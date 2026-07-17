package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.robot.BulkReads;
import org.firstinspires.ftc.teamcode.utils.DashboardTelemetryPacketAccess;
import org.firstinspires.ftc.teamcode.utils.HubHelper;
import org.marsroboticsassociation.controllib.mechanism.ArmSysId;

import java.util.ArrayList;
import java.util.List;

/**
 * On-robot arm system ID using ControlLab's integrated equation-of-motion method
 * ({@link ArmSysId}).
 *
 * <p>Recovers the five-parameter model used by {@code ArmModel} / mechanism controllers:
 *
 * <pre>
 *   V = kS·sign(ω) + kV·ω + kA·α + kCos·cos(θ) + kSin·sin(θ)
 * </pre>
 *
 * by logging encoder position (not hub velocity) and regressing on integrated intervals so
 * acceleration is never differentiated from noisy velocity.
 *
 * <p><b>Two-stage fit.</b> Constant-velocity <b>holds</b> (Y / right bumper) pin {@code kS} and
 * gravity quasi-statically — at a held speed α ≈ 0, so a lashy/flexy drivetrain cannot inflate
 * {@code kS} (the direction-flipping flex/lash deflection would otherwise land in the {@code sign(ω)}
 * term). Constant-voltage <b>runs</b> (A / X) excite {@code Δω} and pin {@code kV} and {@code kA}.
 * {@link ArmSysId#solveTwoStage} combines them. An over-estimated {@code kS} is an anti-braking
 * feedforward term that brings back arrival overshoot, so the holds matter on a real arm.
 *
 * <p>Each control loop reads the hub battery voltage and commands
 * {@code power = clamp(V_cmd / V_batt)} so the applied voltage tracks the requested volts under
 * battery sag. Samples are taken every loop with wall-clock timestamps — no sleep, no paced
 * sample rate. The fit uses measured applied voltage and actual loop {@code dt}.
 *
 * <p><b>Fixed range of motion</b> — set {@link Params#MIN_ANGLE_DEG} and
 * {@link Params#MAX_ANGLE_DEG} (deg from horizontal) to your mechanical hard stops. Open-loop
 * runs soft-stop {@link Params#RUN_STOP_MARGIN_DEG} inside that range; the OLS fit also discards
 * samples near the stops so contact forces do not pollute kS / gravity.
 *
 * <p><b>Procedure</b>
 * <ol>
 *   <li>Set range, motor name, and {@code TICKS_PER_ARM_REV} on Dashboard (or in code).</li>
 *   <li>Press Start. Idle loop shows θ vs the configured range and live battery voltage.</li>
 *   <li><b>Holds (kS, gravity):</b> park the arm near one hard stop, then press <b>Y</b> (sweep up)
 *       or <b>right bumper</b> (sweep down) to hold {@code HOLD_SPEED} across the range. Collect a
 *       few speeds in both directions (change {@code HOLD_SPEED} between holds).</li>
 *   <li><b>Runs (kV, kA):</b> press <b>A</b> for a +voltage run, <b>X</b> for a −voltage run;
 *       change {@code RUN_VOLTAGE} between runs. Near-horizontal starts help.</li>
 *   <li>Press <b>B</b> to solve (two-stage) and show results until stop.</li>
 * </ol>
 *
 * <p>Unlike {@link ArmFeedforwardTuning} (quasistatic + separate step for kA), this recovers all
 * five coefficients from the same encoder-log battery as ControlLab's "Run SysID".
 */
@Config
@TeleOp(name = "ArmSysIdTuning", group = "Tuning")
public class ArmSysIdTuning extends LinearOpMode {

    public static class Params {
        /** Hardware map name of the arm motor. */
        public String MOTOR_NAME = "arm";
        /**
         * Encoder ticks for one full arm revolution (geared).
         * = motor encoder PPR × external gear ratio (e.g. 28 × 19.2 ≈ 537.6).
         */
        public double TICKS_PER_ARM_REV = 537.7;
        /**
         * Angle from horizontal (deg) when the encoder reads 0. If unknown, leave 0 — kSin absorbs
         * residual phase; use {@link ArmSysId.Result#phiRad()} to correct later.
         */
        public double ENCODER_ZERO_OFFSET_DEG = 0.0;
        /**
         * Fixed range of motion: lower hard stop (deg from horizontal). Soft-stop and OLS both
         * respect this bound. Example: −45 = 45° below horizontal in front.
         */
        public double MIN_ANGLE_DEG = -45.0;
        /**
         * Fixed range of motion: upper hard stop (deg from horizontal). Must be greater than
         * {@link #MIN_ANGLE_DEG}. Example: 225 for a 270° over-the-top workspace from −45°.
         */
        public double MAX_ANGLE_DEG = 225.0;
        /**
         * Soft-stop buffer (deg) inside min…max so open-loop runs never drive into the hard stops.
         * The fit uses a slightly smaller margin (~4°) of its own.
         */
        public double RUN_STOP_MARGIN_DEG = 8.0;
        /**
         * Commanded voltage magnitude (volts) for A/X runs. Power is recomputed each loop as
         * {@code V_cmd / V_batt} from the live hub reading.
         */
        public double RUN_VOLTAGE = 6.0;
        /** Duration of each run (seconds). ControlLab uses 0.9 s. */
        public double RUN_DURATION_S = 0.9;
        /**
         * Target speed magnitude (rad/s) for a Y / right-bumper constant-velocity hold. Collect
         * several holds at a few speeds in both directions: the holds pin kS and gravity
         * quasi-statically (α ≈ 0), which keeps a lashy/flexy drivetrain from inflating kS. Change
         * this between holds for a wider speed spread (helps kV).
         */
        public double HOLD_SPEED = 1.5;
        /** Velocity-hold P gain, volts per rad/s (saturates warmup, then holds the speed). */
        public double HOLD_KP = 8.0;
        /** Velocity-hold I gain, volts per rad. */
        public double HOLD_KI = 20.0;
        /** Max duration of a single hold sweep (seconds) before it gives up. */
        public double HOLD_MAX_S = 5.0;
    }

    public static Params PARAMS = new Params();

    private BulkReads bulkReads;
    private DcMotorEx motor;
    private LynxModule hub;
    private double ticksToRad;

    // Stage 2 data (kV, kA): constant-voltage A/X runs.
    private final List<double[]> movingRows = new ArrayList<>();
    private final List<Double> movingRhs = new ArrayList<>();
    // Stage 1 data (kS, gravity): constant-velocity Y / right-bumper holds.
    private final List<double[]> holdRows = new ArrayList<>();
    private final List<Double> holdRhs = new ArrayList<>();
    private int runsCompleted = 0;
    private int holdsCompleted = 0;
    private int totalSamples = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, PARAMS.MOTOR_NAME);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        var packetAccess = new DashboardTelemetryPacketAccess();
        telemetry = new MultipleTelemetry(telemetry, packetAccess.dashboardTelemetry);

        hub = HubHelper.getHubForMotor(motor, hardwareMap);
        bulkReads = new BulkReads(hardwareMap);
        ticksToRad = 2.0 * Math.PI / PARAMS.TICKS_PER_ARM_REV;

        telemetry.addLine("ArmSysId — two-stage (holds for kS/gravity, runs for kV/kA)");
        telemetry.addLine("Set MIN/MAX_ANGLE_DEG to your hard stops (Dashboard).");
        telemetry.addLine("A = +V run, X = −V run, Y = +speed hold, RB = −speed hold, B = solve.");
        addRangeTelemetry();
        telemetry.update();
        waitForStart();
        if (isStopRequested()) {
            return;
        }

        // Collect runs until B.
        while (opModeIsActive()) {
            bulkReads.readAll();
            cutPower();

            double battV = hub.getInputVoltage(VoltageUnit.VOLTS);
            telemetry.addLine("── collect ──");
            telemetry.addData("A / X", "±%.1f V run (kV, kA)", PARAMS.RUN_VOLTAGE);
            telemetry.addData("Y / RB", "±%.1f rad/s hold (kS, gravity)", PARAMS.HOLD_SPEED);
            telemetry.addData("B", "solve (two-stage)");
            addRangeTelemetry();
            telemetry.addData("V_batt (live)", "%.2f V", battV);
            telemetry.addData("RUN_VOLTAGE / HOLD_SPEED", "%.2f V / %.2f rad/s",
                    PARAMS.RUN_VOLTAGE, PARAMS.HOLD_SPEED);
            telemetry.addData("runs / moving rows", "%d / %d", runsCompleted, movingRows.size());
            telemetry.addData("holds / hold rows", "%d / %d", holdsCompleted, holdRows.size());
            telemetry.addData("raw samples", totalSamples);
            telemetry.update();

            if (gamepad1.aWasPressed()) {
                if (!runVoltageCommand(+PARAMS.RUN_VOLTAGE, "+V")) {
                    return;
                }
            } else if (gamepad1.xWasPressed()) {
                if (!runVoltageCommand(-PARAMS.RUN_VOLTAGE, "−V")) {
                    return;
                }
            } else if (gamepad1.yWasPressed()) {
                if (!runVelocityHold(+PARAMS.HOLD_SPEED, "+hold")) {
                    return;
                }
            } else if (gamepad1.rightBumperWasPressed()) {
                if (!runVelocityHold(-PARAMS.HOLD_SPEED, "−hold")) {
                    return;
                }
            } else if (gamepad1.bWasPressed()) {
                break;
            }
        }

        cutPower();
        if (!opModeIsActive()) {
            return;
        }

        ArmSysId.Result r =
                ArmSysId.solveTwoStage(holdRows, holdRhs, movingRows, movingRhs);
        showResults(r);
    }

    /**
     * Open-loop run targeting {@code voltageCmd} volts for {@link Params#RUN_DURATION_S}.
     * Every loop: bulk-read, measure hub voltage, set {@code power = clamp(V_cmd / V_batt)},
     * log θ / V / wall time at the natural loop rate. Soft-stops at the fixed range margins.
     *
     * @return false if the OpMode stopped
     */
    private boolean runVoltageCommand(double voltageCmd, String label) {
        List<Double> thetaList = new ArrayList<>();
        List<Double> voltList = new ArrayList<>();
        List<Double> timeList = new ArrayList<>();
        ElapsedTime run = new ElapsedTime();
        double loSoft = minAngleRad() + stopMarginRad();
        double hiSoft = maxAngleRad() - stopMarginRad();
        double lastT = 0.0;

        run.reset();

        while (opModeIsActive() && run.seconds() < PARAMS.RUN_DURATION_S) {
            bulkReads.readAll();

            double t = run.seconds();
            double dt = t - lastT;
            lastT = t;

            double battV = hub.getInputVoltage(VoltageUnit.VOLTS);
            // Guard divide-by-zero / brownout; recompute power every loop from live V_batt.
            double power = battV > 0.5 ? Range.clip(voltageCmd / battV, -1.0, 1.0) : 0.0;
            motor.setPower(power);
            double vApplied = power * battV;

            double theta = readThetaRad();

            // Soft stop only in the direction of commanded voltage (respects fixed ROM).
            if ((voltageCmd > 0 && theta >= hiSoft) || (voltageCmd < 0 && theta <= loSoft)) {
                telemetry.addData("Run", "%s — soft stop at θ=%.1f° (range %.0f…%.0f°)",
                        label, Math.toDegrees(theta), PARAMS.MIN_ANGLE_DEG, PARAMS.MAX_ANGLE_DEG);
                telemetry.update();
                break;
            }

            // One sample per loop; wall-clock time carries the actual dt into the fit.
            thetaList.add(theta);
            voltList.add(vApplied);
            timeList.add(t);

            telemetry.addData("Run", "%s  V_cmd=%.2f", label, voltageCmd);
            telemetry.addData("t", "%.2f / %.2f s", t, PARAMS.RUN_DURATION_S);
            telemetry.addData("loop dt", "%.1f ms", dt * 1000.0);
            telemetry.addData("V_batt (live)", "%.2f V", battV);
            telemetry.addData("power (cmd)", "%.3f", power);
            telemetry.addData("V applied", "%.2f V", vApplied);
            addRangeTelemetry();
            telemetry.addData("samples this run", thetaList.size());
            telemetry.addData("moving rows so far", movingRows.size());
            telemetry.update();
        }

        cutPower();
        if (!opModeIsActive()) {
            return false;
        }

        int n = thetaList.size();
        if (n < 10) {
            telemetry.addData("Run", "%s — too few samples (%d), skipped", label, n);
            telemetry.update();
            return true;
        }

        double[] theta = new double[n];
        double[] voltage = new double[n];
        double[] timeSec = new double[n];
        for (int i = 0; i < n; i++) {
            theta[i] = thetaList.get(i);
            voltage[i] = voltList.get(i);
            timeSec[i] = timeList.get(i);
        }

        int before = movingRows.size();
        ArmSysId.accumulateRun(
                theta,
                voltage,
                timeSec,
                minAngleRad(),
                maxAngleRad(),
                ArmSysId.DEFAULT_PARAMS,
                movingRows,
                movingRhs);
        totalSamples += n;
        runsCompleted++;

        double meanDtMs = n >= 2
                ? 1000.0 * (timeSec[n - 1] - timeSec[0]) / (n - 1)
                : 0.0;
        telemetry.addData("Run", "%s done: %d samples mean dt=%.1f ms → +%d moving rows",
                label, n, meanDtMs, movingRows.size() - before);
        telemetry.update();
        return true;
    }

    /**
     * Constant-velocity hold targeting {@code targetVel} rad/s: a PI loop on velocity error drives
     * the arm across the range at a held speed, logging θ / applied V / wall time each loop. The
     * held speed keeps acceleration ≈ 0, so {@link ArmSysId#accumulateHold} recovers a quasi-static
     * {@code kS} and gravity that a lashy/flexy drivetrain would otherwise corrupt. Start the arm
     * near the opposite hard stop so the hold sweeps a wide angle band. Soft-stops at the fixed
     * range margins.
     *
     * @return false if the OpMode stopped
     */
    private boolean runVelocityHold(double targetVel, String label) {
        List<Double> thetaList = new ArrayList<>();
        List<Double> voltList = new ArrayList<>();
        List<Double> timeList = new ArrayList<>();
        ElapsedTime run = new ElapsedTime();
        double loSoft = minAngleRad() + stopMarginRad();
        double hiSoft = maxAngleRad() - stopMarginRad();
        double lastT = 0.0;
        double integralV = 0.0;

        run.reset();

        while (opModeIsActive() && run.seconds() < PARAMS.HOLD_MAX_S) {
            bulkReads.readAll();

            double t = run.seconds();
            double dt = t - lastT;
            lastT = t;

            double battV = hub.getInputVoltage(VoltageUnit.VOLTS);
            double vel = motor.getVelocity() * ticksToRad; // encoder velocity, rad/s
            double errV = targetVel - vel;
            integralV += errV * dt;
            double vCmd = PARAMS.HOLD_KP * errV + PARAMS.HOLD_KI * integralV;
            double power = battV > 0.5 ? Range.clip(vCmd / battV, -1.0, 1.0) : 0.0;
            motor.setPower(power);
            double vApplied = power * battV;

            double theta = readThetaRad();

            // Sweep done once we reach the far margin in the direction of travel.
            if ((targetVel > 0 && theta >= hiSoft) || (targetVel < 0 && theta <= loSoft)) {
                if (!thetaList.isEmpty()) {
                    break;
                }
            } else {
                // Log while inside the range; accumulateHold's α gate drops the warmup samples.
                thetaList.add(theta);
                voltList.add(vApplied);
                timeList.add(t);
            }

            telemetry.addData("Hold", "%s  target=%.2f rad/s", label, targetVel);
            telemetry.addData("t", "%.2f / %.2f s", t, PARAMS.HOLD_MAX_S);
            telemetry.addData("ω (meas)", "%.2f rad/s", vel);
            telemetry.addData("V applied", "%.2f V", vApplied);
            addRangeTelemetry();
            telemetry.addData("samples this hold", thetaList.size());
            telemetry.addData("hold rows so far", holdRows.size());
            telemetry.update();
        }

        cutPower();
        if (!opModeIsActive()) {
            return false;
        }

        int n = thetaList.size();
        if (n < 12) {
            telemetry.addData("Hold", "%s — too few samples (%d), skipped. "
                    + "Start near the opposite stop.", label, n);
            telemetry.update();
            return true;
        }

        double[] theta = new double[n];
        double[] voltage = new double[n];
        double[] timeSec = new double[n];
        for (int i = 0; i < n; i++) {
            theta[i] = thetaList.get(i);
            voltage[i] = voltList.get(i);
            timeSec[i] = timeList.get(i);
        }

        int before = holdRows.size();
        ArmSysId.accumulateHold(
                theta,
                voltage,
                timeSec,
                minAngleRad(),
                maxAngleRad(),
                ArmSysId.DEFAULT_PARAMS,
                holdRows,
                holdRhs);
        totalSamples += n;
        holdsCompleted++;

        telemetry.addData("Hold", "%s done: %d samples → +%d hold rows (steady)",
                label, n, holdRows.size() - before);
        telemetry.update();
        return true;
    }

    private void showResults(ArmSysId.Result r) {
        while (opModeIsActive()) {
            bulkReads.readAll();
            double battV = hub.getInputVoltage(VoltageUnit.VOLTS);

            telemetry.addLine("── ArmSysId RESULTS (two-stage) ──");
            if (r.samples < 5) {
                telemetry.addData("Fit", "FAILED – only %d rows (need ≥ 5). Add Y/RB holds "
                        + "(kS, gravity) and A/X runs (kV, kA) at several speeds and angles.",
                        r.samples);
            } else {
                telemetry.addData("kS", "%.4f V", r.kS);
                telemetry.addData("kV", "%.4f V·s/rad", r.kV);
                telemetry.addData("kA", "%.4f V·s²/rad", r.kA);
                telemetry.addData("kCos", "%.4f V", r.kCos);
                telemetry.addData("kSin", "%.4f V", r.kSin);
                telemetry.addData("R²", "%.4f", r.rSquared);
                telemetry.addData("rows (hold+moving)", "%d + %d", holdRows.size(), movingRows.size());
                telemetry.addData("runs / holds / samples", "%d / %d / %d",
                        runsCompleted, holdsCompleted, totalSamples);
                telemetry.addLine("");
                telemetry.addLine("── derived (ArmFeedforward form) ──");
                telemetry.addData("kG", "%.4f V  (= √(kCos²+kSin²))", r.kG());
                telemetry.addData("φ", "%.1f°  (encoder zero from horizontal)",
                        Math.toDegrees(r.phiRad()));
                telemetry.addLine("");
                telemetry.addLine("── PASTE INTO ArmModel ──");
                telemetry.addData("  kS", "%.4f", r.kS);
                telemetry.addData("  kV", "%.4f", r.kV);
                telemetry.addData("  kA", "%.4f", r.kA);
                telemetry.addData("  kCos", "%.4f", r.kCos);
                telemetry.addData("  kSin", "%.4f", r.kSin);
            }
            telemetry.addData("range (deg)", "%.0f … %.0f", PARAMS.MIN_ANGLE_DEG, PARAMS.MAX_ANGLE_DEG);
            telemetry.addData("V_batt (live)", "%.2f V", battV);
            telemetry.update();
        }
    }

    /** Live θ and configured ROM (degrees) for Dashboard / DS. */
    private void addRangeTelemetry() {
        double thetaDeg = Math.toDegrees(readThetaRad());
        double softLo = PARAMS.MIN_ANGLE_DEG + PARAMS.RUN_STOP_MARGIN_DEG;
        double softHi = PARAMS.MAX_ANGLE_DEG - PARAMS.RUN_STOP_MARGIN_DEG;
        telemetry.addData("θ", "%.1f°  (ticks %d)", thetaDeg, motor.getCurrentPosition());
        telemetry.addData("ROM hard", "%.0f° … %.0f°", PARAMS.MIN_ANGLE_DEG, PARAMS.MAX_ANGLE_DEG);
        telemetry.addData("ROM soft", "%.0f° … %.0f°  (stop margin %.0f°)",
                softLo, softHi, PARAMS.RUN_STOP_MARGIN_DEG);
    }

    private double minAngleRad() {
        return Math.toRadians(PARAMS.MIN_ANGLE_DEG);
    }

    private double maxAngleRad() {
        return Math.toRadians(PARAMS.MAX_ANGLE_DEG);
    }

    private double stopMarginRad() {
        return Math.toRadians(PARAMS.RUN_STOP_MARGIN_DEG);
    }

    private double readThetaRad() {
        return motor.getCurrentPosition() * ticksToRad
                + Math.toRadians(PARAMS.ENCODER_ZERO_OFFSET_DEG);
    }

    private void cutPower() {
        if (motor != null) {
            motor.setPower(0);
        }
    }
}
