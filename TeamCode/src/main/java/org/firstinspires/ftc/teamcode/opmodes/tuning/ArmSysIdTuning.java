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
 * by logging encoder position (not hub velocity) during open-loop voltage runs and regressing on
 * integrated intervals so acceleration is never differentiated from noisy velocity.
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
 *   <li>Press <b>A</b> for a +voltage run, <b>X</b> for a −voltage run. Change
 *       {@code RUN_VOLTAGE} between runs for more excitation.</li>
 *   <li>Collect several runs in both directions (near-horizontal starts help gravity).</li>
 *   <li>Press <b>B</b> to solve OLS and show results until stop.</li>
 * </ol>
 *
 * <p>Unlike {@link ArmFeedforwardTuning} (quasistatic + separate step for kA), this identifies
 * all five coefficients in one multi-run OLS fit — the same algorithm as ControlLab's "Run SysID".
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
    }

    public static Params PARAMS = new Params();

    private BulkReads bulkReads;
    private DcMotorEx motor;
    private LynxModule hub;
    private double ticksToRad;

    private final List<double[]> rows = new ArrayList<>();
    private final List<Double> rhs = new ArrayList<>();
    private int runsCompleted = 0;
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

        telemetry.addLine("ArmSysId — integrated EOM (ControlLab method)");
        telemetry.addLine("Set MIN/MAX_ANGLE_DEG to your hard stops (Dashboard).");
        telemetry.addLine("A = +V run, X = −V run, B = solve.");
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
            telemetry.addLine("── collect runs ──");
            telemetry.addData("A", "+%.1f V run", PARAMS.RUN_VOLTAGE);
            telemetry.addData("X", "−%.1f V run", PARAMS.RUN_VOLTAGE);
            telemetry.addData("B", "solve OLS");
            addRangeTelemetry();
            telemetry.addData("V_batt (live)", "%.2f V", battV);
            telemetry.addData("RUN_VOLTAGE", "%.2f V (Dashboard)", PARAMS.RUN_VOLTAGE);
            telemetry.addData("runs / OLS rows", "%d / %d", runsCompleted, rows.size());
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
            } else if (gamepad1.bWasPressed()) {
                break;
            }
        }

        cutPower();
        if (!opModeIsActive()) {
            return;
        }

        ArmSysId.Result r = ArmSysId.solve(rows, rhs);
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
            telemetry.addData("OLS rows so far", rows.size());
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

        int before = rows.size();
        ArmSysId.accumulateRun(
                theta,
                voltage,
                timeSec,
                minAngleRad(),
                maxAngleRad(),
                ArmSysId.DEFAULT_PARAMS,
                rows,
                rhs);
        totalSamples += n;
        runsCompleted++;

        double meanDtMs = n >= 2
                ? 1000.0 * (timeSec[n - 1] - timeSec[0]) / (n - 1)
                : 0.0;
        telemetry.addData("Run", "%s done: %d samples mean dt=%.1f ms → +%d OLS rows",
                label, n, meanDtMs, rows.size() - before);
        telemetry.update();
        return true;
    }

    private void showResults(ArmSysId.Result r) {
        while (opModeIsActive()) {
            bulkReads.readAll();
            double battV = hub.getInputVoltage(VoltageUnit.VOLTS);

            telemetry.addLine("── ArmSysId RESULTS (integrated EOM) ──");
            if (r.samples < 5) {
                telemetry.addData("Fit", "FAILED – only %d intervals (need ≥ 5). "
                        + "Add more A/X runs at several voltages and angles.",
                        r.samples);
            } else {
                telemetry.addData("kS", "%.4f V", r.kS);
                telemetry.addData("kV", "%.4f V·s/rad", r.kV);
                telemetry.addData("kA", "%.4f V·s²/rad", r.kA);
                telemetry.addData("kCos", "%.4f V", r.kCos);
                telemetry.addData("kSin", "%.4f V", r.kSin);
                telemetry.addData("R²", "%.4f", r.rSquared);
                telemetry.addData("intervals", r.samples);
                telemetry.addData("runs / raw samples", "%d / %d", runsCompleted, totalSamples);
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
