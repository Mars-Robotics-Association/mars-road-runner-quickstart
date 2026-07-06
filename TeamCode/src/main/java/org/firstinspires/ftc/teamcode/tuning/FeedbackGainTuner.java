package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.TwoDeadWheelLocalizer;

/**
 * Automatic on-robot tuning of the trajectory-follower feedback gains — the automated counterpart
 * of eyeballing {@code ManualFeedbackTuner}.
 *
 * <p>The identified feedforward already contains a plant model: with voltage-compensated
 * feedforward, each axis closes a loop whose error dynamics are second order with motor time
 * constant {@code tau = kA / kV}. For a chosen bandwidth {@code omega_n} and damping {@code zeta},
 * the gains follow directly: {@code posGain = omega_n^2 * tau} and
 * {@code velGain = 2 * zeta * omega_n * tau - 1} (clamped at 0). So instead of guessing gains, the
 * tuner searches over a single knob — bandwidth.
 *
 * <p>For each axis (heading, then axial, then lateral on mecanum; turn only on tank) it runs step
 * tests: offset the target pose, let the controller drive to it, and measure overshoot, reversals,
 * and settling. Starting from a conservative bandwidth, it raises {@code omega_n} by {@link #GROWTH}
 * each time the response stays clean, and stops at the last clean setting once overshoot or
 * oscillation appears — the classic "turn it up until it rings, then back off," automated. Each
 * iteration steps out and back, so the robot finishes where it started.
 *
 * <p>Prerequisites: localization and feedforward must be tuned first, including {@code kA}
 * (run {@code AxialFeedforwardTuner}; {@code kA} sets the time constant the synthesis relies on).
 * Clear about 2 ft around the robot in every direction it will step. The step tests drive through
 * {@link MecanumDrive#setDriveCommand} / {@link TankDrive#setDriveCommand}, so the exact production
 * feedforward path (anisotropic constants, yaw coupling, voltage compensation) is in the loop.
 *
 * <p>When it finishes, the gains are written into {@code PARAMS} (live for this session) and shown
 * on telemetry to paste into {@code Params}. Verify with {@code ManualFeedbackTuner} and
 * {@code SplineTest}.
 */
@Config
public final class FeedbackGainTuner extends LinearOpMode {
    /** Translational step size for the axial/lateral tests, in inches. */
    public static double STEP_INCHES = 8.0;
    /** Heading step size for the heading/turn tests, in degrees. */
    public static double STEP_DEGREES = 40.0;

    /** Target damping ratio for the synthesized gains (1 = critically damped). */
    public static double ZETA = 1.0;
    /** Initial bandwidth, as a fraction of the plant pole 1/tau. */
    public static double START_BANDWIDTH = 0.7;
    /** Bandwidth multiplier applied after a clean (or sluggish) step response. */
    public static double GROWTH = 1.4;
    /** Bandwidth multiplier applied after a bad response before any clean one is found. */
    public static double BACKOFF = 0.6;
    /** Maximum step-test iterations per axis. */
    public static int MAX_ITERS = 6;
    /** Position gain ceiling; the search stops growing once synthesis would exceed it. */
    public static double MAX_POS_GAIN = 20.0;

    /** Overshoot (fraction of the step) above which a response is rejected. */
    public static double MAX_OVERSHOOT_FRAC = 0.12;
    /** Error sign reversals (beyond the settle band) above which a response is rejected. */
    public static int MAX_REVERSALS = 2;
    /** Settle band for translational steps, in inches. */
    public static double SETTLE_TOL_IN = 0.6;
    /** Settle band for heading steps, in degrees. */
    public static double SETTLE_TOL_DEG = 2.5;
    /** Time the error must stay inside the settle band, in seconds. */
    public static double SETTLE_HOLD_SEC = 0.35;
    /** Per-step timeout, in seconds. */
    public static double STEP_TIMEOUT_SEC = 3.0;

    /** Metrics accumulated over one step response. */
    private static final class StepResponse {
        double e0 = Double.NaN;
        double eLast = Double.NaN;
        double minNorm = 1.0;
        int reversals = 0;
        int lastRegion = 0;
        double settledAt = Double.NaN;
        boolean settled = false;

        /** Ingests one (t, error) sample; returns true once the response has settled. */
        boolean update(double t, double e, double tol) {
            if (Double.isNaN(e0)) e0 = e;
            eLast = e;
            double mag0 = Math.max(Math.abs(e0), 1e-6);
            double n = e / (e0 == 0 ? mag0 : e0);
            minNorm = Math.min(minNorm, n);
            double band = tol / mag0;
            int region = n > band ? 1 : (n < -band ? -1 : 0);
            if (region != 0) {
                if (lastRegion != 0 && region != lastRegion) reversals++;
                lastRegion = region;
            }
            if (Math.abs(e) <= tol) {
                if (Double.isNaN(settledAt)) settledAt = t;
                if (t - settledAt >= SETTLE_HOLD_SEC) {
                    settled = true;
                    return true;
                }
            } else {
                settledAt = Double.NaN;
            }
            return false;
        }

        double overshootFrac() {
            return Math.max(0, -minNorm);
        }

        /** Fraction of the initial error closed by the end of the step. */
        double movedFrac() {
            if (Double.isNaN(e0) || Math.abs(e0) < 1e-6) return 0;
            return 1 - Math.abs(eLast) / Math.abs(e0);
        }

        boolean oscillatory() {
            return overshootFrac() > MAX_OVERSHOOT_FRAC || reversals > MAX_REVERSALS;
        }
    }

    /** Runs one step test on a specific axis: outward from the start pose or back to it. */
    private interface StepTest {
        StepResponse run(boolean outward, double posGain, double velGain);
    }

    private static final class AxisResult {
        final boolean ok;
        final double posGain;
        final double velGain;
        final String note;

        AxisResult(boolean ok, double posGain, double velGain, String note) {
            this.ok = ok;
            this.posGain = posGain;
            this.velGain = velGain;
            this.note = note;
        }
    }

    private MultipleTelemetry telem;

    @Override
    public void runOpMode() throws InterruptedException {
        telem = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            runMecanum();
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            runTank();
        } else {
            throw new RuntimeException("Unknown DRIVE_CLASS");
        }
    }

    private void runMecanum() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        checkDeadWheels(drive.localizer);
        requirePositive(MecanumDrive.PARAMS.kV, MecanumDrive.PARAMS.kA);

        double tauAxial = MecanumDrive.PARAMS.kA / MecanumDrive.PARAMS.kV;
        double tauLateral = tauAxial;
        if (MecanumDrive.PARAMS.useAnisotropicFeedforward
                && MecanumDrive.PARAMS.lateralKV > 0 && MecanumDrive.PARAMS.lateralKA > 0) {
            tauLateral = MecanumDrive.PARAMS.lateralKA / MecanumDrive.PARAMS.lateralKV;
        }

        telem.addLine("Mecanum feedback-gain tuner.");
        telem.addLine("Clear ~2 ft around the robot. It will rotate in place, then step");
        telem.addLine("forward/back, then sideways, adjusting gains after each step pair.");
        telem.update();
        waitForStart();
        if (isStopRequested()) return;

        // Gains active while stepping: index 0-2 pos (axial, lateral, heading), 3-5 vel.
        // Seeded from PARAMS so axes tuned in an earlier session help hold the robot; a failed
        // axis keeps its existing value rather than being zeroed. Heading is tuned first so it
        // can hold the robot straight during the translation steps.
        double[] gains = {
                MecanumDrive.PARAMS.axialGain, MecanumDrive.PARAMS.lateralGain, MecanumDrive.PARAMS.headingGain,
                MecanumDrive.PARAMS.axialVelGain, MecanumDrive.PARAMS.lateralVelGain, MecanumDrive.PARAMS.headingVelGain,
        };

        AxisResult heading = searchAxis("heading", tauAxial,
                mecanumStepTest(drive, gains, 2, new Pose2d(0, 0, Math.toRadians(STEP_DEGREES))));
        if (heading.ok) {
            gains[2] = heading.posGain;
            gains[5] = heading.velGain;
        }

        AxisResult axial = searchAxis("axial", tauAxial,
                mecanumStepTest(drive, gains, 0, new Pose2d(STEP_INCHES, 0, 0)));
        if (axial.ok) {
            gains[0] = axial.posGain;
            gains[3] = axial.velGain;
        }

        AxisResult lateral = searchAxis("lateral", tauLateral,
                mecanumStepTest(drive, gains, 1, new Pose2d(0, STEP_INCHES, 0)));
        if (lateral.ok) {
            gains[1] = lateral.posGain;
            gains[4] = lateral.velGain;
        }

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));

        // Live for the rest of the session; followers read PARAMS gains every loop.
        MecanumDrive.PARAMS.axialGain = gains[0];
        MecanumDrive.PARAMS.lateralGain = gains[1];
        MecanumDrive.PARAMS.headingGain = gains[2];
        MecanumDrive.PARAMS.axialVelGain = gains[3];
        MecanumDrive.PARAMS.lateralVelGain = gains[4];
        MecanumDrive.PARAMS.headingVelGain = gains[5];

        while (opModeIsActive()) {
            telem.addLine("=== Paste into MecanumDrive.Params ===");
            telem.addData("axialGain", "%.2f", gains[0]);
            telem.addData("lateralGain", "%.2f", gains[1]);
            telem.addData("headingGain", "%.2f", gains[2]);
            telem.addData("axialVelGain", "%.2f", gains[3]);
            telem.addData("lateralVelGain", "%.2f", gains[4]);
            telem.addData("headingVelGain", "%.2f", gains[5]);
            telem.addLine();
            telem.addData("axial", axial.note);
            telem.addData("lateral", lateral.note);
            telem.addData("heading", heading.note);
            telem.addLine("Gains are active now — verify with ManualFeedbackTuner / SplineTest.");
            telem.update();
        }
    }

    private void runTank() {
        TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));
        checkDeadWheels(drive.localizer);
        requirePositive(TankDrive.PARAMS.kV, TankDrive.PARAMS.kA);

        double tau = TankDrive.PARAMS.kA / TankDrive.PARAMS.kV;

        telem.addLine("Tank feedback-gain tuner (turnGain / turnVelGain).");
        telem.addLine("The robot will rotate in place; make sure it can spin freely.");
        telem.addLine("Path following uses Ramsete, whose defaults rarely need tuning.");
        telem.update();
        waitForStart();
        if (isStopRequested()) return;

        AxisResult turn = searchAxis("turn", tau,
                tankTurnStepTest(drive, Math.toRadians(STEP_DEGREES)));

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));

        if (turn.ok) {
            TankDrive.PARAMS.turnGain = turn.posGain;
            TankDrive.PARAMS.turnVelGain = turn.velGain;
        }

        while (opModeIsActive()) {
            telem.addLine("=== Paste into TankDrive.Params ===");
            telem.addData("turnGain", "%.2f", TankDrive.PARAMS.turnGain);
            telem.addData("turnVelGain", "%.2f", TankDrive.PARAMS.turnVelGain);
            telem.addLine();
            telem.addData("turn", turn.note);
            telem.addLine("Gains are active now — verify with a TurnAction.");
            telem.update();
        }
    }

    /**
     * Bandwidth search for one axis. Synthesizes (posGain, velGain) from the candidate bandwidth,
     * measures a step pair (out and back), then grows the bandwidth while the response stays clean
     * and stops at the last clean setting once it turns oscillatory.
     */
    private AxisResult searchAxis(String name, double tau, StepTest test) {
        double omega = START_BANDWIDTH / tau;
        double bestKp = Double.NaN, bestKd = Double.NaN;
        String note = "not tuned";

        for (int iter = 1; iter <= MAX_ITERS && opModeIsActive(); iter++) {
            double kp = omega * omega * tau;
            boolean capped = false;
            if (kp > MAX_POS_GAIN) {
                kp = MAX_POS_GAIN;
                omega = Math.sqrt(kp / tau);
                capped = true;
            }
            double kd = Math.max(0, 2 * ZETA * omega * tau - 1);

            telem.addData("axis", name);
            telem.addData("iteration", "%d / %d", iter, MAX_ITERS);
            telem.addData("trying posGain", "%.2f", kp);
            telem.update();

            StepResponse out = test.run(true, kp, kd);
            if (!opModeIsActive()) break;

            if (iter == 1 && out.movedFrac() < 0.5) {
                // The very first, conservative step should easily close most of the error.
                test.run(false, kp, kd); // best effort return to start
                return new AxisResult(false, 0, 0,
                        "FAILED: robot barely moved on the first step — check feedforward and localization");
            }

            StepResponse back = test.run(false, kp, kd);
            if (!opModeIsActive()) break;

            boolean oscillatory = out.oscillatory() || back.oscillatory();
            boolean settled = out.settled && back.settled;

            if (oscillatory) {
                // Return to the start pose gently before the next attempt.
                test.run(false, kp * 0.25, kd);
                if (!Double.isNaN(bestKp)) {
                    note = String.format("ok: rang at posGain %.2f, kept last clean setting", kp);
                    break;
                }
                omega *= BACKOFF;
                note = "FAILED: oscillatory even at the lowest bandwidth tried";
            } else if (settled) {
                bestKp = kp;
                bestKd = kd;
                note = String.format("ok: clean at posGain %.2f (overshoot %.0f%%)",
                        kp, 100 * Math.max(out.overshootFrac(), back.overshootFrac()));
                if (capped) break;
                omega *= GROWTH;
            } else {
                // No ringing but never settled inside the band: sluggish (e.g. stiction floor).
                // More bandwidth is the right direction, but don't record this as a clean setting.
                omega *= GROWTH;
                if (Double.isNaN(bestKp)) {
                    note = "FAILED: never settled — check kS or widen SETTLE_TOL";
                }
                if (capped) break;
            }
        }

        if (Double.isNaN(bestKp)) {
            return new AxisResult(false, 0, 0, note);
        }
        return new AxisResult(true, bestKp, bestKd, note);
    }

    /** Step test for one mecanum axis: 0 = axial, 1 = lateral, 2 = heading. */
    private StepTest mecanumStepTest(MecanumDrive drive, double[] gains, int axis, Pose2d delta) {
        final Pose2d start = drive.localizer.getPose();
        final Pose2d out = start.times(delta);
        final double tol = axis == 2 ? Math.toRadians(SETTLE_TOL_DEG) : SETTLE_TOL_IN;

        return (outward, posGain, velGain) -> {
            Pose2d target = outward ? out : start;
            Pose2dDual<Time> txWorldTarget = Pose2dDual.constant(target, 3);

            double[] g = gains.clone();
            g[axis] = posGain;
            g[axis + 3] = velGain;

            StepResponse resp = new StepResponse();
            ElapsedTime timer = new ElapsedTime();
            while (opModeIsActive() && timer.seconds() < STEP_TIMEOUT_SEC) {
                PoseVelocity2d vel = drive.updatePoseEstimate();
                Pose2d pose = drive.localizer.getPose();
                Pose2d error = target.minusExp(pose);
                double e = axis == 0 ? error.position.x
                        : axis == 1 ? error.position.y
                        : error.heading.log();
                if (resp.update(timer.seconds(), e, tol)) break;

                PoseVelocity2dDual<Time> command = new HolonomicController(
                        g[0], g[1], g[2], g[3], g[4], g[5]
                ).compute(txWorldTarget, pose, vel);
                drive.setDriveCommand(command);
            }
            return resp;
        };
    }

    /** Step test for the tank turn controller, mirroring TurnAction's command structure. */
    private StepTest tankTurnStepTest(TankDrive drive, double stepRad) {
        final Rotation2d start = drive.localizer.getPose().heading;
        final Rotation2d out = start.plus(stepRad);
        final double tol = Math.toRadians(SETTLE_TOL_DEG);

        return (outward, posGain, velGain) -> {
            Rotation2d target = outward ? out : start;

            StepResponse resp = new StepResponse();
            ElapsedTime timer = new ElapsedTime();
            while (opModeIsActive() && timer.seconds() < STEP_TIMEOUT_SEC) {
                PoseVelocity2d vel = drive.updatePoseEstimate();
                double e = target.minus(drive.localizer.getPose().heading);
                if (resp.update(timer.seconds(), e, tol)) break;

                PoseVelocity2dDual<Time> command = new PoseVelocity2dDual<>(
                        Vector2dDual.constant(new Vector2d(0, 0), 3),
                        DualNum.constant(posGain * e - velGain * vel.angVel, 3)
                );
                drive.setDriveCommand(command);
            }
            return resp;
        };
    }

    private static void requirePositive(double kV, double kA) {
        if (kV <= 0 || kA <= 0) {
            throw new RuntimeException(
                    "FeedbackGainTuner needs positive kV and kA (tau = kA/kV sets the plant model)."
                            + " Run AxialFeedforwardTuner first.");
        }
    }

    private static void checkDeadWheels(Object localizer) {
        if (localizer instanceof TwoDeadWheelLocalizer) {
            if (TwoDeadWheelLocalizer.PARAMS.perpXTicks == 0 && TwoDeadWheelLocalizer.PARAMS.parYTicks == 0) {
                throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
            }
        } else if (localizer instanceof ThreeDeadWheelLocalizer) {
            if (ThreeDeadWheelLocalizer.PARAMS.perpXTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par0YTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par1YTicks == 1) {
                throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
            }
        }
    }
}
