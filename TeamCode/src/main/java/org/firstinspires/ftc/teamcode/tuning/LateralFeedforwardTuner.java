package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

/**
 * Automatic on-robot identification of the anisotropic (strafe) feedforward constants {@code
 * lateralKS} / {@code lateralKV} / {@code lateralKA} for a mecanum drive.
 *
 * <p>Same two-phase procedure as {@link AxialFeedforwardTuner} (slow ramp → kS/kV, reverse square
 * wave → kA), applied as a pure strafe with a light heading hold so open-loop roller scrub does not
 * accumulate yaw. The ramp drives <b>LEFT</b> (robot +y); it ends early if motion collapses under
 * power (or on gamepad1 A), then reverses RIGHT for kA. See {@link ReversalFeedforwardId}.
 *
 * <p>Prerequisites: localization and axial feedforward already tuned. Leave room on the right for
 * reverse; a wall on the left is fine. After pasting, set {@code useAnisotropicFeedforward = true}.
 * Expect lateral constants to exceed their axial counterparts (roller scrub).
 */
@Config
public final class LateralFeedforwardTuner extends LinearOpMode {
    /** Power increase per second during the lateral ramp. */
    public static double RAMP_POWER_PER_SEC = 0.1;

    /** Peak power of the ramp phase. */
    public static double RAMP_MAX = 0.9;

    /** |Power| of the reverse square wave used only for lateralKA. */
    public static double KA_POWER = 0.6;

    /**
     * Fallback seconds per reverse direction when ramp travel is too short for a position-based
     * corridor (see {@link #MIN_TRAVEL_IN}).
     */
    public static double HALF_CYCLE = 1.0;

    /** Number of reverse half-cycles. */
    public static int KA_HALF_CYCLES = 4;

    /** Inches kept clear of each end of the measured ramp corridor. */
    public static double END_MARGIN_IN = ReversalFeedforwardId.DEFAULT_END_MARGIN_IN;

    /** Minimum |ramp travel| (in) to use position-based reverse half-cycles. */
    public static double MIN_TRAVEL_IN = ReversalFeedforwardId.DEFAULT_MIN_TRAVEL_IN;

    /** Speeds (in/s) within this of zero use sign=0 in the kA residual. */
    public static double SIGN_DEADBAND = 1.0;

    /**
     * Ramp samples slower than this (encoder-tick units per second) are excluded from the kS/kV
     * fit. Converted with {@code inPerTick}; default 1000 cuts the breakaway knee at low speed.
     */
    public static double MIN_RAMP_TICKS_PER_SEC =
            ReversalFeedforwardId.DEFAULT_MIN_RAMP_TICKS_PER_SEC;

    /** Absolute speed (in/s) treated as stalled after the robot has been moving. */
    public static double STALL_SPEED = ReversalFeedforwardId.DEFAULT_STALL_SPEED;

    /** Seconds velocity must stay collapsed before ending the ramp for a wall/stall. */
    public static double STALL_TIME = ReversalFeedforwardId.DEFAULT_STALL_TIME;

    /** Speed (in/s) that must be reached once before stall detection arms. */
    public static double MOVING_SPEED = ReversalFeedforwardId.DEFAULT_MOVING_SPEED;

    /** Minimum commanded power for a stall to count. */
    public static double STALL_MIN_POWER = ReversalFeedforwardId.DEFAULT_STALL_MIN_POWER;

    /** Fraction of peak ramp speed below which velocity counts as collapsed. */
    public static double STALL_FRAC = ReversalFeedforwardId.DEFAULT_STALL_FRAC;

    /**
     * |Travel| (in) that arms stall detection even if speed never reached {@link #MOVING_SPEED}
     * (lateral ramps often hit a wall before that).
     */
    public static double ARM_TRAVEL_IN = ReversalFeedforwardId.DEFAULT_ARM_TRAVEL_IN;

    /**
     * When armed, |d(pos)/dt| below this (in/s) under power counts as wall contact (helps when
     * mecanum rollers keep velocity slightly nonzero against a wall).
     */
    public static double POS_STALL_SPEED = ReversalFeedforwardId.DEFAULT_POS_STALL_SPEED;

    /**
     * Heading hold gain (power per rad of error). Open-loop strafe otherwise curls; this adds a
     * pure yaw correction so lateral sysid stays roughly straight. 0 disables.
     */
    public static double HEADING_GAIN = 1.2;

    /** Heading-rate damping (power per rad/s). */
    public static double HEADING_VEL_GAIN = 0.08;

    /** Max |yaw power| blended into the strafe command. */
    public static double HEADING_MAX_CORR = 0.35;

    @Override
    public void runOpMode() throws InterruptedException {
        if (!TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            throw new RuntimeException(
                    "LateralFeedforwardTuner is mecanum-only; a "
                            + (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)
                                    ? "tank"
                                    : "non-mecanum")
                            + " drive has no lateral motion.");
        }

        MultipleTelemetry telemetry =
                new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        double inPerTick = MecanumDrive.PARAMS.inPerTick;
        double lateralMultiplier = drive.kinematics.lateralMultiplier;

        telemetry.addLine("Mecanum lateral (strafe) feedforward tuner (kS, kV, kA).");
        telemetry.addLine("Phase 1: ramps LEFT (robot +y) → lateralKS/KV.");
        telemetry.addLine("  Press gamepad1 A just before wall/mat edge (recommended).");
        telemetry.addLine("  Auto-stops if stalled against a wall (wheels may jam).");
        telemetry.addLine("  Heading hold on (HEADING_GAIN) to limit yaw curl.");
        telemetry.addLine("Phase 2: reverse square wave → lateralKA (first goes RIGHT).");
        telemetry.addLine("  Half-cycles use the distance driven on the ramp when possible.");
        telemetry.addLine("Press START. Leave room on the RIGHT for reverse.");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        // Hold the heading at START so open-loop strafe reversals do not walk in yaw.
        drive.localizer.update();
        final Rotation2d headingTarget = drive.localizer.getPose().heading;

        // +power => strafe (+y = robot LEFT) plus a pure-yaw heading correction.
        // Kinematics: strafe p → (-p, +p, -p, +p); +angVel u → (-u, -u, +u, +u).
        // Prefer preserving lateral |p| for the sysid voltage model: shrink yaw correction
        // if |p| + |u| would saturate, rather than scaling all four wheels down.
        DoubleConsumer setPower =
                p -> {
                    PoseVelocity2d tw = drive.localizer.update();
                    double err = headingTarget.minus(drive.localizer.getPose().heading);
                    double u = HEADING_GAIN * err - HEADING_VEL_GAIN * tw.angVel;
                    double uMax = Math.min(HEADING_MAX_CORR, Math.max(0.0, 1.0 - Math.abs(p)));
                    u = Range.clip(u, -uMax, uMax);

                    drive.leftFront.setPower(-p - u);
                    drive.leftBack.setPower(+p - u);
                    drive.rightBack.setPower(-p + u);
                    drive.rightFront.setPower(+p + u);
                };
        // Localizer already updated in setPower; read lateral velocity without a second update
        // when possible. identify() calls setPower then wheelVel each loop — one update in
        // setPower is enough; wheelVel re-reads pose velocity from the last update by calling
        // update() again (Pinpoint is cheap / consistent).
        DoubleSupplier wheelVel = () -> drive.localizer.update().linearVel.y * lateralMultiplier;
        DoubleSupplier axisPos = () -> drive.localizer.getPose().position.y;

        ReversalFeedforwardId.Result fit =
                ReversalFeedforwardId.identify(
                        this,
                        telemetry,
                        setPower,
                        wheelVel,
                        axisPos,
                        drive.voltageSensor,
                        inPerTick,
                        RAMP_POWER_PER_SEC,
                        RAMP_MAX,
                        KA_POWER,
                        HALF_CYCLE,
                        KA_HALF_CYCLES,
                        SIGN_DEADBAND,
                        STALL_SPEED,
                        STALL_TIME,
                        MOVING_SPEED,
                        STALL_MIN_POWER,
                        STALL_FRAC,
                        MIN_RAMP_TICKS_PER_SEC,
                        END_MARGIN_IN,
                        MIN_TRAVEL_IN,
                        ARM_TRAVEL_IN,
                        POS_STALL_SPEED);

        while (opModeIsActive()) {
            if (fit.singular) {
                telemetry.addLine("Fit failed: " + fit.message);
                telemetry.addData("samples", fit.samples);
                telemetry.addData("ramp samples used", fit.rampSamples);
                // Ramp kS/kV may still be usable when only the reverse/kA phase failed.
                if (fit.rampSamples > 0 && Double.isFinite(fit.kS) && fit.kS != 0) {
                    telemetry.addLine("Ramp-only (kA not trusted):");
                    telemetry.addData("lateralKS", "%.5f", fit.kS);
                    telemetry.addData("lateralKV", "%.6f", fit.kV);
                    telemetry.addData("ramp R^2", "%.3f", fit.rampR2);
                }
            } else {
                telemetry.addLine("=== Paste into MecanumDrive.Params ===");
                telemetry.addData("useAnisotropicFeedforward", true);
                telemetry.addData("lateralKS", "%.5f", fit.kS);
                telemetry.addData("lateralKV", "%.6f", fit.kV);
                telemetry.addData("lateralKA", "%.6f", fit.kA);
                telemetry.addLine();
                telemetry.addData("samples", fit.samples);
                telemetry.addData("ramp samples used", fit.rampSamples);
                telemetry.addData("ramp R^2", "%.3f", fit.rampR2);
                telemetry.addLine("Expect lateralKS ≫ axial kS; kV/kA often only modestly higher.");
            }
            telemetry.update();
        }
    }
}
