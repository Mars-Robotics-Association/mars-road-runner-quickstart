package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

/**
 * Automatic on-robot identification of the anisotropic (strafe) feedforward constants
 * {@code lateralKS} / {@code lateralKV} / {@code lateralKA} for a mecanum drive. Mecanum rollers scrub
 * sideways when strafing, which raises the effective feedforward relative to forward motion.
 *
 * <p>This is the strafe counterpart of {@link AxialFeedforwardTuner}: it drives an open-loop square
 * wave sideways (left/right reversals) and fits all three lateral constants at once via the same
 * integral method (see {@link ReversalFeedforwardId}) — including {@code lateralKA}, which the earlier
 * ramp-only approach couldn't get.
 *
 * <p>Wheel velocity is derived from the localizer's lateral chassis velocity times the drive's
 * {@code lateralMultiplier} (a pure strafe drives each wheel at that speed), not the drive-motor
 * encoders, which are frequently not wired.
 *
 * <p>Prerequisites: localization and the axial feedforward already tuned; a clear sideways lane of a
 * few feet in both directions. After pasting the results, set {@code useAnisotropicFeedforward = true}.
 * Expect the lateral constants to exceed their axial counterparts (roller scrub).
 */
@Config
public final class LateralFeedforwardTuner extends LinearOpMode {
    /** Peak power of the square wave. */
    public static double MAX_POWER = 0.6;
    /** Seconds held in each direction before reversing. Shorter = more accel content, less travel. */
    public static double HALF_CYCLE = 0.75;
    /** Number of reversals to run. */
    public static int CYCLES = 8;
    /** Speeds (in/s) within this of zero contribute no kS term (avoids sign chatter at reversals). */
    public static double SIGN_DEADBAND = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {
        if (!TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            throw new RuntimeException(
                    "LateralFeedforwardTuner is mecanum-only; a "
                            + (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class) ? "tank" : "non-mecanum")
                            + " drive has no lateral motion.");
        }

        MultipleTelemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        double inPerTick = MecanumDrive.PARAMS.inPerTick;
        // pure strafe drives each wheel at |chassis lateral velocity| * lateralMultiplier
        double lateralMultiplier = drive.kinematics.lateralMultiplier;

        // +power => strafe pattern (lf, lb, rb, rf) = (-, +, -, +), which is +y (robot-left). The
        // reference wheel (leftBack) then has power = +power and velocity = +linearVel.y*multiplier,
        // so power and wheelVel share a sign convention as ReversalFeedforwardId requires.
        DoubleConsumer setPower = p -> {
            drive.leftFront.setPower(-p);
            drive.leftBack.setPower(p);
            drive.rightBack.setPower(-p);
            drive.rightFront.setPower(p);
        };
        DoubleSupplier wheelVel = () -> drive.localizer.update().linearVel.y * lateralMultiplier;

        telemetry.addLine("Mecanum lateral (strafe) feedforward tuner (kS, kV, kA).");
        telemetry.addLine("Press START; the robot will strafe LEFT and RIGHT repeatedly.");
        telemetry.addLine("Keep a few feet clear on both sides.");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        ReversalFeedforwardId.Result fit = ReversalFeedforwardId.identify(
                this, telemetry, setPower, wheelVel, drive.voltageSensor,
                inPerTick, MAX_POWER, HALF_CYCLE, CYCLES, SIGN_DEADBAND);

        while (opModeIsActive()) {
            if (fit.singular) {
                telemetry.addLine("Fit failed (singular): the maneuver didn't excite all terms.");
                telemetry.addLine("Increase MAX_POWER or CYCLES, or shorten HALF_CYCLE, and retry.");
            } else {
                telemetry.addLine("=== Paste into MecanumDrive.Params ===");
                telemetry.addData("useAnisotropicFeedforward", true);
                telemetry.addData("lateralKS", "%.5f", fit.kS);
                telemetry.addData("lateralKV", "%.6f", fit.kV);
                telemetry.addData("lateralKA", "%.6f", fit.kA);
                telemetry.addLine();
                telemetry.addData("samples", fit.samples);
                telemetry.addLine("Expect lateralKS/KV/KA to exceed the axial kS/kV/kA (roller scrub).");
            }
            telemetry.update();
        }
    }
}
