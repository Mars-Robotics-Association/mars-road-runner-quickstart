package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

/**
 * Automatic on-robot identification of the axial feedforward — including {@code kA}, which the stock
 * tuner leaves to eyeballing a target-vs-actual velocity graph.
 *
 * <p>It drives an open-loop square wave (forward/back reversals) and fits kS, kV, kA together via the
 * integral method; see {@link ReversalFeedforwardId} for why the reversals are what make kA
 * identifiable and how the fit avoids differentiating velocity. Velocity comes from the localizer
 * (chassis forward velocity), not the drive-motor encoders, which are frequently not wired.
 *
 * <p>Prerequisites: localization tuned; a clear lane of a few feet in <em>both</em> directions (the
 * robot lurches forward and back). Compare the reported kS/kV against your ramp values as a sanity
 * check; use the reported kA in place of the hand-matched one.
 */
@Config
public final class AxialFeedforwardTuner extends LinearOpMode {
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
        MultipleTelemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        DoubleConsumer setPower;
        DoubleSupplier forwardVel; // signed chassis forward velocity, in/s, from the localizer
        VoltageSensor voltageSensor;
        double inPerTick;
        String driveName;

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
            setPower = p -> {
                drive.leftFront.setPower(p);
                drive.leftBack.setPower(p);
                drive.rightBack.setPower(p);
                drive.rightFront.setPower(p);
            };
            forwardVel = () -> drive.localizer.update().linearVel.x;
            voltageSensor = drive.voltageSensor;
            inPerTick = MecanumDrive.PARAMS.inPerTick;
            driveName = "MecanumDrive";
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));
            List<DcMotorEx> allMotors = new ArrayList<>(drive.leftMotors);
            allMotors.addAll(drive.rightMotors);
            setPower = p -> {
                for (DcMotorEx m : allMotors) {
                    m.setPower(p);
                }
            };
            forwardVel = () -> drive.localizer.update().linearVel.x;
            voltageSensor = drive.voltageSensor;
            inPerTick = TankDrive.PARAMS.inPerTick;
            driveName = "TankDrive";
        } else {
            throw new RuntimeException("Unknown DRIVE_CLASS");
        }

        telemetry.addLine("Axial feedforward tuner (identifies kS, kV, kA).");
        telemetry.addLine("Press START; the robot will lurch FORWARD and BACK repeatedly.");
        telemetry.addLine("Keep a few feet clear in both directions.");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        ReversalFeedforwardId.Result fit = ReversalFeedforwardId.identify(
                this, telemetry, setPower, forwardVel, voltageSensor,
                inPerTick, MAX_POWER, HALF_CYCLE, CYCLES, SIGN_DEADBAND);

        while (opModeIsActive()) {
            if (fit.singular) {
                telemetry.addLine("Fit failed (singular): the maneuver didn't excite all terms.");
                telemetry.addLine("Increase MAX_POWER or CYCLES, or shorten HALF_CYCLE, and retry.");
            } else {
                telemetry.addLine("=== Paste into " + driveName + ".Params ===");
                telemetry.addData("kS", "%.5f", fit.kS);
                telemetry.addData("kV", "%.6f", fit.kV);
                telemetry.addData("kA", "%.6f", fit.kA);
                telemetry.addLine();
                telemetry.addData("samples", fit.samples);
                telemetry.addLine("Sanity-check kS/kV against your ramp values; use kA as-is.");
            }
            telemetry.update();
        }
    }
}
