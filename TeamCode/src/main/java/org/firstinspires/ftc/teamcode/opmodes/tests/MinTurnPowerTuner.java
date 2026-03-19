package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.BulkReads;
import org.firstinspires.ftc.teamcode.utils.RoadRunnerTeleOpDrive;

/**
 * Tuning opmode to find the minimum turn power that overcomes static friction.
 *
 * <p>Hold A to begin the test. The opmode will slowly ramp up turn power until the
 * Pinpoint localizer detects rotation, then report the minimum power needed.
 * Release A to stop and reset.
 */
@Config
@TeleOp(name = "Min Turn Power Tuner", group = "Tuning")
public class MinTurnPowerTuner extends LinearOpMode {

    public static class TunerConfig {
        public double powerIncrement = 0.005;
        public double movementThreshold = 5.0; // degrees per second
    }

    public static TunerConfig config = new TunerConfig();

    @Override
    public void runOpMode() {
        var bulkReads = new BulkReads(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        var drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        telemetry.addLine("Min Turn Power Tuner");
        telemetry.addLine("Hold A to ramp up turn power");
        telemetry.addLine("Release to stop and reset");
        telemetry.update();

        waitForStart();

        double currentPower = 0;
        double detectedMinPower = 0;
        boolean movementDetected = false;

        while (opModeIsActive()) {
            bulkReads.readAll();

            PoseVelocity2d velocity = drive.localizer.update();
            Pose2d pose = drive.localizer.getPose();
            double angVelDegrees = Math.toDegrees(Math.abs(velocity.angVel));

            if (gamepad1.a) {
                if (!movementDetected) {
                    // Ramp up power
                    currentPower += config.powerIncrement;
                    currentPower = Math.min(currentPower, 1.0);

                    // Check if movement detected
                    if (angVelDegrees > config.movementThreshold) {
                        movementDetected = true;
                        detectedMinPower = currentPower;
                    }
                }

                // Apply turn power
                drive.setDrivePowers(
                        RoadRunnerTeleOpDrive.toDrivePowers(
                                false,
                                0,
                                0,
                                currentPower,
                                1.0,
                                pose.heading
                        )
                );
            } else {
                // Reset when A is released
                currentPower = 0;
                movementDetected = false;
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
            }

            telemetry.addData("Current Power", "%.3f", currentPower);
            telemetry.addData("Angular Velocity (deg/s)", "%.1f", angVelDegrees);
            telemetry.addLine();

            if (movementDetected) {
                telemetry.addLine(">>> MOVEMENT DETECTED <<<");
                telemetry.addData(">>> Min Turn Power", "%.3f", detectedMinPower);
            } else if (gamepad1.a) {
                telemetry.addLine("Ramping up...");
            } else {
                telemetry.addLine("Hold A to start test");
            }

            telemetry.addLine();
            telemetry.addData("Threshold (deg/s)", "%.1f", config.movementThreshold);
            telemetry.addData("Power Increment", "%.4f", config.powerIncrement);
            telemetry.update();
        }
    }
}
