package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;

/**
 * Demo TeleOp for the multi-hypothesis bank localizer (the quickstart equivalent of the source
 * project's {@code VisionFusionViz}). Drives a mecanum robot while a {@link
 * HypothesisBankRoadRunnerLocalizer} fuses Pinpoint/odometry with Limelight AprilTag branches, and
 * shows the fused pose + commitment on telemetry and the FTC Dashboard field overlay.
 *
 * <p>Setup: a Limelight configured as {@code "limelight"} with an AprilTag pipeline on index 0 and
 * "output corners" enabled, plus the standard quickstart drive/odometry config. Point the camera at
 * a mapped tag so the bank can commit (watch {@code bank_committed}). The tag field-pose table and
 * camera extrinsic come from {@code VisionPoseSolverConfig} — replace those with your field/robot.
 */
@TeleOp(name = "Hypothesis Bank Localizer Demo", group = "vision")
public class HypothesisBankLocalizerDemo extends LinearOpMode {

    // Starting field pose (inches, degrees). The bank seeds here and refines from vision.
    public static double START_X = 0;
    public static double START_Y = 0;
    public static double START_HEADING_DEG = 0;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Pose2d startPose = new Pose2d(START_X, START_Y, Math.toRadians(START_HEADING_DEG));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        LimelightVisionSource source =
                new LimelightVisionSource(limelight, telemetry, new OpenCvPlanarPnpSolver());
        HypothesisBankRoadRunnerLocalizer localizer =
                new HypothesisBankRoadRunnerLocalizer(
                        drive.localizer, source, startPose, telemetry);

        // Load the ambiguity-gate calibration BEFORE start(): the SDK's calibration GET has a 100ms
        // timeout that reliably loses to the result poller once it is running, so fetch it while
        // the
        // camera's HTTP server is still idle. Intrinsics never change, so this one-time fetch
        // sticks.
        source.prefetchCalibration(3000);
        limelight.start();

        telemetry.addLine("Ready. Field view on FTC Dashboard. Point at a mapped tag to commit.");
        telemetry.addLine("Left stick translate, right stick rotate. Y = reset to start pose.");
        telemetry.update();

        waitForStart();

        boolean prevY = false;
        while (opModeIsActive()) {
            // The vision localizer owns the odometry update this loop (do NOT also call
            // drive.updatePoseEstimate(), which would double-update the same localizer).
            localizer.update();
            Pose2d pose = localizer.getPose();

            if (gamepad1.y && !prevY) {
                localizer.setPose(startPose);
            }
            prevY = gamepad1.y;

            // Robot-centric drive powers straight from the sticks.
            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x),
                            -gamepad1.right_stick_x));

            telemetry.addData("x (in)", "%.1f", pose.position.x);
            telemetry.addData("y (in)", "%.1f", pose.position.y);
            telemetry.addData("heading (deg)", "%.1f", Math.toDegrees(pose.heading.toDouble()));
            telemetry.addData("committed", localizer.isCommitted());
            telemetry.addData("dominant weight", "%.2f", localizer.dominantWeight());

            com.acmerobotics.dashboard.telemetry.TelemetryPacket packet =
                    new com.acmerobotics.dashboard.telemetry.TelemetryPacket();
            packet.fieldOverlay().setStroke("#4CAF50");
            Drawing.drawRobot(packet.fieldOverlay(), pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            telemetry.update();
        }
    }
}
