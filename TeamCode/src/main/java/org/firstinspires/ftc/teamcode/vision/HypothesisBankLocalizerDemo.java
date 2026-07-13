package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.DashboardTelemetryPacketAccess;

/**
 * Demo TeleOp for the multi-hypothesis bank localizer (the quickstart equivalent of the source
 * project's {@code VisionFusionViz}). Drives a mecanum robot while a {@link
 * HypothesisBankRoadRunnerLocalizer} fuses Pinpoint/odometry with Limelight AprilTag branches,
 * shows the fused pose + commitment on telemetry and the FTC Dashboard field overlay, and records a
 * per-run CSV under {@code FIRST/} for offline analysis.
 *
 * <p>Setup: a Limelight configured as {@code "limelight"} with an AprilTag pipeline on index 0 and
 * "output corners" enabled, plus the standard quickstart drive/odometry config. Point the camera at
 * a mapped tag so the bank can commit (watch {@code bank_committed}). Geometry is quickstart-owned:
 * camera extrinsics in {@link HypothesisBankRoadRunnerLocalizer.Params} {@code camera*} fields (or
 * FTC Dashboard), field tag poses in {@link FieldTagMap}. ControlLib only supplies frame
 * conventions and the pure solver.
 */
@TeleOp(name = "Hypothesis Bank Localizer Demo", group = "vision")
public class HypothesisBankLocalizerDemo extends LinearOpMode {

    // Starting field pose (inches, degrees). The bank seeds here and refines from vision.
    public static double START_X = 0;
    public static double START_Y = 0;
    public static double START_HEADING_DEG = 0;

    @Override
    public void runOpMode() {
        // Route telemetry to both the Driver Station and the dashboard, and keep a handle on the
        // dashboard's own TelemetryPacket so the field overlay is drawn onto the same packet that
        // telemetry.update() flushes (one update per loop, no competing sendTelemetryPacket call).
        DashboardTelemetryPacketAccess packetAccess = new DashboardTelemetryPacketAccess();
        telemetry = new MultipleTelemetry(telemetry, packetAccess.dashboardTelemetry);

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

        // Per-run offline-analysis log under FIRST/. The shared logger pulls the full debug row
        // (poses, frame quality, branch poses, bank state) from the localizer each loop; rows are
        // buffered and flushed in bursts so the control loop isn't stalled on disk I/O.
        BankLocalizerCsvLogger csv = new BankLocalizerCsvLogger("bank_localizer");

        telemetry.addLine("Ready. Field view on FTC Dashboard. Point at a mapped tag to commit.");
        telemetry.addLine("Left stick translate, right stick rotate. Y = reset to start pose.");
        telemetry.addData("csv", csv.fileName());
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

            // Draw the fused robot onto the dashboard's own packet (guarded — reflection can fail
            // on
            // an SDK/dashboard version change), then let telemetry.update() flush it.
            TelemetryPacket packet = packetAccess.getTelemetryPacket();
            if (packet != null) {
                Canvas c = packet.fieldOverlay();
                c.setStroke("#4CAF50");
                Drawing.drawRobot(c, pose);
            }

            // Record the full debug row (poses, frame quality, branch poses, bank state).
            csv.log(localizer);

            telemetry.update();
        }

        csv.close(); // final flush of any buffered rows
    }
}
