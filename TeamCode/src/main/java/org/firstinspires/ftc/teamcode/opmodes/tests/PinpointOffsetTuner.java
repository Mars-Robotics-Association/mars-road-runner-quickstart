package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.utils.DashboardTelemetryPacketAccess;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.robot.BulkReads;
import org.firstinspires.ftc.teamcode.utils.RoadRunnerTeleOpDrive;

import java.util.ArrayDeque;

/**
 * Tuning opmode to calibrate the parY and perpX encoder offsets in {@link PinpointLocalizer.Params}.
 *
 * <p>When the robot spins in place, incorrect offsets cause the reported position to trace circles
 * on the dashboard field view. This tuner measures the drift and computes corrected offset values.
 *
 * <h3>How to use</h3>
 * <ol>
 *   <li>Run this opmode and spin the robot in place using the right stick (left stick is ignored).</li>
 *   <li>Spin consistently in one direction for several full revolutions.</li>
 *   <li>Read the corrected parY and perpX values from telemetry.</li>
 *   <li>Update {@link PinpointLocalizer.Params} with those values.</li>
 *   <li>Re-run the tuner — the position should stay nearly fixed.</li>
 * </ol>
 *
 * <h3>Math</h3>
 * After a 180° heading change starting at heading 0:
 * <ul>
 *   <li>Field x displacement = −2 × perpX_error (inches)</li>
 *   <li>Field y displacement = −2 × parY_error (inches)</li>
 * </ul>
 * So: corrected offset = current offset − measured displacement / 2
 */
@Config
@TeleOp(name = "Pinpoint Offset Tuner", group = "Tuning")
public class PinpointOffsetTuner extends LinearOpMode {

    public static int poseHistorySize = 200;

    @Override
    public void runOpMode() {
        var bulkReads = new BulkReads(hardwareMap);
        var packetAccess = new DashboardTelemetryPacketAccess();
        telemetry = new MultipleTelemetry(telemetry, packetAccess.dashboardTelemetry);

        var drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        var poseHistory = new ArrayDeque<Pose2d>(poseHistorySize + 1);

        // Starting position
        final double startX = 0;
        final double startY = 0;

        // Heading tracking (unwrapped)
        double totalHeadingChange = 0;
        double lastHeading = 0;

        // Half-revolution sample tracking
        int lastOddHalfRev = -1; // last odd half-rev count at which we took a sample
        int sampleCount = 0;
        double sumDx = 0;
        double sumDy = 0;

        // Current configured offsets in inches
        double currentParYInches = PinpointLocalizer.PARAMS.parYTicks * MecanumDrive.PARAMS.inPerTick;
        double currentPerpXInches = PinpointLocalizer.PARAMS.perpXTicks * MecanumDrive.PARAMS.inPerTick;

        telemetry.addLine("Pinpoint Offset Tuner");
        telemetry.addLine("Spin the robot in place using the right stick.");
        telemetry.addLine("Keep spinning in one direction for several revolutions.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            bulkReads.readAll();

            PoseVelocity2d velocity = drive.localizer.update();
            Pose2d pose = drive.localizer.getPose();

            // Update pose history
            poseHistory.add(pose);
            while (poseHistory.size() > poseHistorySize) {
                poseHistory.removeFirst();
            }

            // Accumulate heading change (unwrap across ±π boundary)
            double currentHeading = pose.heading.toDouble();
            double delta = AngleUnit.normalizeRadians(currentHeading - lastHeading);
            totalHeadingChange += delta;
            lastHeading = currentHeading;

            // Check for new odd half-revolution crossing
            int halfRevs = (int) Math.floor(Math.abs(totalHeadingChange) / Math.PI);
            if (halfRevs > 0 && halfRevs % 2 == 1 && halfRevs != lastOddHalfRev) {
                // At an odd multiple of π, position should be at (2*perpX_err, 2*parY_err)
                double dx = pose.position.x - startX;
                double dy = pose.position.y - startY;
                sumDx += dx;
                sumDy += dy;
                sampleCount++;
                lastOddHalfRev = halfRevs;
            }

            // Drive: turn only (ignore left stick)
            drive.setDrivePowers(
                    RoadRunnerTeleOpDrive.toDrivePowers(
                            false,
                            0,
                            0,
                            gamepad1.right_stick_x,
                            1.0,
                            pose.heading
                    )
            );

            // Telemetry
            telemetry.addData("Position", "(%.3f, %.3f)", pose.position.x, pose.position.y);
            telemetry.addData("Total rotation", "%.1f°", Math.toDegrees(totalHeadingChange));
            telemetry.addData("Half-revs completed", halfRevs);
            telemetry.addLine();

            if (sampleCount > 0) {
                double avgDx = sumDx / sampleCount;
                double avgDy = sumDy / sampleCount;

                double perpXError = avgDx / 2.0;
                double parYError = avgDy / 2.0;

                double correctedPerpXInches = currentPerpXInches - perpXError;
                double correctedParYInches = currentParYInches - parYError;

                telemetry.addData("Samples", sampleCount);
                telemetry.addData("Avg displacement", "(%.4f, %.4f) in", avgDx, avgDy);
                telemetry.addLine();
                telemetry.addData("perpX error", "%.4f in", perpXError);
                telemetry.addData("parY error", "%.4f in", parYError);
                telemetry.addLine();
                telemetry.addData("Current perpX", "%.4f in", currentPerpXInches);
                telemetry.addData("Current parY", "%.4f in", currentParYInches);
                telemetry.addLine();
                telemetry.addLine(">>> CORRECTED VALUES <<<");
                telemetry.addData(">>> perpX", "%.4f in", correctedPerpXInches);
                telemetry.addData(">>> parY", "%.4f in", correctedParYInches);
            } else {
                telemetry.addLine("Spin the robot to collect samples...");
                telemetry.addLine("(Samples taken at each 180° of rotation)");
            }

            telemetry.addLine();
            telemetry.addData("Current configured perpX", "%.4f in", currentPerpXInches);
            telemetry.addData("Current configured parY", "%.4f in", currentParYInches);

            // Dashboard field overlay
            TelemetryPacket packet = packetAccess.getTelemetryPacket();
            Canvas c = packet.fieldOverlay();
            Drawing.drawRobot(c, pose);
            Drawing.drawPoseHistory(c, "#FF0000", poseHistory);
            // Draw a marker at the origin to show where position should stay
            c.setStroke("#00FF00");
            c.setStrokeWidth(2);
            c.strokeCircle(startX, startY, 1);

            telemetry.update();
        }
    }
}
