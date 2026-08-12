package org.firstinspires.ftc.teamcode.opmodes.tests

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.Drawing
import org.firstinspires.ftc.teamcode.MecanumDrive
import org.firstinspires.ftc.teamcode.PinpointLocalizer
import org.firstinspires.ftc.teamcode.opmodes.base.MarsLinearOpMode
import org.firstinspires.ftc.teamcode.utils.RoadRunnerTeleOpDrive
import java.util.ArrayDeque
import kotlin.math.abs
import kotlin.math.floor

/**
 * Tuning opmode to calibrate the parY and perpX encoder offsets in
 * [PinpointLocalizer.Params].
 *
 * <p>When the robot spins in place, incorrect offsets cause the reported position to trace circles
 * on the dashboard field view. This tuner measures the drift and computes corrected offset values.
 *
 * <h3>How to use</h3>
 *
 * <ol>
 *   <li>Run this opmode and spin the robot in place using the right stick (left stick is ignored).
 *   <li>Spin consistently in one direction for several full revolutions.
 *   <li>Read the corrected parY and perpX values from telemetry.
 *   <li>Update [PinpointLocalizer.Params] with those values.
 *   <li>Re-run the tuner — the position should stay nearly fixed.
 * </ol>
 *
 * <h3>Math</h3>
 *
 * After a 180° heading change starting at heading 0:
 *
 * <ul>
 *   <li>Field x displacement = −2 × perpX_error (inches)
 *   <li>Field y displacement = −2 × parY_error (inches)
 * </ul>
 *
 * So: corrected offset = current offset − measured displacement / 2
 */
@Config
@TeleOp(name = "Pinpoint Offset Tuner", group = "Tuning")
class PinpointOffsetTuner : MarsLinearOpMode() {
    companion object {
        @JvmField
        var poseHistorySize = 200
    }

    override fun runOpMode() {
        initRobot()
        val drive =
            MecanumDrive.forMarsLinear(hardwareMap, Pose2d(0.0, 0.0, 0.0), this::batteryVoltage)

        val poseHistory = ArrayDeque<Pose2d>(poseHistorySize + 1)

        val startX = 0.0
        val startY = 0.0

        var totalHeadingChange = 0.0
        var lastHeading = 0.0

        var lastOddHalfRev = -1
        var sampleCount = 0
        var sumDx = 0.0
        var sumDy = 0.0

        val currentParYInches =
            PinpointLocalizer.PARAMS.parYTicks * MecanumDrive.PARAMS.inPerTick
        val currentPerpXInches =
            PinpointLocalizer.PARAMS.perpXTicks * MecanumDrive.PARAMS.inPerTick

        telemetry.addLine("Pinpoint Offset Tuner")
        telemetry.addLine("Spin the robot in place using the right stick.")
        telemetry.addLine("Keep spinning in one direction for several revolutions.")
        telemetry.update()

        waitForStart()

        while (nextFrame()) {
            drive.localizer.update()
            val pose = drive.localizer.getPose()

            poseHistory.add(pose)
            while (poseHistory.size > poseHistorySize) {
                poseHistory.removeFirst()
            }

            val currentHeading = pose.heading.toDouble()
            val delta = AngleUnit.normalizeRadians(currentHeading - lastHeading)
            totalHeadingChange += delta
            lastHeading = currentHeading

            val halfRevs = floor(abs(totalHeadingChange) / Math.PI).toInt()
            if (halfRevs > 0 && halfRevs % 2 == 1 && halfRevs != lastOddHalfRev) {
                val dx = pose.position.x - startX
                val dy = pose.position.y - startY
                sumDx += dx
                sumDy += dy
                sampleCount++
                lastOddHalfRev = halfRevs
            }

            drive.setDrivePowers(
                RoadRunnerTeleOpDrive.toDrivePowers(
                    false, 0.0, 0.0, gamepad1.right_stick_x.toDouble(), 1.0, pose.heading,
                ),
            )

            telemetry.addData("Position", "(%.3f, %.3f)", pose.position.x, pose.position.y)
            telemetry.addData("Total rotation", "%.1f°", Math.toDegrees(totalHeadingChange))
            telemetry.addData("Half-revs completed", halfRevs)
            telemetry.addLine()

            if (sampleCount > 0) {
                val avgDx = sumDx / sampleCount
                val avgDy = sumDy / sampleCount

                val perpXError = avgDx / 2.0
                val parYError = avgDy / 2.0

                val correctedPerpXInches = currentPerpXInches - perpXError
                val correctedParYInches = currentParYInches - parYError

                telemetry.addData("Samples", sampleCount)
                telemetry.addData("Avg displacement", "(%.4f, %.4f) in", avgDx, avgDy)
                telemetry.addLine()
                telemetry.addData("perpX error", "%.4f in", perpXError)
                telemetry.addData("parY error", "%.4f in", parYError)
                telemetry.addLine()
                telemetry.addData("Current perpX", "%.4f in", currentPerpXInches)
                telemetry.addData("Current parY", "%.4f in", currentParYInches)
                telemetry.addLine()
                telemetry.addLine(">>> CORRECTED VALUES <<<")
                telemetry.addData(">>> perpX", "%.4f in", correctedPerpXInches)
                telemetry.addData(">>> parY", "%.4f in", correctedParYInches)
            } else {
                telemetry.addLine("Spin the robot to collect samples...")
                telemetry.addLine("(Samples taken at each 180° of rotation)")
            }

            telemetry.addLine()
            telemetry.addData("Current configured perpX", "%.4f in", currentPerpXInches)
            telemetry.addData("Current configured parY", "%.4f in", currentParYInches)

            val packet: TelemetryPacket? = dashboardPacket()
            if (packet != null) {
                val c = packet.fieldOverlay()
                Drawing.drawRobot(c, pose)
                Drawing.drawPoseHistory(c, "#FF0000", poseHistory)
                c.setStroke("#00FF00")
                c.setStrokeWidth(2)
                c.strokeCircle(startX, startY, 1.0)
            }
        }
    }
}
