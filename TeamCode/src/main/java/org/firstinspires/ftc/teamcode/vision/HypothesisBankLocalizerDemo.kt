package org.firstinspires.ftc.teamcode.vision

import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Drawing
import org.firstinspires.ftc.teamcode.MecanumDrive
import org.firstinspires.ftc.teamcode.opmodes.base.MarsLinearOpMode

/**
 * Demo TeleOp for the multi-hypothesis bank localizer (the quickstart equivalent of the source
 * project's `VisionFusionViz`). Drives a mecanum robot while a [HypothesisBankRoadRunnerLocalizer]
 * fuses Pinpoint/odometry with Limelight AprilTag branches, shows the fused pose + commitment on
 * telemetry and the FTC Dashboard field overlay, and records a per-run CSV under `FIRST/` for
 * offline analysis.
 *
 * Setup: a Limelight configured as `"limelight"` with an AprilTag pipeline on index 0 and "output
 * corners" enabled, plus the standard quickstart drive/odometry config. Point the camera at a
 * mapped tag so the bank can commit (watch `bank_committed`). Geometry is quickstart-owned: camera
 * extrinsics in [HypothesisBankRoadRunnerLocalizer.Params] `camera*` fields (or FTC Dashboard),
 * field tag poses in [FieldTagMap]. ControlLib only supplies frame conventions and the pure solver.
 */
@TeleOp(name = "Hypothesis Bank Localizer Demo", group = "vision")
class HypothesisBankLocalizerDemo : MarsLinearOpMode() {

    override fun runOpMode() {
        val startPose = Pose2d(START_X, START_Y, Math.toRadians(START_HEADING_DEG))
        initRobot()
        val drive = MecanumDrive.forMarsLinear(hardwareMap, startPose) { batteryVoltage() }

        val limelight = hardwareMap.get(Limelight3A::class.java, "limelight")
        limelight.pipelineSwitch(0)

        val source = LimelightVisionSource(limelight, telemetry, OpenCvPlanarPnpSolver())
        val localizer =
            HypothesisBankRoadRunnerLocalizer(drive.localizer, source, startPose, telemetry)

        // Load the ambiguity-gate calibration BEFORE start(): the SDK's calibration GET has a 100ms
        // timeout that reliably loses to the result poller once it is running, so fetch it while
        // the camera's HTTP server is still idle. Intrinsics never change, so this one-time fetch
        // sticks.
        source.prefetchCalibration(3000)
        limelight.start()

        // Per-run offline-analysis log under FIRST/. The shared logger pulls the full debug row
        // (poses, frame quality, branch poses, bank state) from the localizer each loop; rows are
        // buffered and flushed in bursts so the control loop isn't stalled on disk I/O.
        val csv = BankLocalizerCsvLogger("bank_localizer")

        telemetry.addLine("Ready. Field view on FTC Dashboard. Point at a mapped tag to commit.")
        telemetry.addLine("Left stick translate, right stick rotate. Y = reset to start pose.")
        telemetry.addData("csv", csv.fileName())
        telemetry.update()

        waitForStart()

        var prevY = false
        while (nextFrame()) {
            // The vision localizer owns the odometry update this loop (do NOT also call
            // drive.updatePoseEstimate(), which would double-update the same localizer).
            localizer.update()
            val pose = localizer.getPose()

            if (gamepad1.y && !prevY) {
                localizer.setPose(startPose)
            }
            prevY = gamepad1.y

            drive.setDrivePowers(
                PoseVelocity2d(
                    Vector2d((-gamepad1.left_stick_y).toDouble(), (-gamepad1.left_stick_x).toDouble()),
                    (-gamepad1.right_stick_x).toDouble(),
                )
            )

            telemetry.addData("x (in)", "%.1f", pose.position.x)
            telemetry.addData("y (in)", "%.1f", pose.position.y)
            telemetry.addData("heading (deg)", "%.1f", Math.toDegrees(pose.heading.toDouble()))
            telemetry.addData("committed", localizer.isCommitted())
            telemetry.addData("dominant weight", "%.2f", localizer.dominantWeight())

            val packet: TelemetryPacket? = dashboardPacket()
            if (packet != null) {
                val c: Canvas = packet.fieldOverlay()
                c.setStroke("#4CAF50")
                Drawing.drawRobot(c, pose)
            }

            csv.log(localizer)
        }

        csv.close()
    }

    companion object {
        // Starting field pose (inches, degrees). The bank seeds here and refines from vision.
        @JvmField
        var START_X = 0.0
        @JvmField
        var START_Y = 0.0
        @JvmField
        var START_HEADING_DEG = 0.0
    }
}
