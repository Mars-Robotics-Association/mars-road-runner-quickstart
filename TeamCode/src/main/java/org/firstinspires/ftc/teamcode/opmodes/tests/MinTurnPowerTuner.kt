package org.firstinspires.ftc.teamcode.opmodes.tests

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.MecanumDrive
import org.firstinspires.ftc.teamcode.opmodes.base.MarsLinearOpMode
import org.firstinspires.ftc.teamcode.utils.RoadRunnerTeleOpDrive
import kotlin.math.abs
import kotlin.math.min

/**
 * Tuning opmode to find the minimum turn power that overcomes static friction.
 *
 * <p>Hold A to begin the test. The opmode will slowly ramp up turn power until the Pinpoint
 * localizer detects rotation, then report the minimum power needed. Release A to stop and reset.
 */
@Config
@TeleOp(name = "Min Turn Power Tuner", group = "Tuning")
class MinTurnPowerTuner : MarsLinearOpMode() {
    class TunerConfig {
        @JvmField var powerIncrement = 0.005
        @JvmField var movementThreshold = 5.0 // degrees per second
    }

    companion object {
        @JvmField
        var config = TunerConfig()
    }

    override fun runOpMode() {
        initRobot()
        val drive =
            MecanumDrive.forMarsLinear(hardwareMap, Pose2d(0.0, 0.0, 0.0), this::batteryVoltage)

        telemetry.addLine("Min Turn Power Tuner")
        telemetry.addLine("Hold A to ramp up turn power")
        telemetry.addLine("Release to stop and reset")
        telemetry.update()

        waitForStart()

        var currentPower = 0.0
        var detectedMinPower = 0.0
        var movementDetected = false

        while (nextFrame()) {
            val velocity = drive.localizer.update()
            val pose = drive.localizer.getPose()
            val angVelDegrees = Math.toDegrees(abs(velocity.angVel))

            if (gamepad1.a) {
                if (!movementDetected) {
                    currentPower += config.powerIncrement
                    currentPower = min(currentPower, 1.0)

                    if (angVelDegrees > config.movementThreshold) {
                        movementDetected = true
                        detectedMinPower = currentPower
                    }
                }

                drive.setDrivePowers(
                    RoadRunnerTeleOpDrive.toDrivePowers(
                        false, 0.0, 0.0, currentPower, 1.0, pose.heading,
                    ),
                )
            } else {
                currentPower = 0.0
                movementDetected = false
                drive.setDrivePowers(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0))
            }

            telemetry.addData("Current Power", "%.3f", currentPower)
            telemetry.addData("Angular Velocity (deg/s)", "%.1f", angVelDegrees)
            telemetry.addLine()

            if (movementDetected) {
                telemetry.addLine(">>> MOVEMENT DETECTED <<<")
                telemetry.addData(">>> Min Turn Power", "%.3f", detectedMinPower)
            } else if (gamepad1.a) {
                telemetry.addLine("Ramping up...")
            } else {
                telemetry.addLine("Hold A to start test")
            }

            telemetry.addLine()
            telemetry.addData("Threshold (deg/s)", "%.1f", config.movementThreshold)
            telemetry.addData("Power Increment", "%.4f", config.powerIncrement)
        }
    }
}
