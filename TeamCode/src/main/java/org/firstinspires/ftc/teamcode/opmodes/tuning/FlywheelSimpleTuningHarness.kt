package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.qualcomm.robotcore.hardware.DcMotor

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.opmodes.base.MarsLinearOpMode
import org.firstinspires.ftc.teamcode.robot.LinkedMotorGroup
import org.firstinspires.ftc.teamcode.robot.MotorConfig
import org.firstinspires.ftc.teamcode.robot.QuantizedPowerMotor
import org.marsroboticsassociation.controllib.control.FlywheelSimple

/**
 * Teleop test harness for tuning [FlywheelSimple].
 *
 * <p>All [FlywheelSimple.Params] (kS, kV, kA, kP, velLpfCutoffHz, readyThreshold, etc.) are
 * exposed live in FTC Dashboard under "FlywheelSimple". Change them while the flywheel is spinning
 * and watch the effect on the Dashboard telemetry graphs.
 *
 * <p>Controls:
 *
 * <ul>
 *   <li>Right bumper — spin at [TARGET_TPS]
 *   <li>Left bumper — coast (stop)
 * </ul>
 *
 * <p>Tuning procedure:
 *
 * <ol>
 *   <li>Run `FlywheelsFeedforwardTuning` first to measure kS, kV, and kA, then paste those
 *       values into [FlywheelSimple.PARAMS].
 *   <li>Set [TARGET_TPS] to a target speed in the Dashboard.
 *   <li>Press right bumper. Watch "velocity (smooth)" converge to "setpoint".
 *   <li>If there is steady-state error, increase kP. If the velocity oscillates, decrease kP.
 *   <li>Check `isReady`: it should go true within a second or two of spin-up at a good kP.
 * </ol>
 */
@Config
@TeleOp(name = "FlywheelSimple Tuning Harness", group = "Tuning")
class FlywheelSimpleTuningHarness : MarsLinearOpMode() {
    companion object {
        /** Target flywheel speed in ticks per second. Adjust via FTC Dashboard. */
        @JvmField
        var TARGET_TPS = 2000.0
    }

    override fun runOpMode() {
        initRobot()

        val group =
            LinkedMotorGroup(
                hardwareMap,
                MotorConfig("flywheel", DcMotorSimple.Direction.REVERSE),
                MotorConfig("flywheel2", DcMotorSimple.Direction.REVERSE),
            )
        group.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        group.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        val flywheelMotor = QuantizedPowerMotor(group, 0.01)
        val flywheel = FlywheelSimple(
            { caption, format, value -> telemetry.addData(caption, format, value) },
            flywheelMotor,
        )

        telemetry.addLine("Right bumper → spin at TARGET_TPS")
        telemetry.addLine("Left bumper  → coast")
        telemetry.addLine("Edit TARGET_TPS and FlywheelSimple params in FTC Dashboard.")
        telemetry.update()
        waitForStart()

        var spinning = false

        while (nextFrame()) {
            if (gamepad1.rightBumperWasPressed()) spinning = true
            if (gamepad1.leftBumperWasPressed()) spinning = false

            flywheel.setTps(if (spinning) TARGET_TPS else 0.0)
            flywheel.update()

            flywheel.writeTelemetry()
            telemetry.addData("Spinning", spinning)
            telemetry.addData("isPowerTooLow", flywheel.isPowerTooLowForTargetVelocity)
        }
    }
}
