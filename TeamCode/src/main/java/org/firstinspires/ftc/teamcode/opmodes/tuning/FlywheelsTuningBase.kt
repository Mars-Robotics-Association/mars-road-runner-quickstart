package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.qualcomm.robotcore.hardware.DcMotor

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.opmodes.base.MarsLinearOpMode

@Config
abstract class FlywheelsTuningBase : MarsLinearOpMode() {
    class HardwareConfig {
        @JvmField var MOTORS_COUPLED = true
        @JvmField var MOTOR_NAMES = arrayOf("flywheel", "flywheel2")
        @JvmField
        var MOTOR_DIRECTIONS = arrayOf(
            DcMotorSimple.Direction.REVERSE,
            DcMotorSimple.Direction.REVERSE,
        )
    }

    companion object {
        @JvmField
        var HARDWARE = HardwareConfig()
    }

    protected lateinit var motors: Array<DcMotorEx>

    protected fun initHardware() {
        initRobot()

        motors = Array(HARDWARE.MOTOR_NAMES.size) { i ->
            hardwareMap.get(DcMotorEx::class.java, HARDWARE.MOTOR_NAMES[i]).also { motor ->
                motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                motor.direction = HARDWARE.MOTOR_DIRECTIONS[i]
                motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
            }
        }
    }

    fun stopMotors() {
        for (m in motors) m.power = 0.0
    }

    final override fun runOpMode() {
        initHardware()
        runOpModeInternal()
    }

    protected abstract fun runOpModeInternal()
}
