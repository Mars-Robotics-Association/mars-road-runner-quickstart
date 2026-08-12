package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import edu.wpi.first.math.MathUtil

class QuantizedPowerMotor : EncapsulatedDcMotorEx {
    private val quantStep: Double
    private var lastPower = Double.NaN

    constructor(hardwareMap: HardwareMap, deviceName: String, motorPowerChangeTolerance: Double) :
        super(hardwareMap, deviceName) {
        quantStep = motorPowerChangeTolerance
    }

    constructor(motor: DcMotorEx, motorPowerChangeTolerance: Double) : super(motor) {
        quantStep = motorPowerChangeTolerance
    }

    /**
     * Quantizes the motor power based on motorPowerChangeTolerance set in the constructor
     *
     * @param power the new power level of the motor, a value in the interval [-1.0, 1.0]
     */
    override fun setPower(power: Double) {
        // Clamp to [-1, 1]
        var power = MathUtil.clamp(power, -1.0, 1.0)
        // Quantize to step
        val quantPower = Math.round(power / quantStep) * quantStep
        // Update motor only if different
        if (lastPower.isNaN() || quantPower != lastPower) {
            super.setPower(quantPower)
            lastPower = quantPower
        }
    }
}
