package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import edu.wpi.first.math.MathUtil;

public class QuantizedPowerMotor extends EncapsulatedDcMotorEx {
    private final double quantStep;
    private double lastPower = Double.NaN;

    public QuantizedPowerMotor(HardwareMap hardwareMap, String deviceName, double motorPowerChangeTolerance) {
        super(hardwareMap, deviceName);
        quantStep = motorPowerChangeTolerance;
    }

    public QuantizedPowerMotor(DcMotorEx motor, double motorPowerChangeTolerance) {
        super(motor);
        quantStep = motorPowerChangeTolerance;
    }

    /**
     * Quantizes the motor power based on motorPowerChangeTolerance set in the constructor
     *
     * @param power the new power level of the motor, a value in the interval [-1.0, 1.0]
     */
    @Override
    public void setPower(double power) {
        // Clamp to [-1, 1]
        power = MathUtil.clamp(power, -1.0, 1.0);
        // Quantize to step
        double quantPower = Math.round(power / quantStep) * quantStep;
        // Update motor only if different
        if (Double.isNaN(lastPower) || quantPower != lastPower) {
            super.setPower(quantPower);
            lastPower = quantPower;
        }
    }
}
