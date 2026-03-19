package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.utils.HubHelper;
import edu.wpi.first.math.MathUtil;
import org.marsroboticsassociation.controllib.util.SetOnChange;
import org.marsroboticsassociation.controllib.util.TelemetryAddData;

import java.util.Arrays;

/**
 * Abstract base class for controlling one or more motors in tandem.
 * <p>
 * This class supports driving multiple motors simultaneously with identical power commands,
 * which is useful for mechanisms that use multiple motors geared or belted together
 * (e.g., dual-motor flywheels, lift systems). When multiple motors are provided:
 * <ul>
 *   <li>The same power is applied to all motors.</li>
 *   <li>Only the first motor's encoder is read for feedback.</li>
 *   <li>All motors must be of the same type (same PPR, gear ratio).</li>
 *   <li>Motors should ideally be mechanically coupled (geared, belted, or on a shared shaft)
 *       to ensure they stay in sync. Without mechanical coupling, motors may drift apart
 *       over time due to minor differences in load or motor characteristics.</li>
 * </ul>
 * <p>
 * The class also provides voltage monitoring with low-pass filtering for feedforward
 * compensation, and quantized power updates to reduce unnecessary motor commands.
 */
public abstract class MotorBase {
    protected final DcMotorEx[] motors;
    protected final TelemetryAddData telemetry;
    protected final String[] deviceNames;
    protected final double gearRatio;
    protected final double motorPPR;

    private double lastPower = Double.NaN;
    private final double quantStep;
    protected final SetOnChange<DcMotor.RunMode> motorMode;
    private final double hubVoltage;

    /**
     * Constructs a MotorBase with one or more motors running in tandem.
     *
     * @param hardwareMap              the hardware map to retrieve motors from
     * @param telemetry                callback for telemetry output
     * @param gearRatio                the external gear ratio (output/input), use 1.0 if direct drive
     * @param motorPPR                 pulses per revolution of the motor encoder
     * @param motorPowerChangeTolerance minimum power change threshold to reduce command noise
     * @param deviceNames              one or more motor device names; if multiple are provided,
     *                                 the first motor's encoder is used for feedback and all motors
     *                                 receive the same power. All motors must be of the same type.
     */
    public MotorBase(HardwareMap hardwareMap, TelemetryAddData telemetry, double gearRatio, double motorPPR, double motorPowerChangeTolerance, String... deviceNames) {
        this.deviceNames = deviceNames;
        this.gearRatio = gearRatio;
        this.motorPPR = motorPPR;
        this.motors = Arrays.stream(deviceNames).map(deviceName -> hardwareMap.get(DcMotorEx.class, deviceName)).toArray(DcMotorEx[]::new);
        this.telemetry = telemetry;

        LynxModule hub = HubHelper.getHubForMotor(motors[0], hardwareMap);
        hubVoltage = hub.getInputVoltage(VoltageUnit.VOLTS);

        quantStep = motorPowerChangeTolerance;
        motorMode = SetOnChange.of(DcMotor.RunMode.STOP_AND_RESET_ENCODER, mode -> Arrays.stream(motors).forEach(motor -> motor.setMode(mode)));
        motorMode.set(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /** For unit tests only — bypasses HardwareMap, LynxModule, and HubHelper. */
    MotorBase(TelemetryAddData telemetry, double gearRatio, double motorPPR,
              double motorPowerChangeTolerance, double hubVoltage,
              String deviceName, DcMotorEx motor) {
        this.deviceNames = new String[]{deviceName};
        this.gearRatio = gearRatio;
        this.motorPPR = motorPPR;
        this.motors = new DcMotorEx[]{motor};
        this.telemetry = telemetry;
        this.hubVoltage = hubVoltage;
        this.quantStep = motorPowerChangeTolerance;
        this.motorMode = SetOnChange.of(DcMotor.RunMode.RUN_WITHOUT_ENCODER, m -> {});
    }

    /**
     * Sets power to all motors, only if the quantized value changes from the last command.
     * <p>
     * Power is clamped to [-1, 1] and quantized to reduce command noise. The same power
     * value is applied to all motors in the group.
     *
     * @param power the desired power level (-1.0 to 1.0)
     */
    public void setPower(double power) {
        if (Double.isNaN(power)) {
            telemetry.addData("power", "%s", "NaN");
            return;
        } else if (Double.isInfinite(power)) {
            telemetry.addData("power", "%s", "Infinite");
            return;
        } else {
            telemetry.addData("power", "%.2f", power);
        }

        // Clamp to [-1, 1]
        power = MathUtil.clamp(power, -1.0, 1.0);
        // Quantize to step
        double quantPower = Math.round(power / quantStep) * quantStep;
        // Update motor only if different
        if (Double.isNaN(lastPower) || quantPower != lastPower) {
            for (DcMotorEx motor : motors) {
                motor.setPower(quantPower);
            }
            lastPower = quantPower;
        }
    }

    public double getVoltage() {
        return hubVoltage;
    }

    public final void update(double dt) {
        if (dt < 1e-6) return; // don't even run the update if we're in the same frame
        updateInternal(dt);
    }

    protected abstract void updateInternal(double dt);

    public abstract void writeTelemetry();

    public abstract void stop();
}
