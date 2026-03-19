package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.LongSupplier;

import org.marsroboticsassociation.controllib.util.TelemetryAddData;
import org.marsroboticsassociation.controllib.filter.BiquadLowPassVarDt;
import org.marsroboticsassociation.controllib.filter.LowPassFilter;
import org.marsroboticsassociation.controllib.motion.VelocityTrajectoryManager;

/**
 * Abstract base class for velocity-controlled motors with encoder feedback.
 * <p>
 * Extends {@link MotorBase} to add velocity measurement and filtering capabilities.
 * Subclasses implement specific control strategies (e.g., PID, feedforward) to achieve
 * target velocities.
 * <p>
 * Like the parent class, this supports multiple motors running in tandem. When multiple
 * motors are used:
 * <ul>
 *   <li>Velocity is measured from the first motor's encoder only.</li>
 *   <li>The same power command is sent to all motors.</li>
 *   <li>Motors should be mechanically coupled to maintain synchronization.</li>
 * </ul>
 *
 * @see MotorBase
 */
public abstract class VelocityMotorBase extends MotorBase {
    /**
     * Constructs a VelocityMotorBase with one or more motors.
     *
     * @param hardwareMap              the hardware map to retrieve motors from
     * @param telemetry                callback for telemetry output
     * @param gearRatio                the external gear ratio (output/input)
     * @param motorPPR                 pulses per revolution of the motor encoder
     * @param motorPowerChangeTolerance minimum power change threshold
     * @param deviceNames              one or more motor device names; velocity feedback
     *                                 is read from the first motor's encoder
     */
    public VelocityMotorBase(HardwareMap hardwareMap, TelemetryAddData telemetry, double gearRatio, double motorPPR, double motorPowerChangeTolerance, double aMax, double jInc, double jDec, double vChangeTolerance, String... deviceNames) {
        super(hardwareMap, telemetry, gearRatio, motorPPR, motorPowerChangeTolerance, deviceNames);
        tpsLpf = new BiquadLowPassVarDt(12, 1.0 / Math.sqrt(2.0));
        trajectory = new VelocityTrajectoryManager(aMax, jInc, vChangeTolerance, telemetry);
        trajectory.updateConfig(aMax, jInc, jDec);
    }

    /** For unit tests only — bypasses HardwareMap, LynxModule, and HubHelper. */
    VelocityMotorBase(TelemetryAddData telemetry, double gearRatio, double motorPPR,
                      double motorPowerChangeTolerance, double hubVoltage,
                      String deviceName, DcMotorEx motor,
                      double aMax, double jInc, double jDec, double vChangeTolerance,
                      LongSupplier clock) {
        super(telemetry, gearRatio, motorPPR, motorPowerChangeTolerance, hubVoltage, deviceName, motor);
        tpsLpf = new BiquadLowPassVarDt(12, 1.0 / Math.sqrt(2.0));
        trajectory = new VelocityTrajectoryManager(aMax, jInc, vChangeTolerance, telemetry, clock);
        trajectory.updateConfig(aMax, jInc, jDec);
    }

    protected final VelocityTrajectoryManager trajectory;

    private double tpsActual;
    private final LowPassFilter tpsLpf;

    public double tpsToRpm(double tps) {
        return tps * 60 / motorPPR / gearRatio;
    }

    public double rpmToTps(double rpm) {
        return rpm * gearRatio * motorPPR / 60.0;
    }

    protected abstract void setTPS(double tps);

    public void setRPM(double rpm) {
        setTPS(rpmToTps(rpm));
    }

    public double getTpsSetpoint() { return trajectory.getTarget(); }

    public double getRpmSetpoint() {
        return tpsToRpm(getTpsSetpoint());
    }

    protected void setFilterCutoff(double cutoffHz) {
        tpsLpf.setCutoffHz(cutoffHz);
    }

    public abstract boolean isAtTargetSpeed();

    @Override
    protected void updateInternal(double dt) {
        tpsActual = motors[0].getVelocity();
        tpsLpf.update(tpsActual, dt);
    }

    @Override
    public void writeTelemetry() {
        telemetry.addData(deviceNames[0] + " TPS setpoint", "%.0f", getTpsSetpoint());
        telemetry.addData(deviceNames[0] + " RPM setpoint", "%.0f", getRpmSetpoint());
        telemetry.addData(deviceNames[0] + " TPS measured", "%.0f", tpsActual);
        telemetry.addData(deviceNames[0] + " RPM measured", "%.0f", tpsToRpm(tpsActual));
        //telemetry.addData(deviceName + " power", motor.getPower());
    }

    public double getTpsMeasurement() {
        return tpsActual;
    }

    public double getTpsFiltered() {
        return tpsLpf.getValue();
    }
}
