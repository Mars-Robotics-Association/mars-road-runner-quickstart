package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.LongSupplier;

import edu.wpi.first.math.MathUtil;
import org.marsroboticsassociation.controllib.util.TelemetryAddData;
import org.marsroboticsassociation.controllib.filter.BiquadLowPassVarDt;
import org.marsroboticsassociation.controllib.filter.LowPassFilter;

/**
 * A velocity-controlled motor implementation using custom proportional + feedforward control
 * rather than the PIDF built into the FTC SDK.
 * <p>
 * This controller features:
 * <ul>
 *   <li>Jerk-limited motion profiling for smooth acceleration/deceleration</li>
 *   <li>Feedforward compensation (kS, kV, kA) with battery voltage scaling</li>
 *   <li>Proportional feedback for disturbance rejection</li>
 *   <li>Low-pass filtered velocity and acceleration measurements</li>
 * </ul>
 * <p>
 * <b>Multi-Motor Support:</b> This class supports driving multiple motors in tandem.
 * When multiple device names are provided:
 * <ul>
 *   <li>All motors receive the same power command.</li>
 *   <li>Only the first motor's encoder is used for velocity feedback.</li>
 *   <li>Each motor can have its own direction specified in {@link VelocityMotorPFConfig#motorDirection}.</li>
 *   <li>All motors must be of the same type (same PPR and gear ratio).</li>
 *   <li>For best results, motors should be mechanically coupled (geared, belted, or on a shared
 *       shaft) to ensure they remain synchronized. Without mechanical coupling, motors may
 *       drift apart due to differences in load or motor characteristics.</li>
 * </ul>
 *
 * @see VelocityMotorBase
 * @see MotorBase
 */
public class VelocityMotorPF extends VelocityMotorBase {
    /**
     * Configuration parameters for the Proportional-Feedforward (PF) velocity controller.
     * <p>
     * This class encapsulates all tuning constants and physical properties required to
     * operate the {@link VelocityMotorPF} controller. It includes coefficients for
     * feedback (kP) and feedforward (kS, kV, kA) control, as well as constraints for
     * motion profiling (acceleration, jerk) and hardware specifications.
     */
    public static class VelocityMotorPFConfig {
        /**
         * Proportional Gain.
         * <p>
         * Multiplies the velocity error (target velocity - measured velocity).
         * Used to correct small errors and disturbances during steady-state motion.
         * Unit: Motor Power per (Ticks/Second error).
         */
        public double kP = 0.0045;

        /**
         * Static Friction Feedforward.
         * <p>
         * The y-intercept or offset for the velocity feedforward.
         * Unit: Volts.
         */
        public double kS = 0.8931;

        /**
         * Velocity Feedforward.
         * <p>
         * Models the voltage required to maintain a specific velocity, ignoring acceleration.
         * Ideally calculated as {@code Battery_Voltage / Max_Ticks_Per_Second}.
         * Unit: Volts per (Ticks/Second).
         */
        public double kV = 12.5 / 2632.1;

        /**
         * Acceleration Feedforward.
         * <p>
         * Models the voltage required to produce acceleration (F=ma).
         * Helps the controller anticipate power needs during the ramp-up and ramp-down phases
         * of the motion profile.
         * Unit: Volts per (Ticks/Second^2).
         */
        public double kA = 12.5 / 2087.9;

        /**
         * Fraction of the theoretical max velocity to reserve as headroom.
         * {@link #getMaxVelocity()} returns {@code (1 - headroomAllowance)} times
         * the physical max derived from kS, kV, and battery voltage.
         */
        public double headroomAllowance = 0.2;

        /**
         * Maximum Acceleration Constraint.
         * <p>
         * The maximum rate at which velocity can change. No sense having it
         * exceed what the motor is physically capable of
         * Unit: Ticks per Second^2.
         */
        public double accelMax = 2500; //994.7; //1196.7;

        /**
         * Jerk Constraint (Increasing Acceleration).
         * <p>
         * The derivative of acceleration. Controls the smoothness of the motion when
         * beginning a move (ramping up acceleration). Lower values reduce mechanical stress.
         * Unit: Ticks per Second^3.
         */
        public double jerkIncreasing = 2000;

        /**
         * Jerk Constraint (Decreasing Acceleration).
         * <p>
         * Controls the smoothness of the motion when approaching constant speed or
         * when beginning to decelerate.
         * Unit: Ticks per Second^3.
         */
        public double jerkDecreasing = 1000;

        /**
         * Measurement Low-Pass Filter Cutoff.
         * <p>
         * The frequency cutoff for filtering the raw encoder velocity. Lower values smooth out noise
         * but introduce phase lag (latency).
         * Unit: Hertz (Hz).
         */
        public double measurementLpfCutoffHz = 6.5;

        /**
         * Acceleration Low-Pass Filter Cutoff.
         * <p>
         * The frequency cutoff for filtering the calculated acceleration. This is used primarily
         * for the {@link #isAtTargetSpeed()} stability check.
         * Unit: Hertz (Hz).
         */
        public double accelLpfCutoffHz = 2;

        /**
         * Target Speed Tolerance.
         * <p>
         * The acceptable error range between target velocity and measured velocity
         * to consider the motor "at speed".
         * Unit: Ticks per Second.
         */
        public double targetSpeedTolerance = 15;

        /**
         * Acceleration Tolerance.
         * <p>
         * The acceptable range of acceleration fluctuation to consider the motor "stable".
         * Ensures the motor isn't oscillating even if the average speed is correct.
         * Unit: Ticks per Second^2.
         */
        public double accelerationTolerance = 75;

        /**
         * Motor Directions.
         * <p>
         * Defines the logical direction for each motor (FORWARD or REVERSE).
         * When using multiple motors, this array must have one entry per motor,
         * in the same order as the device names passed to the constructor.
         * <p>
         * Example for two motors spinning in opposite physical directions:
         * <pre>
         * config.motorDirection = new Direction[]{Direction.FORWARD, Direction.REVERSE};
         * </pre>
         */
        public Direction[] motorDirection = new Direction[]{
                Direction.REVERSE,
                Direction.REVERSE
        };

        /**
         * Feedforward Source Logic.
         * <p>
         * If {@code true}, feedforward is calculated based on the instantaneous velocity/acceleration
         * of the generated motion profile (smoother).
         * <p>
         * If {@code false}, feedforward is calculated based on the final target setpoint immediately.
         */
        public boolean ffFromTrajectory = true;
    }


    private final VelocityMotorPFConfig _config;
    private final LowPassFilter accelLpf;
    private double lastTpsSet = 0;
    private double voltage = 12.0;
    private double lastFilteredTps = 0;

    /**
     * Constructs a VelocityMotorPF controller with one or more motors.
     *
     * @param hardwareMap               the hardware map to retrieve motors from
     * @param telemetry                 callback for telemetry output
     * @param gearRatio                 the external gear ratio (output/input), use 1.0 if direct drive
     * @param motorPPR                  pulses per revolution of the motor encoder
     * @param motorPowerChangeTolerance minimum power change to send a new command (reduces noise)
     * @param config                    configuration parameters for the controller; when using multiple
     *                                  motors, ensure {@link VelocityMotorPFConfig#motorDirection} has
     *                                  one entry per motor
     * @param deviceNames               one or more motor device names; the first motor's encoder is used
     *                                  for feedback, and all motors receive the same power command
     */
    public VelocityMotorPF(HardwareMap hardwareMap, TelemetryAddData telemetry, double gearRatio, double motorPPR, double motorPowerChangeTolerance, VelocityMotorPFConfig config, String... deviceNames) {
        super(hardwareMap, telemetry, gearRatio, motorPPR, motorPowerChangeTolerance, config.accelMax, config.jerkIncreasing, config.jerkDecreasing, 10.0, deviceNames);
        _config = config;
        for (int i = 0; i < motors.length; i++) {
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motors[i].setDirection(config.motorDirection[i]);
        }

        accelLpf = new BiquadLowPassVarDt(_config.accelLpfCutoffHz, 0.5);
    }

    /**
     * Constructs a VelocityMotorPF controller using a predefined motor type.
     *
     * @param hardwareMap               the hardware map to retrieve motors from
     * @param telemetry                 callback for telemetry output
     * @param motorType                 predefined motor type containing PPR and gear ratio
     * @param motorPowerChangeTolerance minimum power change to send a new command
     * @param config                    configuration parameters for the controller
     * @param deviceNames               one or more motor device names
     * @see #VelocityMotorPF(HardwareMap, TelemetryAddData, double, double, double, VelocityMotorPFConfig, String...)
     */
    public VelocityMotorPF(HardwareMap hardwareMap, TelemetryAddData telemetry, MotorType motorType, double motorPowerChangeTolerance, VelocityMotorPFConfig config, String... deviceNames) {
        this(hardwareMap, telemetry, motorType.getGearRatio(), motorType.getPulsesPerRevolution(), motorPowerChangeTolerance, config, deviceNames);
    }

    /** For unit tests only — bypasses HardwareMap, LynxModule, HubHelper, and motor hardware init. */
    VelocityMotorPF(TelemetryAddData telemetry, double gearRatio, double motorPPR,
                    double motorPowerChangeTolerance, double hubVoltage,
                    String deviceName, DcMotorEx motor, VelocityMotorPFConfig config,
                    LongSupplier clock) {
        super(telemetry, gearRatio, motorPPR, motorPowerChangeTolerance, hubVoltage, deviceName, motor,
                config.accelMax, config.jerkIncreasing, config.jerkDecreasing, 10.0, clock);
        _config = config;
        accelLpf = new BiquadLowPassVarDt(_config.accelLpfCutoffHz, 0.5);
    }

    @Override
    protected void setTPS(double tps) {
        if (tps != lastTpsSet) {
            voltage = getVoltage();
            double vmax = getMaxVelocity();
            tps = MathUtil.clamp(tps, -vmax, vmax);
            if (lastTpsSet == 0) {
                trajectory.resetFromMeasurement(getTpsFiltered());
            }
            trajectory.setTarget(tps);
            lastTpsSet = tps;
        }
    }

    /**
     * Returns the maximum achievable velocity at the last known battery voltage,
     * reduced by {@link VelocityMotorPFConfig#headroomAllowance}.
     * Derived from the feedforward model: {@code (1 - headroomAllowance) * (voltage - kS) / kV}.
     *
     * @return A conservative maximum velocity in ticks per second.
     */
    public double getMaxVelocity() {
        return (1.0 - _config.headroomAllowance) * (voltage - _config.kS) / _config.kV;
    }

    @Override
    protected void updateInternal(double dt) {
        super.setFilterCutoff(_config.measurementLpfCutoffHz);
        super.updateInternal(dt);

        accelLpf.setCutoffHz(_config.accelLpfCutoffHz);

        trajectory.updateConfig(_config.accelMax, _config.jerkIncreasing, _config.jerkDecreasing);
        trajectory.update();

        if (trajectory.getAcceleration() == 0) {
            voltage = getVoltage();
        }

        double v = trajectory.getVelocity();
        double t = _config.ffFromTrajectory ? v : trajectory.getTarget();
        double a = trajectory.getAcceleration();
        double ff = _config.kS * Math.signum(t) + _config.kV * t + _config.kA * a;
        ff /= voltage;
        double tpsFiltered = getTpsFiltered();
        if (dt > 1e-6) {
            double accel = (tpsFiltered - lastFilteredTps) / dt;
            if (!Double.isInfinite(accel) && !Double.isNaN(accel)) {
                accelLpf.update(accel, dt); // acceleration in ticks / sec^2
            }
        }
        lastFilteredTps = tpsFiltered;
        double ve = v - tpsFiltered;
        // Scale kP down during acceleration to prevent LPF lag from inflating ve and causing overshoot.
        // Fades from 0 at full acceleration to full kP at steady state, avoiding a discontinuous step.
        double kPEffective = _config.kP * Math.max(0, 1.0 - Math.abs(a) / _config.accelMax);
        double power = ff + kPEffective * ve;

        if (Math.abs(trajectory.getTarget()) < 1e-6) {
            setPower(0);
        } else {
            setPower(power);
        }
    }

    /**
     * Determines if the motor has reached its target speed and stabilized.
     * <p>
     * This method checks several conditions to ensure the motor is truly "at speed":
     * <ul>
     *   <li>The target speed is non-zero (specifically greater than 200 TPS), ignoring idle states.</li>
     *   <li>The internal trajectory profile has finished accelerating (acceleration is 0).</li>
     *   <li>The error between the target speed and the filtered measured speed is within the configured tolerance ({@code targetSpeedTolerance}).</li>
     *   <li>The filtered acceleration of the motor is within the configured tolerance ({@code accelerationTolerance}), indicating stability.</li>
     * </ul>
     *
     * @return {@code true} if the motor is moving at the target speed within tolerances and is stable; {@code false} otherwise.
     */
    @Override
    public boolean isAtTargetSpeed() {
        return Math.abs(trajectory.getTarget()) > 200 && // 200 isn't that important, but we don't want target 0
                trajectory.getAcceleration() == 0 && // it should be done following the trajectory
                Math.abs(trajectory.getTarget() - getTpsFiltered()) < _config.targetSpeedTolerance &&
                Math.abs(accelLpf.getValue()) < _config.accelerationTolerance;
    }

    @Override
    public void writeTelemetry() {
        super.writeTelemetry();
        telemetry.addData(deviceNames[0] + " TPS LPF", "%.1f", getTpsFiltered());
        telemetry.addData(deviceNames[0] + " RPM LPF", "%.1f", tpsToRpm(getTpsFiltered()));
        telemetry.addData(deviceNames[0] + " accel LPF", "%.1f", accelLpf.getValue());
        telemetry.addData("voltage lpf", "%.2f", getVoltage());
        telemetry.addData("voltage used", "%.2f", voltage);
    }

    /**
     * Stops all motors immediately by resetting the velocity trajectory target to zero
     * and cutting power.
     * <p>
     * This method sets the internal trajectory target to 0.0, forces motor power to 0,
     * and resets the last set TPS value to allow for a fresh trajectory calculation
     * upon the next velocity command.
     */
    @Override
    public void stop() {
        trajectory.setTarget(0.0);
        setPower(0);
        lastTpsSet = 0;
    }
}
