package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.LongSupplier;

import edu.wpi.first.math.MathUtil;
import org.marsroboticsassociation.controllib.util.SetOnChange;
import org.marsroboticsassociation.controllib.util.TelemetryAddData;

public class VelocityMotorSdkPidf extends VelocityMotorBase {
    public static class MotorPIDFConfig {
        public double Kv;
        public double nominalVoltage;
        public double Kp;
        public double Ki;
        public double Kd;
        public double maxSettableVelocity;
        public double maxAccel;
        public double jerkIncreasing;
        public double jerkDecreasing;
        public double lpfCutoff;
        public Direction motorDirection = Direction.FORWARD;

        public MotorPIDFConfig(double maxTPS) {
            Kv = 32767.0 / maxTPS;
            nominalVoltage = 12.0;
            Kp = 0.1 * Kv;
            Ki = 0.1 * Kp;
            Kd = 0.01 * Kp;
            maxSettableVelocity = 0.8 * maxTPS;
            maxAccel = 1196.7;
            jerkIncreasing = 2669.2;
            jerkDecreasing = 800;
            lpfCutoff = 4;
        }
    }

    public final MotorPIDFConfig config;

    private final SetOnChange<Double> motorVelocitySetpoint;
    private final SetOnChange<Double> voltageFactor;
    private final SetOnChange<Double> Kp;
    private final SetOnChange<Double> Ki;
    private final SetOnChange<Double> Kd;
    private final SetOnChange<Double> Kv;


    public VelocityMotorSdkPidf(HardwareMap hardwareMap, String deviceName, TelemetryAddData telemetry, MotorPIDFConfig config, double gearRatio, double motorPPR) {
        super(hardwareMap, telemetry, gearRatio, motorPPR, 0.05, config.maxAccel, config.jerkIncreasing, config.jerkDecreasing, 1.0, deviceName);
        this.config = config;
        motors[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motors[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (config.motorDirection != null) {
            motors[0].setDirection(config.motorDirection);
        }
        motorVelocitySetpoint = SetOnChange.ofDouble(0.0, 1.0, motors[0]::setVelocity);
        Runnable updatePIDF = () -> motors[0].setVelocityPIDFCoefficients(
                config.Kp,
                config.Ki,
                config.Kd,
                config.Kv * config.nominalVoltage / getVoltage());
        voltageFactor = SetOnChange.ofDouble(config.nominalVoltage / getVoltage(), 0.025, (vf) -> updatePIDF.run());
        Kp = SetOnChange.ofDouble(config.Kp, (kp) -> updatePIDF.run());
        Ki = SetOnChange.ofDouble(config.Ki, (ki) -> updatePIDF.run());
        Kd = SetOnChange.ofDouble(config.Kd, (kd) -> updatePIDF.run());
        Kv = SetOnChange.ofDouble(config.Kv, (kv) -> updatePIDF.run());
    }

    public VelocityMotorSdkPidf(HardwareMap hardwareMap, String deviceName, TelemetryAddData telemetry, MotorPIDFConfig config, MotorType motorType) {
        this(hardwareMap, deviceName, telemetry, config, motorType.getGearRatio(), motorType.getPulsesPerRevolution());
    }

    /** For unit tests only — bypasses HardwareMap, LynxModule, HubHelper, and motor hardware init. */
    VelocityMotorSdkPidf(TelemetryAddData telemetry, double gearRatio, double motorPPR,
                          double motorPowerChangeTolerance, double hubVoltage,
                          String deviceName, DcMotorEx motor, MotorPIDFConfig config,
                          LongSupplier clock) {
        super(telemetry, gearRatio, motorPPR, motorPowerChangeTolerance, hubVoltage, deviceName, motor,
                config.maxAccel, config.jerkIncreasing, config.jerkDecreasing, 1.0, clock);
        this.config = config;
        motorVelocitySetpoint = SetOnChange.ofDouble(0.0, 1.0, motor::setVelocity);
        Runnable updatePIDF = () -> motor.setVelocityPIDFCoefficients(
                config.Kp,
                config.Ki,
                config.Kd,
                config.Kv * config.nominalVoltage / getVoltage());
        voltageFactor = SetOnChange.ofDouble(config.nominalVoltage / getVoltage(), 0.025, (vf) -> updatePIDF.run());
        Kp = SetOnChange.ofDouble(config.Kp, (kp) -> updatePIDF.run());
        Ki = SetOnChange.ofDouble(config.Ki, (ki) -> updatePIDF.run());
        Kd = SetOnChange.ofDouble(config.Kd, (kd) -> updatePIDF.run());
        Kv = SetOnChange.ofDouble(config.Kv, (kv) -> updatePIDF.run());
    }

    @Override
    protected void setTPS(double tps) {
        tps = MathUtil.clamp(tps, -config.maxSettableVelocity, config.maxSettableVelocity);
        trajectory.setTarget(tps);
        motorMode.set(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public boolean isAtTargetSpeed() {
        return trajectory.getAcceleration() == 0 && Math.abs(getTpsFiltered() - trajectory.getTarget()) < 30;
    }

    public void setPower(double power) {
        power = MathUtil.clamp(power, -1.0, 1.0);
        motorMode.set(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        super.setPower(power);
    }

    @Override
    protected void updateInternal(double dt) {
        super.setFilterCutoff(config.lpfCutoff);
        super.updateInternal(dt);
        trajectory.updateConfig(config.maxAccel, config.jerkIncreasing, config.jerkDecreasing);
        Kp.set(config.Kp);
        Ki.set(config.Ki);
        Kd.set(config.Kd);
        Kv.set(config.Kv);
        voltageFactor.set(config.nominalVoltage / getVoltage());
        trajectory.update();
        if (Math.abs(trajectory.getTarget()) < 1e-6) {
            stop();
        } else {
            motorVelocitySetpoint.set(trajectory.getVelocity());
        }

    }

    @Override
    public void writeTelemetry() {
        super.writeTelemetry();
        telemetry.addData(deviceNames[0] + " Voltage Factor", "%.3f", voltageFactor.get());
        telemetry.addData(deviceNames[0] + " TPS LPF", "%.1f", getTpsFiltered());
        telemetry.addData(deviceNames[0] + " RPM LPF", "%.1f", tpsToRpm(getTpsFiltered()));
    }

    @Override
    public void stop() {
        trajectory.setTarget(0.0);
        motorMode.set(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorVelocitySetpoint.set(0.0);
        super.setPower(0.0);
    }
}
