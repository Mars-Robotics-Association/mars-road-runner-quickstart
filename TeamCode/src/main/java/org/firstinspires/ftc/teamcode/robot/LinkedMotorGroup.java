package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

/**
 * A group of mechanically coupled motors that act as one. Power and enable/disable are fanned
 * out to every motor; read operations come from the first (primary) motor. Closed-loop modes
 * (velocity, position) are not supported — tandem motors running independent closed-loop
 * controllers would fight each other.
 */
public class LinkedMotorGroup extends EncapsulatedDcMotorEx {
    private final List<DcMotorEx> motors;

    public LinkedMotorGroup(HardwareMap hardwareMap, MotorConfig... configs) {
        super(hardwareMap, configs[0].name());
        motors = new ArrayList<>(configs.length);
        for (MotorConfig config : configs) {
            DcMotorEx m = hardwareMap.get(DcMotorEx.class, config.name());
            m.setDirection(config.direction());
            motors.add(m);
        }
    }

    private void forAll(Consumer<DcMotorEx> action) {
        motors.forEach(action);
    }

    @Override
    public void setPower(double power) {
        forAll(m -> m.setPower(power));
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        forAll(m -> m.setZeroPowerBehavior(zeroPowerBehavior));
    }

    @Override
    public void setMotorEnable() {
        forAll(DcMotorEx::setMotorEnable);
    }

    @Override
    public void setMotorDisable() {
        forAll(DcMotorEx::setMotorDisable);
    }

    // Closed-loop control is not supported for tandem motors — independent controllers
    // on mechanically coupled shafts will fight each other.

    @Override
    public void setMode(DcMotor.RunMode mode) {
        if (mode == DcMotor.RunMode.RUN_USING_ENCODER || mode == DcMotor.RunMode.RUN_TO_POSITION) {
            throw new UnsupportedOperationException("LinkedMotorGroup does not support closed-loop mode " + mode);
        }
        forAll(m -> m.setMode(mode));
    }

    @Override
    public void setVelocity(double angularRate) {
        throw new UnsupportedOperationException("LinkedMotorGroup does not support setVelocity");
    }

    @Override
    public void setVelocity(double angularRate, AngleUnit unit) {
        throw new UnsupportedOperationException("LinkedMotorGroup does not support setVelocity");
    }

    @Override
    public void setTargetPosition(int position) {
        throw new UnsupportedOperationException("LinkedMotorGroup does not support setTargetPosition");
    }

    @Override
    public void setTargetPositionTolerance(int tolerance) {
        throw new UnsupportedOperationException("LinkedMotorGroup does not support setTargetPositionTolerance");
    }

    @Override
    public void setPIDFCoefficients(DcMotor.RunMode mode, PIDFCoefficients pidfCoefficients) {
        throw new UnsupportedOperationException("LinkedMotorGroup does not support setPIDFCoefficients");
    }

    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        throw new UnsupportedOperationException("LinkedMotorGroup does not support setVelocityPIDFCoefficients");
    }

    @Override
    public void setPositionPIDFCoefficients(double p) {
        throw new UnsupportedOperationException("LinkedMotorGroup does not support setPositionPIDFCoefficients");
    }

    @Override
    @Deprecated
    public void setPIDCoefficients(DcMotor.RunMode mode, PIDCoefficients pidCoefficients) {
        throw new UnsupportedOperationException("LinkedMotorGroup does not support setPIDCoefficients");
    }

    @Override
    public void setCurrentAlert(double current, CurrentUnit unit) {
        forAll(m -> m.setCurrentAlert(current, unit));
    }

    @Override
    @Deprecated
    public void setPowerFloat() {
        forAll(DcMotorEx::setPowerFloat);
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        forAll(DcMotorEx::resetDeviceConfigurationForOpMode);
    }

    @Override
    public void close() {
        forAll(DcMotorEx::close);
    }
}
