package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDCoefficients
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit

/**
 * A group of mechanically coupled motors that act as one. Power and enable/disable are fanned
 * out to every motor; read operations come from the first (primary) motor. Closed-loop modes
 * (velocity, position) are not supported — tandem motors running independent closed-loop
 * controllers would fight each other.
 */
class LinkedMotorGroup(hardwareMap: HardwareMap, vararg configs: MotorConfig) :
    EncapsulatedDcMotorEx(hardwareMap, configs[0].name) {
    private val motors: MutableList<DcMotorEx> = ArrayList(configs.size)

    init {
        for (config in configs) {
            val m = hardwareMap.get(DcMotorEx::class.java, config.name)
            m.direction = config.direction
            motors.add(m)
        }
    }

    private fun forAll(action: (DcMotorEx) -> Unit) {
        motors.forEach(action)
    }

    override fun setPower(power: Double) {
        forAll { m -> m.power = power }
    }

    override fun setZeroPowerBehavior(zeroPowerBehavior: DcMotor.ZeroPowerBehavior) {
        forAll { m -> m.zeroPowerBehavior = zeroPowerBehavior }
    }

    override fun setMotorEnable() {
        forAll { m -> m.setMotorEnable() }
    }

    override fun setMotorDisable() {
        forAll { m -> m.setMotorDisable() }
    }

    // Closed-loop control is not supported for tandem motors — independent controllers
    // on mechanically coupled shafts will fight each other.

    override fun setMode(mode: DcMotor.RunMode) {
        if (mode == DcMotor.RunMode.RUN_USING_ENCODER || mode == DcMotor.RunMode.RUN_TO_POSITION) {
            throw UnsupportedOperationException("LinkedMotorGroup does not support closed-loop mode $mode")
        }
        forAll { m -> m.mode = mode }
    }

    override fun setVelocity(angularRate: Double) {
        throw UnsupportedOperationException("LinkedMotorGroup does not support setVelocity")
    }

    override fun setVelocity(angularRate: Double, unit: AngleUnit) {
        throw UnsupportedOperationException("LinkedMotorGroup does not support setVelocity")
    }

    override fun setTargetPosition(position: Int) {
        throw UnsupportedOperationException("LinkedMotorGroup does not support setTargetPosition")
    }

    override fun setTargetPositionTolerance(tolerance: Int) {
        throw UnsupportedOperationException("LinkedMotorGroup does not support setTargetPositionTolerance")
    }

    override fun setPIDFCoefficients(mode: DcMotor.RunMode, pidfCoefficients: PIDFCoefficients) {
        throw UnsupportedOperationException("LinkedMotorGroup does not support setPIDFCoefficients")
    }

    override fun setVelocityPIDFCoefficients(p: Double, i: Double, d: Double, f: Double) {
        throw UnsupportedOperationException("LinkedMotorGroup does not support setVelocityPIDFCoefficients")
    }

    override fun setPositionPIDFCoefficients(p: Double) {
        throw UnsupportedOperationException("LinkedMotorGroup does not support setPositionPIDFCoefficients")
    }

    @Deprecated("Deprecated in Java")
    override fun setPIDCoefficients(mode: DcMotor.RunMode, pidCoefficients: PIDCoefficients) {
        throw UnsupportedOperationException("LinkedMotorGroup does not support setPIDCoefficients")
    }

    override fun setCurrentAlert(current: Double, unit: CurrentUnit) {
        forAll { m -> m.setCurrentAlert(current, unit) }
    }

    @Deprecated("Deprecated in Java")
    override fun setPowerFloat() {
        forAll { m -> m.setPowerFloat() }
    }

    override fun resetDeviceConfigurationForOpMode() {
        forAll { m -> m.resetDeviceConfigurationForOpMode() }
    }

    override fun close() {
        forAll { m -> m.close() }
    }
}
