package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorController
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDCoefficients
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit
import org.firstinspires.ftc.teamcode.utils.HubHelper
import org.marsroboticsassociation.controllib.hardware.IMotor

open class EncapsulatedDcMotorEx : DcMotorEx, IMotor {
    // Nullable so unit-test stubs can pass null and override all used methods.
    private val motor: DcMotorEx?
    val hub: LynxModule?
    private val deviceName: String?

    constructor(hardwareMap: HardwareMap, deviceName: String) {
        val m = hardwareMap.get(DcMotorEx::class.java, deviceName)
        motor = m
        this.hub = HubHelper.getHubForMotor(m, hardwareMap)
        this.deviceName = deviceName
    }

    protected constructor(motor: DcMotorEx?) {
        this.motor = motor
        this.hub =
            if (motor is EncapsulatedDcMotorEx) {
                motor.hub
            } else {
                null
            }
        this.deviceName = null
    }

    private fun requireMotor(): DcMotorEx {
        return motor ?: throw UnsupportedOperationException("motor not available on this stub")
    }

    override val name: String
        get() {
            val m = this.motor
            if (m is IMotor) {
                return m.name
            }
            // deviceName is null for test stubs constructed without HardwareMap.
            return deviceName ?: "stub"
        }

    override val position: Int
        get() = currentPosition

    override val hubVoltage: Double
        get() {
            val hub =
                this.hub
                    ?: throw UnsupportedOperationException(
                        "hubVoltage requires construction from HardwareMap"
                    )
            return hub.getInputVoltage(VoltageUnit.VOLTS)
        }

    /**
     * Individually energizes this particular motor
     *
     * @see #setMotorDisable()
     * @see #isMotorEnabled()
     */
    override fun setMotorEnable() {
        requireMotor().setMotorEnable()
    }

    /**
     * Individually de-energizes this particular motor
     *
     * @see #setMotorEnable()
     * @see #isMotorEnabled()
     */
    override fun setMotorDisable() {
        requireMotor().setMotorDisable()
    }

    /**
     * Returns whether this motor is energized
     *
     * @see #setMotorEnable()
     * @see #setMotorDisable()
     */
    override fun isMotorEnabled(): Boolean {
        return requireMotor().isMotorEnabled
    }

    /**
     * Sets the velocity of the motor
     *
     * @param angularRate the desired ticks per second
     */
    override fun setVelocity(angularRate: Double) {
        requireMotor().setVelocity(angularRate)
    }

    /**
     * Sets the velocity of the motor
     *
     * @param angularRate the desired angular rate, in units per second
     * @param unit the units in which angularRate is expressed
     * @see #getVelocity(AngleUnit)
     */
    override fun setVelocity(angularRate: Double, unit: AngleUnit) {
        requireMotor().setVelocity(angularRate, unit)
    }

    override val encoderVelocity: Double
        get() = requireMotor().velocity

    override fun getVelocity(): Double = encoderVelocity

    /**
     * Returns the current velocity of the motor, in angular units per second
     *
     * @param unit the units in which the angular rate is desired
     * @return the current velocity of the motor
     * @see #setVelocity(double, AngleUnit)
     */
    override fun getVelocity(unit: AngleUnit): Double {
        return requireMotor().getVelocity(unit)
    }

    /**
     * Sets the PID control coefficients for one of the PID modes of this motor. Note that in some
     * controller implementations, setting the PID coefficients for one mode on a motor might affect
     * other modes on that motor, or might affect the PID coefficients used by other motors on the
     * same controller (this is not true on the REV Expansion Hub).
     *
     * @param mode either [RunMode.RUN_USING_ENCODER] or [RunMode.RUN_TO_POSITION]
     * @param pidCoefficients the new coefficients to use when in that mode on this motor
     * @see #getPIDCoefficients(RunMode)
     */
    @Deprecated("Use setPIDFCoefficients(RunMode, PIDFCoefficients) instead")
    override fun setPIDCoefficients(mode: DcMotor.RunMode, pidCoefficients: PIDCoefficients) {
        requireMotor().setPIDCoefficients(mode, pidCoefficients)
    }

    /**
     * [setPIDFCoefficients] is a superset enhancement to [setPIDCoefficients]. In addition to the
     * proportional, integral, and derivative coefficients previously supported, a feed-forward
     * coefficient may also be specified. Further, a selection of motor control algorithms is
     * offered: the originally-shipped Legacy PID algorithm, and a PIDF algorithm which avails
     * itself of the feed-forward coefficient. Note that the feed-forward coefficient is not used by
     * the Legacy PID algorithm; thus, the feed-forward coefficient must be indicated as zero if the
     * Legacy PID algorithm is used. Also: the internal implementation of these algorithms may be
     * different: it is not the case that the use of PIDF with the F term as zero necessarily
     * exhibits exactly the same behavior as the use of the LegacyPID algorithm, though in practice
     * they will be quite close.
     *
     * Readers are reminded that [DcMotor.RunMode.RUN_TO_POSITION] mode makes use of *both* the
     * coefficients set for RUN_TO_POSITION *and* the coefficients set for RUN_WITH_ENCODER, due to
     * the fact that internally the RUN_TO_POSITION logic calculates an on-the-fly velocity goal on
     * each control cycle, then (logically) runs the RUN_WITH_ENCODER logic. Because of that double-
     * layering, only the proportional ('p') coefficient makes logical sense for use in the
     * RUN_TO_POSITION coefficients.
     *
     * @see #setVelocityPIDFCoefficients(double, double, double, double)
     * @see #setPositionPIDFCoefficients(double)
     * @see #getPIDFCoefficients(RunMode)
     */
    override fun setPIDFCoefficients(mode: DcMotor.RunMode, pidfCoefficients: PIDFCoefficients) {
        requireMotor().setPIDFCoefficients(mode, pidfCoefficients)
    }

    /**
     * A shorthand for setting the PIDF coefficients for the [DcMotor.RunMode.RUN_USING_ENCODER]
     * mode. [com.qualcomm.robotcore.hardware.MotorControlAlgorithm.PIDF] is used.
     *
     * @see #setPIDFCoefficients(RunMode, PIDFCoefficients)
     */
    override fun setVelocityPIDFCoefficients(p: Double, i: Double, d: Double, f: Double) {
        requireMotor().setVelocityPIDFCoefficients(p, i, d, f)
    }

    /**
     * A shorthand for setting the PIDF coefficients for the [DcMotor.RunMode.RUN_TO_POSITION] mode.
     * [com.qualcomm.robotcore.hardware.MotorControlAlgorithm.PIDF] is used.
     *
     * Readers are reminded that [DcMotor.RunMode.RUN_TO_POSITION] mode makes use of *both* the
     * coefficients set for RUN_TO_POSITION *and* the coefficients set for RUN_WITH_ENCODER, due to
     * the fact that internally the RUN_TO_POSITION logic calculates an on-the-fly velocity goal on
     * each control cycle, then (logically) runs the RUN_WITH_ENCODER logic. Because of that double-
     * layering, only the proportional ('p') coefficient makes logical sense for use in the
     * RUN_TO_POSITION coefficients.
     *
     * @see #setVelocityPIDFCoefficients(double, double, double, double)
     * @see #setPIDFCoefficients(RunMode, PIDFCoefficients)
     */
    override fun setPositionPIDFCoefficients(p: Double) {
        requireMotor().setPositionPIDFCoefficients(p)
    }

    /**
     * Returns the PID control coefficients used when running in the indicated mode on this motor.
     *
     * @param mode either [RunMode.RUN_USING_ENCODER] or [RunMode.RUN_TO_POSITION]
     * @return the PID control coefficients used when running in the indicated mode on this motor
     * @see #getPIDFCoefficients(RunMode)
     */
    @Deprecated("Use getPIDFCoefficients(RunMode) instead")
    override fun getPIDCoefficients(mode: DcMotor.RunMode): PIDCoefficients {
        return requireMotor().getPIDCoefficients(mode)
    }

    /**
     * Returns the PIDF control coefficients used when running in the indicated mode on this motor.
     *
     * @param mode either [RunMode.RUN_USING_ENCODER] or [RunMode.RUN_TO_POSITION]
     * @return the PIDF control coefficients used when running in the indicated mode on this motor
     * @see #setPIDFCoefficients(RunMode, PIDFCoefficients)
     */
    override fun getPIDFCoefficients(mode: DcMotor.RunMode): PIDFCoefficients {
        return requireMotor().getPIDFCoefficients(mode)
    }

    /**
     * Sets the target positioning tolerance of this motor
     *
     * @param tolerance the desired tolerance, in encoder ticks
     * @see DcMotor#setTargetPosition(int)
     */
    override fun setTargetPositionTolerance(tolerance: Int) {
        requireMotor().targetPositionTolerance = tolerance
    }

    /**
     * Returns the current target positioning tolerance of this motor
     *
     * @return the current target positioning tolerance of this motor
     */
    override fun getTargetPositionTolerance(): Int {
        return requireMotor().targetPositionTolerance
    }

    /**
     * Returns the current consumed by this motor.
     *
     * @param unit current units
     * @return the current consumed by this motor.
     */
    override fun getCurrent(unit: CurrentUnit): Double {
        return requireMotor().getCurrent(unit)
    }

    /**
     * Returns the current alert for this motor.
     *
     * @param unit current units
     * @return the current alert for this motor
     */
    override fun getCurrentAlert(unit: CurrentUnit): Double {
        return requireMotor().getCurrentAlert(unit)
    }

    /**
     * Sets the current alert for this motor
     *
     * @param current current alert
     * @param unit current units
     */
    override fun setCurrentAlert(current: Double, unit: CurrentUnit) {
        requireMotor().setCurrentAlert(current, unit)
    }

    /**
     * Returns whether the current consumption of this motor exceeds the alert threshold.
     *
     * @return whether the current consumption of this motor exceeds the alert threshold.
     */
    override fun isOverCurrent(): Boolean {
        return requireMotor().isOverCurrent
    }

    /**
     * Returns the assigned type for this motor. If no particular motor type has been configured,
     * then [MotorConfigurationType.getUnspecifiedMotorType] will be returned. Note that the motor
     * type for a given motor is initially assigned in the robot configuration user interface,
     * though it may subsequently be modified using methods herein.
     *
     * @return the assigned type for this motor
     */
    override fun getMotorType(): MotorConfigurationType {
        return requireMotor().motorType
    }

    /**
     * Sets the assigned type of this motor. Usage of this method is very rare.
     *
     * @param motorType the new assigned type for this motor
     * @see #getMotorType()
     */
    override fun setMotorType(motorType: MotorConfigurationType) {
        requireMotor().motorType = motorType
    }

    /**
     * Returns the underlying motor controller on which this motor is situated.
     *
     * @return the underlying motor controller on which this motor is situated.
     * @see #getPortNumber()
     */
    override fun getController(): DcMotorController {
        return requireMotor().controller
    }

    /**
     * Returns the port number on the underlying motor controller on which this motor is situated.
     *
     * @return the port number on the underlying motor controller on which this motor is situated.
     * @see #getController()
     */
    override fun getPortNumber(): Int {
        return requireMotor().portNumber
    }

    /**
     * Sets the behavior of the motor when a power level of zero is applied.
     *
     * @param zeroPowerBehavior the new behavior of the motor when a power level of zero is applied.
     * @see ZeroPowerBehavior
     * @see #setPower(double)
     */
    override fun setZeroPowerBehavior(zeroPowerBehavior: DcMotor.ZeroPowerBehavior) {
        requireMotor().zeroPowerBehavior = zeroPowerBehavior
    }

    /**
     * Returns the current behavior of the motor were a power level of zero to be applied.
     *
     * @return the current behavior of the motor were a power level of zero to be applied.
     */
    override fun getZeroPowerBehavior(): DcMotor.ZeroPowerBehavior {
        return requireMotor().zeroPowerBehavior
    }

    /**
     * Sets the zero power behavior of the motor to [DcMotor.ZeroPowerBehavior.FLOAT], then applies
     * zero power to that requireMotor().
     *
     * Note that the change of the zero power behavior to [DcMotor.ZeroPowerBehavior.FLOAT] remains
     * in effect even following the return of this method. **This is a breaking change** in behavior
     * from previous releases of the SDK.
     *
     * @see #setPower(double)
     * @see #getPowerFloat()
     * @see #setZeroPowerBehavior(ZeroPowerBehavior)
     */
    @Deprecated(
        "This method is deprecated in favor of direct use of setZeroPowerBehavior() and setPower()."
    )
    override fun setPowerFloat() {
        requireMotor().setPowerFloat()
    }

    /**
     * Returns whether the motor is currently in a float power level.
     *
     * @return whether the motor is currently in a float power level.
     * @see #setPowerFloat()
     */
    override fun getPowerFloat(): Boolean {
        return requireMotor().powerFloat
    }

    /**
     * Sets the desired encoder target position to which the motor should advance or retreat and
     * then actively hold thereat. This behavior is similar to the operation of a servo. The maximum
     * speed at which this advance or retreat occurs is governed by the power level currently set on
     * the requireMotor(). While the motor is advancing or retreating to the desired target
     * position, [isBusy] will return true.
     *
     * Note that adjustment to a target position is only effective when the motor is in
     * [RunMode.RUN_TO_POSITION] RunMode. Note further that, clearly, the motor must be equipped
     * with an encoder in order for this mode to function properly.
     *
     * @param position the desired encoder target position
     * @see #getCurrentPosition()
     * @see #setMode(RunMode)
     * @see RunMode#RUN_TO_POSITION
     * @see #getTargetPosition()
     * @see #isBusy()
     */
    override fun setTargetPosition(position: Int) {
        requireMotor().targetPosition = position
    }

    /**
     * Returns the current target encoder position for this motor.
     *
     * @return the current target encoder position for this motor.
     * @see #setTargetPosition(int)
     */
    override fun getTargetPosition(): Int {
        return requireMotor().targetPosition
    }

    /**
     * Returns true if the motor is currently advancing or retreating to a target position.
     *
     * @return true if the motor is currently advancing or retreating to a target position.
     * @see #setTargetPosition(int)
     */
    override fun isBusy(): Boolean {
        return requireMotor().isBusy
    }

    /**
     * Returns the current reading of the encoder for this motor. The units for this reading, that
     * is, the number of ticks per revolution, are specific to the motor/encoder in question, and
     * thus are not specified here.
     *
     * @return the current reading of the encoder for this motor
     * @see #getTargetPosition()
     * @see RunMode#STOP_AND_RESET_ENCODER
     */
    override fun getCurrentPosition(): Int {
        return requireMotor().currentPosition
    }

    /**
     * Sets the current run mode for this motor
     *
     * @param mode the new current run mode for this motor
     * @see RunMode
     * @see #getMode()
     */
    override fun setMode(mode: DcMotor.RunMode) {
        requireMotor().mode = mode
    }

    /**
     * Returns the current run mode for this motor
     *
     * @return the current run mode for this motor
     * @see RunMode
     * @see #setMode(RunMode)
     */
    override fun getMode(): DcMotor.RunMode {
        return requireMotor().mode
    }

    /**
     * Sets the logical direction in which this motor operates.
     *
     * @param direction the direction to set for this motor
     * @see #getDirection()
     */
    override fun setDirection(direction: DcMotorSimple.Direction) {
        requireMotor().direction = direction
    }

    /**
     * Returns the current logical direction in which this motor is set as operating.
     *
     * @return the current logical direction in which this motor is set as operating.
     * @see #setDirection(Direction)
     */
    override fun getDirection(): DcMotorSimple.Direction {
        return requireMotor().direction
    }

    /**
     * Sets the power level of the motor, expressed as a fraction of the maximum possible power /
     * speed supported according to the run mode in which the motor is operating.
     *
     * Setting a power level of zero will brake the motor
     *
     * @param power the new power level of the motor, a value in the interval [-1.0, 1.0]
     * @see #getPower()
     * @see DcMotor#setMode(DcMotor.RunMode)
     * @see DcMotor#setPowerFloat()
     */
    override fun setPower(power: Double) {
        requireMotor().power = power
    }

    /**
     * Returns the current configured power level of the requireMotor().
     *
     * @return the current level of the motor, a value in the interval [0.0, 1.0]
     * @see #setPower(double)
     */
    override fun getPower(): Double {
        return requireMotor().power
    }

    /**
     * Returns an indication of the manufacturer of this device.
     *
     * @return the device's manufacturer
     */
    override fun getManufacturer(): com.qualcomm.robotcore.hardware.HardwareDevice.Manufacturer {
        return requireMotor().manufacturer
    }

    /**
     * Returns a string suitable for display to the user as to the type of device. Note that this is
     * a device-type-specific name; it has nothing to do with the name by which a user might have
     * configured the device in a robot configuration.
     *
     * @return device manufacturer and name
     */
    override fun getDeviceName(): String {
        return requireMotor().deviceName
    }

    /**
     * Get connection information about this device in a human readable format
     *
     * @return connection info
     */
    override fun getConnectionInfo(): String {
        return requireMotor().connectionInfo
    }

    /**
     * Version
     *
     * @return get the version of this device
     */
    override fun getVersion(): Int {
        return requireMotor().version
    }

    /**
     * Resets the device's configuration to that which is expected at the beginning of an OpMode.
     * For example, motors will reset the their direction to 'forward'.
     */
    override fun resetDeviceConfigurationForOpMode() {
        requireMotor().resetDeviceConfigurationForOpMode()
    }

    /** Closes this device */
    override fun close() {
        requireMotor().close()
    }
}
