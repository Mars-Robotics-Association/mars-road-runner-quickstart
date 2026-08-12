package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import com.qualcomm.robotcore.util.Range
import kotlin.math.abs
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit

/**
 * Wrapper for GoBilda servos that provides angle-based control.
 *
 * <p>This class maps angular positions to servo positions and automatically configures the PWM
 * range based on the servo's angular range (300°, 1800°, or 180°).
 */
class GoBildaServo {

    var servo: ServoImplEx

    // Always stored internally as radians
    private var maxAngle: Double
    private var minAngle: Double

    private val maxPosition = 1.0
    private val minPosition = 0.0

    /**
     * Constructs a GoBildaServo with the specified angular range.
     *
     * <p>Automatically configures PWM range based on the angular range:
     * <ul>
     * <li>300° or 1800°: PWM 500-2500</li>
     * <li>180°: PWM 600-2400</li>
     * </ul>
     *
     * @param hw The hardware map.
     * @param servoName The name of the servo in the hardware configuration.
     * @param minAngle The minimum angle of the servo's range.
     * @param maxAngle The maximum angle of the servo's range.
     * @param angleUnit The unit for minAngle and maxAngle.
     */
    constructor(
        hw: HardwareMap,
        servoName: String,
        minAngle: Double,
        maxAngle: Double,
        angleUnit: AngleUnit,
    ) {
        servo = hw.get(ServoImplEx::class.java, servoName)

        val range = abs(Math.round(fromRadians(maxAngle - minAngle, angleUnit)))

        if (range == 300L || range == 1800L) {
            servo.setPwmRange(PwmControl.PwmRange(500.0, 2500.0))
        } else if (range == 180L) {
            servo.setPwmRange(PwmControl.PwmRange(600.0, 2400.0))
        }

        this.minAngle = toRadians(minAngle, angleUnit)
        this.maxAngle = toRadians(maxAngle, angleUnit)
    }

    /**
     * Constructs a GoBildaServo with the specified angular range in degrees.
     *
     * @param hw The hardware map.
     * @param servoName The name of the servo in the hardware configuration.
     * @param minDegree The minimum angle in degrees.
     * @param maxDegree The maximum angle in degrees.
     */
    constructor(
        hw: HardwareMap,
        servoName: String,
        minDegree: Double,
        maxDegree: Double,
    ) : this(hw, servoName, minDegree, maxDegree, AngleUnit.DEGREES)

    /**
     * Rotates the servo by a relative angle from its current position.
     *
     * @param angle The angle to rotate by.
     * @param angleUnit The unit for the angle.
     */
    fun rotateByAngle(angle: Double, angleUnit: AngleUnit) {
        val target = getAngle(angleUnit) + angle
        turnToAngle(target, angleUnit)
    }

    /**
     * Rotates the servo by a relative angle in degrees from its current position.
     *
     * @param degrees The angle to rotate by in degrees.
     */
    fun rotateByAngle(degrees: Double) {
        rotateByAngle(degrees, AngleUnit.DEGREES)
    }

    /**
     * Turns the servo to an absolute angle.
     *
     * @param angle The target angle (clamped to the servo's configured range).
     * @param angleUnit The unit for the angle.
     */
    fun turnToAngle(angle: Double, angleUnit: AngleUnit) {
        val angleRadians = Range.clip(toRadians(angle, angleUnit), minAngle, maxAngle)
        setPosition((angleRadians - minAngle) / (getAngleRange(AngleUnit.RADIANS)))
    }

    /**
     * Turns the servo to an absolute angle in degrees.
     *
     * @param degrees The target angle in degrees.
     */
    fun turnToAngle(degrees: Double) {
        turnToAngle(degrees, AngleUnit.DEGREES)
    }

    /**
     * Rotates the servo by a relative position value.
     *
     * @param position The position delta (added to current position).
     */
    fun rotateBy(position: Double) {
        val target = getPosition() + position
        setPosition(target)
    }

    /**
     * Sets the servo to an absolute position.
     *
     * <p>Automatically enables PWM if it was disabled.
     *
     * @param position The target position (0.0 to 1.0, clamped).
     */
    fun setPosition(position: Double) {
        if (!servo.isPwmEnabled) {
            servo.setPwmEnable()
        }
        servo.position = Range.clip(position, minPosition, maxPosition)
    }

    /**
     * Updates the servo's angular range mapping.
     *
     * @param min The new minimum angle.
     * @param max The new maximum angle.
     * @param angleUnit The unit for the angles.
     */
    fun setRange(min: Double, max: Double, angleUnit: AngleUnit) {
        this.minAngle = toRadians(min, angleUnit)
        this.maxAngle = toRadians(max, angleUnit)
    }

    /**
     * Updates the servo's angular range mapping in degrees.
     *
     * @param min The new minimum angle in degrees.
     * @param max The new maximum angle in degrees.
     */
    fun setRange(min: Double, max: Double) {
        setRange(min, max, AngleUnit.DEGREES)
    }

    /** Sets whether the servo direction is inverted. */
    fun setInverted(isInverted: Boolean) {
        servo.direction = if (isInverted) Servo.Direction.REVERSE else Servo.Direction.FORWARD
    }

    /** Returns whether the servo direction is inverted. */
    fun getInverted(): Boolean {
        return Servo.Direction.REVERSE == servo.direction
    }

    /** Returns the current servo position (0.0 to 1.0). */
    fun getPosition(): Double {
        return servo.position
    }

    /**
     * Returns the current servo angle.
     *
     * @param angleUnit The unit for the returned angle.
     * @return The current angle.
     */
    fun getAngle(angleUnit: AngleUnit): Double {
        return getPosition() * getAngleRange(angleUnit) + fromRadians(minAngle, angleUnit)
    }

    /** Returns the current servo angle in degrees. */
    fun getAngle(): Double {
        return getAngle(AngleUnit.DEGREES)
    }

    /**
     * Returns the servo's total angular range.
     *
     * @param angleUnit The unit for the returned range.
     * @return The angular range (maxAngle - minAngle).
     */
    fun getAngleRange(angleUnit: AngleUnit): Double {
        return fromRadians(maxAngle - minAngle, angleUnit)
    }

    /** Returns the servo's total angular range in degrees. */
    fun getAngleRange(): Double {
        return getAngleRange(AngleUnit.DEGREES)
    }

    /** Disables PWM output to the servo, allowing it to be moved freely. */
    fun disable() {
        servo.setPwmDisable()
    }

    /** Enables PWM output to the servo. */
    fun enable() {
        servo.setPwmEnable()
    }

    /** Returns a string describing the servo's port and controller. */
    fun getDeviceType(): String {
        val port = Integer.toString(servo.portNumber)
        val controller = servo.controller.toString()
        return "LimpServo: $port; $controller"
    }

    private fun toRadians(angle: Double, angleUnit: AngleUnit): Double {
        return if (angleUnit == AngleUnit.DEGREES) Math.toRadians(angle) else angle
    }

    private fun fromRadians(angle: Double, angleUnit: AngleUnit): Double {
        return if (angleUnit == AngleUnit.DEGREES) Math.toDegrees(angle) else angle
    }
}
