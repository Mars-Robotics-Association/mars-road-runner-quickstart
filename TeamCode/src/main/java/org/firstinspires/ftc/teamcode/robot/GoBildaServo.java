package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Wrapper for GoBilda servos that provides angle-based control.
 *
 * <p>This class maps angular positions to servo positions and automatically configures
 * the PWM range based on the servo's angular range (300°, 1800°, or 180°).
 */
public class GoBildaServo {

    ServoImplEx servo;

    // Always stored internally as radians
    private double maxAngle, minAngle;

    private final double maxPosition = 1;
    private final double minPosition = 0;

    /**
     * Constructs a GoBildaServo with the specified angular range.
     *
     * <p>Automatically configures PWM range based on the angular range:
     * <ul>
     *   <li>300° or 1800°: PWM 500-2500</li>
     *   <li>180°: PWM 600-2400</li>
     * </ul>
     *
     * @param hw        The hardware map.
     * @param servoName The name of the servo in the hardware configuration.
     * @param minAngle  The minimum angle of the servo's range.
     * @param maxAngle  The maximum angle of the servo's range.
     * @param angleUnit The unit for minAngle and maxAngle.
     */
    public GoBildaServo(HardwareMap hw, String servoName, double minAngle, double maxAngle, AngleUnit angleUnit) {
        servo = hw.get(ServoImplEx.class, servoName);

        long range = Math.abs(Math.round(fromRadians(maxAngle - minAngle, angleUnit)));

        if (range == 300 || range == 1800) {
            servo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        } else if (range == 180) {
            servo.setPwmRange(new PwmControl.PwmRange(600, 2400));
        }

        this.minAngle = toRadians(minAngle, angleUnit);
        this.maxAngle = toRadians(maxAngle, angleUnit);
    }

    /**
     * Constructs a GoBildaServo with the specified angular range in degrees.
     *
     * @param hw        The hardware map.
     * @param servoName The name of the servo in the hardware configuration.
     * @param minDegree The minimum angle in degrees.
     * @param maxDegree The maximum angle in degrees.
     */
    public GoBildaServo(HardwareMap hw, String servoName, double minDegree, double maxDegree) {
        this(hw, servoName, minDegree, maxDegree, AngleUnit.DEGREES);
    }


    /**
     * Rotates the servo by a relative angle from its current position.
     *
     * @param angle     The angle to rotate by.
     * @param angleUnit The unit for the angle.
     */
    public void rotateByAngle(double angle, AngleUnit angleUnit) {
        angle = getAngle(angleUnit) + angle;
        turnToAngle(angle, angleUnit);
    }

    /**
     * Rotates the servo by a relative angle in degrees from its current position.
     *
     * @param degrees The angle to rotate by in degrees.
     */
    public void rotateByAngle(double degrees) {
        rotateByAngle(degrees, AngleUnit.DEGREES);
    }

    /**
     * Turns the servo to an absolute angle.
     *
     * @param angle     The target angle (clamped to the servo's configured range).
     * @param angleUnit The unit for the angle.
     */
    public void turnToAngle(double angle, AngleUnit angleUnit) {
        double angleRadians = Range.clip(toRadians(angle, angleUnit), minAngle, maxAngle);
        setPosition((angleRadians - minAngle) / (getAngleRange(AngleUnit.RADIANS)));
    }

    /**
     * Turns the servo to an absolute angle in degrees.
     *
     * @param degrees The target angle in degrees.
     */
    public void turnToAngle(double degrees) {
        turnToAngle(degrees, AngleUnit.DEGREES);
    }

    /**
     * Rotates the servo by a relative position value.
     *
     * @param position The position delta (added to current position).
     */
    public void rotateBy(double position) {
        position = getPosition() + position;
        setPosition(position);
    }

    /**
     * Sets the servo to an absolute position.
     *
     * <p>Automatically enables PWM if it was disabled.
     *
     * @param position The target position (0.0 to 1.0, clamped).
     */
    public void setPosition(double position) {
        if (!servo.isPwmEnabled()) {
            servo.setPwmEnable();
        }
        servo.setPosition(Range.clip(position, minPosition, maxPosition));
    }

    /**
     * Updates the servo's angular range mapping.
     *
     * @param min       The new minimum angle.
     * @param max       The new maximum angle.
     * @param angleUnit The unit for the angles.
     */
    public void setRange(double min, double max, AngleUnit angleUnit) {
        this.minAngle = toRadians(min, angleUnit);
        this.maxAngle = toRadians(max, angleUnit);
    }

    /**
     * Updates the servo's angular range mapping in degrees.
     *
     * @param min The new minimum angle in degrees.
     * @param max The new maximum angle in degrees.
     */
    public void setRange(double min, double max) {
        setRange(min, max, AngleUnit.DEGREES);
    }

    /** Sets whether the servo direction is inverted. */
    public void setInverted(boolean isInverted) {
        servo.setDirection(isInverted ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
    }

    /** Returns whether the servo direction is inverted. */
    public boolean getInverted() {
        return Servo.Direction.REVERSE == servo.getDirection();
    }

    /** Returns the current servo position (0.0 to 1.0). */
    public double getPosition() {
        return servo.getPosition();
    }

    /**
     * Returns the current servo angle.
     *
     * @param angleUnit The unit for the returned angle.
     * @return The current angle.
     */
    public double getAngle(AngleUnit angleUnit) {
        return getPosition() * getAngleRange(angleUnit) + fromRadians(minAngle, angleUnit);
    }

    /** Returns the current servo angle in degrees. */
    public double getAngle() {
        return getAngle(AngleUnit.DEGREES);
    }

    /**
     * Returns the servo's total angular range.
     *
     * @param angleUnit The unit for the returned range.
     * @return The angular range (maxAngle - minAngle).
     */
    public double getAngleRange(AngleUnit angleUnit) {
        return fromRadians(maxAngle - minAngle, angleUnit);
    }

    /** Returns the servo's total angular range in degrees. */
    public double getAngleRange() {
        return getAngleRange(AngleUnit.DEGREES);
    }

    /** Disables PWM output to the servo, allowing it to be moved freely. */
    public void disable() {
        servo.setPwmDisable();
    }

    /** Enables PWM output to the servo. */
    public void enable() {
        servo.setPwmEnable();
    }

    /** Returns a string describing the servo's port and controller. */
    public String getDeviceType() {
        String port = Integer.toString(servo.getPortNumber());
        String controller = servo.getController().toString();
        return "LimpServo: " + port + "; " + controller;
    }

    private double toRadians(double angle, AngleUnit angleUnit) {
        return angleUnit == AngleUnit.DEGREES ? Math.toRadians(angle) : angle;
    }

    private double fromRadians(double angle, AngleUnit angleUnit) {
        return angleUnit == AngleUnit.DEGREES ? Math.toDegrees(angle) : angle;
    }

}
