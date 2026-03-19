package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;

/**
 * This is a convenience class that exists to make it easier to modify poses in FTC dashboard because the fields are public and mutable
 */
public class AutoPose {
    public double x;
    public double y;
    public double hd;


    public AutoPose(double x, double y, double hd) {
        this.x = x;
        this.y = y;
        this.hd = hd;
    }

    public AutoPose(Pose2d p) {
        this.x = p.position.x;
        this.y = p.position.y;
        this.hd = Math.toDegrees(p.heading.toDouble());
    }

    /**
     * Creates a new AutoPose by applying a relative movement (twist) to the current pose.
     * <p>
     * The movement is calculated relative to the current position and heading.
     * This uses RoadRunner's {@link Twist2d} to ensure the resulting pose correctly
     * accounts for simultaneous translation and rotation.
     *
     * @return A new {@code AutoPose} representing the position after the movement.
     */
    public AutoPose moveBy(AutoMove move) {
        return new AutoPose(this.getPose().plus(move.getTwist()));
    }

    public Vector2d getPosition() {
        return new Vector2d(x, y);
    }

    public double getHeadingRadians() {
        return Math.toRadians(hd);
    }

    public Pose2d getPose() {
        return new Pose2d(getPosition(), getHeadingRadians());
    }

    /**
     * Returns a new AutoPose mirrored across the X-axis (negates Y and heading).
     *
     * <p>This is useful for creating blue alliance poses from red alliance poses,
     * since the FTC field is symmetric across the X-axis.
     *
     * @return A new AutoPose with negated y and heading values.
     */
    public AutoPose mirrored() {
        return new AutoPose(x, -y, -hd);
    }
}
