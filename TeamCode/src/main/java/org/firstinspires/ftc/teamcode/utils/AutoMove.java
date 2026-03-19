package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;

/**
 * This is a convenience class that exists to make it easier to modify poses in FTC dashboard because the fields are public and mutable
 */
public class AutoMove {
    public double dx;
    public double dy;
    public double dTheta;


    public AutoMove(double dx, double dy, double dTheta) {
        this.dx = dx;
        this.dy = dy;
        this.dTheta = dTheta;
    }

    public Twist2d getTwist() {
        return new Twist2d(new Vector2d(dx, dy), Math.toRadians(dTheta));
    }

    /**
     * Returns a new AutoMove mirrored across the X-axis (negates dy and dTheta).
     *
     * <p>This is useful for creating blue alliance movements from red alliance movements.
     *
     * @return A new AutoMove with negated dy and dTheta values.
     */
    public AutoMove mirrored() {
        return new AutoMove(dx, -dy, -dTheta);
    }
}
