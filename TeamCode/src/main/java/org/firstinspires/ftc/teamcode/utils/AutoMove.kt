package org.firstinspires.ftc.teamcode.utils

import com.acmerobotics.roadrunner.Twist2d
import com.acmerobotics.roadrunner.Vector2d

/**
 * This is a convenience class that exists to make it easier to modify poses in FTC dashboard because the fields are public and mutable
 */
class AutoMove(
    @JvmField var dx: Double,
    @JvmField var dy: Double,
    @JvmField var dTheta: Double,
) {
    fun getTwist(): Twist2d {
        return Twist2d(Vector2d(dx, dy), Math.toRadians(dTheta))
    }

    /**
     * Returns a new AutoMove mirrored across the X-axis (negates dy and dTheta).
     *
     * <p>This is useful for creating blue alliance movements from red alliance movements.
     *
     * @return A new AutoMove with negated dy and dTheta values.
     */
    fun mirrored(): AutoMove {
        return AutoMove(dx, -dy, -dTheta)
    }
}
