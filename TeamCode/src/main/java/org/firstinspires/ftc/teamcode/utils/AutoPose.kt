package org.firstinspires.ftc.teamcode.utils

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d

/**
 * This is a convenience class that exists to make it easier to modify poses in FTC dashboard
 * because the fields are public and mutable
 */
open class AutoPose(
    @JvmField var x: Double,
    @JvmField var y: Double,
    @JvmField var hd: Double,
) {
    constructor(
        p: Pose2d
    ) : this(
        p.position.x,
        p.position.y,
        Math.toDegrees(p.heading.toDouble()),
    )

    /**
     * Creates a new AutoPose by applying a relative movement (twist) to the current pose.
     *
     * <p>
     * The movement is calculated relative to the current position and heading. This uses
     * RoadRunner's [Twist2d][com.acmerobotics.roadrunner.Twist2d] to ensure the resulting pose
     * correctly accounts for simultaneous translation and rotation.
     *
     * @return A new `AutoPose` representing the position after the movement.
     */
    fun moveBy(move: AutoMove): AutoPose {
        return AutoPose(this.getPose().plus(move.getTwist()))
    }

    fun getPosition(): Vector2d {
        return Vector2d(x, y)
    }

    fun getHeadingRadians(): Double {
        return Math.toRadians(hd)
    }

    fun getPose(): Pose2d {
        return Pose2d(getPosition(), getHeadingRadians())
    }

    /**
     * Returns a new AutoPose mirrored across the X-axis (negates Y and heading).
     *
     * <p>This is useful for creating blue alliance poses from red alliance poses, since the FTC
     * field is symmetric across the X-axis.
     *
     * @return A new AutoPose with negated y and heading values.
     */
    fun mirrored(): AutoPose {
        return AutoPose(x, -y, -hd)
    }
}
