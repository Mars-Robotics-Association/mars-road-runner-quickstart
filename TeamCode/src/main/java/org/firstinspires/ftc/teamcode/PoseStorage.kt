package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.Pose2d

/**
 * Static storage for passing the robot's pose from autonomous to teleop.
 *
 * <p>The auto opmode writes its final pose here before ending, and the teleop opmode reads it on
 * init to start with an accurate position and immediately enable vision fusion.
 */
class PoseStorage {
    companion object {
        @JvmField var lastAutoPose: Pose2d? = null
    }
}
