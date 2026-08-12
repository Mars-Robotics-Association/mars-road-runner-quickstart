package org.firstinspires.ftc.teamcode.vision

import org.marsroboticsassociation.controllib.localization.vision.Transform3D
import org.marsroboticsassociation.controllib.localization.vision.VisionPoseSolverConfig
import java.util.Collections
import java.util.HashMap

/**
 * Season / field AprilTag poses for the vision localizer (`field←tag`, metres, Limelight field
 * frame).
 *
 * This is **quickstart-owned geometry** — not shared across robots or seasons in MarsCommonFtc.
 * Replace the table when the game map changes (new `.fmap`) or when you need a different subset of
 * tags. Values below are the DECODE goal tags from `ftc2025DECODE.fmap`: each entry is a 4×4
 * row-major rigid transform (2×2 rotation is pure yaw of ±54°; z is tag height 0.7493 m).
 */
class FieldTagMap private constructor() {

    companion object {
        // ftc2025DECODE.fmap : field←tag, 4×4 row-major, metres
        private val TAG20_FIELD_FROM_TAG =
            doubleArrayOf(
                0.5877852522924731,
                -0.8090169943749473,
                0.0,
                -1.4827,
                0.8090169943749473,
                0.5877852522924731,
                0.0,
                -1.4133,
                0.0,
                0.0,
                1.0,
                0.7493,
                0.0,
                0.0,
                0.0,
                1.0,
            )
        private val TAG24_FIELD_FROM_TAG =
            doubleArrayOf(
                0.5877852522924731,
                0.8090169943749473,
                0.0,
                -1.4827,
                -0.8090169943749473,
                0.5877852522924731,
                0.0,
                1.4133,
                0.0,
                0.0,
                1.0,
                0.7493,
                0.0,
                0.0,
                0.0,
                1.0,
            )

        private val DECODE_2025_GOAL_TAGS: Map<Int, Transform3D> =
            Collections.unmodifiableMap(
                HashMap<Int, Transform3D>().apply {
                    put(20, VisionPoseSolverConfig.fromRowMajor4x4(TAG20_FIELD_FROM_TAG))
                    put(24, VisionPoseSolverConfig.fromRowMajor4x4(TAG24_FIELD_FROM_TAG))
                }
            )

        /**
         * DECODE goal tags 20 and 24 (`ftc2025DECODE.fmap`). Unmodifiable; safe to pass into
         * [VisionPoseSolverConfig] as-is.
         */
        @JvmStatic
        fun decode2025GoalTags(): Map<Int, Transform3D> = DECODE_2025_GOAL_TAGS
    }
}
