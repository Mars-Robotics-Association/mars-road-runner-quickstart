package org.firstinspires.ftc.teamcode.vision;

import org.marsroboticsassociation.controllib.localization.vision.Transform3D;
import org.marsroboticsassociation.controllib.localization.vision.VisionPoseSolverConfig;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

/**
 * Season / field AprilTag poses for the vision localizer ({@code field←tag}, metres, Limelight
 * field frame).
 *
 * <p>This is <b>quickstart-owned geometry</b> — not shared across robots or seasons in
 * MarsCommonFtc. Replace the table when the game map changes (new {@code .fmap}) or when you need
 * a different subset of tags. Values below are the DECODE goal tags from {@code
 * ftc2025DECODE.fmap}: each entry is a 4×4 row-major rigid transform (2×2 rotation is pure yaw of
 * ±54°; z is tag height 0.7493 m).
 */
public final class FieldTagMap {

    // ftc2025DECODE.fmap : field←tag, 4×4 row-major, metres
    private static final double[] TAG20_FIELD_FROM_TAG = {
        0.5877852522924731,
        -0.8090169943749473,
        0,
        -1.4827,
        0.8090169943749473,
        0.5877852522924731,
        0,
        -1.4133,
        0,
        0,
        1,
        0.7493,
        0,
        0,
        0,
        1
    };
    private static final double[] TAG24_FIELD_FROM_TAG = {
        0.5877852522924731,
        0.8090169943749473,
        0,
        -1.4827,
        -0.8090169943749473,
        0.5877852522924731,
        0,
        1.4133,
        0,
        0,
        1,
        0.7493,
        0,
        0,
        0,
        1
    };

    private static final Map<Integer, Transform3D> DECODE_2025_GOAL_TAGS;

    static {
        Map<Integer, Transform3D> m = new HashMap<>();
        m.put(20, VisionPoseSolverConfig.fromRowMajor4x4(TAG20_FIELD_FROM_TAG));
        m.put(24, VisionPoseSolverConfig.fromRowMajor4x4(TAG24_FIELD_FROM_TAG));
        DECODE_2025_GOAL_TAGS = Collections.unmodifiableMap(m);
    }

    private FieldTagMap() {}

    /**
     * DECODE goal tags 20 and 24 ({@code ftc2025DECODE.fmap}). Unmodifiable; safe to pass into
     * {@link VisionPoseSolverConfig} as-is.
     */
    public static Map<Integer, Transform3D> decode2025GoalTags() {
        return DECODE_2025_GOAL_TAGS;
    }
}
