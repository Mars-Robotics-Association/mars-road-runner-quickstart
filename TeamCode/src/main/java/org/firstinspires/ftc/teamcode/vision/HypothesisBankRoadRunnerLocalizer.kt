package org.firstinspires.ftc.teamcode.vision

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.PoseVelocity2d
import edu.wpi.first.math.geometry.Rotation2d
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Localizer
import org.marsroboticsassociation.controllib.localization.vision.HypothesisBankLocalizer
import org.marsroboticsassociation.controllib.localization.vision.Transform3D
import org.marsroboticsassociation.controllib.localization.vision.VisionPoseSolver
import org.marsroboticsassociation.controllib.localization.vision.VisionPoseSolverConfig
import org.marsroboticsassociation.controllib.localization.vision.VisionSource
import java.util.function.LongSupplier

/**
 * Road Runner [Localizer] adapter around the library's pure [HypothesisBankLocalizer]. This is the
 * thin FTC/Road Runner glue: it owns the odometry [Localizer] (e.g. a Pinpoint), the vision
 * [VisionSource], the clock, and the FtcDashboard `@Config` tuning, and each loop it reads
 * odometry, converts Road Runner ↔ WPILib `Pose2d`/`PoseVelocity2d`, and delegates the estimation
 * to the library core.
 *
 * All the multi-hypothesis estimation, gating, and latency compensation lives in ControlLib and is
 * desktop-testable; nothing library-specific leaks into it. Swap this adapter for a Pedro Pathing
 * one (same core, different pose conversions) to use the localizer with another drive library.
 *
 * **Per-robot camera extrinsics** live in [Params] (Limelight `rs*` style). The **field tag map**
 * lives in [FieldTagMap]. Neither is shared across robots/seasons in MarsCommonFtc — edit them in
 * this quickstart for your robot and game map.
 */
class HypothesisBankRoadRunnerLocalizer : Localizer {

    /** FtcDashboard-tunable configuration (live for core/tag; extrinsics applied at construction). */
    @Config
    class Params {
        /** The pure library core's tuning (bank + commit + latency + blur gate). */
        @JvmField
        var core: HypothesisBankLocalizer.Params = HypothesisBankLocalizer.Params()

        /** AprilTag black-square side length (metres) for the ambiguity PnP re-solve. */
        @JvmField
        var tagSizeMeters: Double = 0.1651

        /**
         * Minimum tag image area (percent of frame) for a tag to qualify as the ambiguity anchor; 0
         * disables the area filter (any tag qualifies).
         */
        @JvmField
        var ambiguityMinTagAreaPct: Double = 0.0

        // --- camera extrinsic (Limelight .vpr "camera pose in robot space", per-robot) ----------
        // Match this robot's Limelight config (or a hand-eye solve). Units: metres / degrees.
        // Defaults below are one known good mount (Curiosity DECODE); replace for every other robot.

        /** Camera forward of robot origin (m). Limelight `rsforward`. */
        @JvmField
        var cameraForwardM: Double = 0.0816

        /** Camera left of robot origin (m); right is negative. Limelight `rsside`. */
        @JvmField
        var cameraSideM: Double = -0.0400

        /** Camera height above floor / robot origin (m). Limelight `rsup`. */
        @JvmField
        var cameraUpM: Double = 0.3835

        /** Camera roll (deg). 180 = mounted upside-down. Limelight `rsroll`. */
        @JvmField
        var cameraRollDeg: Double = 180.0

        /** Camera pitch (deg); positive = nose up. Limelight `rspitch`. */
        @JvmField
        var cameraPitchDeg: Double = 0.23

        /** Camera yaw (deg); positive = left. Limelight `rsyaw`. */
        @JvmField
        var cameraYawDeg: Double = -0.78
    }

    private val odometry: Localizer
    private val source: VisionSource
    private val telemetry: Telemetry?
    private val clock: LongSupplier
    private val core: HypothesisBankLocalizer

    /**
     * Constructs the adapter with a solver built from [PARAMS] camera extrinsics and the
     * [System.nanoTime] clock.
     *
     * @param odometry the odometry localizer (the rigid backbone; its raw pose need not be
     *     field-aligned — the bank maps it to the field)
     * @param source the vision-acquisition source (e.g. [LimelightVisionSource])
     * @param startPose the starting field pose to seed (Road Runner)
     * @param telemetry for status logging; may be null
     */
    constructor(
        odometry: Localizer,
        source: VisionSource,
        startPose: com.acmerobotics.roadrunner.Pose2d,
        telemetry: Telemetry?,
    ) : this(odometry, source, solverFromParams(PARAMS), startPose, telemetry, LongSupplier { System.nanoTime() })

    constructor(
        odometry: Localizer,
        source: VisionSource,
        solver: VisionPoseSolver,
        startPose: com.acmerobotics.roadrunner.Pose2d?,
        telemetry: Telemetry?,
        clock: LongSupplier,
    ) {
        this.odometry = odometry
        this.source = source
        this.telemetry = telemetry
        this.clock = clock
        this.core = HypothesisBankLocalizer(PARAMS.core, solver)
        if (startPose != null) {
            core.setPose(toWpi(startPose), toWpi(odometry.getPose()))
        }
    }

    override fun setPose(pose: com.acmerobotics.roadrunner.Pose2d) {
        core.setPose(toWpi(pose), toWpi(odometry.getPose()))
    }

    override fun getPose(): com.acmerobotics.roadrunner.Pose2d {
        return toRr(core.getPose(toWpi(odometry.getPose())))
    }

    override fun update(): PoseVelocity2d {
        val vel = odometry.update()
        val odoRr = odometry.getPose()

        // Keep the vision source's robot-orientation seed current (cheap; MT1 doesn't need it, but
        // it keeps MT2 usable if ever reintroduced as a check).
        source.updateRobotOrientation(Math.toDegrees(getPose().heading.toDouble()))

        core.update(clock.asLong, toWpi(odoRr), vel.angVel, source.latest())

        if (telemetry != null) {
            telemetry.addData("bank_committed", core.isCommitted)
            telemetry.addData("bank_domWeight", String.format("%.2f", core.dominantWeight()))
            telemetry.addData("bank_size", core.bank().size())
        }
        return vel
    }

    /** Whether the bank has committed (gate auto-aim / trusted-pose cues on this). */
    fun isCommitted(): Boolean = core.isCommitted

    /** Commitment level: dominant-hypothesis weight in `(0,1]`. */
    fun dominantWeight(): Double = core.dominantWeight()

    /**
     * The raw (uncorrected) odometry pose this loop, Road Runner frame — for logging/diagnostics.
     */
    fun getOdometryPose(): com.acmerobotics.roadrunner.Pose2d = odometry.getPose()

    /** The pure library core, for diagnostic access (see [BankLocalizerCsvLogger]). */
    fun core(): HypothesisBankLocalizer = core

    /**
     * The vision source, for its calibration (intrinsics/distortion) when logging the replay row.
     */
    fun visionSource(): VisionSource = source

    companion object {
        @JvmField
        var PARAMS: Params = Params()

        /**
         * Builds a [VisionPoseSolver] from this robot's [Params] camera extrinsics and this
         * quickstart's [FieldTagMap].
         */
        @JvmStatic
        fun solverFromParams(p: Params): VisionPoseSolver {
            val robotFromCamera: Transform3D =
                VisionPoseSolverConfig.robotFromCameraFromLimelightRs(
                    p.cameraForwardM,
                    p.cameraSideM,
                    p.cameraUpM,
                    p.cameraRollDeg,
                    p.cameraPitchDeg,
                    p.cameraYawDeg,
                )
            return VisionPoseSolverConfig(robotFromCamera, FieldTagMap.decode2025GoalTags()).solver()
        }

        private fun toWpi(p: com.acmerobotics.roadrunner.Pose2d): edu.wpi.first.math.geometry.Pose2d {
            return edu.wpi.first.math.geometry.Pose2d(
                p.position.x,
                p.position.y,
                Rotation2d(p.heading.toDouble()),
            )
        }

        private fun toRr(p: edu.wpi.first.math.geometry.Pose2d): com.acmerobotics.roadrunner.Pose2d {
            return com.acmerobotics.roadrunner.Pose2d(
                p.x,
                p.y,
                p.rotation.radians,
            )
        }
    }
}
