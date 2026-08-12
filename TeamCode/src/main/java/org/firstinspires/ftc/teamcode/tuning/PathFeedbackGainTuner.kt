package org.firstinspires.ftc.teamcode.tuning

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.HolonomicController
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Pose2dDual
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.PoseVelocity2dDual
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.Time
import com.acmerobotics.roadrunner.TimeTrajectory
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.MecanumDrive
import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer
import org.firstinspires.ftc.teamcode.TwoDeadWheelLocalizer
import org.firstinspires.ftc.teamcode.opmodes.base.MarsLinearOpMode
import org.firstinspires.ftc.teamcode.utils.CsvLogger
import org.firstinspires.ftc.teamcode.utils.GatedTelemetry
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import kotlin.math.abs
import kotlin.math.ceil
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt

/**
 * ManualFeedback-style automatic gain search: long line paths, discrete position-gain ladder, back
 * off when chatter appears.
 *
 * Does what you do by eye with [ManualFeedbackTuner], without short profiled bumps that happily
 * climb to unusable gains of 10–20:
 *
 * 1. **Axial** — [DISTANCE] in forward/back along field X (heading 0) with a modest heading hold.
 *    Ladder `axialGain`. Targets are absolute field anchors from the start pose so overshoot cannot
 *    walk the out-and-back into the near wall.
 * 2. **Heading** — same path, axial locked at its best. Ladder `headingGain`.
 * 3. **Lateral** — return near field home, turn to [LATERAL_HEADING_RAD] (default 90°), then
 *    out-and-back on the same field-X strip via `strafeToConstantHeading` (not `lineToX` — RR lineToX
 *    follows path tangent, so at 90° it drives field ±Y = robot reverse). Ladder `lateralGain`.
 *
 * Reusing the field-X corridor after a 90° turn avoids needing a second long strip of floor:
 * strafe-to keeps heading while the position target moves along field X.
 *
 * Velocity gains stay at [VEL_GAIN] (default 0). Chatter is scored on the command channel of the
 * axis under test only.
 *
 * Prerequisites: localization and feedforward tuned (including lateral FF if anisotropic). Clear a
 * straight corridor of about `DISTANCE + 12` inches and room to spin in place at the start.
 * Production feedforward path via [MecanumDrive.setDriveCommand].
 *
 * On finish, writes gains into live `MecanumDrive.PARAMS`, then drives back to the OpMode start pose
 * and heading so the next run can begin without re-staging. Optional CSVs: `path_fbgain_samples_*` /
 * `path_fbgain_summary_*`. Pull with `telemetry/pull.sh`.
 */
@Config
class PathFeedbackGainTuner : MarsLinearOpMode() {
    private enum class Axis {
        AXIAL,
        LATERAL,
        HEADING,
    }

    /** Path orientation for out-and-back cycles. */
    private enum class PathMode {
        /** Heading ~0, lineToX — robot axial along field X. */
        AXIAL_LINE,

        /**
         * Heading ~[LATERAL_HEADING_RAD], strafeToConstantHeading along field X — robot lateral on
         * the same corridor.
         */
        LATERAL_LINE,
    }

    private class AxisResult(
        @JvmField val ok: Boolean,
        @JvmField val posGain: Double,
        @JvmField val note: String,
    )

    /** Metrics accumulated over one or more trajectory legs at a fixed gain. */
    private class PathMetrics(
        @JvmField val axisUnderTest: Axis,
    ) {
        @JvmField
        var samples = 0

        @JvmField
        var durationS = 0.0

        /** Time spent in the low-ref-speed chatter window (for flip rates / diagnostics). */
        @JvmField
        var chatterWindowS = 0.0

        @JvmField
        var axialCmdFlips = 0

        @JvmField
        var lateralCmdFlips = 0

        @JvmField
        var headingCmdFlips = 0

        private var lastAxSign = 0
        private var lastLatSign = 0
        private var lastHSign = 0

        // Diagnostic only (not used for reject): second-diff of command during chatter window.
        private var axialJerkSum = 0.0
        private var lateralJerkSum = 0.0
        private var headingJerkSum = 0.0
        private var jerkSamples = 0
        private var errXSqSum = 0.0
        private var errYSqSum = 0.0
        private var errHSqSum = 0.0
        private var prevCmdX = Double.NaN
        private var prevCmdY = Double.NaN
        private var prevCmdH = Double.NaN
        private var prevDCmdX = Double.NaN
        private var prevDCmdY = Double.NaN
        private var prevDCmdH = Double.NaN

        /** Mid-path stop-then-go events on the axis under test (high gain fighting the profile). */
        @JvmField
        var hesitations = 0

        /** 0 = cruising, 1 = saw near-stop mid-path (waiting for re-accel). */
        private var hesitationState = 0

        /**
         * Always counts sample time / tracking RMS and late-path hesitations. Command sign flips
         * only when `scoreChatter` — low reference speed (see [CHATTER_REF_SPEED]).
         *
         * @param remainingAlongPath distance still short of the traj end along the path (inches for
         *     translation axes, rad for heading). Near-stops past the target (negative remaining) do
         *     not arm hesitation.
         */
        fun observe(
            dt: Double,
            errX: Double,
            errY: Double,
            errH: Double,
            cmdX: Double,
            cmdY: Double,
            cmdH: Double,
            axisVel: Double,
            progress01: Double,
            remainingAlongPath: Double,
            scoreChatter: Boolean,
        ) {
            samples++
            durationS += max(dt, 0.0)
            errXSqSum += errX * errX
            errYSqSum += errY * errY
            errHSqSum += errH * errH

            // Late-path stop-short-then-lunge (independent of low-speed chatter window).
            if (progress01 >= HESITATION_PROGRESS_MIN && progress01 <= HESITATION_PROGRESS_MAX) {
                val vSlow =
                    if (axisUnderTest == Axis.HEADING) HESITATION_V_SLOW_HEADING else HESITATION_V_SLOW
                val vFast =
                    if (axisUnderTest == Axis.HEADING) HESITATION_V_FAST_HEADING else HESITATION_V_FAST
                val remMin =
                    if (axisUnderTest == Axis.HEADING) {
                        HESITATION_REMAINING_MIN_HEADING
                    } else {
                        HESITATION_REMAINING_MIN
                    }
                val av = abs(axisVel)
                if (hesitationState == 0 && av < vSlow && remainingAlongPath >= remMin) {
                    hesitationState = 1
                } else if (hesitationState == 1 && av > vFast) {
                    hesitations++
                    hesitationState = 0
                }
            } else {
                hesitationState = 0
            }

            if (!scoreChatter) {
                // Break derivative chain across cruise → low-speed so planned decel is not thrash.
                prevCmdX = Double.NaN
                prevCmdY = Double.NaN
                prevCmdH = Double.NaN
                prevDCmdX = Double.NaN
                prevDCmdY = Double.NaN
                prevDCmdH = Double.NaN
                lastAxSign = 0
                lastLatSign = 0
                lastHSign = 0
                return
            }

            chatterWindowS += max(dt, 0.0)

            val axSign = signWithDeadband(cmdX, TRANS_CMD_FLIP_DEADBAND)
            if (axSign != 0) {
                if (lastAxSign != 0 && axSign != lastAxSign) axialCmdFlips++
                lastAxSign = axSign
            }
            val latSign = signWithDeadband(cmdY, TRANS_CMD_FLIP_DEADBAND)
            if (latSign != 0) {
                if (lastLatSign != 0 && latSign != lastLatSign) lateralCmdFlips++
                lastLatSign = latSign
            }
            val hSign = signWithDeadband(cmdH, HEADING_CMD_FLIP_DEADBAND)
            if (hSign != 0) {
                if (lastHSign != 0 && hSign != lastHSign) headingCmdFlips++
                lastHSign = hSign
            }

            // Diagnostic jerk only (not used for reject).
            if (!prevCmdX.isNaN() && dt > 1e-4) {
                val dX = (cmdX - prevCmdX) / dt
                val dY = (cmdY - prevCmdY) / dt
                val dH = (cmdH - prevCmdH) / dt
                if (!prevDCmdX.isNaN()) {
                    axialJerkSum += abs((dX - prevDCmdX) / dt)
                    lateralJerkSum += abs((dY - prevDCmdY) / dt)
                    headingJerkSum += abs((dH - prevDCmdH) / dt)
                    jerkSamples++
                }
                prevDCmdX = dX
                prevDCmdY = dY
                prevDCmdH = dH
            }
            prevCmdX = cmdX
            prevCmdY = cmdY
            prevCmdH = cmdH
        }

        fun axialJerk(): Double = if (jerkSamples == 0) 0.0 else axialJerkSum / jerkSamples

        fun lateralJerk(): Double = if (jerkSamples == 0) 0.0 else lateralJerkSum / jerkSamples

        fun headingJerk(): Double = if (jerkSamples == 0) 0.0 else headingJerkSum / jerkSamples

        fun axialFlipsPerSec(): Double =
            if (chatterWindowS < 1e-3) 0.0 else axialCmdFlips / chatterWindowS

        fun lateralFlipsPerSec(): Double =
            if (chatterWindowS < 1e-3) 0.0 else lateralCmdFlips / chatterWindowS

        fun headingFlipsPerSec(): Double =
            if (chatterWindowS < 1e-3) 0.0 else headingCmdFlips / chatterWindowS

        fun errXRms(): Double = if (samples == 0) 0.0 else sqrt(errXSqSum / samples)

        fun errYRms(): Double = if (samples == 0) 0.0 else sqrt(errYSqSum / samples)

        fun errHRms(): Double = if (samples == 0) 0.0 else sqrt(errHSqSum / samples)

        fun activeFlips(axis: Axis): Int {
            return when (axis) {
                Axis.AXIAL -> axialCmdFlips
                Axis.LATERAL -> lateralCmdFlips
                Axis.HEADING -> headingCmdFlips
            }
        }

        fun maxFlips(axis: Axis): Int {
            return when (axis) {
                Axis.AXIAL -> MAX_AXIAL_CMD_FLIPS
                Axis.LATERAL -> MAX_LATERAL_CMD_FLIPS
                Axis.HEADING -> MAX_HEADING_CMD_FLIPS
            }
        }

        /**
         * Hard reject: too many low-speed command flips, or repeated mid-path stop-then-go
         * (high-gain profile fighting).
         */
        fun chatter(axis: Axis): Boolean {
            return activeFlips(axis) > maxFlips(axis) || hesitations >= MAX_HESITATIONS_REJECT
        }

        fun edge(axis: Axis): Boolean {
            return activeFlips(axis) >= ceil(EDGE_FRAC * maxFlips(axis)).toInt() ||
                hesitations >= MAX_HESITATIONS_EDGE
        }

        companion object {
            private fun signWithDeadband(v: Double, db: Double): Int {
                return if (v > db) 1 else if (v < -db) -1 else 0
            }
        }
    }

    companion object {
        /** When false, skip writing CSVs. */
        @JvmField
        var LOG_CSV = true

        @JvmField
        var LOG_FLUSH_EVERY = 256

        /** Out-and-back distance, inches — same default as [ManualFeedbackTuner.DISTANCE]. */
        @JvmField
        var DISTANCE = 64.0

        /** Complete out-and-back cycles at each gain step before deciding clean vs chatter. */
        @JvmField
        var CYCLES = 2

        /** First position gain tried for the axis under test. */
        @JvmField
        var START_GAIN = 2.0

        /**
         * Position gain ceiling (inclusive). Field experience on this chassis: axial ~4–5 is the
         * quiet sweet spot before stop-then-lunge; audible thrash starts around ~10. Ceiling is
         * above that so the ladder can find the edge and back off.
         */
        @JvmField
        var MAX_GAIN = 12.0

        /** Position gain increment after a clean step. */
        @JvmField
        var GAIN_STEP = 1.0

        /**
         * Heading position gain while the axial ladder runs. Soft enough not to thrash, strong
         * enough to keep the line from curling. 2 was too weak on this chassis (line wanders); 4
         * matches the production default. Replaced by the heading-phase result afterward.
         */
        @JvmField
        var HOLD_HEADING_GAIN = 4.0

        /**
         * First heading gain tried in the heading phase. Higher than [START_GAIN]: 2 felt limp for
         * yaw hold on long paths, and a false soft-edge at 2 used to strand the ladder there.
         */
        @JvmField
        var HEADING_START_GAIN = 3.0

        /**
         * When true, after axial+heading: home, turn to [LATERAL_HEADING_RAD], and ladder
         * lateralGain on the same field-X corridor (constant heading = pure strafe).
         */
        @JvmField
        var TUNE_LATERAL = true

        /**
         * Field heading (rad) for the lateral phase. Default `π/2`: robot faces +Y while
         * lineToXConstantHeading still travels field X → robot-frame lateral motion on the same
         * strip.
         */
        @JvmField
        var LATERAL_HEADING_RAD = Math.PI / 2.0

        /** Velocity gains for all axes (default 0 — D amplifies Pinpoint noise on long paths). */
        @JvmField
        var VEL_GAIN = 0.0

        /**
         * Absolute axial command sign flips (low-speed window only) over a full gain step before
         * hard reject. Quiet gains (2–8) should stay near 0 flips; real thrash near ~10 produces
         * many. Raw command "jerk" is logged only — profile decel makes it huge even when quiet.
         */
        @JvmField
        var MAX_AXIAL_CMD_FLIPS = 16

        @JvmField
        var MAX_LATERAL_CMD_FLIPS = 18

        @JvmField
        var MAX_HEADING_CMD_FLIPS = 16

        /**
         * Soft edge: flip count at or above this fraction of the hard max stops the ladder (even
         * without a hard reject). The edge gain itself is *not* kept — we back off to the last fully
         * clean step so companion phases (heading/lateral) are not poisoned by a stop-then-lunge
         * axial. Kept high so mild noise at 3–5 does not freeze the ladder early.
         */
        @JvmField
        var EDGE_FRAC = 0.75

        /** |cmd| below this does not count toward axial/lateral command sign flips (in/s). */
        @JvmField
        var TRANS_CMD_FLIP_DEADBAND = 3.0

        /** |cmd| below this does not count toward heading command sign flips (rad/s). */
        @JvmField
        var HEADING_CMD_FLIP_DEADBAND = 0.25

        /**
         * Only score command chatter while the trajectory reference speed is below this (in/s).
         * Cruise and planned accel/decel are ignored; thrash shows up when the profile has nearly
         * stopped.
         */
        @JvmField
        var CHATTER_REF_SPEED = 6.0

        /** Same idea for reference angular rate (rad/s) when scoring heading chatter. */
        @JvmField
        var CHATTER_REF_OMEGA = 0.35

        /**
         * Terminal hesitation: while still short of the path end, axis speed falls below
         * [HESITATION_V_SLOW] then rises above [HESITATION_V_FAST] again (progress in
         * [[HESITATION_PROGRESS_MIN], [HESITATION_PROGRESS_MAX]]). That is the high-gain "brake,
         * stop short, then lunge" before reverse (axial ≥6 on this chassis; by-eye sweet spot ~4–5).
         *
         * Arming only when remaining path distance ≥ [HESITATION_REMAINING_MIN] avoids counting
         * low-gain overshoot recovery (past the target) as hesitation. Soft-edge backs off to the
         * last fully clean gain; hard-reject at this many events.
         */
        @JvmField
        var MAX_HESITATIONS_REJECT = 3

        /**
         * Soft-edge when hesitation count reaches this. Ladder stops and keeps the *previous* fully
         * clean gain (not this edge step). 2 allows a single mild re-lunge at the sweet spot (~5)
         * while still rejecting clear stop-then-go at ~6+.
         */
        @JvmField
        var MAX_HESITATIONS_EDGE = 2

        /** Fraction of trajectory duration treated as late-path for hesitation scoring. */
        @JvmField
        var HESITATION_PROGRESS_MIN = 0.55

        /**
         * Include late terminal re-lunge (stop short then catch residual error before reverse). Was
         * 0.90, which missed re-accels that only peak after ~92% of the leg.
         */
        @JvmField
        var HESITATION_PROGRESS_MAX = 0.95

        /** |axis velocity| below this (in/s or rad/s) counts as a near-stop for hesitation. */
        @JvmField
        var HESITATION_V_SLOW = 5.0

        /**
         * |axis velocity| above this after a near-stop counts as a re-accel hesitation. 12 was too
         * high — mild stop-then-go at axial ~6 only re-accelerates to ~8–10 in/s and looked clean to
         * the counter while still feeling wrong by eye.
         */
        @JvmField
        var HESITATION_V_FAST = 8.0

        /**
         * Only arm a hesitation near-stop when this much path distance remains to the traj endpoint
         * (inches). Positive remaining = still short of the end along the path direction. Filters
         * out low-gain overshoot recovery past the target.
         */
        @JvmField
        var HESITATION_REMAINING_MIN = 1.5

        /** Heading uses smaller velocity thresholds (rad/s). */
        @JvmField
        var HESITATION_V_SLOW_HEADING = 0.15

        @JvmField
        var HESITATION_V_FAST_HEADING = 0.35

        /** Heading remaining (rad) required to arm a heading hesitation near-stop. */
        @JvmField
        var HESITATION_REMAINING_MIN_HEADING = 0.05

        /**
         * How often to refresh DS / Dashboard status during long `nextSample()` path loops (ms).
         * Those loops close gated telemetry by default, which freezes the last pre-step line for the
         * whole out-and-back (~10 s per gain) unless we open and flush periodically.
         */
        @JvmField
        var STATUS_MS = 250

        private const val SAMPLE_HEADER =
            "t_s,axis,gain,cycle,leg,pose_x,pose_y,pose_h,ref_x,ref_y,ref_h," +
                "err_x,err_y,err_h,vel_x,vel_y,vel_h,cmd_vx,cmd_vy,cmd_w," +
                "ax_flips,lat_flips,h_flips"

        private const val SUMMARY_HEADER =
            "axis,gain,cycles,duration_s,ax_jerk,lat_jerk,h_jerk," +
                "ax_flips_per_s,lat_flips_per_s,h_flips_per_s," +
                "err_x_rms,err_y_rms,err_h_rms,hesitations,chatter,edge,verdict"

        private fun skipped(note: String): AxisResult {
            return AxisResult(false, 0.0, note)
        }

        /**
         * When the highest clean gain still showed mild hesitation, prefer the last fully quiet
         * (zero-hes) clean step. Matches by-eye "7 looked high — back off to 6" after a hard reject
         * at 8: last clean 7 had hes=1 while 6 was clean with hes=0.
         */
        private fun quieterBest(bestGain: Double, bestHes: Int, bestZeroHesGain: Double): Double {
            if (bestGain.isNaN()) {
                return bestGain
            }
            if (bestHes > 0 && !bestZeroHesGain.isNaN() && bestZeroHesGain + 1e-9 < bestGain) {
                return bestZeroHesGain
            }
            return bestGain
        }

        /**
         * Writes PARAMS gains for this step. The axis under test gets `gain`; NaN companion args
         * mean "this is the axis under test" and are ignored for that slot.
         */
        private fun applyGains(
            axis: Axis,
            gain: Double,
            axialGainOrNaN: Double,
            lateralGainOrNaN: Double,
            headingGainOrNaN: Double,
        ) {
            MecanumDrive.PARAMS.axialGain = if (axis == Axis.AXIAL) gain else axialGainOrNaN
            MecanumDrive.PARAMS.lateralGain = if (axis == Axis.LATERAL) gain else lateralGainOrNaN
            MecanumDrive.PARAMS.headingGain = if (axis == Axis.HEADING) gain else headingGainOrNaN
        }

        private fun flatten(action: Action): List<Action> {
            val out = ArrayList<Action>()
            flattenInto(action, out)
            return out
        }

        private fun flattenInto(action: Action, out: MutableList<Action>) {
            if (action is SequentialAction) {
                for (child in action.initialActions) {
                    flattenInto(child, out)
                }
            } else {
                out.add(action)
            }
        }

        private fun requirePositive(kV: Double, kA: Double) {
            if (kV <= 0 || kA <= 0) {
                throw RuntimeException(
                    "PathFeedbackGainTuner needs positive kV and kA. Run AxialFeedforwardTuner" +
                        " first.",
                )
            }
        }

        private fun checkDeadWheels(localizer: Any) {
            if (localizer is TwoDeadWheelLocalizer) {
                if (TwoDeadWheelLocalizer.PARAMS.perpXTicks == 0.0 &&
                    TwoDeadWheelLocalizer.PARAMS.parYTicks == 0.0
                ) {
                    throw RuntimeException(
                        "Odometry wheel locations not set! Run AngularRampLogger to tune them.",
                    )
                }
            } else if (localizer is ThreeDeadWheelLocalizer) {
                if (ThreeDeadWheelLocalizer.PARAMS.perpXTicks == 0.0 &&
                    ThreeDeadWheelLocalizer.PARAMS.par0YTicks == 0.0 &&
                    ThreeDeadWheelLocalizer.PARAMS.par1YTicks == 1.0
                ) {
                    throw RuntimeException(
                        "Odometry wheel locations not set! Run AngularRampLogger to tune them.",
                    )
                }
            }
        }
    }

    private var telem: Telemetry? = null
    private var sampleLog: CsvLogger? = null
    private var summaryLog: CsvLogger? = null
    private var logAxis = ""
    private var logGain = 0.0
    private var logCycle = 0
    private var logLeg = ""

    /** Best clean gain for the active axis (for status lines during the long path). */
    private var logBestSoFar = Double.NaN

    private var lastStatusNs = 0L

    /** Field pose at OpMode start — absolute out-and-back anchors (not relative to drift). */
    private var fieldHome = Pose2d(0.0, 0.0, 0.0)

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        initRobot()
        telem = telemetry

        if (TuningOpModes.DRIVE_CLASS != MecanumDrive::class.java) {
            telem!!.addLine("PathFeedbackGainTuner is mecanum-only (holonomic path gains).")
            telem!!.addLine("Use ManualFeedbackTuner for tank.")
            telem!!.update()
            waitForStart()
            return
        }

        val drive =
            MecanumDrive.forMarsLinear(hardwareMap, Pose2d(0.0, 0.0, 0.0), this::batteryVoltage)
        checkDeadWheels(drive.localizer)
        requirePositive(MecanumDrive.PARAMS.kV, MecanumDrive.PARAMS.kA)

        openLogs()

        telem!!.addLine("Path feedback-gain tuner (ManualFeedback-style).")
        telem!!.addLine(
            String.format(
                Locale.US,
                "Corridor: ~%.0f in on field X. %d cycle(s) per gain.",
                DISTANCE + 12.0,
                CYCLES,
            ),
        )
        telem!!.addLine(
            String.format(
                Locale.US,
                "1) axial  2) heading  3) home+turn %.0f° + lateral (TUNE_LATERAL=%s)",
                Math.toDegrees(LATERAL_HEADING_RAD),
                TUNE_LATERAL,
            ),
        )
        telem!!.addLine(
            String.format(
                Locale.US,
                "ladder %.0f..%.0f step %.0f, vel=%.1f — chatter → keep last clean",
                START_GAIN,
                MAX_GAIN,
                GAIN_STEP,
                VEL_GAIN,
            ),
        )
        if (sampleLog != null) {
            telem!!.addData("sample log", sampleLog!!.fileName())
            telem!!.addData("summary log", summaryLog!!.fileName())
            telem!!.addLine("Pull with: telemetry/pull.sh")
        }
        telem!!.update()
        waitForStart()
        if (isStopRequested) {
            closeLogs()
            return
        }

        drive.updatePoseEstimate()
        fieldHome = drive.localizer.getPose()

        MecanumDrive.PARAMS.axialVelGain = VEL_GAIN
        MecanumDrive.PARAMS.lateralVelGain = VEL_GAIN
        MecanumDrive.PARAMS.headingVelGain = VEL_GAIN

        // --- Phase 1: axial (lateral=0, heading hold) ---
        val axial =
            searchAxis(
                drive,
                Axis.AXIAL,
                PathMode.AXIAL_LINE,
                /* axial */ Double.NaN,
                /* lateral */ 0.0,
                /* heading */ HOLD_HEADING_GAIN,
            )
        if (!opModeIsActive()) {
            finish(drive, axial, skipped("skipped"), skipped("skipped"))
            return
        }

        val axialBest = if (axial.ok) axial.posGain else START_GAIN
        MecanumDrive.PARAMS.axialGain = axialBest
        MecanumDrive.PARAMS.lateralGain = 0.0

        // --- Phase 2: heading (axial locked, lateral=0) ---
        val heading =
            searchAxis(
                drive,
                Axis.HEADING,
                PathMode.AXIAL_LINE,
                axialBest,
                0.0,
                /* heading under test */ Double.NaN,
            )
        if (!opModeIsActive()) {
            finish(drive, axial, skipped("skipped"), heading)
            return
        }

        val headingBest = if (heading.ok) heading.posGain else HOLD_HEADING_GAIN
        MecanumDrive.PARAMS.headingGain = headingBest

        // --- Phase 3: lateral on same corridor after home + 90° turn ---
        val lateral: AxisResult
        if (TUNE_LATERAL && opModeIsActive()) {
            telem!!.addLine("Reorienting: return home, turn for lateral corridor…")
            telem!!.update()
            lateral =
                if (!reorientForLateral(drive, fieldHome)) {
                    AxisResult(false, 0.0, "FAILED: reorient home/turn aborted")
                } else {
                    searchAxis(
                        drive,
                        Axis.LATERAL,
                        PathMode.LATERAL_LINE,
                        axialBest,
                        /* lateral under test */ Double.NaN,
                        headingBest,
                    )
                }
        } else {
            lateral = AxisResult(true, 0.0, "skipped (TUNE_LATERAL=false)")
        }

        finish(drive, axial, lateral, heading)
    }

    private fun finish(
        drive: MecanumDrive,
        axial: AxisResult,
        lateral: AxisResult,
        heading: AxisResult,
    ) {
        // Apply best gains first so the return-home path uses them.
        MecanumDrive.PARAMS.axialGain = if (axial.ok) axial.posGain else START_GAIN
        MecanumDrive.PARAMS.lateralGain = if (lateral.ok) lateral.posGain else 0.0
        MecanumDrive.PARAMS.headingGain = if (heading.ok) heading.posGain else HOLD_HEADING_GAIN
        MecanumDrive.PARAMS.axialVelGain = VEL_GAIN
        MecanumDrive.PARAMS.lateralVelGain = VEL_GAIN
        MecanumDrive.PARAMS.headingVelGain = VEL_GAIN

        // Park at the OpMode start pose/heading so the next run can begin without re-staging.
        if (opModeIsActive()) {
            telem!!.addLine("Returning to start pose / heading…")
            telem!!.update()
            returnToStart(drive, fieldHome)
        }

        drive.setDrivePowers(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0))
        closeLogs()

        while (nextFrame()) {
            telem!!.addLine("=== Written to live MecanumDrive.PARAMS ===")
            telem!!.addData("axialGain", "%.2f", MecanumDrive.PARAMS.axialGain)
            telem!!.addData("lateralGain", "%.2f", MecanumDrive.PARAMS.lateralGain)
            telem!!.addData("headingGain", "%.2f", MecanumDrive.PARAMS.headingGain)
            telem!!.addData("axialVelGain", "%.2f", MecanumDrive.PARAMS.axialVelGain)
            telem!!.addData("lateralVelGain", "%.2f", MecanumDrive.PARAMS.lateralVelGain)
            telem!!.addData("headingVelGain", "%.2f", MecanumDrive.PARAMS.headingVelGain)
            telem!!.addLine()
            telem!!.addData("axial", axial.note)
            telem!!.addData("lateral", lateral.note)
            telem!!.addData("heading", heading.note)
            if (LOG_CSV) {
                telem!!.addLine("CSVs on hub under /sdcard/FIRST/ — pull with telemetry/pull.sh")
            }
            telem!!.addLine("Live for later OpModes this session. Paste into source to keep.")
            telem!!.addLine("Robot should be back at start pose — ready to re-run.")
            telem!!.addLine("Verify with ManualFeedbackTuner / SplineTest.")
        }
    }

    /**
     * Drive back near field home (constant heading), then turn to [LATERAL_HEADING_RAD] so the next
     * strafe legs are pure robot-lateral on the same field-X strip.
     */
    private fun reorientForLateral(drive: MecanumDrive, home: Pose2d): Boolean {
        return goToPose(drive, home.position.x, home.position.y, LATERAL_HEADING_RAD, "reorient")
    }

    /** Return to the OpMode start pose (field home XY + original heading) after tuning finishes. */
    private fun returnToStart(drive: MecanumDrive, home: Pose2d): Boolean {
        return goToPose(
            drive,
            home.position.x,
            home.position.y,
            home.heading.toDouble(),
            "return_home",
        )
    }

    /**
     * Strafe to field (x, y) at constant heading, then `turnTo` the requested heading. Unscored —
     * used for reorient and end-of-run park.
     */
    private fun goToPose(
        drive: MecanumDrive,
        x: Double,
        y: Double,
        headingRad: Double,
        logTag: String,
    ): Boolean {
        drive.updatePoseEstimate()
        val here = drive.localizer.getPose()
        logAxis = logTag
        logGain = 0.0
        logCycle = 0
        val path =
            drive.actionBuilder(here)
                .strafeToConstantHeading(Vector2d(x, y))
                .turnTo(headingRad)
                .build()
        val ok = runActionUnscored(drive, path)
        drive.setDrivePowers(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0))
        return ok && opModeIsActive()
    }

    /**
     * Discrete gain ladder for one axis. Pass [Double.NaN] for the slot under test; the other two
     * slots are fixed companion gains for that phase.
     */
    private fun searchAxis(
        drive: MecanumDrive,
        axis: Axis,
        pathMode: PathMode,
        axialGainOrNaN: Double,
        lateralGainOrNaN: Double,
        headingGainOrNaN: Double,
    ): AxisResult {
        var bestGain = Double.NaN
        var bestHes = 0
        // Highest clean step with zero hesitations — preferred when the top clean still had mild
        // hes (e.g. heading 7 clean-with-hes=1, reject at 8 → keep 6, not 7).
        var bestZeroHesGain = Double.NaN
        var note = "not tuned"
        logAxis = axis.name.lowercase(Locale.US)

        val startGain = if (axis == Axis.HEADING) HEADING_START_GAIN else START_GAIN
        var gain = startGain
        while (gain <= MAX_GAIN + 1e-9 && opModeIsActive()) {
            logGain = gain
            logBestSoFar = bestGain
            applyGains(axis, gain, axialGainOrNaN, lateralGainOrNaN, headingGainOrNaN)

            // Immediate status before the long nextSample() path (which would otherwise freeze DS).
            lastStatusNs = 0L
            maybePublishStatus(null, null, Double.NaN)

            val metrics = PathMetrics(axis)
            val ok = runGainStep(drive, pathMode, metrics)
            if (!ok || !opModeIsActive()) {
                val picked = quieterBest(bestGain, bestHes, bestZeroHesGain)
                return AxisResult(
                    !picked.isNaN(),
                    if (picked.isNaN()) 0.0 else picked,
                    "aborted (stop or path failure)",
                )
            }

            val chatter = metrics.chatter(axis)
            val edge = !chatter && metrics.edge(axis)
            val verdict =
                if (chatter) {
                    if (metrics.hesitations >= MAX_HESITATIONS_REJECT) "hesitation" else "chatter"
                } else if (edge) {
                    if (metrics.hesitations >= MAX_HESITATIONS_EDGE) "edge_hesitation" else "edge_keep"
                } else {
                    "clean"
                }
            logSummary(axis, gain, metrics, chatter, edge, verdict)

            if (chatter) {
                if (!bestGain.isNaN()) {
                    var picked = quieterBest(bestGain, bestHes, bestZeroHesGain)
                    note =
                        String.format(
                            Locale.US,
                            "ok: chatter at %.1f — kept %.1f%s",
                            gain,
                            picked,
                            if (picked + 1e-9 < bestGain) {
                                String.format(
                                    Locale.US,
                                    " (backed off from last clean %.1f; hes=%d)",
                                    bestGain,
                                    bestHes,
                                )
                            } else {
                                " (last clean)"
                            },
                        )
                    bestGain = picked
                } else {
                    val lower = gain - GAIN_STEP
                    if (lower >= 1.0) {
                        bestGain = lower
                        note =
                            String.format(
                                Locale.US,
                                "ok: chatter at start %.1f — backed off to %.1f (verify by" +
                                    " ear)",
                                gain,
                                bestGain,
                            )
                    } else {
                        note =
                            String.format(
                                Locale.US,
                                "FAILED: chatter at start gain %.1f — check feedforward /" +
                                    " lower START_GAIN",
                                gain,
                            )
                        return AxisResult(false, 0.0, note)
                    }
                }
                break
            }

            // Soft edge: do not keep this gain. The edge step already shows stop-then-lunge or
            // near-chatter; keeping it poisons later phases (heading ladder on an axial path that
            // already hesitates) and picks a gain that feels wrong by eye before reverse.
            if (edge) {
                if (!bestGain.isNaN()) {
                    var picked = quieterBest(bestGain, bestHes, bestZeroHesGain)
                    if (metrics.hesitations >= MAX_HESITATIONS_EDGE) {
                        note =
                            String.format(
                                Locale.US,
                                "ok: hesitation edge at %.1f (hes=%d) — kept %.1f%s",
                                gain,
                                metrics.hesitations,
                                picked,
                                if (picked + 1e-9 < bestGain) {
                                    String.format(
                                        Locale.US,
                                        " (backed off from %.1f; hes=%d)",
                                        bestGain,
                                        bestHes,
                                    )
                                } else {
                                    " (last clean)"
                                },
                            )
                    } else {
                        note =
                            String.format(
                                Locale.US,
                                "ok: near chatter edge at %.1f — kept %.1f%s",
                                gain,
                                picked,
                                if (picked + 1e-9 < bestGain) {
                                    String.format(
                                        Locale.US,
                                        " (backed off from %.1f; hes=%d)",
                                        bestGain,
                                        bestHes,
                                    )
                                } else {
                                    " (last clean)"
                                },
                            )
                    }
                    bestGain = picked
                } else {
                    // First step is already soft-edge: keep it. Backing off below START would
                    // strand heading/axial at an unusably weak gain (the old "heading=2 feels
                    // limp" failure mode when a false edge fired at the floor).
                    bestGain = gain
                    note =
                        String.format(
                            Locale.US,
                            "ok: edge at start %.1f (hes=%d) — kept (no lower clean step;" +
                                " verify by eye)",
                            bestGain,
                            metrics.hesitations,
                        )
                }
                break
            }

            bestGain = gain
            bestHes = metrics.hesitations
            if (metrics.hesitations == 0) {
                bestZeroHesGain = gain
            }
            note =
                String.format(
                    Locale.US,
                    "ok: clean at %.1f (flips %d, hes %d)",
                    bestGain,
                    metrics.activeFlips(axis),
                    metrics.hesitations,
                )

            if (gain + GAIN_STEP > MAX_GAIN + 1e-9) {
                var picked = quieterBest(bestGain, bestHes, bestZeroHesGain)
                if (picked + 1e-9 < bestGain) {
                    note =
                        String.format(
                            Locale.US,
                            "ok: clean through max — backed off %.1f→%.1f (last zero-hes)",
                            bestGain,
                            picked,
                        )
                    bestGain = picked
                } else {
                    note = String.format(Locale.US, "ok: clean through max gain %.1f", bestGain)
                }
            }
            gain += GAIN_STEP
        }

        if (bestGain.isNaN()) {
            return AxisResult(false, 0.0, note)
        }
        applyGains(axis, bestGain, axialGainOrNaN, lateralGainOrNaN, headingGainOrNaN)
        return AxisResult(true, bestGain, note)
    }

    /**
     * [CYCLES] out-and-back paths in the requested path mode. Endpoints are absolute field anchors
     * from [fieldHome] so relative overshoot cannot march the start pose into a wall.
     */
    private fun runGainStep(
        drive: MecanumDrive,
        pathMode: PathMode,
        metrics: PathMetrics,
    ): Boolean {
        val xNear = fieldHome.position.x
        val xFar = fieldHome.position.x + DISTANCE
        val yHome = fieldHome.position.y

        var cycle = 1
        while (cycle <= CYCLES && opModeIsActive()) {
            logCycle = cycle
            drive.updatePoseEstimate()
            val begin = drive.localizer.getPose()

            val path: Action =
                if (pathMode == PathMode.LATERAL_LINE) {
                    // Do NOT use lineToX* here: RR lineToX goes along path tangent until x=target, so
                    // with heading ≈ 90° the segment is almost field ±Y (robot reverse), not a
                    // strafe. Explicit field-XY endpoints + constant heading = pure robot-lateral
                    // on the strip.
                    drive.actionBuilder(begin)
                        .strafeToConstantHeading(Vector2d(xFar, yHome))
                        .strafeToConstantHeading(Vector2d(xNear, yHome))
                        .build()
                } else {
                    // Absolute field-X anchors (not begin.x ± DISTANCE).
                    drive.actionBuilder(begin).lineToX(xFar).lineToX(xNear).build()
                }

            if (!runActionSampling(drive, path, metrics)) {
                return false
            }
            cycle++
        }
        return opModeIsActive()
    }

    /** Run an action for reorient (turns + short strafe) without chatter scoring. */
    private fun runActionUnscored(drive: MecanumDrive, action: Action): Boolean {
        for (leaf in flatten(action)) {
            if (!opModeIsActive()) return false
            if (leaf is MecanumDrive.FollowTrajectoryAction) {
                val traj = leaf.timeTrajectory
                if (!followTrajectory(drive, traj, /* metrics */ null)) return false
            } else if (leaf is MecanumDrive.TurnAction) {
                logLeg = "turn"
                val packet = TelemetryPacket()
                while (nextSample() && leaf.run(packet)) {
                    drive.updatePoseEstimate()
                    maybePublishStatus(drive.localizer.getPose(), null, Double.NaN)
                }
                if (isStopRequested) return false
            } else {
                val packet = TelemetryPacket()
                while (nextSample() && leaf.run(packet)) {
                    drive.updatePoseEstimate()
                    maybePublishStatus(drive.localizer.getPose(), null, Double.NaN)
                }
                if (isStopRequested) return false
            }
        }
        return true
    }

    private fun runActionSampling(
        drive: MecanumDrive,
        action: Action,
        metrics: PathMetrics,
    ): Boolean {
        for (leaf in flatten(action)) {
            if (!opModeIsActive()) return false
            if (leaf is MecanumDrive.FollowTrajectoryAction) {
                val traj = leaf.timeTrajectory
                if (!followTrajectory(drive, traj, metrics)) return false
            } else {
                val packet = TelemetryPacket()
                while (nextSample() && leaf.run(packet)) {
                    // stock
                }
                if (isStopRequested) return false
            }
        }
        drive.setDrivePowers(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0))
        return true
    }

    /**
     * Same control law as [MecanumDrive.FollowTrajectoryAction], plus optional chatter metrics /
     * CSV. Uses live `PARAMS` gains.
     */
    private fun followTrajectory(
        drive: MecanumDrive,
        traj: TimeTrajectory,
        metrics: PathMetrics?,
    ): Boolean {
        logLeg = "traj"
        val clock = ElapsedTime()
        var beginTs = -1.0
        var lastT = 0.0
        val duration = max(traj.duration, 1e-6)
        val trajBegin = traj.get(0.0).value()
        val trajEnd = traj.get(duration).value()
        val pathDx = trajEnd.position.x - trajBegin.position.x
        val pathDy = trajEnd.position.y - trajBegin.position.y
        val pathLen = hypot(pathDx, pathDy)

        while (nextSample()) {
            if (beginTs < 0) {
                beginTs = clock.seconds()
            }
            val t = clock.seconds() - beginTs
            if (t >= traj.duration) {
                drive.setDrivePowers(PoseVelocity2d(Vector2d(0.0, 0.0), 0.0))
                break
            }

            val txWorldTarget: Pose2dDual<Time> = traj.get(t)
            val robotVel = drive.updatePoseEstimate()
            val pose = drive.localizer.getPose()

            val command: PoseVelocity2dDual<Time> =
                HolonomicController(
                        MecanumDrive.PARAMS.axialGain,
                        MecanumDrive.PARAMS.lateralGain,
                        MecanumDrive.PARAMS.headingGain,
                        MecanumDrive.PARAMS.axialVelGain,
                        MecanumDrive.PARAMS.lateralVelGain,
                        MecanumDrive.PARAMS.headingVelGain,
                    )
                    .compute(txWorldTarget, pose, robotVel)

            drive.setDriveCommand(command)

            val ref = txWorldTarget.value()
            val error = ref.minusExp(pose)
            val cmdX = command.value().linearVel.x
            val cmdY = command.value().linearVel.y
            val cmdW = command.value().angVel
            val dt = t - lastT
            lastT = t
            // Dual first derivatives = reference velocity (field frame for position).
            val refSpeed = hypot(txWorldTarget.position.x[1], txWorldTarget.position.y[1])
            // heading.velocity() is already dθ/dt as a DualNum; [0] is the rate value.
            val refOmega = abs(txWorldTarget.heading.velocity()[0])
            val scoreChatter = refSpeed <= CHATTER_REF_SPEED && refOmega <= CHATTER_REF_OMEGA
            val progress01 = t / duration
            if (metrics != null && dt > 1e-6) {
                val axisVel =
                    when (metrics.axisUnderTest) {
                        Axis.AXIAL -> robotVel.linearVel.x
                        Axis.LATERAL -> robotVel.linearVel.y
                        Axis.HEADING -> robotVel.angVel
                    }
                // Positive remaining = still short of traj end along the path (or heading).
                val remainingAlongPath =
                    if (metrics.axisUnderTest == Axis.HEADING) {
                        // Rotation2d.minus already returns the wrapped log angle (rad).
                        abs(trajEnd.heading.minus(pose.heading))
                    } else if (pathLen > 1e-3) {
                        val toEndX = trajEnd.position.x - pose.position.x
                        val toEndY = trajEnd.position.y - pose.position.y
                        (toEndX * pathDx + toEndY * pathDy) / pathLen
                    } else {
                        0.0
                    }
                metrics.observe(
                    dt,
                    error.position.x,
                    error.position.y,
                    error.heading.log(),
                    cmdX,
                    cmdY,
                    cmdW,
                    axisVel,
                    progress01,
                    remainingAlongPath,
                    scoreChatter,
                )
            }

            maybePublishStatus(pose, ref, t / duration)

            if (sampleLog != null) {
                sampleLog!!.row(
                    t,
                    logAxis,
                    logGain,
                    logCycle,
                    logLeg,
                    pose.position.x,
                    pose.position.y,
                    pose.heading.toDouble(),
                    ref.position.x,
                    ref.position.y,
                    ref.heading.toDouble(),
                    error.position.x,
                    error.position.y,
                    error.heading.log(),
                    robotVel.linearVel.x,
                    robotVel.linearVel.y,
                    robotVel.angVel,
                    cmdX,
                    cmdY,
                    cmdW,
                    if (metrics == null) 0 else metrics.axialCmdFlips,
                    if (metrics == null) 0 else metrics.lateralCmdFlips,
                    if (metrics == null) 0 else metrics.headingCmdFlips,
                )
                maybeFlushSamples()
            }
        }

        sampleLog?.flush()
        return !isStopRequested
    }

    /**
     * Refresh DS / Dashboard during [nextSample] bursts. `nextSample` keeps gated telemetry closed,
     * so without this the station freezes on the last line for the whole path.
     */
    private fun maybePublishStatus(pose: Pose2d?, ref: Pose2d?, progress01: Double) {
        if (STATUS_MS < 0) {
            return
        }
        val nowNs = System.nanoTime()
        if (STATUS_MS > 0 &&
            lastStatusNs != 0L &&
            (nowNs - lastStatusNs) < STATUS_MS * 1_000_000L
        ) {
            return
        }
        lastStatusNs = nowNs

        val t = telem!!
        val gated = if (t is GatedTelemetry) t else null
        val wasOpen = gated == null || gated.isOpen()
        gated?.setOpen(true)

        t.addData("phase", if (logAxis.isEmpty()) "?" else logAxis)
        t.addData("trying gain", "%.1f", logGain)
        t.addData(
            "best so far",
            if (logBestSoFar.isNaN()) {
                "none yet"
            } else {
                String.format(Locale.US, "%.1f", logBestSoFar)
            },
        )
        t.addData("cycle", "%d / %d", logCycle, CYCLES)
        t.addData("leg", if (logLeg.isEmpty()) "-" else logLeg)
        if (!progress01.isNaN()) {
            t.addData("path %", "%.0f", 100.0 * max(0.0, min(1.0, progress01)))
        }
        t.addData(
            "gains ax/lat/h",
            "%.1f / %.1f / %.1f",
            MecanumDrive.PARAMS.axialGain,
            MecanumDrive.PARAMS.lateralGain,
            MecanumDrive.PARAMS.headingGain,
        )
        if (pose != null) {
            t.addData(
                "pose",
                "x=%.1f y=%.1f h=%.0f°",
                pose.position.x,
                pose.position.y,
                Math.toDegrees(pose.heading.toDouble()),
            )
        }
        if (ref != null) {
            t.addData(
                "ref",
                "x=%.1f y=%.1f h=%.0f°",
                ref.position.x,
                ref.position.y,
                Math.toDegrees(ref.heading.toDouble()),
            )
        }
        if (sampleLog != null) {
            t.addData("log", sampleLog!!.fileName())
        }

        if (gated != null) {
            gated.forceUpdate()
            gated.setOpen(wasOpen)
        } else {
            t.update()
        }
    }

    private fun openLogs() {
        if (!LOG_CSV) return
        val stamp = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(Date())
        sampleLog = CsvLogger("path_fbgain_samples_$stamp.csv", SAMPLE_HEADER)
        summaryLog = CsvLogger("path_fbgain_summary_$stamp.csv", SUMMARY_HEADER)
        summaryLog!!.row(
            "meta",
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            false,
            false,
            String.format(
                Locale.US,
                "DISTANCE=%.1f;CYCLES=%d;START=%.1f;H_START=%.1f;MAX=%.1f;STEP=%.1f;" +
                    "HOLD_H=%.1f;TUNE_LAT=%s;LAT_HEAD_DEG=%.1f;VEL=%.2f;" +
                    "MAX_AX_FLIPS=%d;MAX_LAT_FLIPS=%d;MAX_H_FLIPS=%d;" +
                    "CHATTER_REF_SPEED=%.1f;MAX_HES_EDGE=%d;MAX_HES_REJ=%d;" +
                    "HES_VFAST=%.1f;HES_PMAX=%.2f;HES_REMAIN=%.1f",
                DISTANCE,
                CYCLES,
                START_GAIN,
                HEADING_START_GAIN,
                MAX_GAIN,
                GAIN_STEP,
                HOLD_HEADING_GAIN,
                TUNE_LATERAL,
                Math.toDegrees(LATERAL_HEADING_RAD),
                VEL_GAIN,
                MAX_AXIAL_CMD_FLIPS,
                MAX_LATERAL_CMD_FLIPS,
                MAX_HEADING_CMD_FLIPS,
                CHATTER_REF_SPEED,
                MAX_HESITATIONS_EDGE,
                MAX_HESITATIONS_REJECT,
                HESITATION_V_FAST,
                HESITATION_PROGRESS_MAX,
                HESITATION_REMAINING_MIN,
            ),
        )
        summaryLog!!.flush()
    }

    private fun logSummary(
        axis: Axis,
        gain: Double,
        m: PathMetrics,
        chatter: Boolean,
        edge: Boolean,
        verdict: String,
    ) {
        if (summaryLog == null) return
        summaryLog!!.row(
            axis.name.lowercase(Locale.US),
            gain,
            CYCLES,
            m.durationS,
            m.axialJerk(),
            m.lateralJerk(),
            m.headingJerk(),
            m.axialFlipsPerSec(),
            m.lateralFlipsPerSec(),
            m.headingFlipsPerSec(),
            m.errXRms(),
            m.errYRms(),
            m.errHRms(),
            m.hesitations,
            chatter,
            edge,
            verdict,
        )
        summaryLog!!.flush()
    }

    private fun closeLogs() {
        sampleLog?.close()
        sampleLog = null
        summaryLog?.close()
        summaryLog = null
    }

    private fun maybeFlushSamples() {
        val log = sampleLog
        if (log != null && log.bufferedRows() >= LOG_FLUSH_EVERY) {
            log.flush()
        }
    }
}
