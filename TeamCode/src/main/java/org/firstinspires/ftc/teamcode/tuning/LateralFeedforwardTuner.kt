package org.firstinspires.ftc.teamcode.tuning

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Rotation2d
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.MecanumDrive
import org.firstinspires.ftc.teamcode.TankDrive
import org.firstinspires.ftc.teamcode.opmodes.base.MarsLinearOpMode
import org.firstinspires.ftc.teamcode.utils.CsvLogger
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.function.DoubleConsumer
import java.util.function.DoubleSupplier
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

/**
 * Automatic on-robot identification of the anisotropic (strafe) feedforward constants
 * `lateralKS` / `lateralKV` / `lateralKA` for a mecanum drive.
 *
 * Same two-phase procedure as [AxialFeedforwardTuner] (slow ramp → kS/kV, reverse square wave →
 * kA), applied as a pure strafe with a light heading hold so open-loop roller scrub does not
 * accumulate yaw. The ramp drives **LEFT** (robot +y); it ends early if motion collapses under
 * power (or on gamepad1 A), then reverses RIGHT for kA. See [ReversalFeedforwardId].
 *
 * Prerequisites: localization and axial feedforward already tuned. Leave room on the right for
 * reverse; a wall on the left is fine. Expect lateral constants to exceed their axial counterparts
 * (roller scrub).
 *
 * On a successful full fit, writes `lateralKS`/`lateralKV`/`lateralKA` and sets
 * `useAnisotropicFeedforward = true` on the live `PARAMS` statics so later OpModes in the same RC
 * process can chain without a paste. A ramp-only fit still writes `lateralKS`/`lateralKV` and
 * enables anisotropic mode. Paste into source to keep values across restart or redeploy.
 *
 * Each run writes two CSVs under `/sdcard/FIRST/` (via [CsvLogger]):
 * - `lateral_ff_samples_<stamp>.csv` — raw loop measurements (power, battery, velocity, pose; see
 *   [ReversalFeedforwardId.SAMPLE_HEADER])
 * - `lateral_ff_meta_<stamp>.csv` — run config / plant scale for offline re-fit (see
 *   [ReversalFeedforwardId.META_HEADER]); not the fitted lateralKS/KV/KA
 *
 * Pull them with `telemetry/pull.sh`.
 */
@Config
class LateralFeedforwardTuner : MarsLinearOpMode() {
    companion object {
        /** When false, skip writing CSVs (useful if the hub disk is full). */
        @JvmField
        var LOG_CSV = true

        /** Power increase per second during the lateral ramp. */
        @JvmField
        var RAMP_POWER_PER_SEC = 0.1

        /** Peak power of the ramp phase. */
        @JvmField
        var RAMP_MAX = 0.9

        /** |Power| of the reverse square wave used only for lateralKA. */
        @JvmField
        var KA_POWER = 0.6

        /**
         * Fallback seconds per reverse direction when ramp travel is too short for a position-based
         * corridor (see [MIN_TRAVEL_IN]).
         */
        @JvmField
        var HALF_CYCLE = 1.0

        /** Number of reverse half-cycles. */
        @JvmField
        var KA_HALF_CYCLES = 4

        /** Inches kept clear of each end of the measured ramp corridor. */
        @JvmField
        var END_MARGIN_IN = ReversalFeedforwardId.DEFAULT_END_MARGIN_IN

        /** Minimum |ramp travel| (in) to use position-based reverse half-cycles. */
        @JvmField
        var MIN_TRAVEL_IN = ReversalFeedforwardId.DEFAULT_MIN_TRAVEL_IN

        /** Speeds (in/s) within this of zero use sign=0 in the kA residual. */
        @JvmField
        var SIGN_DEADBAND = 1.0

        /**
         * Ramp samples slower than this (encoder-tick units per second) are excluded from the kS/kV
         * fit. Converted with `inPerTick`; default 1000 cuts the breakaway knee at low speed.
         */
        @JvmField
        var MIN_RAMP_TICKS_PER_SEC = ReversalFeedforwardId.DEFAULT_MIN_RAMP_TICKS_PER_SEC

        /** Absolute speed (in/s) treated as stalled after the robot has been moving. */
        @JvmField
        var STALL_SPEED = ReversalFeedforwardId.DEFAULT_STALL_SPEED

        /** Seconds velocity must stay collapsed before ending the ramp for a wall/stall. */
        @JvmField
        var STALL_TIME = ReversalFeedforwardId.DEFAULT_STALL_TIME

        /** Speed (in/s) that must be reached once before stall detection arms. */
        @JvmField
        var MOVING_SPEED = ReversalFeedforwardId.DEFAULT_MOVING_SPEED

        /** Minimum commanded power for a stall to count. */
        @JvmField
        var STALL_MIN_POWER = ReversalFeedforwardId.DEFAULT_STALL_MIN_POWER

        /** Fraction of peak ramp speed below which velocity counts as collapsed. */
        @JvmField
        var STALL_FRAC = ReversalFeedforwardId.DEFAULT_STALL_FRAC

        /**
         * |Travel| (in) that arms stall detection even if speed never reached [MOVING_SPEED]
         * (lateral ramps often hit a wall before that).
         */
        @JvmField
        var ARM_TRAVEL_IN = ReversalFeedforwardId.DEFAULT_ARM_TRAVEL_IN

        /**
         * When armed, |d(pos)/dt| below this (in/s) under power counts as wall contact (helps when
         * mecanum rollers keep velocity slightly nonzero against a wall).
         */
        @JvmField
        var POS_STALL_SPEED = ReversalFeedforwardId.DEFAULT_POS_STALL_SPEED

        /**
         * Heading hold gain (power per rad of error). Open-loop strafe otherwise curls; this adds a
         * pure yaw correction so lateral sysid stays roughly straight. 0 disables.
         */
        @JvmField
        var HEADING_GAIN = 1.2

        /** Heading-rate damping (power per rad/s). */
        @JvmField
        var HEADING_VEL_GAIN = 0.08

        /** Max |yaw power| blended into the strafe command. */
        @JvmField
        var HEADING_MAX_CORR = 0.35

        private fun closeLogs(sampleLog: CsvLogger?, metaLog: CsvLogger?) {
            sampleLog?.close()
            metaLog?.close()
        }
    }

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        if (TuningOpModes.DRIVE_CLASS != MecanumDrive::class.java) {
            throw RuntimeException(
                "LateralFeedforwardTuner is mecanum-only; a " +
                    (if (TuningOpModes.DRIVE_CLASS == TankDrive::class.java) "tank" else "non-mecanum") +
                    " drive has no lateral motion.",
            )
        }

        initRobot()
        val drive =
            MecanumDrive.forMarsLinear(hardwareMap, Pose2d(0.0, 0.0, 0.0), this::batteryVoltage)
        val inPerTick = MecanumDrive.PARAMS.inPerTick
        val lateralMultiplier = drive.kinematics.lateralMultiplier

        var sampleLog: CsvLogger? = null
        var metaLog: CsvLogger? = null
        if (LOG_CSV) {
            val stamp = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(Date())
            sampleLog =
                CsvLogger("lateral_ff_samples_$stamp.csv", ReversalFeedforwardId.SAMPLE_HEADER)
            // Shared identify knobs + lateral-only plant/hold config (not fit results).
            val metaHeader =
                ReversalFeedforwardId.META_HEADER +
                    ",lateral_multiplier,heading_gain,heading_vel_gain,heading_max_corr"
            metaLog = CsvLogger("lateral_ff_meta_$stamp.csv", metaHeader)
            metaLog.row(
                "MecanumDrive",
                "lateral",
                inPerTick,
                RAMP_POWER_PER_SEC,
                RAMP_MAX,
                KA_POWER,
                HALF_CYCLE,
                KA_HALF_CYCLES,
                SIGN_DEADBAND,
                STALL_SPEED,
                STALL_TIME,
                MOVING_SPEED,
                STALL_MIN_POWER,
                STALL_FRAC,
                MIN_RAMP_TICKS_PER_SEC,
                END_MARGIN_IN,
                MIN_TRAVEL_IN,
                ARM_TRAVEL_IN,
                POS_STALL_SPEED,
                ReversalFeedforwardId.DEFAULT_KA_WINDOW,
                ReversalFeedforwardId.DEFAULT_KA_STRIDE,
                lateralMultiplier,
                HEADING_GAIN,
                HEADING_VEL_GAIN,
                HEADING_MAX_CORR,
            )
            metaLog.flush()
        }

        telemetry.addLine("Mecanum lateral (strafe) feedforward tuner (kS, kV, kA).")
        telemetry.addLine("Phase 1: ramps LEFT (robot +y) → lateralKS/KV.")
        telemetry.addLine("  Press gamepad1 A just before wall/mat edge (recommended).")
        telemetry.addLine("  Auto-stops if stalled against a wall (wheels may jam).")
        telemetry.addLine("  Heading hold on (HEADING_GAIN) to limit yaw curl.")
        telemetry.addLine("Phase 2: reverse square wave → lateralKA (first goes RIGHT).")
        telemetry.addLine("  Half-cycles use the distance driven on the ramp when possible.")
        telemetry.addLine("Press START. Leave room on the RIGHT for reverse.")
        if (sampleLog != null) {
            telemetry.addData("sample log", sampleLog.fileName())
            telemetry.addData("meta log", metaLog!!.fileName())
            telemetry.addLine("Pull with: telemetry/pull.sh")
        }
        telemetry.update()
        waitForStart()
        if (isStopRequested) {
            closeLogs(sampleLog, metaLog)
            return
        }

        // Hold the heading at START so open-loop strafe reversals do not walk in yaw.
        drive.localizer.update()
        val headingTarget: Rotation2d = drive.localizer.getPose().heading

        // +power => strafe (+y = robot LEFT) plus a pure-yaw heading correction.
        // Kinematics: strafe p → (-p, +p, -p, +p); +angVel u → (-u, -u, +u, +u).
        // Prefer preserving lateral |p| for the sysid voltage model: shrink yaw correction
        // if |p| + |u| would saturate, rather than scaling all four wheels down.
        val setPower =
            DoubleConsumer { p ->
                val tw: PoseVelocity2d = drive.localizer.update()
                val err = headingTarget.minus(drive.localizer.getPose().heading)
                var u = HEADING_GAIN * err - HEADING_VEL_GAIN * tw.angVel
                val uMax = min(HEADING_MAX_CORR, max(0.0, 1.0 - abs(p)))
                u = Range.clip(u, -uMax, uMax)

                drive.leftFront.power = -p - u
                drive.leftBack.power = +p - u
                drive.rightBack.power = -p + u
                drive.rightFront.power = +p + u
            }
        // Localizer already updated in setPower; read lateral velocity without a second update
        // when possible. identify() calls setPower then wheelVel each loop — one update in
        // setPower is enough; wheelVel re-reads pose velocity from the last update by calling
        // update() again (Pinpoint is cheap / consistent).
        val wheelVel = DoubleSupplier { drive.localizer.update().linearVel.y * lateralMultiplier }
        val axisPos = DoubleSupplier { drive.localizer.getPose().position.y }

        // identify() advances samples via nextFrame under MANUAL bulk.
        val fit =
            ReversalFeedforwardId.identify(
                this,
                telemetry,
                setPower,
                wheelVel,
                axisPos,
                inPerTick,
                RAMP_POWER_PER_SEC,
                RAMP_MAX,
                KA_POWER,
                HALF_CYCLE,
                KA_HALF_CYCLES,
                SIGN_DEADBAND,
                STALL_SPEED,
                STALL_TIME,
                MOVING_SPEED,
                STALL_MIN_POWER,
                STALL_FRAC,
                MIN_RAMP_TICKS_PER_SEC,
                END_MARGIN_IN,
                MIN_TRAVEL_IN,
                ARM_TRAVEL_IN,
                POS_STALL_SPEED,
                sampleLog,
            )

        var wroteRampOnly = false
        if (!fit.singular) {
            MecanumDrive.PARAMS.lateralKS = fit.kS
            MecanumDrive.PARAMS.lateralKV = fit.kV
            MecanumDrive.PARAMS.lateralKA = fit.kA
            MecanumDrive.PARAMS.useAnisotropicFeedforward = true
        } else if (
            fit.rampSamples > 0 &&
                fit.kS.isFinite() &&
                fit.kS != 0.0 &&
                fit.kV.isFinite() &&
                fit.kV > 0
        ) {
            MecanumDrive.PARAMS.lateralKS = fit.kS
            MecanumDrive.PARAMS.lateralKV = fit.kV
            MecanumDrive.PARAMS.useAnisotropicFeedforward = true
            wroteRampOnly = true
        }

        closeLogs(sampleLog, metaLog)

        while (nextFrame()) {
            if (fit.singular) {
                telemetry.addLine("Fit failed: " + fit.message)
                telemetry.addData("samples", fit.samples)
                telemetry.addData("ramp samples used", fit.rampSamples)
                // Ramp kS/kV may still be usable when only the reverse/kA phase failed.
                if (wroteRampOnly) {
                    telemetry.addLine(
                        "Ramp-only lateralKS/KV written to live PARAMS (kA not updated):",
                    )
                    telemetry.addData("useAnisotropicFeedforward", true)
                    telemetry.addData("lateralKS", "%.5f", fit.kS)
                    telemetry.addData("lateralKV", "%.5e", fit.kV)
                    telemetry.addData("ramp R^2", "%.3f", fit.rampR2)
                    telemetry.addLine("Still paste into source to keep after restart.")
                } else if (fit.rampSamples > 0 && fit.kS.isFinite() && fit.kS != 0.0) {
                    telemetry.addLine("Ramp-only (kA not trusted; not written — kV unusable):")
                    telemetry.addData("lateralKS", "%.5f", fit.kS)
                    telemetry.addData("lateralKV", "%.5e", fit.kV)
                    telemetry.addData("ramp R^2", "%.3f", fit.rampR2)
                }
            } else {
                telemetry.addLine("=== Written to live MecanumDrive.PARAMS ===")
                telemetry.addData("useAnisotropicFeedforward", true)
                telemetry.addData("lateralKS", "%.5f", fit.kS)
                telemetry.addData("lateralKV", "%.5e", fit.kV)
                telemetry.addData("lateralKA", "%.5e", fit.kA)
                telemetry.addLine()
                telemetry.addData("samples", fit.samples)
                telemetry.addData("ramp samples used", fit.rampSamples)
                telemetry.addData("ramp R^2", "%.3f", fit.rampR2)
                telemetry.addLine("Live for later OpModes this session. Paste into source to keep.")
                telemetry.addLine("Expect lateralKS ≫ axial kS; kV/kA often only modestly higher.")
            }
            if (LOG_CSV) {
                telemetry.addLine("CSVs on hub under /sdcard/FIRST/ — pull with telemetry/pull.sh")
            }
        }
    }
}
