package org.firstinspires.ftc.teamcode.tuning

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.Pose2d
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.function.DoubleConsumer
import java.util.function.DoubleSupplier
import org.firstinspires.ftc.teamcode.MecanumDrive
import org.firstinspires.ftc.teamcode.TankDrive
import org.firstinspires.ftc.teamcode.opmodes.base.MarsLinearOpMode
import org.firstinspires.ftc.teamcode.utils.CsvLogger

/**
 * Automatic on-robot identification of the axial feedforward — including `kA`, which the stock
 * tuner leaves to eyeballing a target-vs-actual velocity graph.
 *
 * Two phases (see [ReversalFeedforwardId]):
 *
 * 1. Slow open-loop forward ramp — same model as `ForwardRampLogger` → `kS`, `kV`. Ends early if
 *    the robot stops moving under power (e.g. hits a wall).
 * 2. Reverse square wave (starts reverse) → residual `kA`. Half-cycles use the measured ramp travel
 *    (start → wall) so the kA phase spans the free corridor, not a fixed 1 s.
 *
 * Prerequisites: localization tuned and sign-correct (drive forward → +x). A wall ahead is OK — the
 * ramp stops on contact and reverse half-cycles run within the distance just driven.
 *
 * On a successful full fit, writes `kS`/`kV`/`kA` into the live `PARAMS` statics so later tuning
 * OpModes in the same RC process can chain without a paste. A ramp-only (kA failed) fit still
 * writes `kS`/`kV`. Paste into source to keep values across restart or redeploy.
 *
 * Each run writes two CSVs under `/sdcard/FIRST/` (via [CsvLogger]):
 * - `axial_ff_samples_<stamp>.csv` — raw loop measurements (power, battery, velocity, pose; see
 *   [ReversalFeedforwardId.SAMPLE_HEADER])
 * - `axial_ff_meta_<stamp>.csv` — run config / plant scale for offline re-fit (see
 *   [ReversalFeedforwardId.META_HEADER]); not the fitted kS/kV/kA
 *
 * Pull them with `telemetry/pull.sh`.
 */
@Config
class AxialFeedforwardTuner : MarsLinearOpMode() {
    companion object {
        /** When false, skip writing CSVs (useful if the hub disk is full). */
        @JvmField var LOG_CSV = true

        /** Power increase per second during the kS/kV ramp (stock ForwardRampLogger uses 0.1). */
        @JvmField var RAMP_POWER_PER_SEC = 0.1

        /** Peak power of the ramp phase. */
        @JvmField var RAMP_MAX = 0.9

        /** |Power| of the reverse square wave used only for kA. */
        @JvmField var KA_POWER = 0.6

        /**
         * Fallback seconds per reverse direction when ramp travel is too short for a position-based
         * corridor (see [MIN_TRAVEL_IN]).
         */
        @JvmField var HALF_CYCLE = 1.0

        /** Number of reverse half-cycles (direction holds). 4 ≈ two full round trips. */
        @JvmField var KA_HALF_CYCLES = 4

        /**
         * Inches kept clear of each end of the measured ramp corridor during position-based reverse
         * half-cycles.
         */
        @JvmField var END_MARGIN_IN = ReversalFeedforwardId.DEFAULT_END_MARGIN_IN

        /**
         * Minimum |ramp travel| (in) before reverse half-cycles use the full corridor instead of
         * timed [HALF_CYCLE] holds.
         */
        @JvmField var MIN_TRAVEL_IN = ReversalFeedforwardId.DEFAULT_MIN_TRAVEL_IN

        /** Speeds (in/s) within this of zero use sign=0 in the kA residual. */
        @JvmField var SIGN_DEADBAND = 1.0

        /**
         * Ramp samples slower than this (encoder-tick units per second) are excluded from the kS/kV
         * fit. Converted with `inPerTick`; default 1000 cuts the breakaway knee at low speed.
         */
        @JvmField var MIN_RAMP_TICKS_PER_SEC = ReversalFeedforwardId.DEFAULT_MIN_RAMP_TICKS_PER_SEC

        /**
         * Absolute speed (in/s) treated as stalled after the robot has been moving; see also
         * [STALL_FRAC].
         */
        @JvmField var STALL_SPEED = ReversalFeedforwardId.DEFAULT_STALL_SPEED

        /** Seconds velocity must stay collapsed before ending the ramp for a wall/stall. */
        @JvmField var STALL_TIME = ReversalFeedforwardId.DEFAULT_STALL_TIME

        /** Speed (in/s) that must be reached once before stall detection arms. */
        @JvmField var MOVING_SPEED = ReversalFeedforwardId.DEFAULT_MOVING_SPEED

        /** Minimum commanded power for a stall to count. */
        @JvmField var STALL_MIN_POWER = ReversalFeedforwardId.DEFAULT_STALL_MIN_POWER

        /** Fraction of peak ramp speed below which velocity counts as collapsed. */
        @JvmField var STALL_FRAC = ReversalFeedforwardId.DEFAULT_STALL_FRAC

        /** |Travel| (in) that arms stall detection even if speed never reached [MOVING_SPEED]. */
        @JvmField var ARM_TRAVEL_IN = ReversalFeedforwardId.DEFAULT_ARM_TRAVEL_IN

        /** When armed, |d(pos)/dt| below this (in/s) under power counts as wall contact. */
        @JvmField var POS_STALL_SPEED = ReversalFeedforwardId.DEFAULT_POS_STALL_SPEED

        private fun closeLogs(sampleLog: CsvLogger?, metaLog: CsvLogger?) {
            sampleLog?.close()
            metaLog?.close()
        }

        /**
         * Writes axial feedforward into the drive's live `PARAMS`. When `writeKA` is false, only
         * `kS`/`kV` are updated (partial ramp fit).
         */
        private fun writeAxialParams(
            driveName: String,
            kS: Double,
            kV: Double,
            kA: Double,
            writeKA: Boolean,
        ) {
            if (driveName == "MecanumDrive") {
                MecanumDrive.PARAMS.kS = kS
                MecanumDrive.PARAMS.kV = kV
                if (writeKA) {
                    MecanumDrive.PARAMS.kA = kA
                }
            } else {
                TankDrive.PARAMS.kS = kS
                TankDrive.PARAMS.kV = kV
                if (writeKA) {
                    TankDrive.PARAMS.kA = kA
                }
            }
        }
    }

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        initRobot()

        val setPower: DoubleConsumer
        // signed chassis forward velocity, in/s, from the localizer
        val forwardVel: DoubleSupplier
        // forward position (in); read after forwardVel updates localizer
        val axisPos: DoubleSupplier
        val inPerTick: Double
        val driveName: String

        when (TuningOpModes.DRIVE_CLASS) {
            MecanumDrive::class.java -> {
                val drive =
                    MecanumDrive.forMarsLinear(hardwareMap, Pose2d(0.0, 0.0, 0.0)) {
                        batteryVoltage()
                    }
                setPower = DoubleConsumer { p ->
                    drive.leftFront.power = p
                    drive.leftBack.power = p
                    drive.rightBack.power = p
                    drive.rightFront.power = p
                }
                forwardVel = DoubleSupplier { drive.localizer.update().linearVel.x }
                axisPos = DoubleSupplier { drive.localizer.getPose().position.x }
                inPerTick = MecanumDrive.PARAMS.inPerTick
                driveName = "MecanumDrive"
            }
            TankDrive::class.java -> {
                val drive =
                    TankDrive.forMarsLinear(hardwareMap, Pose2d(0.0, 0.0, 0.0)) {
                        batteryVoltage()
                    }
                val allMotors = drive.leftMotors + drive.rightMotors
                setPower = DoubleConsumer { p -> allMotors.forEach { it.power = p } }
                forwardVel = DoubleSupplier { drive.localizer.update().linearVel.x }
                axisPos = DoubleSupplier { drive.localizer.getPose().position.x }
                inPerTick = TankDrive.PARAMS.inPerTick
                driveName = "TankDrive"
            }
            else -> error("Unknown TuningOpModes.DRIVE_CLASS: ${TuningOpModes.DRIVE_CLASS}")
        }

        var sampleLog: CsvLogger? = null
        var metaLog: CsvLogger? = null
        if (LOG_CSV) {
            val stamp = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(Date())
            sampleLog =
                CsvLogger("axial_ff_samples_$stamp.csv", ReversalFeedforwardId.SAMPLE_HEADER)
            metaLog = CsvLogger("axial_ff_meta_$stamp.csv", ReversalFeedforwardId.META_HEADER)
            // Config only — fit outputs are recomputed from samples offline.
            metaLog.row(
                driveName,
                "axial",
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
            )
            metaLog.flush()
        }

        telemetry.addLine("Axial feedforward tuner (identifies kS, kV, kA).")
        telemetry.addLine("Phase 1: ramps FORWARD (+x) → kS, kV (like ForwardRampLogger).")
        telemetry.addLine("  Press gamepad1 A just before wall/mat edge (recommended).")
        telemetry.addLine("  Auto-stops if stalled against a wall (wheels may jam).")
        telemetry.addLine("Phase 2: reverse square wave → kA (first goes BACKWARD).")
        telemetry.addLine("  Half-cycles use the distance driven on the ramp when possible.")
        telemetry.addLine("Press START. Leave room BEHIND the start for reverse.")
        telemetry.addLine("Localization must be sign-correct (forward → +x).")
        sampleLog?.let {
            telemetry.addData("sample log", it.fileName())
            telemetry.addData("meta log", metaLog!!.fileName())
            telemetry.addLine("Pull with: telemetry/pull.sh")
        }
        telemetry.update()
        waitForStart()
        if (isStopRequested) {
            closeLogs(sampleLog, metaLog)
            return
        }

        val fit =
            ReversalFeedforwardId.identify(
                this,
                telemetry,
                setPower,
                forwardVel,
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
            writeAxialParams(driveName, fit.kS, fit.kV, fit.kA, true)
        } else if (
            fit.rampSamples > 0 &&
                fit.kS.isFinite() &&
                fit.kS != 0.0 &&
                fit.kV.isFinite() &&
                fit.kV > 0
        ) {
            // Unblocks TrackWidthTuner / YawCoupling; leave kA untouched if reverse phase failed.
            writeAxialParams(driveName, fit.kS, fit.kV, Double.NaN, false)
            wroteRampOnly = true
        }

        closeLogs(sampleLog, metaLog)

        while (nextFrame()) {
            if (fit.singular) {
                telemetry.addLine("Fit failed: " + fit.message)
                telemetry.addData("samples", fit.samples)
                telemetry.addData("ramp samples used", fit.rampSamples)
                if (wroteRampOnly) {
                    telemetry.addLine("Ramp-only kS/kV written to live PARAMS (kA not updated):")
                    telemetry.addData("kS", "%.5f", fit.kS)
                    telemetry.addData("kV", "%.5e", fit.kV)
                    telemetry.addData("ramp R^2", "%.3f", fit.rampR2)
                    telemetry.addLine("Still paste into source to keep after restart.")
                } else if (fit.rampSamples > 0 && fit.kS.isFinite() && fit.kS != 0.0) {
                    telemetry.addLine("Ramp-only (kA not trusted; not written — kV unusable):")
                    telemetry.addData("kS", "%.5f", fit.kS)
                    telemetry.addData("kV", "%.5e", fit.kV)
                    telemetry.addData("ramp R^2", "%.3f", fit.rampR2)
                }
            } else {
                telemetry.addLine("=== Written to live $driveName.PARAMS ===")
                telemetry.addData("kS", "%.5f", fit.kS)
                telemetry.addData("kV", "%.5e", fit.kV)
                telemetry.addData("kA", "%.5e", fit.kA)
                telemetry.addLine()
                telemetry.addData("samples", fit.samples)
                telemetry.addData("ramp samples used", fit.rampSamples)
                telemetry.addData("ramp R^2", "%.3f", fit.rampR2)
                telemetry.addLine("Live for later OpModes this session. Paste into source to keep.")
                telemetry.addLine(
                    "kS/kV should match ForwardRampLogger closely; kA is the new piece."
                )
            }
            if (LOG_CSV) {
                telemetry.addLine("CSVs on hub under /sdcard/FIRST/ — pull with telemetry/pull.sh")
            }
        }
    }
}
