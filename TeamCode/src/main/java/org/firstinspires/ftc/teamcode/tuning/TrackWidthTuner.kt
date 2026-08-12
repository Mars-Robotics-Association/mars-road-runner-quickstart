package org.firstinspires.ftc.teamcode.tuning

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.DualNum
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2dDual
import com.acmerobotics.roadrunner.Time
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.Vector2dDual
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.util.ElapsedTime
import java.util.function.Consumer
import java.util.function.DoubleSupplier
import kotlin.math.abs
import kotlin.math.hypot
import kotlin.math.ln
import kotlin.math.sign
import kotlin.math.sqrt
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit
import org.firstinspires.ftc.teamcode.Localizer
import org.firstinspires.ftc.teamcode.MecanumDrive
import org.firstinspires.ftc.teamcode.PinpointLocalizer
import org.firstinspires.ftc.teamcode.TankDrive
import org.firstinspires.ftc.teamcode.opmodes.base.MarsLinearOpMode

/**
 * Automatic on-robot correction of `trackWidthTicks` — the effective track width used by the
 * kinematics to convert angular velocity into wheel speeds.
 *
 * Intended primarily for Pinpoint / OTOS setups, where `AngularRampLogger` cannot measure the track
 * width (the regression needs drive encoders in the tuning view). On any setup it also serves as a
 * quick verification of an already-tuned value.
 *
 * How it works: it spins in place, ramping a commanded angular velocity through the production
 * `setDriveCommand` feedforward path — which converts `ω` to wheel speeds using the *current*
 * `trackWidthTicks`. If that value is wrong by a factor, the actual yaw rate is wrong by the
 * inverse factor: `ω_actual = ω_commanded · W_param / W_true`. The tuner ramps counterclockwise
 * then clockwise, regresses actual yaw rate against commanded yaw rate, and reports
 * `trackWidthTicks / slope` as the corrected value.
 *
 * Yaw measurement:
 * - **Pinpoint** (when the drive localizer is a [PinpointLocalizer]): uses onboard heading velocity
 *   — independent of Control Hub mount orientation.
 * - **Hub IMU** otherwise: does *not* assume Z is yaw. Each sample records the full
 *   angular-velocity vector (deg/s → rad/s; see FtcRobotController#1070) and a rate from
 *   differentiating unwrapped IMU yaw. After both ramps, spin axis `û ∝ Σ ω_cmd · ω` is estimated
 *   from the data and rates are projected as `ω · û`. Differentiated yaw is also fit; whichever
 *   candidate has the more plausible slope and better R² wins. A pure in-place spin is always about
 *   field-vertical — if hub orientation is misconfigured that appears on X/Y instead of Z, and the
 *   projection still recovers the signed rate.
 *
 * Prerequisites: the drive feedforward (`kS`/`kV`, ideally `kA`) must be tuned — the spin is driven
 * open-loop through it. `trackWidthTicks` may be left at 0; the tuner then seeds a nominal
 * geometric track width ([DEFAULT_TRACK_WIDTH_IN] / `inPerTick`) so the multiplicative fit can run.
 * A tape-measure value is still fine if you have one. On a successful fit, writes the corrected
 * `trackWidthTicks` into the live `PARAMS` statics so later OpModes in the same RC process can
 * chain without a paste. Re-run to verify: the slope should come out ≈ 1.00. Paste into source to
 * keep values across restart or redeploy.
 */
@Config
class TrackWidthTuner : MarsLinearOpMode() {
    companion object {
        /** Peak commanded angular velocity at the end of each ramp, in rad/s. */
        @JvmField var MAX_ANG_VEL = Math.PI

        /** Seconds spent ramping the commanded angular velocity from 0 to [MAX_ANG_VEL]. */
        @JvmField var RAMP_TIME = 3.0

        /**
         * Samples with commanded angular velocity below this (rad/s) are ignored as start-up
         * transient.
         */
        @JvmField var MIN_ANG_VEL = 0.5

        /**
         * Nominal geometric track width in inches used when `Params.trackWidthTicks` is unset (`<=
         * 0`). Converted to ticks via `/ inPerTick`. The robot size limit is 18 in; 16 in leaves
         * margin under that for a typical wheel-center track. Mecanum effective width often comes
         * out somewhat larger after correction because of roller scrub.
         */
        @JvmField var DEFAULT_TRACK_WIDTH_IN = 16.0

        private fun requireFeedforward(kV: Double) {
            require(kV > 0) {
                "Tune the drive feedforward (kS/kV) before running TrackWidthTuner — the spin" +
                    " is driven through it."
            }
        }

        /**
         * Returns the seed for the multiplicative fit: the existing Params value when set,
         * otherwise [DEFAULT_TRACK_WIDTH_IN] / `inPerTick`.
         */
        private fun resolveTrackWidthTicks(trackWidthTicks: Double, inPerTick: Double): Double {
            if (trackWidthTicks > 0) {
                return trackWidthTicks
            }
            require(inPerTick > 0) {
                "Set inPerTick (ForwardPushTest) before TrackWidthTuner — needed to seed" +
                    " trackWidthTicks from $DEFAULT_TRACK_WIDTH_IN in."
            }
            return DEFAULT_TRACK_WIDTH_IN / inPerTick
        }

        /**
         * Builds (cmd, rate) series from raw samples and picks the best of the available
         * estimators.
         */
        private fun chooseFit(
            ccwRaw: List<RawSample>,
            cwRaw: List<RawSample>,
            usePinpoint: Boolean,
        ): FitChoice {
            if (usePinpoint) {
                return FitChoice(
                    fitDirect(ccwRaw),
                    fitDirect(cwRaw),
                    "Pinpoint heading velocity",
                    null,
                )
            }

            // Spin axis û ∝ Σ ω_cmd · ω over both ramps (sign of cmd flips with spin direction, so
            // products reinforce). Pure field-vertical spin maps to some fixed direction in the
            // (possibly misconfigured) robot frame.
            var sx = 0.0
            var sy = 0.0
            var sz = 0.0
            for (s in ccwRaw) {
                sx += s.omegaCmd * s.wx
                sy += s.omegaCmd * s.wy
                sz += s.omegaCmd * s.wz
            }
            for (s in cwRaw) {
                sx += s.omegaCmd * s.wx
                sy += s.omegaCmd * s.wy
                sz += s.omegaCmd * s.wz
            }
            val norm = sqrt(sx * sx + sy * sy + sz * sz)
            val ux: Double
            val uy: Double
            val uz: Double
            val axisNote: String
            if (norm > 1e-9) {
                ux = sx / norm
                uy = sy / norm
                uz = sz / norm
                axisNote = String.format("û=(%.2f, %.2f, %.2f)", ux, uy, uz)
            } else {
                // No correlation — fall back to Z (legacy behavior).
                ux = 0.0
                uy = 0.0
                uz = 1.0
                axisNote = "û=(0, 0, 1) fallback (no spin correlation)"
            }

            val ccwProj = fitProjected(ccwRaw, ux, uy, uz)
            val cwProj = fitProjected(cwRaw, ux, uy, uz)
            val ccwYaw = fitYawDiff(ccwRaw)
            val cwYaw = fitYawDiff(cwRaw)

            val scoreProj = scorePair(ccwProj, cwProj)
            val scoreYaw = scorePair(ccwYaw, cwYaw)

            if (scoreYaw > scoreProj) {
                return FitChoice(ccwYaw, cwYaw, "Hub IMU d(yaw)/dt", axisNote)
            }
            return FitChoice(ccwProj, cwProj, "Hub IMU ω·û (orientation-robust)", axisNote)
        }

        /**
         * Higher is better. Prefers slopes near 1 and high R²; strongly penalizes out-of-range
         * slopes.
         */
        private fun scorePair(
            a: TunerRegression.Result?,
            b: TunerRegression.Result?,
        ): Double {
            if (a == null || b == null) {
                return Double.NEGATIVE_INFINITY
            }
            val slope = 0.5 * (a.slope + b.slope)
            val r2 = 0.5 * (a.r2 + b.r2)
            if (!(slope > 0.05 && slope < 20)) {
                return Double.NEGATIVE_INFINITY
            }
            // Plausible band gets a bonus; then reward proximity to 1 and fit quality.
            val inBand = if (slope > 0.2 && slope < 5.0) 2.0 else 0.0
            val nearOne = -abs(ln(slope))
            return inBand + nearOne + r2
        }

        private fun fitDirect(raw: List<RawSample>): TunerRegression.Result {
            val pts = ArrayList<DoubleArray>(raw.size)
            for (s in raw) {
                pts.add(doubleArrayOf(s.omegaCmd, s.directRate))
            }
            return TunerRegression.robustFit(pts)!!
        }

        private fun fitProjected(
            raw: List<RawSample>,
            ux: Double,
            uy: Double,
            uz: Double,
        ): TunerRegression.Result {
            val pts = ArrayList<DoubleArray>(raw.size)
            for (s in raw) {
                pts.add(doubleArrayOf(s.omegaCmd, s.wx * ux + s.wy * uy + s.wz * uz))
            }
            return TunerRegression.robustFit(pts)!!
        }

        private fun fitYawDiff(raw: List<RawSample>): TunerRegression.Result {
            val pts = ArrayList<DoubleArray>(raw.size)
            for (s in raw) {
                if (s.yawRate.isNaN()) {
                    continue
                }
                pts.add(doubleArrayOf(s.omegaCmd, s.yawRate))
            }
            if (pts.size < 2) {
                // Degenerate — robustFit needs at least 2 points; return a zero-slope placeholder.
                pts.add(doubleArrayOf(0.0, 0.0))
                pts.add(doubleArrayOf(1.0, 0.0))
            }
            return TunerRegression.robustFit(pts)!!
        }
    }

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        initRobot()
        // MarsLinearOpMode already wraps DS + Dashboard in GatedTelemetry — use Telemetry, do not
        // cast to MultipleTelemetry.
        val telem: Telemetry = telemetry

        val setCommand: Consumer<PoseVelocity2dDual<Time>>
        val imu: IMU
        val localizer: Localizer
        val trackWidthTicks: Double
        val usedDefaultSeed: Boolean
        val paramsClass: String
        when (TuningOpModes.DRIVE_CLASS) {
            MecanumDrive::class.java -> {
                requireFeedforward(MecanumDrive.PARAMS.kV)
                usedDefaultSeed = MecanumDrive.PARAMS.trackWidthTicks <= 0
                // Kinematics bake trackWidthTicks at construction — seed Params first when unset.
                trackWidthTicks =
                    resolveTrackWidthTicks(
                        MecanumDrive.PARAMS.trackWidthTicks,
                        MecanumDrive.PARAMS.inPerTick,
                    )
                MecanumDrive.PARAMS.trackWidthTicks = trackWidthTicks
                val drive =
                    MecanumDrive.forMarsLinear(hardwareMap, Pose2d(0.0, 0.0, 0.0)) {
                        batteryVoltage()
                    }
                setCommand = Consumer { drive.setDriveCommand(it) }
                imu = drive.lazyImu.get()
                localizer = drive.localizer
                paramsClass = "MecanumDrive.Params"
            }
            TankDrive::class.java -> {
                requireFeedforward(TankDrive.PARAMS.kV)
                usedDefaultSeed = TankDrive.PARAMS.trackWidthTicks <= 0
                trackWidthTicks =
                    resolveTrackWidthTicks(
                        TankDrive.PARAMS.trackWidthTicks,
                        TankDrive.PARAMS.inPerTick,
                    )
                TankDrive.PARAMS.trackWidthTicks = trackWidthTicks
                val drive =
                    TankDrive.forMarsLinear(hardwareMap, Pose2d(0.0, 0.0, 0.0)) {
                        batteryVoltage()
                    }
                setCommand = Consumer { drive.setDriveCommand(it) }
                imu = drive.lazyImu.get()
                localizer = drive.localizer
                paramsClass = "TankDrive.Params"
            }
            else -> error("Unknown TuningOpModes.DRIVE_CLASS: ${TuningOpModes.DRIVE_CLASS}")
        }

        val pinpointRate: DoubleSupplier?
        val usePinpoint: Boolean
        if (localizer is PinpointLocalizer) {
            val pl = localizer
            usePinpoint = true
            pinpointRate = DoubleSupplier {
                pl.driver.update()
                pl.driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS)
            }
        } else {
            usePinpoint = false
            pinpointRate = null
        }

        telem.addLine("Track width tuner.")
        telem.addData(
            "yaw source",
            if (usePinpoint) "Pinpoint heading velocity" else "Hub IMU (axis projection + yaw)",
        )
        if (usedDefaultSeed) {
            telem.addData(
                "seed",
                "default %.1f in → trackWidthTicks=%.2f",
                DEFAULT_TRACK_WIDTH_IN,
                trackWidthTicks,
            )
        } else {
            telem.addData("seed", "Params trackWidthTicks=%.2f", trackWidthTicks)
        }
        telem.addLine(
            "Press START, then the robot spins in place: counterclockwise, then clockwise."
        )
        telem.addLine("Make sure it can rotate freely.")
        telem.update()
        waitForStart()
        if (isStopRequested) return

        val ccwRaw = rampAndSample(setCommand, imu, pinpointRate, +1, telem, "CCW")
        val cwRaw = rampAndSample(setCommand, imu, pinpointRate, -1, telem, "CW")

        val fit = chooseFit(ccwRaw, cwRaw, usePinpoint)
        val slope = 0.5 * (fit.ccw.slope + fit.cw.slope)
        val valid = slope > 0.2 && slope < 5.0
        val corrected = if (valid) trackWidthTicks / slope else Double.NaN

        // Kinematics bake trackWidthTicks at drive construction, so this OpMode's instance keeps
        // the seed; the next OpMode init picks up the corrected static.
        if (valid) {
            when (TuningOpModes.DRIVE_CLASS) {
                MecanumDrive::class.java -> MecanumDrive.PARAMS.trackWidthTicks = corrected
                TankDrive::class.java -> TankDrive.PARAMS.trackWidthTicks = corrected
            }
        }

        while (nextFrame()) {
            if (valid) {
                telemetry.addLine("=== Written to live $paramsClass ===")
                telemetry.addData("trackWidthTicks", "%.2f", corrected)
                telemetry.addLine()
                telemetry.addData(
                    "seed used",
                    "%.2f%s",
                    trackWidthTicks,
                    if (usedDefaultSeed) " (default geometric)" else "",
                )
                telemetry.addData("yaw source", fit.label)
                telemetry.addData("slope (actual/commanded)", "%.4f", slope)
                if (fit.axisNote != null) {
                    telemetry.addData("spin axis (robot frame)", fit.axisNote)
                }
                telemetry.addData(
                    "ccw fit",
                    "slope %.4f, R^2 %.4f (%d/%d pts)",
                    fit.ccw.slope,
                    fit.ccw.r2,
                    fit.ccw.used,
                    fit.ccw.total,
                )
                telemetry.addData(
                    "cw fit",
                    "slope %.4f, R^2 %.4f (%d/%d pts)",
                    fit.cw.slope,
                    fit.cw.r2,
                    fit.cw.used,
                    fit.cw.total,
                )
                telemetry.addLine("Live for later OpModes this session. Paste into source to keep.")
                telemetry.addLine("Re-run to verify: the slope should be ~1.00.")
            } else {
                telemetry.addLine(
                    "FAILED: fitted slope " +
                        String.format("%.3f", slope) +
                        " is not plausible (need 0.2–5.0)."
                )
                telemetry.addLine("Check motor directions and feedforward (kS/kV), then retry.")
                telemetry.addData("yaw source", fit.label)
                if (fit.axisNote != null) {
                    telemetry.addData("spin axis (robot frame)", fit.axisNote)
                }
                telemetry.addData(
                    "seed used",
                    "%.2f%s",
                    trackWidthTicks,
                    if (usedDefaultSeed) " (default geometric)" else "",
                )
                telemetry.addData(
                    "ccw fit",
                    "slope %.4f, R^2 %.4f (%d/%d pts)",
                    fit.ccw.slope,
                    fit.ccw.r2,
                    fit.ccw.used,
                    fit.ccw.total,
                )
                telemetry.addData(
                    "cw fit",
                    "slope %.4f, R^2 %.4f (%d/%d pts)",
                    fit.cw.slope,
                    fit.cw.r2,
                    fit.cw.used,
                    fit.cw.total,
                )
                if (slope >= 5.0) {
                    telemetry.addLine(
                        "Slope too high: actual yaw >> commanded. Check units / free spin."
                    )
                } else if (slope <= 0.2) {
                    telemetry.addLine(
                        "Slope too low: robot spun much slower than commanded. Watch" +
                            " commanded vs actual mid-ramp; check kS/kV and free spin."
                    )
                }
            }
        }
    }

    /**
     * Ramps commanded angular velocity through the production feedforward path. Each sample stores
     * enough IMU data to reconstruct yaw rate after both ramps (axis projection + differentiated
     * yaw), or a direct Pinpoint rate when `pinpointRate` is non-null.
     */
    private fun rampAndSample(
        setCommand: Consumer<PoseVelocity2dDual<Time>>,
        imu: IMU,
        pinpointRate: DoubleSupplier?,
        dir: Int,
        telemetry: Telemetry,
        label: String,
    ): List<RawSample> {
        val samples = ArrayList<RawSample>()
        val alpha = dir * MAX_ANG_VEL / RAMP_TIME
        val timer = ElapsedTime()
        var lastYaw = Double.NaN
        var lastT = Double.NaN

        while (nextFrame() && timer.seconds() < RAMP_TIME) {
            val t = timer.seconds()
            val omegaCmd = alpha * t
            setCommand.accept(
                PoseVelocity2dDual(
                    Vector2dDual.constant(Vector2d(0.0, 0.0), 3),
                    DualNum(doubleArrayOf(omegaCmd, alpha, 0.0)),
                )
            )

            val s = RawSample()
            s.omegaCmd = omegaCmd

            if (pinpointRate != null) {
                s.directRate = pinpointRate.asDouble
            } else {
                // Angular velocity: read deg/s and convert (SDK may ignore AngleUnit.RADIANS;
                // FtcRobotController#1070). Keep all three axes for orientation-robust projection.
                val avDeg: AngularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES)
                s.wx = Math.toRadians(avDeg.xRotationRate.toDouble())
                s.wy = Math.toRadians(avDeg.yRotationRate.toDouble())
                s.wz = Math.toRadians(avDeg.zRotationRate.toDouble())

                // Differentiated unwrapped yaw — correct when hub orientation is configured right;
                // may be near zero if orientation is wrong (spin appears as pitch/roll instead).
                val yaw = imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)
                if (!lastYaw.isNaN() && !lastT.isNaN()) {
                    val dt = t - lastT
                    if (dt > 1e-6) {
                        s.yawRate = AngleUnit.normalizeRadians(yaw - lastYaw) / dt
                    }
                }
                lastYaw = yaw
                lastT = t
            }

            if (abs(omegaCmd) > MIN_ANG_VEL) {
                samples.add(s)
            }

            val display =
                if (pinpointRate != null) {
                    s.directRate
                } else {
                    hypot(s.wx, hypot(s.wy, s.wz)) * sign(omegaCmd)
                }

            telemetry.addData("phase", label)
            telemetry.addData("commanded (rad/s)", "%.2f", omegaCmd)
            telemetry.addData("actual (rad/s)", "%.2f", display)
            if (pinpointRate == null) {
                telemetry.addData("ω xyz (rad/s)", "%.2f, %.2f, %.2f", s.wx, s.wy, s.wz)
                telemetry.addData("d(yaw)/dt (rad/s)", "%.2f", s.yawRate)
            }
            telemetry.addData("samples", samples.size)
        }
        setCommand.accept(
            PoseVelocity2dDual(
                Vector2dDual.constant(Vector2d(0.0, 0.0), 3),
                DualNum.constant(0.0, 3),
            )
        )
        // brief coast so the next ramp starts from rest
        val settle = ElapsedTime()
        while (nextFrame() && settle.seconds() < 1.0) {
            telemetry.addData("phase", "$label settling")
        }
        return samples
    }

    private class RawSample {
        var omegaCmd = 0.0
        var wx = 0.0
        var wy = 0.0
        var wz = 0.0
        var yawRate = Double.NaN
        var directRate = Double.NaN
    }

    private class FitChoice(
        @JvmField val ccw: TunerRegression.Result,
        @JvmField val cw: TunerRegression.Result,
        @JvmField val label: String,
        @JvmField val axisNote: String?,
    )
}
