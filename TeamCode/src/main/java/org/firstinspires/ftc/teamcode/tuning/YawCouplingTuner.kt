package org.firstinspires.ftc.teamcode.tuning

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit
import org.firstinspires.ftc.teamcode.Localizer
import org.firstinspires.ftc.teamcode.MecanumDrive
import org.firstinspires.ftc.teamcode.PinpointLocalizer
import org.firstinspires.ftc.teamcode.TankDrive
import org.firstinspires.ftc.teamcode.opmodes.base.MarsLinearOpMode
import org.firstinspires.ftc.teamcode.utils.CsvLogger
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.function.DoubleSupplier
import kotlin.math.abs
import kotlin.math.max

/**
 * Automatic on-robot identification of the `yawCoupling*` feedforward constants — the parasitic yaw
 * (curl) a robot picks up when driving straight open-loop.
 *
 * Unlike the stock ramp loggers (which record data for the offline tuning site), this OpMode drives
 * the ramp, fits the constants on the robot, writes them into the live `PARAMS` statics, and prints
 * values to paste into source for persistence across restart or redeploy.
 *
 * How it works: it drives an open-loop power ramp (feedforward off, no heading correction) and, for
 * each sample above [MIN_SPEED], records the chassis velocity (from the localizer) against the yaw
 * rate. It regresses yaw rate `ω = a + b·v`, then converts to the yaw-mode voltage that cancels it
 * using the small-signal yaw plant gain `kV·trackWidth`: `kS = −kV·trackWidth·a`, `kV_yaw =
 * −kV·trackWidth·b`. Mecanum additionally runs a strafe ramp for the lateral constants; tank uses
 * the axial pair only.
 *
 * Yaw rate source: Pinpoint heading velocity when available (preferred — independent of hub mount).
 * Otherwise Control Hub IMU angular velocity, always read as deg/s and converted to rad/s
 * (FtcRobotController#1070), using the axis of largest |rate| during the ramp so a misconfigured
 * hub orientation still yields a signed spin rate.
 *
 * Fit note: curl is often a weak signal (small ω over IMU/localizer noise). This tuner uses a plain
 * least-squares fit over samples above [MIN_SPEED] rather than [TunerRegression.robustFit]'s 0.95
 * R² chase, which was designed for strong V-vs-v feedforward ramps and can discard most points
 * without ever getting a high R².
 *
 * Prerequisites: localization, drive feedforward (`kS`/`kV`/`kA`), and a sane `trackWidthTicks` must
 * already be tuned. Give the robot a clear straight lane of roughly 6–8 ft in each direction it will
 * ramp (forward, and sideways on mecanum). Press gamepad1 A during a ramp to cut power early (e.g.
 * before a wall); the fit uses whatever samples were collected.
 *
 * Dashboard knobs: raise [MAX_POWER] / [RAMP_TIME] if the robot barely moves or R² stays near zero
 * with a short speed range.
 *
 * Each run writes two CSVs under `/sdcard/FIRST/` (via [CsvLogger]):
 * - `yaw_coupling_samples_<stamp>.csv` — raw ramp measurements (command, chassis velocity,
 *   yaw-source rate, localizer pose/angVel)
 * - `yaw_coupling_meta_<stamp>.csv` — plant scale and OpMode knobs needed to re-fit offline (not
 *   the fitted yawCoupling* values)
 *
 * Pull them with `telemetry/pull.sh`.
 */
@Config
class YawCouplingTuner : MarsLinearOpMode() {
    companion object {
        /** When false, skip writing CSVs (useful if the hub disk is full). */
        @JvmField
        var LOG_CSV = true

        /** Flush the sample CSV to disk after this many buffered rows. */
        @JvmField
        var LOG_FLUSH_EVERY = 256

        /**
         * Peak open-loop power at the end of the ramp. 0.4 was too gentle for many mecanum bases
         * (only a couple feet of travel); 0.65 needs ~6–8 ft of clear lane but gives a usable speed
         * span.
         */
        @JvmField
        var MAX_POWER = 0.65

        /** Seconds spent ramping power from 0 to [MAX_POWER]. */
        @JvmField
        var RAMP_TIME = 5.0

        /** Samples slower than this (in/s) are ignored as start-up transient / noise. */
        @JvmField
        var MIN_SPEED = 5.0

        /** R² below this is flagged as low-confidence (curl may be tiny or yaw measurement noisy). */
        @JvmField
        var WARN_R2 = 0.4

        /**
         * Raw ramp measurements. `omega_src` is the yaw rate used by the fit (Pinpoint or hub IMU);
         * `omega_loc` is localizer angVel for a second independent trace. Applied voltage is not
         * part of the curl model — power is logged only as the open-loop command.
         */
        private const val SAMPLE_HEADER =
            "t_s,phase,power,vel_in_s,omega_src_rad_s,omega_loc_rad_s,pose_x,pose_y,pose_h"

        /**
         * One-row (or few-row) run config for offline re-fit. `factor = -kV_wheel * track_width`
         * converts ω=a+b·v into coupling kS/kV — log the plant pieces, not the fit.
         */
        private const val META_HEADER =
            "drive,yaw_source,kV_wheel,track_width,in_per_tick,kV_params,max_power,ramp_time,min_speed"

        private fun requireCalibrated(kVWheel: Double, trackWidth: Double) {
            if (kVWheel <= 0 || trackWidth <= 0) {
                throw RuntimeException(
                    "Tune the drive feedforward (kV) and track width before running" +
                        " YawCouplingTuner",
                )
            }
        }

        private fun yawSourceLabel(localizer: Localizer): String {
            return if (localizer is PinpointLocalizer) {
                "Pinpoint heading velocity"
            } else {
                "Hub IMU (deg/s→rad, dominant axis)"
            }
        }

        /**
         * Prefer Pinpoint heading rate. Else hub IMU with deg→rad conversion; during the ramp the
         * caller still records a single omega via [readHubYawRate], which picks the dominant axis so
         * hub orientation misconfiguration does not zero out the signal.
         */
        private fun makeYawRateSource(localizer: Localizer, imu: IMU): DoubleSupplier {
            if (localizer is PinpointLocalizer) {
                val pl = localizer
                return DoubleSupplier {
                    pl.driver.update()
                    pl.driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS)
                }
            }
            // Stateful dominant-axis estimate shared across reads in one OpMode run.
            val axis = doubleArrayOf(0.0, 0.0, 1.0) // default Z until enough samples accumulate
            val sum = doubleArrayOf(0.0, 0.0, 0.0) // Σ |ω_i| to pick dominant; sign from that axis
            val n = intArrayOf(0)
            return DoubleSupplier { readHubYawRate(imu, axis, sum, n) }
        }

        /**
         * Hub IMU yaw rate in rad/s. Always converts from degrees (SDK unit bug). After a few
         * samples, locks onto the axis with largest mean |rate| (field-vertical spin under wrong
         * orientation).
         */
        private fun readHubYawRate(
            imu: IMU,
            axis: DoubleArray,
            sumAbs: DoubleArray,
            n: IntArray,
        ): Double {
            val avDeg: AngularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES)
            val wx = Math.toRadians(avDeg.xRotationRate.toDouble())
            val wy = Math.toRadians(avDeg.yRotationRate.toDouble())
            val wz = Math.toRadians(avDeg.zRotationRate.toDouble())

            // Early samples: build which axis is active during pure translation (curl is small, so
            // this mainly matters if orientation dumps true yaw onto X/Y when the robot *does*
            // spin). For open-loop straight drive, curl is the signal — use Z unless another axis is
            // clearly larger in magnitude over time.
            sumAbs[0] += abs(wx)
            sumAbs[1] += abs(wy)
            sumAbs[2] += abs(wz)
            n[0]++
            if (n[0] >= 10) {
                if (sumAbs[0] >= sumAbs[1] && sumAbs[0] >= sumAbs[2]) {
                    axis[0] = 1.0
                    axis[1] = 0.0
                    axis[2] = 0.0
                } else if (sumAbs[1] >= sumAbs[0] && sumAbs[1] >= sumAbs[2]) {
                    axis[0] = 0.0
                    axis[1] = 1.0
                    axis[2] = 0.0
                } else {
                    axis[0] = 0.0
                    axis[1] = 0.0
                    axis[2] = 1.0
                }
            }
            return wx * axis[0] + wy * axis[1] + wz * axis[2]
        }

        /** Plain OLS on (v, ω) — better for weak curl than chasing R² ≥ 0.95. */
        private fun fitCurl(samples: List<DoubleArray>): TunerRegression.Result {
            if (samples.size < 2) {
                return TunerRegression.Result(0.0, 0.0, 0.0, 0.0, 0, samples.size)
            }
            return TunerRegression.fitAll(samples)
        }

        private fun addFitTelemetry(
            telemetry: Telemetry,
            label: String,
            fit: TunerRegression.Result,
            ramp: RampResult,
        ) {
            telemetry.addData(
                "$label fit",
                "R^2 %.3f  a=%.4f  b=%.5f  (%d pts)",
                fit.r2,
                fit.intercept,
                fit.slope,
                fit.used,
            )
            telemetry.addData(
                "$label speed",
                "peak %.1f in/s, travel ~%.0f in",
                ramp.peakSpeed,
                ramp.travelIn,
            )
            if (ramp.peakSpeed < MIN_SPEED * 2) {
                telemetry.addData(
                    "$label warn",
                    "peak speed low — raise MAX_POWER or free the lane",
                )
            }
        }

        private fun setMecanumPowers(
            drive: MecanumDrive,
            lf: Double,
            lb: Double,
            rb: Double,
            rf: Double,
        ) {
            drive.leftFront.power = lf
            drive.leftBack.power = lb
            drive.rightBack.power = rb
            drive.rightFront.power = rf
        }
    }

    private var sampleLog: CsvLogger? = null
    private var metaLog: CsvLogger? = null

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        initRobot()
        // MarsLinearOpMode already wraps DS + Dashboard in GatedTelemetry — use Telemetry, do not
        // cast to MultipleTelemetry.
        val telem: Telemetry = telemetry

        if (TuningOpModes.DRIVE_CLASS == MecanumDrive::class.java) {
            val drive =
                MecanumDrive.forMarsLinear(hardwareMap, Pose2d(0.0, 0.0, 0.0), this::batteryVoltage)
            val kVWheel = MecanumDrive.PARAMS.kV / MecanumDrive.PARAMS.inPerTick
            val trackWidth = drive.kinematics.trackWidth
            requireCalibrated(kVWheel, trackWidth)

            val yawRate = makeYawRateSource(drive.localizer, drive.lazyImu.get())
            val yawSourceName = yawSourceLabel(drive.localizer)

            openLogs(
                "MecanumDrive",
                yawSourceName,
                kVWheel,
                trackWidth,
                MecanumDrive.PARAMS.inPerTick,
                MecanumDrive.PARAMS.kV,
            )

            telem.addLine("Mecanum yaw-coupling tuner.")
            telem.addData("yaw source", yawSourceName)
            telem.addData(
                "ramp",
                "%.0f%% power over %.1fs (need ~6-8 ft clear)",
                MAX_POWER * 100,
                RAMP_TIME,
            )
            telem.addLine("Press START, then the robot ramps FORWARD, stops, then ramps SIDEWAYS.")
            telem.addLine("Press gamepad1 A during a ramp to stop early (e.g. before a wall).")
            telem.addLine("Make sure both lanes are clear.")
            if (sampleLog != null) {
                telem.addData("sample log", sampleLog!!.fileName())
                telem.addData("meta log", metaLog!!.fileName())
                telem.addLine("Pull with: telemetry/pull.sh")
            }
            telem.update()
            waitForStart()
            if (isStopRequested) {
                closeLogs()
                return
            }

            // --- forward ramp -> axial constants ---
            val forward = rampAndSample(drive, yawRate, true, telem, "FORWARD")
            val fwd = fitCurl(forward.samples)

            // --- strafe ramp -> lateral constants ---
            val strafe = rampAndSample(drive, yawRate, false, telem, "SIDEWAYS")
            val lat = fitCurl(strafe.samples)

            val factor = -kVWheel * trackWidth
            val kSAxial = factor * fwd.intercept
            val kVAxial = factor * fwd.slope
            val kSLateral = factor * lat.intercept
            val kVLateral = factor * lat.slope

            MecanumDrive.PARAMS.yawCouplingKsAxial = kSAxial
            MecanumDrive.PARAMS.yawCouplingKvAxial = kVAxial
            MecanumDrive.PARAMS.yawCouplingKsLateral = kSLateral
            MecanumDrive.PARAMS.yawCouplingKvLateral = kVLateral

            closeLogs()

            while (nextFrame()) {
                telemetry.addLine("=== Written to live MecanumDrive.PARAMS ===")
                telemetry.addData("yawCouplingKsAxial", "%.5f", kSAxial)
                telemetry.addData("yawCouplingKvAxial", "%.5e", kVAxial)
                telemetry.addData("yawCouplingKsLateral", "%.5f", kSLateral)
                telemetry.addData("yawCouplingKvLateral", "%.5e", kVLateral)
                telemetry.addLine()
                telemetry.addData("yaw source", yawSourceName)
                addFitTelemetry(telem, "forward", fwd, forward)
                addFitTelemetry(telem, "strafe", lat, strafe)
                if (LOG_CSV) {
                    telemetry.addLine(
                        "CSVs on hub under /sdcard/FIRST/ — pull with telemetry/pull.sh",
                    )
                }
                telemetry.addLine("Live for later OpModes this session. Paste into source to keep.")
                telemetry.addLine("If a re-test shows the curl got WORSE, negate that pair.")
                if (fwd.r2 < WARN_R2 || lat.r2 < WARN_R2) {
                    telemetry.addLine()
                    telemetry.addLine(
                        "Low R^2 is common when curl is small (noise >> signal). Near-zero" +
                            " constants are fine. If curl is visibly large, raise" +
                            " MAX_POWER/RAMP_TIME or check yaw source.",
                    )
                }
            }
        } else if (TuningOpModes.DRIVE_CLASS == TankDrive::class.java) {
            val drive =
                TankDrive.forMarsLinear(hardwareMap, Pose2d(0.0, 0.0, 0.0), this::batteryVoltage)
            val kVWheel = TankDrive.PARAMS.kV / TankDrive.PARAMS.inPerTick
            val trackWidth = drive.kinematics.trackWidth
            requireCalibrated(kVWheel, trackWidth)

            val yawRate = makeYawRateSource(drive.localizer, drive.lazyImu.get())
            val yawSourceName = yawSourceLabel(drive.localizer)

            openLogs(
                "TankDrive",
                yawSourceName,
                kVWheel,
                trackWidth,
                TankDrive.PARAMS.inPerTick,
                TankDrive.PARAMS.kV,
            )

            telem.addLine("Tank yaw-coupling tuner.")
            telem.addData("yaw source", yawSourceName)
            telem.addData("ramp", "%.0f%% power over %.1fs", MAX_POWER * 100, RAMP_TIME)
            telem.addLine(
                "Press START, then the robot ramps FORWARD. Make sure the lane is clear.",
            )
            telem.addLine("Press gamepad1 A during the ramp to stop early (e.g. before a wall).")
            if (sampleLog != null) {
                telem.addData("sample log", sampleLog!!.fileName())
                telem.addData("meta log", metaLog!!.fileName())
                telem.addLine("Pull with: telemetry/pull.sh")
            }
            telem.update()
            waitForStart()
            if (isStopRequested) {
                closeLogs()
                return
            }

            val forward = rampAndSampleTank(drive, yawRate, telem)
            val fwd = fitCurl(forward.samples)

            val factor = -kVWheel * trackWidth
            val kSAxial = factor * fwd.intercept
            val kVAxial = factor * fwd.slope

            TankDrive.PARAMS.yawCouplingKsAxial = kSAxial
            TankDrive.PARAMS.yawCouplingKvAxial = kVAxial

            closeLogs()

            while (nextFrame()) {
                telemetry.addLine("=== Written to live TankDrive.PARAMS ===")
                telemetry.addData("yawCouplingKsAxial", "%.5f", kSAxial)
                telemetry.addData("yawCouplingKvAxial", "%.5e", kVAxial)
                telemetry.addLine()
                telemetry.addData("yaw source", yawSourceName)
                addFitTelemetry(telem, "forward", fwd, forward)
                if (LOG_CSV) {
                    telemetry.addLine(
                        "CSVs on hub under /sdcard/FIRST/ — pull with telemetry/pull.sh",
                    )
                }
                telemetry.addLine("Live for later OpModes this session. Paste into source to keep.")
                telemetry.addLine("If a re-test shows the curl got WORSE, negate the pair.")
                if (fwd.r2 < WARN_R2) {
                    telemetry.addLine()
                    telemetry.addLine(
                        "Low R^2 is common when curl is small. Near-zero constants are fine.",
                    )
                }
            }
        } else {
            throw RuntimeException("Unknown DRIVE_CLASS")
        }
    }

    private fun openLogs(
        drive: String,
        yawSource: String,
        kVWheel: Double,
        trackWidth: Double,
        inPerTick: Double,
        kVParams: Double,
    ) {
        if (!LOG_CSV) {
            return
        }
        val stamp = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(Date())
        sampleLog = CsvLogger("yaw_coupling_samples_$stamp.csv", SAMPLE_HEADER)
        metaLog = CsvLogger("yaw_coupling_meta_$stamp.csv", META_HEADER)
        metaLog!!.row(
            drive,
            yawSource,
            kVWheel,
            trackWidth,
            inPerTick,
            kVParams,
            MAX_POWER,
            RAMP_TIME,
            MIN_SPEED,
        )
        metaLog!!.flush()
    }

    private fun closeLogs() {
        sampleLog?.close()
        sampleLog = null
        metaLog?.close()
        metaLog = null
    }

    private fun maybeFlushSamples() {
        val log = sampleLog
        if (log != null && log.bufferedRows() >= LOG_FLUSH_EVERY) {
            log.flush()
        }
    }

    /**
     * Ramps the mecanum drive open-loop (forward or sideways) and returns (velocity, yawRate)
     * samples. Press gamepad1 A to end the ramp early; edge-detect so a held A from before START
     * does not fire.
     */
    private fun rampAndSample(
        drive: MecanumDrive,
        yawRate: DoubleSupplier,
        forward: Boolean,
        telemetry: Telemetry,
        label: String,
    ): RampResult {
        val samples = ArrayList<DoubleArray>()
        val timer = ElapsedTime()
        var stoppedByDriver = false
        var peakSpeed = 0.0
        var travel = 0.0
        var lastT = 0.0

        // Fresh pose for travel estimate
        drive.localizer.setPose(Pose2d(0.0, 0.0, 0.0))

        while (nextFrame() && timer.seconds() < RAMP_TIME) {
            if (gamepad1.aWasPressed()) {
                stoppedByDriver = true
                break
            }

            val power = MAX_POWER * timer.seconds() / RAMP_TIME
            if (forward) {
                setMecanumPowers(drive, power, power, power, power)
            } else {
                // strafe pattern (lf, lb, rb, rf) = (-, +, -, +) drives +y (robot-left)
                setMecanumPowers(drive, -power, power, -power, power)
            }

            val vel: PoseVelocity2d = drive.localizer.update()
            val v = if (forward) vel.linearVel.x else vel.linearVel.y
            val omegaSrc = yawRate.asDouble
            val t = timer.seconds()
            val dt = t - lastT
            if (dt > 1e-6 && dt < 0.2) {
                travel += abs(v) * dt
            }
            lastT = t
            peakSpeed = max(peakSpeed, abs(v))

            if (abs(v) > MIN_SPEED) {
                samples.add(doubleArrayOf(v, omegaSrc))
            }
            if (sampleLog != null) {
                val pose = drive.localizer.getPose()
                sampleLog!!.row(
                    t,
                    label,
                    power,
                    v,
                    omegaSrc,
                    vel.angVel,
                    pose.position.x,
                    pose.position.y,
                    pose.heading.toDouble(),
                )
                maybeFlushSamples()
            }

            telemetry.addData("phase", label)
            telemetry.addData("power", "%.2f", power)
            telemetry.addData("speed (in/s)", "%.1f", v)
            telemetry.addData("yaw rate (rad/s)", "%.4f", omegaSrc)
            telemetry.addData("travel (in)", "%.0f", travel)
            telemetry.addData("samples", samples.size)
            telemetry.addLine("A = stop ramp early")
        }
        setMecanumPowers(drive, 0.0, 0.0, 0.0, 0.0)
        sampleLog?.flush()
        // brief coast so the next ramp starts from rest
        val settle = ElapsedTime()
        while (nextFrame() && settle.seconds() < 1.0) {
            drive.localizer.update()
            telemetry.addData(
                "phase",
                if (stoppedByDriver) "$label (A stop) settling" else "$label settling",
            )
            telemetry.addData("samples kept", samples.size)
            telemetry.addData("peak speed", "%.1f in/s", peakSpeed)
        }
        return RampResult(samples, peakSpeed, travel)
    }

    private fun rampAndSampleTank(
        drive: TankDrive,
        yawRate: DoubleSupplier,
        telemetry: Telemetry,
    ): RampResult {
        val samples = ArrayList<DoubleArray>()
        val timer = ElapsedTime()
        var peakSpeed = 0.0
        var travel = 0.0
        var lastT = 0.0
        drive.localizer.setPose(Pose2d(0.0, 0.0, 0.0))

        while (nextFrame() && timer.seconds() < RAMP_TIME) {
            if (gamepad1.aWasPressed()) {
                break
            }

            val power = MAX_POWER * timer.seconds() / RAMP_TIME
            for (m in drive.leftMotors) m.power = power
            for (m in drive.rightMotors) m.power = power

            val vel: PoseVelocity2d = drive.localizer.update()
            val v = vel.linearVel.x
            val omegaSrc = yawRate.asDouble
            val t = timer.seconds()
            val dt = t - lastT
            if (dt > 1e-6 && dt < 0.2) {
                travel += abs(v) * dt
            }
            lastT = t
            peakSpeed = max(peakSpeed, abs(v))

            if (abs(v) > MIN_SPEED) {
                samples.add(doubleArrayOf(v, omegaSrc))
            }
            if (sampleLog != null) {
                val pose = drive.localizer.getPose()
                sampleLog!!.row(
                    t,
                    "FORWARD",
                    power,
                    v,
                    omegaSrc,
                    vel.angVel,
                    pose.position.x,
                    pose.position.y,
                    pose.heading.toDouble(),
                )
                maybeFlushSamples()
            }

            telemetry.addData("phase", "FORWARD")
            telemetry.addData("power", "%.2f", power)
            telemetry.addData("speed (in/s)", "%.1f", v)
            telemetry.addData("yaw rate (rad/s)", "%.4f", omegaSrc)
            telemetry.addData("travel (in)", "%.0f", travel)
            telemetry.addData("samples", samples.size)
            telemetry.addLine("A = stop ramp early")
        }
        for (m in drive.leftMotors) m.power = 0.0
        for (m in drive.rightMotors) m.power = 0.0
        sampleLog?.flush()
        return RampResult(samples, peakSpeed, travel)
    }

    private class RampResult(
        @JvmField val samples: List<DoubleArray>,
        @JvmField val peakSpeed: Double,
        @JvmField val travelIn: Double,
    )
}
