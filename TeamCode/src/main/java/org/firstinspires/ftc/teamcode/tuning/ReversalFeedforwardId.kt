package org.firstinspires.ftc.teamcode.tuning

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.opmodes.base.MarsLinearOpMode
import org.firstinspires.ftc.teamcode.utils.CsvLogger
import java.util.function.DoubleConsumer
import java.util.function.DoubleSupplier
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sign

/**
 * Shared core for the feedforward tuners ([AxialFeedforwardTuner], [LateralFeedforwardTuner]).
 *
 * **Two maneuvers, two jobs.**
 *
 * 1. **Slow open-loop ramp** (same idea as `ForwardRampLogger`) — visits many speeds so `kS` and
 *    `kV` separate cleanly via `V = kS + kV·v`. High-speed-only cruise levels pin `kV` well but leave
 *    the intercept (`kS`) poorly leveraged: a few percent slope error shifts `kS` by tenths of a
 *    volt. Ends early if speed collapses under power (wall / hard stop) so the robot does not keep
 *    driving into an obstacle.
 * 2. **Reverse square wave** — first half-cycle is reverse (away from a forward wall). When the ramp
 *    measured a useful travel distance (e.g. start → wall), each half-cycle runs for that corridor
 *    rather than a fixed 1 s, so the kA phase uses the room the robot just proved is free. Then with
 *    `kS`/`kV` fixed, fit residual `∫(V − kS·sign(v) − kV·v) dt = kA·Δv` over short windows (no noisy
 *    `a = dv/dt`).
 *
 * Velocity is chassis axis speed in in/s from the localizer; axis position is inches along the same
 * axis. `power` and velocity must share a sign convention.
 *
 * Optional [CsvLogger] sample logging (header [SAMPLE_HEADER]): raw control-loop measurements during
 * ramp and reverse so the id can be re-run offline. Callers open/close the logger and write a
 * one-row meta file with run config ([META_HEADER]) — not fitted constants (those are recomputed
 * from samples).
 */
class ReversalFeedforwardId private constructor() {
    /** Fitted constants in tick units (kS in volts), matching `Params.kS/kV/kA`. */
    class Result(
        @JvmField val singular: Boolean,
        @JvmField val kS: Double,
        @JvmField val kV: Double,
        @JvmField val kA: Double,
        @JvmField val samples: Int,
        @JvmField val rampSamples: Int,
        /** Coefficient of determination for the ramp (kS/kV) fit; NaN if unavailable. */
        @JvmField val rampR2: Double,
        @JvmField val message: String,
    )

    /**
     * Soft limits for the reverse phase from measured ramp endpoints. Package-visible for unit
     * tests.
     */
    class Corridor(
        @JvmField val usable: Boolean,
        /** Position (in) near the ramp start — reverse half-cycles stop here. */
        @JvmField val nearStart: Double,
        /** Position (in) near the ramp end / wall — forward half-cycles stop here. */
        @JvmField val nearEnd: Double,
        /** `sign(posEnd - posStart)`; 0 if unusable. */
        @JvmField val travelSign: Double,
        @JvmField val travelIn: Double,
    )

    /** Package-visible for unit tests. */
    class RampFit(
        @JvmField val singular: Boolean,
        @JvmField val kS: Double,
        @JvmField val kVInch: Double,
        @JvmField val r2: Double,
        @JvmField val used: Int,
        @JvmField val message: String,
    )

    private class KaFit(
        @JvmField val singular: Boolean,
        @JvmField val kAInch: Double,
        @JvmField val message: String,
    )

    companion object {
        /**
         * Per-loop raw measurements. `phase` is `ramp` or `reverse`; `half_idx` is `-1` on the ramp
         * and 0-based on reverse half-cycles. Applied axis voltage is `power * battery_v`.
         * `pose_in` is the localizer axis reading when available (NaN otherwise) — not the
         * integrated ∫vel used for corridor logic.
         */
        @JvmField
        val SAMPLE_HEADER = "t_s,phase,half_idx,power,battery_v,vel_in_s,pose_in"

        /**
         * One-row run config for offline re-fit (plant scale + OpMode knobs). Not fit results —
         * recompute kS/kV/kA from the sample file.
         */
        @JvmField
        val META_HEADER =
            "drive,axis,in_per_tick,ramp_power_per_sec,ramp_max,ka_power,half_cycle,ka_half_cycles," +
                "sign_deadband,stall_speed,stall_time,moving_speed,stall_min_power,stall_frac," +
                "min_ramp_ticks_per_sec,end_margin_in,min_travel_in,arm_travel_in,pos_stall_speed," +
                "ka_window_s,ka_stride_s"

        /** Flush buffered sample rows after this many are queued. */
        @JvmField
        val LOG_FLUSH_EVERY = 256

        /** Window length (s) for residual kA integral equations. */
        @JvmField
        val DEFAULT_KA_WINDOW = 0.12

        /** Stride (s) between kA windows. */
        @JvmField
        val DEFAULT_KA_STRIDE = 0.04

        /** Seconds held at ramp peak before the reverse phase. */
        @JvmField
        val DEFAULT_RAMP_HOLD = 0.4

        /**
         * Absolute speed (in/s) below which a moving robot is treated as stalled against an
         * obstacle. Combined with [DEFAULT_STALL_FRAC] of peak speed (whichever is larger).
         */
        @JvmField
        val DEFAULT_STALL_SPEED = 2.0

        /** How long (s) velocity must stay collapsed before the ramp ends for a wall/stall. */
        @JvmField
        val DEFAULT_STALL_TIME = 0.15

        /**
         * Speed (in/s) the robot must have exceeded at least once before stall detection arms —
         * avoids ending the ramp during breakaway from rest.
         */
        @JvmField
        val DEFAULT_MOVING_SPEED = 5.0

        /** Commanded |power| must be at least this to count as stalled (ignore near-zero commands). */
        @JvmField
        val DEFAULT_STALL_MIN_POWER = 0.15

        /**
         * Velocity below this fraction of the ramp's peak speed also counts as collapsed (catches
         * wall hits where the robot still creeps a little).
         */
        @JvmField
        val DEFAULT_STALL_FRAC = 0.25

        /**
         * Ramp samples slower than this many encoder-tick units per second are excluded from the
         * kS/kV regression. Converts via `inPerTick` so it tracks wheel resolution. Cuts the
         * non-linear breakaway knee at the start of a forward ramp (~vertical cluster near v=0 on
         * the V-vs-v plot).
         */
        @JvmField
        val DEFAULT_MIN_RAMP_TICKS_PER_SEC = 1000.0

        /**
         * Keep this many inches clear of each end of the measured ramp corridor during the
         * position-based reverse phase (so half-cycles don't grind the wall or start edge).
         */
        @JvmField
        val DEFAULT_END_MARGIN_IN = 6.0

        /**
         * Minimum |ramp travel| (in) before the reverse phase switches from fixed-time half-cycles
         * to position-based half-cycles that use the full corridor.
         */
        @JvmField
        val DEFAULT_MIN_TRAVEL_IN = 18.0

        /**
         * |Travel| (in) from ramp start that arms stall detection even if peak speed never reached
         * [DEFAULT_MOVING_SPEED]. Important for slow lateral ramps that hit a wall early.
         */
        @JvmField
        val DEFAULT_ARM_TRAVEL_IN = 4.0

        /**
         * When stall is armed, |d(position)/dt| below this (in/s) under power counts as stalled —
         * catches wall contact when velocity is noisy or wheels keep slipping a little.
         */
        @JvmField
        val DEFAULT_POS_STALL_SPEED = 1.5

        /**
         * Run a slow ramp (kS/kV) then a reverse square wave (kA).
         *
         * The ramp ends at the usual duration *or* earlier if the robot stops moving while still
         * under power (e.g. hit a wall). Phase 2 always starts with reverse power. When `axisPos`
         * reports enough ramp travel, each reverse half-cycle uses that corridor (minus end
         * margins) instead of a fixed `halfCycle` duration.
         *
         * @param wheelVel axis velocity (in/s); should update the localizer when read
         * @param axisPos axis position (in) along the same axis; read after `wheelVel`
         * @param halfCycle fallback seconds per reverse direction when travel is too short for
         *     position-based half-cycles
         */
        @JvmStatic
        fun identify(
            opMode: MarsLinearOpMode,
            telemetry: Telemetry,
            setPower: DoubleConsumer,
            wheelVel: DoubleSupplier,
            axisPos: DoubleSupplier,
            inPerTick: Double,
            rampPowerPerSec: Double,
            rampMax: Double,
            kaPower: Double,
            halfCycle: Double,
            halfCycles: Int,
            signDeadband: Double,
            stallSpeed: Double,
            stallTime: Double,
            movingSpeed: Double,
            stallMinPower: Double,
            stallFrac: Double,
            minRampTicksPerSec: Double,
            endMarginIn: Double,
            minTravelIn: Double,
        ): Result {
            return identify(
                opMode,
                telemetry,
                setPower,
                wheelVel,
                axisPos,
                inPerTick,
                rampPowerPerSec,
                rampMax,
                kaPower,
                halfCycle,
                halfCycles,
                signDeadband,
                stallSpeed,
                stallTime,
                movingSpeed,
                stallMinPower,
                stallFrac,
                minRampTicksPerSec,
                endMarginIn,
                minTravelIn,
                DEFAULT_ARM_TRAVEL_IN,
                DEFAULT_POS_STALL_SPEED,
                null,
            )
        }

        @JvmStatic
        fun identify(
            opMode: MarsLinearOpMode,
            telemetry: Telemetry,
            setPower: DoubleConsumer,
            wheelVel: DoubleSupplier,
            axisPos: DoubleSupplier,
            inPerTick: Double,
            rampPowerPerSec: Double,
            rampMax: Double,
            kaPower: Double,
            halfCycle: Double,
            halfCycles: Int,
            signDeadband: Double,
            stallSpeed: Double,
            stallTime: Double,
            movingSpeed: Double,
            stallMinPower: Double,
            stallFrac: Double,
            minRampTicksPerSec: Double,
            endMarginIn: Double,
            minTravelIn: Double,
            armTravelIn: Double,
            posStallSpeed: Double,
        ): Result {
            return identify(
                opMode,
                telemetry,
                setPower,
                wheelVel,
                axisPos,
                inPerTick,
                rampPowerPerSec,
                rampMax,
                kaPower,
                halfCycle,
                halfCycles,
                signDeadband,
                stallSpeed,
                stallTime,
                movingSpeed,
                stallMinPower,
                stallFrac,
                minRampTicksPerSec,
                endMarginIn,
                minTravelIn,
                armTravelIn,
                posStallSpeed,
                null,
            )
        }

        /**
         * @param sampleLog optional per-loop sample logger ([SAMPLE_HEADER]); null skips CSV rows.
         *     Caller owns open/close and the meta file; this only appends raw sample rows and
         *     flushes periodically.
         */
        @JvmStatic
        fun identify(
            opMode: MarsLinearOpMode,
            telemetry: Telemetry,
            setPower: DoubleConsumer,
            wheelVel: DoubleSupplier,
            axisPos: DoubleSupplier,
            inPerTick: Double,
            rampPowerPerSec: Double,
            rampMax: Double,
            kaPower: Double,
            halfCycle: Double,
            halfCycles: Int,
            signDeadband: Double,
            stallSpeed: Double,
            stallTime: Double,
            movingSpeed: Double,
            stallMinPower: Double,
            stallFrac: Double,
            minRampTicksPerSec: Double,
            endMarginIn: Double,
            minTravelIn: Double,
            armTravelIn: Double,
            posStallSpeed: Double,
            sampleLog: CsvLogger?,
        ): Result {
            if (rampPowerPerSec <= 0 || rampMax <= 0) {
                return Result(true, 0.0, 0.0, 0.0, 0, 0, Double.NaN, "bad ramp settings")
            }
            if (kaPower <= 0 || halfCycle <= 0 || halfCycles < 2) {
                return Result(true, 0.0, 0.0, 0.0, 0, 0, Double.NaN, "bad reverse-phase settings")
            }

            val rampT = ArrayList<Double>()
            val rampV = ArrayList<Double>()
            val rampVel = ArrayList<Double>()
            val revT = ArrayList<Double>()
            val revV = ArrayList<Double>()
            val revVel = ArrayList<Double>()

            val timer = ElapsedTime()
            val rampDuration = rampMax / rampPowerPerSec + DEFAULT_RAMP_HOLD

            // --- Phase 1: slow forward ramp (ForwardRampLogger-style) ---
            // Ends on schedule, on gamepad1 A (manual — hit before the mat/wall trap), or if
            // motion collapses under power (automatic wall/hard-stop detect).
            //
            // Position for stall + corridor is ∫vel dt along the test axis — same signal as the fit.
            // World pose (axisPos) is only for telemetry: on lateral strafes, heading error makes
            // field-y a bad stand-in for robot-lateral travel and was skipping reverse half-cycles.
            var everMoving = false
            var peakVel = 0.0
            var stallAccum = 0.0
            var lastT = 0.0
            var stoppedForWall = false
            var stoppedByDriver = false
            var s = 0.0 // integrated axis displacement (in), same sign as wheelVel
            var prevS = 0.0
            val s0 = 0.0

            // Seed localizer before the ramp.
            wheelVel.asDouble

            while (opMode.nextFrame() && timer.seconds() < rampDuration) {
                // Manual end: press A just before the robot jams into the wall/mat lip so reverse
                // still has free wheels. Edge-detect so a held A from before START does not fire.
                if (opMode.gamepad1.aWasPressed()) {
                    stoppedByDriver = true
                    break
                }

                val t = timer.seconds()
                val dt = t - lastT
                lastT = t
                val power = min(rampPowerPerSec * t, rampMax)
                setPower.accept(power)
                val batteryV = opMode.batteryVoltage()
                val appliedV = power * batteryV
                val vel = wheelVel.asDouble
                val speed = abs(vel)
                val poseIn = readPoseIn(axisPos)

                if (dt > 1e-4 && dt < 0.5) {
                    s += vel * dt
                }
                val posRate =
                    if (dt > 1e-4 && dt < 0.5) abs(s - prevS) / dt else Double.POSITIVE_INFINITY
                prevS = s

                if (speed > movingSpeed) {
                    everMoving = true
                }
                if (speed > peakVel) {
                    peakVel = speed
                }

                val travelAbs = abs(s - s0)
                val armed = isStallArmed(everMoving, true, travelAbs, armTravelIn)
                val velCollapsed =
                    isVelocityCollapsed(
                        armed,
                        power,
                        speed,
                        peakVel,
                        stallSpeed,
                        stallFrac,
                        stallMinPower,
                    )
                val posFrozen =
                    isPositionFrozen(armed, power, posRate, stallMinPower, posStallSpeed)
                val collapsing = velCollapsed || posFrozen
                if (collapsing) {
                    // Do not put wall-contact samples (high V, near-zero v) into the kS/kV fit.
                    // Still log every loop so offline analysis sees the crash.
                    if (dt > 0 && dt < 0.5) {
                        stallAccum += dt
                    }
                    if (stallTime > 0 && stallAccum >= stallTime) {
                        stoppedForWall = true
                        trimPostPeakCollapse(rampT, rampV, rampVel, peakVel)
                        logSample(sampleLog, t, "ramp", -1, power, batteryV, vel, poseIn)
                        maybeFlush(sampleLog)
                        break
                    }
                } else {
                    stallAccum = 0.0
                    rampT.add(t)
                    rampV.add(appliedV)
                    rampVel.add(vel)
                }
                logSample(sampleLog, t, "ramp", -1, power, batteryV, vel, poseIn)
                maybeFlush(sampleLog)

                telemetry.addData("phase", "ramp")
                telemetry.addLine("Press gamepad1 A to end ramp before wall/mat edge")
                telemetry.addData("power", "%.2f", power)
                telemetry.addData("speed (in/s)", "%.1f", vel)
                telemetry.addData("peak speed", "%.1f", peakVel)
                telemetry.addData("travel (in)", "%.1f", s - s0)
                if (posRate.isFinite()) {
                    telemetry.addData("pos rate (in/s)", "%.1f", posRate)
                }
                telemetry.addData("pose axis (in)", "%.1f", axisPos.asDouble)
                telemetry.addData("stall armed", armed)
                telemetry.addData("ramp samples", rampV.size)
                if (collapsing) {
                    telemetry.addData("stall timer (s)", "%.2f / %.2f", stallAccum, stallTime)
                    telemetry.addData(
                        "stall via",
                        if (posFrozen && !velCollapsed) {
                            "position"
                        } else if (velCollapsed && !posFrozen) {
                            "velocity"
                        } else {
                            "vel+pos"
                        },
                    )
                }
            }

            // Brief coast so reverse does not inherit a wall-pressed stall state; keep ∫vel continuous.
            setPower.accept(0.0)
            run {
                val coast = ElapsedTime()
                var lastCoast = 0.0
                while (opMode.nextFrame() && coast.seconds() < 0.2) {
                    val tc = coast.seconds()
                    val dtc = tc - lastCoast
                    lastCoast = tc
                    val vel = wheelVel.asDouble
                    if (dtc > 1e-4 && dtc < 0.5) {
                        s += vel * dtc
                    }
                    telemetry.addData("phase", "coast before kA")
                    telemetry.addData("travel (in)", "%.1f", s - s0)
                }
            }

            val travel = s - s0
            // Corridor from ramp start → post-coast end (∫vel, same axis as the fit).
            val corridor = corridorFromRamp(s0, s, endMarginIn, minTravelIn)

            val rampEndReason =
                if (stoppedByDriver) {
                    "driver A"
                } else if (stoppedForWall) {
                    "wall/stall"
                } else {
                    "full duration"
                }
            if (stoppedByDriver) {
                telemetry.addLine("Ramp stopped: gamepad1 A (before wall/mat).")
            } else if (stoppedForWall) {
                telemetry.addLine("Ramp stopped: no longer moving under power (wall/obstacle).")
            }
            if (corridor.usable) {
                telemetry.addLine(
                    String.format(
                        "kA phase: corridor %.0f in (margins %.0f in) — %d half-cycles.",
                        abs(travel),
                        endMarginIn,
                        halfCycles,
                    ),
                )
            } else {
                telemetry.addLine(
                    String.format(
                        "kA phase: timed half-cycles (%.1fs × %d) — travel only %.0f in.",
                        halfCycle,
                        halfCycles,
                        abs(travel),
                    ),
                )
            }

            // --- Phase 2: reverse square wave for kA (first half is opposite the ramp) ---
            // Live position continues the same ∫vel integrator so targets match the fit axis.
            val sBox = doubleArrayOf(s)

            if (corridor.usable) {
                runPositionReversePhase(
                    opMode,
                    telemetry,
                    setPower,
                    wheelVel,
                    axisPos,
                    sBox,
                    revT,
                    revV,
                    revVel,
                    kaPower,
                    halfCycle,
                    halfCycles,
                    corridor,
                    stallSpeed,
                    stallTime,
                    stallMinPower,
                    stallFrac,
                    rampEndReason,
                    sampleLog,
                )
            } else {
                runTimedReversePhase(
                    opMode,
                    telemetry,
                    setPower,
                    wheelVel,
                    axisPos,
                    revT,
                    revV,
                    revVel,
                    kaPower,
                    halfCycle,
                    halfCycles,
                    rampEndReason,
                    sampleLog,
                )
            }
            setPower.accept(0.0)
            sampleLog?.flush()

            return fitRampAndReverse(
                toArray(rampT),
                toArray(rampV),
                toArray(rampVel),
                toArray(revT),
                toArray(revV),
                toArray(revVel),
                inPerTick,
                signDeadband,
                DEFAULT_KA_WINDOW,
                DEFAULT_KA_STRIDE,
                minRampTicksPerSec,
            )
        }

        private fun logSample(
            log: CsvLogger?,
            t: Double,
            phase: String,
            halfIdx: Int,
            power: Double,
            batteryV: Double,
            vel: Double,
            poseIn: Double,
        ) {
            if (log == null) {
                return
            }
            log.row(t, phase, halfIdx, power, batteryV, vel, poseIn)
        }

        private fun readPoseIn(axisPos: DoubleSupplier?): Double {
            return if (axisPos == null) Double.NaN else axisPos.asDouble
        }

        private fun maybeFlush(log: CsvLogger?) {
            if (log != null && log.bufferedRows() >= LOG_FLUSH_EVERY) {
                log.flush()
            }
        }

        /**
         * Build reverse-phase limits from ramp start/end positions (inches). `nearStart`/`nearEnd`
         * sit `endMarginIn` inside the measured span.
         */
        @JvmStatic
        fun corridorFromRamp(
            posStart: Double,
            posEnd: Double,
            endMarginIn: Double,
            minTravelIn: Double,
        ): Corridor {
            if (!posStart.isFinite() || !posEnd.isFinite()) {
                return Corridor(false, 0.0, 0.0, 0.0, 0.0)
            }
            val travel = posEnd - posStart
            val absTravel = abs(travel)
            if (absTravel < minTravelIn) {
                return Corridor(false, 0.0, 0.0, 0.0, travel)
            }
            val sign = sign(travel)
            if (sign == 0.0) {
                return Corridor(false, 0.0, 0.0, 0.0, travel)
            }
            val margin = min(max(0.0, endMarginIn), 0.2 * absTravel)
            val nearStart = posStart + sign * margin
            val nearEnd = posEnd - sign * margin
            // Usable span must still leave room for a meaningful reversal.
            if ((nearEnd - nearStart) * sign < minTravelIn * 0.5) {
                return Corridor(false, nearStart, nearEnd, sign, travel)
            }
            return Corridor(true, nearStart, nearEnd, sign, travel)
        }

        /** True when reverse (-ramp direction) has reached the near-start limit. */
        @JvmStatic
        fun reverseReached(pos: Double, c: Corridor): Boolean {
            return (pos - c.nearStart) * c.travelSign <= 0
        }

        /** True when forward (+ramp direction) has reached the near-end limit. */
        @JvmStatic
        fun forwardReached(pos: Double, c: Corridor): Boolean {
            return (pos - c.nearEnd) * c.travelSign >= 0
        }

        private fun runTimedReversePhase(
            opMode: MarsLinearOpMode,
            telemetry: Telemetry,
            setPower: DoubleConsumer,
            wheelVel: DoubleSupplier,
            axisPos: DoubleSupplier?,
            revT: MutableList<Double>,
            revV: MutableList<Double>,
            revVel: MutableList<Double>,
            kaPower: Double,
            halfCycle: Double,
            halfCycles: Int,
            rampEndReason: String,
            sampleLog: CsvLogger?,
        ) {
            val timer = ElapsedTime()
            val totalRev = halfCycles * halfCycle
            while (opMode.nextFrame() && timer.seconds() < totalRev) {
                val t = timer.seconds()
                val halfIdx = (t / halfCycle).toInt()
                if (halfIdx >= halfCycles) {
                    break
                }
                val power = if (halfIdx % 2 == 0) -kaPower else kaPower
                setPower.accept(power)

                val batteryV = opMode.batteryVoltage()
                val appliedV = power * batteryV
                val vel = wheelVel.asDouble
                revT.add(t)
                revV.add(appliedV)
                revVel.add(vel)
                logSample(sampleLog, t, "reverse", halfIdx, power, batteryV, vel, readPoseIn(axisPos))
                maybeFlush(sampleLog)

                telemetry.addData("phase", "reverse %d / %d (timed)", halfIdx + 1, halfCycles)
                telemetry.addData("ramp end", rampEndReason)
                telemetry.addData("power", "%.2f", power)
                telemetry.addData("speed (in/s)", "%.1f", vel)
                telemetry.addData("rev samples", revT.size)
            }
        }

        /**
         * @param sBox single-element box holding live ∫vel position (updated in-place)
         * @param minHalfSec minimum duration of each half-cycle so kA always gets real reversals even
         *     if a position target is already satisfied (or stalls early)
         */
        private fun runPositionReversePhase(
            opMode: MarsLinearOpMode,
            telemetry: Telemetry,
            setPower: DoubleConsumer,
            wheelVel: DoubleSupplier,
            axisPos: DoubleSupplier?,
            sBox: DoubleArray,
            revT: MutableList<Double>,
            revV: MutableList<Double>,
            revVel: MutableList<Double>,
            kaPower: Double,
            minHalfSec: Double,
            halfCycles: Int,
            corridor: Corridor,
            stallSpeed: Double,
            stallTime: Double,
            stallMinPower: Double,
            stallFrac: Double,
            rampEndReason: String,
            sampleLog: CsvLogger?,
        ) {
            val absTravel = abs(corridor.travelIn)
            // Allow long corridors; always at least ~2× minHalf so a full reverse can complete.
            val maxHalfSec = max(minHalfSec * 2.5, absTravel / 6.0 + 1.5)
            val minHalf = max(0.35, minHalfSec)
            val global = ElapsedTime()

            var halfIdx = 0
            while (halfIdx < halfCycles && opMode.opModeIsActive()) {
                val reverse = halfIdx % 2 == 0
                val power = if (reverse) -kaPower else kaPower
                val halfTimer = ElapsedTime()
                var everMoving = false
                var peakSpeed = 0.0
                var stallAccum = 0.0
                var lastT = global.seconds()
                val halfStartS = sBox[0]
                // Require a few inches of progress in the commanded direction before stall may end
                // the half.
                val minProgress = min(4.0, 0.15 * absTravel)

                while (opMode.nextFrame() && halfTimer.seconds() < maxHalfSec) {
                    val t = global.seconds()
                    val dt = t - lastT
                    lastT = t

                    setPower.accept(power)
                    val vel = wheelVel.asDouble
                    if (dt > 1e-4 && dt < 0.5) {
                        sBox[0] += vel * dt
                    }
                    val pos = sBox[0]
                    val speed = abs(vel)
                    val batteryV = opMode.batteryVoltage()
                    val appliedV = power * batteryV

                    revT.add(t)
                    revV.add(appliedV)
                    revVel.add(vel)
                    logSample(
                        sampleLog,
                        t,
                        "reverse",
                        halfIdx,
                        power,
                        batteryV,
                        vel,
                        readPoseIn(axisPos),
                    )
                    maybeFlush(sampleLog)

                    if (speed > DEFAULT_MOVING_SPEED * 0.5) {
                        everMoving = true
                    }
                    if (speed > peakSpeed) {
                        peakSpeed = speed
                    }

                    val progress =
                        if (reverse) {
                            (halfStartS - pos) * corridor.travelSign // motion toward start
                        } else {
                            (pos - halfStartS) * corridor.travelSign // motion toward end
                        }
                    val atTarget =
                        if (reverse) reverseReached(pos, corridor) else forwardReached(pos, corridor)
                    val minTimeOk = halfTimer.seconds() >= minHalf
                    if (atTarget && minTimeOk) {
                        break
                    }

                    // Stall exit only after minimum time + some progress (avoids ending reverse
                    // while still pressed on the wall at the start of the half).
                    val collapsing =
                        isVelocityCollapsed(
                            everMoving,
                            abs(power),
                            speed,
                            peakSpeed,
                            stallSpeed,
                            stallFrac,
                            stallMinPower,
                        )
                    if (collapsing && minTimeOk && progress >= minProgress) {
                        if (dt > 0 && dt < 0.5) {
                            stallAccum += dt
                        }
                        if (stallTime > 0 && stallAccum >= stallTime) {
                            break
                        }
                    } else if (!collapsing) {
                        stallAccum = 0.0
                    }

                    telemetry.addData(
                        "phase",
                        "reverse %d / %d (corridor)",
                        halfIdx + 1,
                        halfCycles,
                    )
                    telemetry.addData("ramp end", rampEndReason)
                    telemetry.addData("power", "%.2f", power)
                    telemetry.addData("speed (in/s)", "%.1f", vel)
                    telemetry.addData("s (in)", "%.1f", pos)
                    telemetry.addData(
                        "target (in)",
                        "%.1f",
                        if (reverse) corridor.nearStart else corridor.nearEnd,
                    )
                    telemetry.addData("progress (in)", "%.1f", progress)
                    telemetry.addData("half t (s)", "%.1f / %.1f", halfTimer.seconds(), maxHalfSec)
                    telemetry.addData("rev samples", revT.size)
                }
                halfIdx++
            }
        }

        /**
         * True when stall detection may fire: either speed once exceeded the moving threshold, or
         * the robot has traveled enough distance (covers slow lateral ramps that hit a wall early).
         */
        @JvmStatic
        fun isStallArmed(
            everMoving: Boolean,
            havePos: Boolean,
            travelAbs: Double,
            armTravelIn: Double,
        ): Boolean {
            if (everMoving) {
                return true
            }
            return havePos && armTravelIn > 0 && travelAbs >= armTravelIn
        }

        /**
         * True when the robot has been moving and speed has collapsed under power — typical of wall
         * contact. `speed` and `peakVel` should be non-negative magnitudes (in/s). `armed` replaces
         * the old "everMoving" gate (see [isStallArmed]). Package-visible for unit tests.
         */
        @JvmStatic
        fun isVelocityCollapsed(
            armed: Boolean,
            power: Double,
            speed: Double,
            peakVel: Double,
            stallSpeed: Double,
            stallFrac: Double,
            stallMinPower: Double,
        ): Boolean {
            if (!armed || power < stallMinPower) {
                return false
            }
            val frac = if (stallFrac > 0) stallFrac else 0.0
            val thresh = max(stallSpeed, peakVel * frac)
            return speed < thresh
        }

        /**
         * True when stall is armed, power is applied, and position is barely advancing — wall
         * contact with noisy / non-zero velocity (common on mecanum strafe).
         */
        @JvmStatic
        fun isPositionFrozen(
            armed: Boolean,
            power: Double,
            posRateInPerSec: Double,
            stallMinPower: Double,
            posStallSpeed: Double,
        ): Boolean {
            if (!armed || power < stallMinPower) {
                return false
            }
            if (!posRateInPerSec.isFinite() || posStallSpeed < 0) {
                return false
            }
            return posRateInPerSec < posStallSpeed
        }

        /**
         * Drop trailing ramp samples from the velocity crash into an obstacle so high-V / low-v
         * points do not bias `kS`. Keeps at least a few samples when possible.
         */
        @JvmStatic
        fun trimPostPeakCollapse(
            t: MutableList<Double>,
            voltage: MutableList<Double>,
            vel: MutableList<Double>,
            peakVel: Double,
        ) {
            if (peakVel <= 0 || t.isEmpty() || t.size != vel.size || t.size != voltage.size) {
                return
            }
            val keepAbove = peakVel * 0.85
            while (t.size > 8 && vel[vel.size - 1] < keepAbove) {
                val i = t.size - 1
                t.removeAt(i)
                voltage.removeAt(i)
                vel.removeAt(i)
            }
        }

        /**
         * Fit kS/kV from a unidirectional ramp series and kA from a reverse series. Package-visible
         * for unit tests.
         */
        @JvmStatic
        fun fitRampAndReverse(
            rampT: DoubleArray,
            rampVoltage: DoubleArray,
            rampVel: DoubleArray,
            revT: DoubleArray,
            revVoltage: DoubleArray,
            revVel: DoubleArray,
            inPerTick: Double,
            signDeadband: Double,
            kaWindow: Double,
            kaStride: Double,
        ): Result {
            return fitRampAndReverse(
                rampT,
                rampVoltage,
                rampVel,
                revT,
                revVoltage,
                revVel,
                inPerTick,
                signDeadband,
                kaWindow,
                kaStride,
                DEFAULT_MIN_RAMP_TICKS_PER_SEC,
            )
        }

        /**
         * @param minRampTicksPerSec ramp samples slower than this (tick units/s × `inPerTick` →
         *     in/s) are excluded from the kS/kV fit
         */
        @JvmStatic
        fun fitRampAndReverse(
            rampT: DoubleArray,
            rampVoltage: DoubleArray,
            rampVel: DoubleArray,
            revT: DoubleArray,
            revVoltage: DoubleArray,
            revVel: DoubleArray,
            inPerTick: Double,
            signDeadband: Double,
            kaWindow: Double,
            kaStride: Double,
            minRampTicksPerSec: Double,
        ): Result {
            // Drop the breakaway knee: V rises with almost no motion until static friction breaks.
            // signDeadband alone is often ~1 in/s (~300 ticks/s); 1000 ticks/s is a better cut.
            val rampMinSpeed = max(signDeadband, max(0.0, minRampTicksPerSec) * inPerTick)
            val ramp = fitRamp(rampVoltage, rampVel, rampT, rampMinSpeed)
            if (ramp.singular) {
                return Result(
                    true,
                    0.0,
                    0.0,
                    0.0,
                    rampVoltage.size + revT.size,
                    ramp.used,
                    ramp.r2,
                    ramp.message,
                )
            }

            val ka =
                fitKa(
                    revT,
                    revVoltage,
                    revVel,
                    ramp.kS,
                    ramp.kVInch,
                    signDeadband,
                    kaWindow,
                    kaStride,
                )
            val total = rampVoltage.size + revT.size
            if (ka.singular) {
                return Result(
                    true,
                    ramp.kS,
                    ramp.kVInch * inPerTick,
                    0.0,
                    total,
                    ramp.used,
                    ramp.r2,
                    ka.message,
                )
            }

            return Result(
                false,
                ramp.kS,
                ramp.kVInch * inPerTick,
                ka.kAInch * inPerTick,
                total,
                ramp.used,
                ramp.r2,
                "ok",
            )
        }

        /**
         * Unidirectional ramp regression matching ForwardRampLogger's `V = kS + kV·v`.
         *
         * Samples with `|v| ≤ signDeadband` are dropped — callers should set that high enough to
         * clear the non-linear breakaway knee (see [DEFAULT_MIN_RAMP_TICKS_PER_SEC]).
         *
         * `kV` (slope) comes from [TunerRegression.robustFit] over the moving ramp — slope is well
         * conditioned. `kS` (intercept) is then re-estimated as the mean residual `V − kV·v` on the
         * *lowest-accel* samples: a continuous ramp never fully settles, so a joint intercept fit
         * absorbs `kA·a` and biases `kS` high; residual mean on near-steady samples matches what
         * the dashboard line fit does after outlier removal.
         */
        @JvmStatic
        fun fitRamp(voltage: DoubleArray, vel: DoubleArray, signDeadband: Double): RampFit {
            return fitRamp(voltage, vel, null, signDeadband)
        }

        /**
         * @param timeSec optional sample times for finite-difference accel; improves kS residual
         *     pick
         */
        @JvmStatic
        fun fitRamp(
            voltage: DoubleArray?,
            vel: DoubleArray?,
            timeSec: DoubleArray?,
            signDeadband: Double,
        ): RampFit {
            if (voltage == null || vel == null || voltage.size != vel.size || voltage.size < 8) {
                return RampFit(true, 0.0, 0.0, Double.NaN, 0, "too few ramp samples")
            }
            if (timeSec != null && timeSec.size != voltage.size) {
                return RampFit(true, 0.0, 0.0, Double.NaN, 0, "ramp time length mismatch")
            }

            var pos = 0
            var neg = 0
            for (v in vel) {
                if (v > signDeadband) {
                    pos++
                } else if (v < -signDeadband) {
                    neg++
                }
            }
            val usePositive = pos >= neg

            val samples = ArrayList<DoubleArray>()
            // parallel lists for residual-kS pass (same quadrant as samples)
            val resV = ArrayList<Double>()
            val resVel = ArrayList<Double>()
            val resAbsA = ArrayList<Double>()

            for (i in voltage.indices) {
                val v = vel[i]
                val y: Double
                val x: Double
                if (usePositive) {
                    if (v <= signDeadband) {
                        continue
                    }
                    x = v
                    y = voltage[i]
                } else {
                    if (v >= -signDeadband) {
                        continue
                    }
                    x = -v
                    y = -voltage[i]
                }
                samples.add(doubleArrayOf(x, y))

                var absA = Double.POSITIVE_INFINITY
                if (timeSec != null && i > 0) {
                    val dt = timeSec[i] - timeSec[i - 1]
                    if (dt > 1e-4) {
                        absA = abs((vel[i] - vel[i - 1]) / dt)
                    }
                }
                resV.add(y)
                resVel.add(x)
                resAbsA.add(absA)
            }

            if (samples.size < 8) {
                return RampFit(
                    true,
                    0.0,
                    0.0,
                    Double.NaN,
                    samples.size,
                    "too few moving ramp samples — check localization sign (forward → +v)",
                )
            }

            val fit = TunerRegression.robustFit(samples)
            if (fit == null || fit.used < 4) {
                return RampFit(true, 0.0, 0.0, Double.NaN, samples.size, "ramp regression failed")
            }
            if (fit.slope <= 0) {
                return RampFit(
                    true,
                    fit.intercept,
                    fit.slope,
                    fit.r2,
                    fit.used,
                    "ramp kV ≤ 0 — localization sign is likely still inverted",
                )
            }

            val kVInch = fit.slope
            // Re-estimate kS at the ramp peak (hold): slope from the full ramp is solid, but the
            // joint intercept absorbs kA·a during the climb. Near peak voltage, a → 0 and
            // kS ≈ V − kV·v.
            val kS = refitKsAtPeak(resV, resVel, resAbsA, kVInch, fit.intercept)

            return RampFit(false, kS, kVInch, fit.r2, fit.used, "ok")
        }

        /**
         * Mean of `V − kV·v` on near-peak-voltage samples with the lowest accel (settled hold at
         * the end of the ramp). Falls back to the joint-fit intercept if too few samples.
         */
        private fun refitKsAtPeak(
            V: List<Double>,
            vel: List<Double>,
            absA: List<Double>,
            kVInch: Double,
            fallbackKS: Double,
        ): Double {
            val n = V.size
            if (n == 0) {
                return fallbackKS
            }
            var vmax = 0.0
            for (y in V) {
                vmax = max(vmax, y)
            }
            if (vmax < 1e-6) {
                return fallbackKS
            }
            // Peak band: top ~5% of applied voltage (the hold at RAMP_MAX).
            val vThresh = 0.95 * vmax

            // Collect residuals in the peak band; prefer low |a| when available.
            val peak = ArrayList<DoubleArray>() // [absA, residual]
            for (i in 0 until n) {
                if (V[i] < vThresh) {
                    continue
                }
                val r = V[i] - kVInch * vel[i]
                val a = absA[i]
                peak.add(doubleArrayOf(if (a.isFinite()) a else 0.0, r))
            }
            if (peak.size < 4) {
                return fallbackKS
            }
            // Use the lowest-accel half of the peak band (most settled).
            peak.sortBy { it[0] }
            val keep = max(4, peak.size / 2)
            var sum = 0.0
            for (i in 0 until keep) {
                sum += peak[i][1]
            }
            return sum / keep
        }

        /** Residual kA from reverse-phase windows: `∫(V − kS·sign(v) − kV·v) dt = kA·Δv`. */
        private fun fitKa(
            t: DoubleArray,
            voltage: DoubleArray,
            vel: DoubleArray,
            kS: Double,
            kVInch: Double,
            signDeadband: Double,
            kaWindow: Double,
            kaStride: Double,
        ): KaFit {
            val n = t.size
            if (n < 8 || voltage.size != n || vel.size != n) {
                return KaFit(true, 0.0, "too few reverse samples for kA")
            }

            var num = 0.0
            var den = 0.0
            var windows = 0
            val tEnd = t[n - 1]
            var wStart = t[0]
            while (wStart + kaWindow <= tEnd + 1e-9) {
                val i0 = lowerBound(t, wStart)
                var i1 = lowerBound(t, wStart + kaWindow)
                if (i1 >= n) {
                    i1 = n - 1
                }
                if (i1 <= i0 + 1) {
                    wStart += kaStride
                    continue
                }
                var residInt = 0.0
                for (k in i0 until i1) {
                    val dt = t[k + 1] - t[k]
                    if (dt <= 0) {
                        continue
                    }
                    val r0 = residual(voltage[k], vel[k], kS, kVInch, signDeadband)
                    val r1 = residual(voltage[k + 1], vel[k + 1], kS, kVInch, signDeadband)
                    residInt += 0.5 * (r0 + r1) * dt
                }
                val dv = vel[i1] - vel[i0]
                num += residInt * dv
                den += dv * dv
                windows++
                wStart += kaStride
            }

            if (den < 1e-6 || windows < 4) {
                return KaFit(
                    true,
                    0.0,
                    "kA not identifiable — more reverse half-cycles or higher KA_POWER",
                )
            }
            val kA = num / den
            // Passive inertia cannot produce a negative kA; a negative fit almost always means the
            // reverse phase did not actually reverse (stuck on a wall, half-cycles skipped, etc.).
            if (!(kA > 0)) {
                return KaFit(
                    true,
                    kA,
                    "kA ≤ 0 (" +
                        String.format("%.4g", kA) +
                        ") — reverse phase likely did not excite accel; re-run and confirm " +
                        "the robot leaves the wall and squares back and forth",
                )
            }
            return KaFit(false, kA, "ok")
        }

        private fun residual(
            voltage: Double,
            vel: Double,
            kS: Double,
            kVInch: Double,
            dead: Double,
        ): Double {
            val s = if (abs(vel) < dead) 0.0 else sign(vel)
            return voltage - kS * s - kVInch * vel
        }

        private fun lowerBound(t: DoubleArray, x: Double): Int {
            var lo = 0
            var hi = t.size
            while (lo < hi) {
                val mid = (lo + hi) ushr 1
                if (t[mid] < x) {
                    lo = mid + 1
                } else {
                    hi = mid
                }
            }
            return lo
        }

        private fun toArray(list: List<Double>): DoubleArray {
            return DoubleArray(list.size) { list[it] }
        }
    }
}
