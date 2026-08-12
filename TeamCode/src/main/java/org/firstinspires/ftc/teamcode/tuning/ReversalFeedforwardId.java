package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.base.MarsLinearOpMode;
import org.firstinspires.ftc.teamcode.utils.CsvLogger;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

/**
 * Shared core for the feedforward tuners ({@link AxialFeedforwardTuner}, {@link
 * LateralFeedforwardTuner}).
 *
 * <p><b>Two maneuvers, two jobs.</b>
 *
 * <ol>
 *   <li><b>Slow open-loop ramp</b> (same idea as {@code ForwardRampLogger}) — visits many speeds so
 *       {@code kS} and {@code kV} separate cleanly via {@code V = kS + kV·v}. High-speed-only
 *       cruise levels pin {@code kV} well but leave the intercept ({@code kS}) poorly leveraged: a
 *       few percent slope error shifts {@code kS} by tenths of a volt. Ends early if speed
 *       collapses under power (wall / hard stop) so the robot does not keep driving into an
 *       obstacle.
 *   <li><b>Reverse square wave</b> — first half-cycle is reverse (away from a forward wall). When
 *       the ramp measured a useful travel distance (e.g. start → wall), each half-cycle runs for
 *       that corridor rather than a fixed 1 s, so the kA phase uses the room the robot just proved
 *       is free. Then with {@code kS}/{@code kV} fixed, fit residual {@code ∫(V − kS·sign(v) −
 *       kV·v) dt = kA·Δv} over short windows (no noisy {@code a = dv/dt}).
 * </ol>
 *
 * <p>Velocity is chassis axis speed in in/s from the localizer; axis position is inches along the
 * same axis. {@code power} and velocity must share a sign convention.
 *
 * <p>Optional {@link CsvLogger} sample logging (header {@link #SAMPLE_HEADER}): raw control-loop
 * measurements during ramp and reverse so the id can be re-run offline. Callers open/close the
 * logger and write a one-row meta file with run config ({@link #META_HEADER}) — not fitted
 * constants (those are recomputed from samples).
 */
public final class ReversalFeedforwardId {
    private ReversalFeedforwardId() {}

    /**
     * Per-loop raw measurements. {@code phase} is {@code ramp} or {@code reverse}; {@code half_idx}
     * is {@code -1} on the ramp and 0-based on reverse half-cycles. Applied axis voltage is {@code
     * power * battery_v}. {@code pose_in} is the localizer axis reading when available (NaN
     * otherwise) — not the integrated ∫vel used for corridor logic.
     */
    public static final String SAMPLE_HEADER =
            "t_s,phase,half_idx,power,battery_v,vel_in_s,pose_in";

    /**
     * One-row run config for offline re-fit (plant scale + OpMode knobs). Not fit results —
     * recompute kS/kV/kA from the sample file.
     */
    public static final String META_HEADER =
            "drive,axis,in_per_tick,ramp_power_per_sec,ramp_max,ka_power,half_cycle,ka_half_cycles,"
                + "sign_deadband,stall_speed,stall_time,moving_speed,stall_min_power,stall_frac,"
                + "min_ramp_ticks_per_sec,end_margin_in,min_travel_in,arm_travel_in,pos_stall_speed,"
                + "ka_window_s,ka_stride_s";

    /** Flush buffered sample rows after this many are queued. */
    public static final int LOG_FLUSH_EVERY = 256;

    /** Window length (s) for residual kA integral equations. */
    public static final double DEFAULT_KA_WINDOW = 0.12;

    /** Stride (s) between kA windows. */
    public static final double DEFAULT_KA_STRIDE = 0.04;

    /** Seconds held at ramp peak before the reverse phase. */
    public static final double DEFAULT_RAMP_HOLD = 0.4;

    /**
     * Absolute speed (in/s) below which a moving robot is treated as stalled against an obstacle.
     * Combined with {@link #DEFAULT_STALL_FRAC} of peak speed (whichever is larger).
     */
    public static final double DEFAULT_STALL_SPEED = 2.0;

    /** How long (s) velocity must stay collapsed before the ramp ends for a wall/stall. */
    public static final double DEFAULT_STALL_TIME = 0.15;

    /**
     * Speed (in/s) the robot must have exceeded at least once before stall detection arms — avoids
     * ending the ramp during breakaway from rest.
     */
    public static final double DEFAULT_MOVING_SPEED = 5.0;

    /** Commanded |power| must be at least this to count as stalled (ignore near-zero commands). */
    public static final double DEFAULT_STALL_MIN_POWER = 0.15;

    /**
     * Velocity below this fraction of the ramp's peak speed also counts as collapsed (catches wall
     * hits where the robot still creeps a little).
     */
    public static final double DEFAULT_STALL_FRAC = 0.25;

    /**
     * Ramp samples slower than this many encoder-tick units per second are excluded from the kS/kV
     * regression. Converts via {@code inPerTick} so it tracks wheel resolution. Cuts the non-linear
     * breakaway knee at the start of a forward ramp (~vertical cluster near v=0 on the V-vs-v
     * plot).
     */
    public static final double DEFAULT_MIN_RAMP_TICKS_PER_SEC = 1000;

    /**
     * Keep this many inches clear of each end of the measured ramp corridor during the
     * position-based reverse phase (so half-cycles don't grind the wall or start edge).
     */
    public static final double DEFAULT_END_MARGIN_IN = 6.0;

    /**
     * Minimum |ramp travel| (in) before the reverse phase switches from fixed-time half-cycles to
     * position-based half-cycles that use the full corridor.
     */
    public static final double DEFAULT_MIN_TRAVEL_IN = 18.0;

    /**
     * |Travel| (in) from ramp start that arms stall detection even if peak speed never reached
     * {@link #DEFAULT_MOVING_SPEED}. Important for slow lateral ramps that hit a wall early.
     */
    public static final double DEFAULT_ARM_TRAVEL_IN = 4.0;

    /**
     * When stall is armed, |d(position)/dt| below this (in/s) under power counts as stalled —
     * catches wall contact when velocity is noisy or wheels keep slipping a little.
     */
    public static final double DEFAULT_POS_STALL_SPEED = 1.5;

    /** Fitted constants in tick units (kS in volts), matching {@code Params.kS/kV/kA}. */
    public static final class Result {
        public final boolean singular;
        public final double kS;
        public final double kV;
        public final double kA;
        public final int samples;
        public final int rampSamples;

        /** Coefficient of determination for the ramp (kS/kV) fit; NaN if unavailable. */
        public final double rampR2;

        public final String message;

        Result(
                boolean singular,
                double kS,
                double kV,
                double kA,
                int samples,
                int rampSamples,
                double rampR2,
                String message) {
            this.singular = singular;
            this.kS = kS;
            this.kV = kV;
            this.kA = kA;
            this.samples = samples;
            this.rampSamples = rampSamples;
            this.rampR2 = rampR2;
            this.message = message;
        }
    }

    /**
     * Run a slow ramp (kS/kV) then a reverse square wave (kA).
     *
     * <p>The ramp ends at the usual duration <em>or</em> earlier if the robot stops moving while
     * still under power (e.g. hit a wall). Phase 2 always starts with reverse power. When {@code
     * axisPos} reports enough ramp travel, each reverse half-cycle uses that corridor (minus end
     * margins) instead of a fixed {@code halfCycle} duration.
     *
     * @param wheelVel axis velocity (in/s); should update the localizer when read
     * @param axisPos axis position (in) along the same axis; read after {@code wheelVel}
     * @param halfCycle fallback seconds per reverse direction when travel is too short for
     *     position-based half-cycles
     */
    public static Result identify(
            MarsLinearOpMode opMode,
            Telemetry telemetry,
            DoubleConsumer setPower,
            DoubleSupplier wheelVel,
            DoubleSupplier axisPos,
            double inPerTick,
            double rampPowerPerSec,
            double rampMax,
            double kaPower,
            double halfCycle,
            int halfCycles,
            double signDeadband,
            double stallSpeed,
            double stallTime,
            double movingSpeed,
            double stallMinPower,
            double stallFrac,
            double minRampTicksPerSec,
            double endMarginIn,
            double minTravelIn) {
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
                null);
    }

    public static Result identify(
            MarsLinearOpMode opMode,
            Telemetry telemetry,
            DoubleConsumer setPower,
            DoubleSupplier wheelVel,
            DoubleSupplier axisPos,
            double inPerTick,
            double rampPowerPerSec,
            double rampMax,
            double kaPower,
            double halfCycle,
            int halfCycles,
            double signDeadband,
            double stallSpeed,
            double stallTime,
            double movingSpeed,
            double stallMinPower,
            double stallFrac,
            double minRampTicksPerSec,
            double endMarginIn,
            double minTravelIn,
            double armTravelIn,
            double posStallSpeed) {
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
                null);
    }

    /**
     * @param sampleLog optional per-loop sample logger ({@link #SAMPLE_HEADER}); null skips CSV
     *     rows. Caller owns open/close and the meta file; this only appends raw sample rows and
     *     flushes periodically.
     */
    public static Result identify(
            MarsLinearOpMode opMode,
            Telemetry telemetry,
            DoubleConsumer setPower,
            DoubleSupplier wheelVel,
            DoubleSupplier axisPos,
            double inPerTick,
            double rampPowerPerSec,
            double rampMax,
            double kaPower,
            double halfCycle,
            int halfCycles,
            double signDeadband,
            double stallSpeed,
            double stallTime,
            double movingSpeed,
            double stallMinPower,
            double stallFrac,
            double minRampTicksPerSec,
            double endMarginIn,
            double minTravelIn,
            double armTravelIn,
            double posStallSpeed,
            CsvLogger sampleLog) {
        if (rampPowerPerSec <= 0 || rampMax <= 0) {
            return new Result(true, 0, 0, 0, 0, 0, Double.NaN, "bad ramp settings");
        }
        if (kaPower <= 0 || halfCycle <= 0 || halfCycles < 2) {
            return new Result(true, 0, 0, 0, 0, 0, Double.NaN, "bad reverse-phase settings");
        }

        List<Double> rampT = new ArrayList<>();
        List<Double> rampV = new ArrayList<>();
        List<Double> rampVel = new ArrayList<>();
        List<Double> revT = new ArrayList<>();
        List<Double> revV = new ArrayList<>();
        List<Double> revVel = new ArrayList<>();

        ElapsedTime timer = new ElapsedTime();
        double rampDuration = rampMax / rampPowerPerSec + DEFAULT_RAMP_HOLD;

        // --- Phase 1: slow forward ramp (ForwardRampLogger-style) ---
        // Ends on schedule, on gamepad1 A (manual — hit before the mat/wall trap), or if
        // motion collapses under power (automatic wall/hard-stop detect).
        //
        // Position for stall + corridor is ∫vel dt along the test axis — same signal as the fit.
        // World pose (axisPos) is only for telemetry: on lateral strafes, heading error makes
        // field-y a bad stand-in for robot-lateral travel and was skipping reverse half-cycles.
        boolean everMoving = false;
        double peakVel = 0;
        double stallAccum = 0;
        double lastT = 0;
        boolean stoppedForWall = false;
        boolean stoppedByDriver = false;
        double s = 0; // integrated axis displacement (in), same sign as wheelVel
        double prevS = 0;
        final double s0 = 0;

        // Seed localizer before the ramp.
        wheelVel.getAsDouble();

        while (opMode.nextFrame() && timer.seconds() < rampDuration) {
            // Manual end: press A just before the robot jams into the wall/mat lip so reverse
            // still has free wheels. Edge-detect so a held A from before START does not fire.
            if (opMode.gamepad1.aWasPressed()) {
                stoppedByDriver = true;
                break;
            }

            double t = timer.seconds();
            double dt = t - lastT;
            lastT = t;
            double power = Math.min(rampPowerPerSec * t, rampMax);
            setPower.accept(power);
            double batteryV = opMode.batteryVoltage();
            double appliedV = power * batteryV;
            double vel = wheelVel.getAsDouble();
            double speed = Math.abs(vel);
            double poseIn = readPoseIn(axisPos);

            if (dt > 1e-4 && dt < 0.5) {
                s += vel * dt;
            }
            double posRate =
                    (dt > 1e-4 && dt < 0.5) ? Math.abs(s - prevS) / dt : Double.POSITIVE_INFINITY;
            prevS = s;

            if (speed > movingSpeed) {
                everMoving = true;
            }
            if (speed > peakVel) {
                peakVel = speed;
            }

            double travelAbs = Math.abs(s - s0);
            boolean armed = isStallArmed(everMoving, true, travelAbs, armTravelIn);
            boolean velCollapsed =
                    isVelocityCollapsed(
                            armed, power, speed, peakVel, stallSpeed, stallFrac, stallMinPower);
            boolean posFrozen =
                    isPositionFrozen(armed, power, posRate, stallMinPower, posStallSpeed);
            boolean collapsing = velCollapsed || posFrozen;
            if (collapsing) {
                // Do not put wall-contact samples (high V, near-zero v) into the kS/kV fit.
                // Still log every loop so offline analysis sees the crash.
                if (dt > 0 && dt < 0.5) {
                    stallAccum += dt;
                }
                if (stallTime > 0 && stallAccum >= stallTime) {
                    stoppedForWall = true;
                    trimPostPeakCollapse(rampT, rampV, rampVel, peakVel);
                    logSample(sampleLog, t, "ramp", -1, power, batteryV, vel, poseIn);
                    maybeFlush(sampleLog);
                    break;
                }
            } else {
                stallAccum = 0;
                rampT.add(t);
                rampV.add(appliedV);
                rampVel.add(vel);
            }
            logSample(sampleLog, t, "ramp", -1, power, batteryV, vel, poseIn);
            maybeFlush(sampleLog);

            telemetry.addData("phase", "ramp");
            telemetry.addLine("Press gamepad1 A to end ramp before wall/mat edge");
            telemetry.addData("power", "%.2f", power);
            telemetry.addData("speed (in/s)", "%.1f", vel);
            telemetry.addData("peak speed", "%.1f", peakVel);
            telemetry.addData("travel (in)", "%.1f", s - s0);
            if (Double.isFinite(posRate)) {
                telemetry.addData("pos rate (in/s)", "%.1f", posRate);
            }
            if (axisPos != null) {
                telemetry.addData("pose axis (in)", "%.1f", axisPos.getAsDouble());
            }
            telemetry.addData("stall armed", armed);
            telemetry.addData("ramp samples", rampV.size());
            if (collapsing) {
                telemetry.addData("stall timer (s)", "%.2f / %.2f", stallAccum, stallTime);
                telemetry.addData(
                        "stall via",
                        posFrozen && !velCollapsed
                                ? "position"
                                : (velCollapsed && !posFrozen ? "velocity" : "vel+pos"));
            }
        }

        // Brief coast so reverse does not inherit a wall-pressed stall state; keep ∫vel continuous.
        setPower.accept(0);
        {
            ElapsedTime coast = new ElapsedTime();
            double lastCoast = 0;
            while (opMode.nextFrame() && coast.seconds() < 0.2) {
                double tc = coast.seconds();
                double dtc = tc - lastCoast;
                lastCoast = tc;
                double vel = wheelVel.getAsDouble();
                if (dtc > 1e-4 && dtc < 0.5) {
                    s += vel * dtc;
                }
                telemetry.addData("phase", "coast before kA");
                telemetry.addData("travel (in)", "%.1f", s - s0);
            }
        }

        double travel = s - s0;
        // Corridor from ramp start → post-coast end (∫vel, same axis as the fit).
        Corridor corridor = corridorFromRamp(s0, s, endMarginIn, minTravelIn);

        String rampEndReason =
                stoppedByDriver ? "driver A" : (stoppedForWall ? "wall/stall" : "full duration");
        if (stoppedByDriver) {
            telemetry.addLine("Ramp stopped: gamepad1 A (before wall/mat).");
        } else if (stoppedForWall) {
            telemetry.addLine("Ramp stopped: no longer moving under power (wall/obstacle).");
        }
        if (corridor.usable) {
            telemetry.addLine(
                    String.format(
                            "kA phase: corridor %.0f in (margins %.0f in) — %d half-cycles.",
                            Math.abs(travel), endMarginIn, halfCycles));
        } else {
            telemetry.addLine(
                    String.format(
                            "kA phase: timed half-cycles (%.1fs × %d) — travel only %.0f in.",
                            halfCycle, halfCycles, Math.abs(travel)));
        }

        // --- Phase 2: reverse square wave for kA (first half is opposite the ramp) ---
        // Live position continues the same ∫vel integrator so targets match the fit axis.
        final double[] sBox = new double[] {s};

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
                    sampleLog);
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
                    sampleLog);
        }
        setPower.accept(0);
        if (sampleLog != null) {
            sampleLog.flush();
        }

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
                minRampTicksPerSec);
    }

    private static void logSample(
            CsvLogger log,
            double t,
            String phase,
            int halfIdx,
            double power,
            double batteryV,
            double vel,
            double poseIn) {
        if (log == null) {
            return;
        }
        log.row(t, phase, halfIdx, power, batteryV, vel, poseIn);
    }

    private static double readPoseIn(DoubleSupplier axisPos) {
        return axisPos == null ? Double.NaN : axisPos.getAsDouble();
    }

    private static void maybeFlush(CsvLogger log) {
        if (log != null && log.bufferedRows() >= LOG_FLUSH_EVERY) {
            log.flush();
        }
    }

    /**
     * Soft limits for the reverse phase from measured ramp endpoints. Package-visible for unit
     * tests.
     */
    static final class Corridor {
        final boolean usable;

        /** Position (in) near the ramp start — reverse half-cycles stop here. */
        final double nearStart;

        /** Position (in) near the ramp end / wall — forward half-cycles stop here. */
        final double nearEnd;

        /** {@code sign(posEnd - posStart)}; 0 if unusable. */
        final double travelSign;

        final double travelIn;

        Corridor(
                boolean usable,
                double nearStart,
                double nearEnd,
                double travelSign,
                double travelIn) {
            this.usable = usable;
            this.nearStart = nearStart;
            this.nearEnd = nearEnd;
            this.travelSign = travelSign;
            this.travelIn = travelIn;
        }
    }

    /**
     * Build reverse-phase limits from ramp start/end positions (inches). {@code nearStart}/{@code
     * nearEnd} sit {@code endMarginIn} inside the measured span.
     */
    static Corridor corridorFromRamp(
            double posStart, double posEnd, double endMarginIn, double minTravelIn) {
        if (!Double.isFinite(posStart) || !Double.isFinite(posEnd)) {
            return new Corridor(false, 0, 0, 0, 0);
        }
        double travel = posEnd - posStart;
        double absTravel = Math.abs(travel);
        if (absTravel < minTravelIn) {
            return new Corridor(false, 0, 0, 0, travel);
        }
        double sign = Math.signum(travel);
        if (sign == 0) {
            return new Corridor(false, 0, 0, 0, travel);
        }
        double margin = Math.min(Math.max(0, endMarginIn), 0.2 * absTravel);
        double nearStart = posStart + sign * margin;
        double nearEnd = posEnd - sign * margin;
        // Usable span must still leave room for a meaningful reversal.
        if ((nearEnd - nearStart) * sign < minTravelIn * 0.5) {
            return new Corridor(false, nearStart, nearEnd, sign, travel);
        }
        return new Corridor(true, nearStart, nearEnd, sign, travel);
    }

    /** True when reverse (-ramp direction) has reached the near-start limit. */
    static boolean reverseReached(double pos, Corridor c) {
        return (pos - c.nearStart) * c.travelSign <= 0;
    }

    /** True when forward (+ramp direction) has reached the near-end limit. */
    static boolean forwardReached(double pos, Corridor c) {
        return (pos - c.nearEnd) * c.travelSign >= 0;
    }

    private static void runTimedReversePhase(
            MarsLinearOpMode opMode,
            Telemetry telemetry,
            DoubleConsumer setPower,
            DoubleSupplier wheelVel,
            DoubleSupplier axisPos,
            List<Double> revT,
            List<Double> revV,
            List<Double> revVel,
            double kaPower,
            double halfCycle,
            int halfCycles,
            String rampEndReason,
            CsvLogger sampleLog) {
        ElapsedTime timer = new ElapsedTime();
        double totalRev = halfCycles * halfCycle;
        while (opMode.nextFrame() && timer.seconds() < totalRev) {
            double t = timer.seconds();
            int halfIdx = (int) (t / halfCycle);
            if (halfIdx >= halfCycles) {
                break;
            }
            double power = (halfIdx % 2 == 0) ? -kaPower : kaPower;
            setPower.accept(power);

            double batteryV = opMode.batteryVoltage();
            double appliedV = power * batteryV;
            double vel = wheelVel.getAsDouble();
            revT.add(t);
            revV.add(appliedV);
            revVel.add(vel);
            logSample(sampleLog, t, "reverse", halfIdx, power, batteryV, vel, readPoseIn(axisPos));
            maybeFlush(sampleLog);

            telemetry.addData("phase", "reverse %d / %d (timed)", halfIdx + 1, halfCycles);
            telemetry.addData("ramp end", rampEndReason);
            telemetry.addData("power", "%.2f", power);
            telemetry.addData("speed (in/s)", "%.1f", vel);
            telemetry.addData("rev samples", revT.size());
        }
    }

    /**
     * @param sBox single-element box holding live ∫vel position (updated in-place)
     * @param minHalfSec minimum duration of each half-cycle so kA always gets real reversals even
     *     if a position target is already satisfied (or stalls early)
     */
    private static void runPositionReversePhase(
            MarsLinearOpMode opMode,
            Telemetry telemetry,
            DoubleConsumer setPower,
            DoubleSupplier wheelVel,
            DoubleSupplier axisPos,
            double[] sBox,
            List<Double> revT,
            List<Double> revV,
            List<Double> revVel,
            double kaPower,
            double minHalfSec,
            int halfCycles,
            Corridor corridor,
            double stallSpeed,
            double stallTime,
            double stallMinPower,
            double stallFrac,
            String rampEndReason,
            CsvLogger sampleLog) {
        double absTravel = Math.abs(corridor.travelIn);
        // Allow long corridors; always at least ~2× minHalf so a full reverse can complete.
        double maxHalfSec = Math.max(minHalfSec * 2.5, absTravel / 6.0 + 1.5);
        double minHalf = Math.max(0.35, minHalfSec);
        ElapsedTime global = new ElapsedTime();

        for (int halfIdx = 0; halfIdx < halfCycles && opMode.opModeIsActive(); halfIdx++) {
            boolean reverse = (halfIdx % 2 == 0);
            double power = reverse ? -kaPower : kaPower;
            ElapsedTime halfTimer = new ElapsedTime();
            boolean everMoving = false;
            double peakSpeed = 0;
            double stallAccum = 0;
            double lastT = global.seconds();
            double halfStartS = sBox[0];
            // Require a few inches of progress in the commanded direction before stall may end the
            // half.
            double minProgress = Math.min(4.0, 0.15 * absTravel);

            while (opMode.nextFrame() && halfTimer.seconds() < maxHalfSec) {
                double t = global.seconds();
                double dt = t - lastT;
                lastT = t;

                setPower.accept(power);
                double vel = wheelVel.getAsDouble();
                if (dt > 1e-4 && dt < 0.5) {
                    sBox[0] += vel * dt;
                }
                double pos = sBox[0];
                double speed = Math.abs(vel);
                double batteryV = opMode.batteryVoltage();
                double appliedV = power * batteryV;

                revT.add(t);
                revV.add(appliedV);
                revVel.add(vel);
                logSample(
                        sampleLog,
                        t,
                        "reverse",
                        halfIdx,
                        power,
                        batteryV,
                        vel,
                        readPoseIn(axisPos));
                maybeFlush(sampleLog);

                if (speed > DEFAULT_MOVING_SPEED * 0.5) {
                    everMoving = true;
                }
                if (speed > peakSpeed) {
                    peakSpeed = speed;
                }

                double progress =
                        reverse
                                ? (halfStartS - pos) * corridor.travelSign // motion toward start
                                : (pos - halfStartS) * corridor.travelSign; // motion toward end
                boolean atTarget =
                        reverse ? reverseReached(pos, corridor) : forwardReached(pos, corridor);
                boolean minTimeOk = halfTimer.seconds() >= minHalf;
                if (atTarget && minTimeOk) {
                    break;
                }

                // Stall exit only after minimum time + some progress (avoids ending reverse
                // while still pressed on the wall at the start of the half).
                boolean collapsing =
                        isVelocityCollapsed(
                                everMoving,
                                Math.abs(power),
                                speed,
                                peakSpeed,
                                stallSpeed,
                                stallFrac,
                                stallMinPower);
                if (collapsing && minTimeOk && progress >= minProgress) {
                    if (dt > 0 && dt < 0.5) {
                        stallAccum += dt;
                    }
                    if (stallTime > 0 && stallAccum >= stallTime) {
                        break;
                    }
                } else if (!collapsing) {
                    stallAccum = 0;
                }

                telemetry.addData("phase", "reverse %d / %d (corridor)", halfIdx + 1, halfCycles);
                telemetry.addData("ramp end", rampEndReason);
                telemetry.addData("power", "%.2f", power);
                telemetry.addData("speed (in/s)", "%.1f", vel);
                telemetry.addData("s (in)", "%.1f", pos);
                telemetry.addData(
                        "target (in)", "%.1f", reverse ? corridor.nearStart : corridor.nearEnd);
                telemetry.addData("progress (in)", "%.1f", progress);
                telemetry.addData("half t (s)", "%.1f / %.1f", halfTimer.seconds(), maxHalfSec);
                telemetry.addData("rev samples", revT.size());
            }
        }
    }

    /**
     * True when stall detection may fire: either speed once exceeded the moving threshold, or the
     * robot has traveled enough distance (covers slow lateral ramps that hit a wall early).
     */
    static boolean isStallArmed(
            boolean everMoving, boolean havePos, double travelAbs, double armTravelIn) {
        if (everMoving) {
            return true;
        }
        return havePos && armTravelIn > 0 && travelAbs >= armTravelIn;
    }

    /**
     * True when the robot has been moving and speed has collapsed under power — typical of wall
     * contact. {@code speed} and {@code peakVel} should be non-negative magnitudes (in/s). {@code
     * armed} replaces the old "everMoving" gate (see {@link #isStallArmed}). Package-visible for
     * unit tests.
     */
    static boolean isVelocityCollapsed(
            boolean armed,
            double power,
            double speed,
            double peakVel,
            double stallSpeed,
            double stallFrac,
            double stallMinPower) {
        if (!armed || power < stallMinPower) {
            return false;
        }
        double frac = stallFrac > 0 ? stallFrac : 0;
        double thresh = Math.max(stallSpeed, peakVel * frac);
        return speed < thresh;
    }

    /**
     * True when stall is armed, power is applied, and position is barely advancing — wall contact
     * with noisy / non-zero velocity (common on mecanum strafe).
     */
    static boolean isPositionFrozen(
            boolean armed,
            double power,
            double posRateInPerSec,
            double stallMinPower,
            double posStallSpeed) {
        if (!armed || power < stallMinPower) {
            return false;
        }
        if (!Double.isFinite(posRateInPerSec) || posStallSpeed < 0) {
            return false;
        }
        return posRateInPerSec < posStallSpeed;
    }

    /**
     * Drop trailing ramp samples from the velocity crash into an obstacle so high-V / low-v points
     * do not bias {@code kS}. Keeps at least a few samples when possible.
     */
    static void trimPostPeakCollapse(
            List<Double> t, List<Double> voltage, List<Double> vel, double peakVel) {
        if (peakVel <= 0 || t.isEmpty() || t.size() != vel.size() || t.size() != voltage.size()) {
            return;
        }
        double keepAbove = peakVel * 0.85;
        while (t.size() > 8 && vel.get(vel.size() - 1) < keepAbove) {
            int i = t.size() - 1;
            t.remove(i);
            voltage.remove(i);
            vel.remove(i);
        }
    }

    /**
     * Fit kS/kV from a unidirectional ramp series and kA from a reverse series. Package-visible for
     * unit tests.
     */
    static Result fitRampAndReverse(
            double[] rampT,
            double[] rampVoltage,
            double[] rampVel,
            double[] revT,
            double[] revVoltage,
            double[] revVel,
            double inPerTick,
            double signDeadband,
            double kaWindow,
            double kaStride) {
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
                DEFAULT_MIN_RAMP_TICKS_PER_SEC);
    }

    /**
     * @param minRampTicksPerSec ramp samples slower than this (tick units/s × {@code inPerTick} →
     *     in/s) are excluded from the kS/kV fit
     */
    static Result fitRampAndReverse(
            double[] rampT,
            double[] rampVoltage,
            double[] rampVel,
            double[] revT,
            double[] revVoltage,
            double[] revVel,
            double inPerTick,
            double signDeadband,
            double kaWindow,
            double kaStride,
            double minRampTicksPerSec) {
        // Drop the breakaway knee: V rises with almost no motion until static friction breaks.
        // signDeadband alone is often ~1 in/s (~300 ticks/s); 1000 ticks/s is a better cut.
        double rampMinSpeed = Math.max(signDeadband, Math.max(0, minRampTicksPerSec) * inPerTick);
        RampFit ramp = fitRamp(rampVoltage, rampVel, rampT, rampMinSpeed);
        if (ramp.singular) {
            return new Result(
                    true,
                    0,
                    0,
                    0,
                    rampVoltage.length + revT.length,
                    ramp.used,
                    ramp.r2,
                    ramp.message);
        }

        KaFit ka =
                fitKa(
                        revT,
                        revVoltage,
                        revVel,
                        ramp.kS,
                        ramp.kVInch,
                        signDeadband,
                        kaWindow,
                        kaStride);
        int total = rampVoltage.length + revT.length;
        if (ka.singular) {
            return new Result(
                    true,
                    ramp.kS,
                    ramp.kVInch * inPerTick,
                    0,
                    total,
                    ramp.used,
                    ramp.r2,
                    ka.message);
        }

        return new Result(
                false,
                ramp.kS,
                ramp.kVInch * inPerTick,
                ka.kAInch * inPerTick,
                total,
                ramp.used,
                ramp.r2,
                "ok");
    }

    /** Package-visible for unit tests. */
    static final class RampFit {
        final boolean singular;
        final double kS;
        final double kVInch;
        final double r2;
        final int used;
        final String message;

        RampFit(boolean singular, double kS, double kVInch, double r2, int used, String message) {
            this.singular = singular;
            this.kS = kS;
            this.kVInch = kVInch;
            this.r2 = r2;
            this.used = used;
            this.message = message;
        }
    }

    private static final class KaFit {
        final boolean singular;
        final double kAInch;
        final String message;

        KaFit(boolean singular, double kAInch, String message) {
            this.singular = singular;
            this.kAInch = kAInch;
            this.message = message;
        }
    }

    /**
     * Unidirectional ramp regression matching ForwardRampLogger's {@code V = kS + kV·v}.
     *
     * <p>Samples with {@code |v| ≤ signDeadband} are dropped — callers should set that high enough
     * to clear the non-linear breakaway knee (see {@link #DEFAULT_MIN_RAMP_TICKS_PER_SEC}).
     *
     * <p>{@code kV} (slope) comes from {@link TunerRegression#robustFit} over the moving ramp —
     * slope is well conditioned. {@code kS} (intercept) is then re-estimated as the mean residual
     * {@code V − kV·v} on the <em>lowest-accel</em> samples: a continuous ramp never fully settles,
     * so a joint intercept fit absorbs {@code kA·a} and biases {@code kS} high; residual mean on
     * near-steady samples matches what the dashboard line fit does after outlier removal.
     */
    static RampFit fitRamp(double[] voltage, double[] vel, double signDeadband) {
        return fitRamp(voltage, vel, null, signDeadband);
    }

    /**
     * @param timeSec optional sample times for finite-difference accel; improves kS residual pick
     */
    static RampFit fitRamp(double[] voltage, double[] vel, double[] timeSec, double signDeadband) {
        if (voltage == null || vel == null || voltage.length != vel.length || voltage.length < 8) {
            return new RampFit(true, 0, 0, Double.NaN, 0, "too few ramp samples");
        }
        if (timeSec != null && timeSec.length != voltage.length) {
            return new RampFit(true, 0, 0, Double.NaN, 0, "ramp time length mismatch");
        }

        int pos = 0, neg = 0;
        for (double v : vel) {
            if (v > signDeadband) {
                pos++;
            } else if (v < -signDeadband) {
                neg++;
            }
        }
        boolean usePositive = pos >= neg;

        List<double[]> samples = new ArrayList<>();
        // parallel lists for residual-kS pass (same quadrant as samples)
        List<Double> resV = new ArrayList<>();
        List<Double> resVel = new ArrayList<>();
        List<Double> resAbsA = new ArrayList<>();

        for (int i = 0; i < voltage.length; i++) {
            double v = vel[i];
            double y;
            double x;
            if (usePositive) {
                if (v <= signDeadband) {
                    continue;
                }
                x = v;
                y = voltage[i];
            } else {
                if (v >= -signDeadband) {
                    continue;
                }
                x = -v;
                y = -voltage[i];
            }
            samples.add(new double[] {x, y});

            double absA = Double.POSITIVE_INFINITY;
            if (timeSec != null && i > 0) {
                double dt = timeSec[i] - timeSec[i - 1];
                if (dt > 1e-4) {
                    absA = Math.abs((vel[i] - vel[i - 1]) / dt);
                }
            }
            resV.add(y);
            resVel.add(x);
            resAbsA.add(absA);
        }

        if (samples.size() < 8) {
            return new RampFit(
                    true,
                    0,
                    0,
                    Double.NaN,
                    samples.size(),
                    "too few moving ramp samples — check localization sign (forward → +v)");
        }

        TunerRegression.Result fit = TunerRegression.robustFit(samples);
        if (fit == null || fit.used < 4) {
            return new RampFit(true, 0, 0, Double.NaN, samples.size(), "ramp regression failed");
        }
        if (fit.slope <= 0) {
            return new RampFit(
                    true,
                    fit.intercept,
                    fit.slope,
                    fit.r2,
                    fit.used,
                    "ramp kV ≤ 0 — localization sign is likely still inverted");
        }

        double kVInch = fit.slope;
        // Re-estimate kS at the ramp peak (hold): slope from the full ramp is solid, but the
        // joint intercept absorbs kA·a during the climb. Near peak voltage, a → 0 and
        // kS ≈ V − kV·v.
        double kS = refitKsAtPeak(resV, resVel, resAbsA, kVInch, fit.intercept);

        return new RampFit(false, kS, kVInch, fit.r2, fit.used, "ok");
    }

    /**
     * Mean of {@code V − kV·v} on near-peak-voltage samples with the lowest accel (settled hold at
     * the end of the ramp). Falls back to the joint-fit intercept if too few samples.
     */
    private static double refitKsAtPeak(
            List<Double> V, List<Double> vel, List<Double> absA, double kVInch, double fallbackKS) {
        int n = V.size();
        if (n == 0) {
            return fallbackKS;
        }
        double vmax = 0;
        for (double y : V) {
            vmax = Math.max(vmax, y);
        }
        if (vmax < 1e-6) {
            return fallbackKS;
        }
        // Peak band: top ~5% of applied voltage (the hold at RAMP_MAX).
        double vThresh = 0.95 * vmax;

        // Collect residuals in the peak band; prefer low |a| when available.
        List<double[]> peak = new ArrayList<>(); // [absA, residual]
        for (int i = 0; i < n; i++) {
            if (V.get(i) < vThresh) {
                continue;
            }
            double r = V.get(i) - kVInch * vel.get(i);
            double a = absA.get(i);
            peak.add(new double[] {Double.isFinite(a) ? a : 0.0, r});
        }
        if (peak.size() < 4) {
            return fallbackKS;
        }
        // Use the lowest-accel half of the peak band (most settled).
        peak.sort(java.util.Comparator.comparingDouble(p -> p[0]));
        int keep = Math.max(4, peak.size() / 2);
        double sum = 0;
        for (int i = 0; i < keep; i++) {
            sum += peak.get(i)[1];
        }
        return sum / keep;
    }

    /** Residual kA from reverse-phase windows: {@code ∫(V − kS·sign(v) − kV·v) dt = kA·Δv}. */
    static KaFit fitKa(
            double[] t,
            double[] voltage,
            double[] vel,
            double kS,
            double kVInch,
            double signDeadband,
            double kaWindow,
            double kaStride) {
        int n = t.length;
        if (n < 8 || voltage.length != n || vel.length != n) {
            return new KaFit(true, 0, "too few reverse samples for kA");
        }

        double num = 0, den = 0;
        int windows = 0;
        double tEnd = t[n - 1];
        for (double wStart = t[0]; wStart + kaWindow <= tEnd + 1e-9; wStart += kaStride) {
            int i0 = lowerBound(t, wStart);
            int i1 = lowerBound(t, wStart + kaWindow);
            if (i1 >= n) {
                i1 = n - 1;
            }
            if (i1 <= i0 + 1) {
                continue;
            }
            double residInt = 0;
            for (int k = i0; k < i1; k++) {
                double dt = t[k + 1] - t[k];
                if (dt <= 0) {
                    continue;
                }
                double r0 = residual(voltage[k], vel[k], kS, kVInch, signDeadband);
                double r1 = residual(voltage[k + 1], vel[k + 1], kS, kVInch, signDeadband);
                residInt += 0.5 * (r0 + r1) * dt;
            }
            double dv = vel[i1] - vel[i0];
            num += residInt * dv;
            den += dv * dv;
            windows++;
        }

        if (den < 1e-6 || windows < 4) {
            return new KaFit(
                    true, 0, "kA not identifiable — more reverse half-cycles or higher KA_POWER");
        }
        double kA = num / den;
        // Passive inertia cannot produce a negative kA; a negative fit almost always means the
        // reverse phase did not actually reverse (stuck on a wall, half-cycles skipped, etc.).
        if (!(kA > 0)) {
            return new KaFit(
                    true,
                    kA,
                    "kA ≤ 0 ("
                            + String.format("%.4g", kA)
                            + ") — reverse phase likely did not excite accel; re-run and confirm "
                            + "the robot leaves the wall and squares back and forth");
        }
        return new KaFit(false, kA, "ok");
    }

    private static double residual(
            double voltage, double vel, double kS, double kVInch, double dead) {
        double s = Math.abs(vel) < dead ? 0.0 : Math.signum(vel);
        return voltage - kS * s - kVInch * vel;
    }

    private static int lowerBound(double[] t, double x) {
        int lo = 0, hi = t.length;
        while (lo < hi) {
            int mid = (lo + hi) >>> 1;
            if (t[mid] < x) {
                lo = mid + 1;
            } else {
                hi = mid;
            }
        }
        return lo;
    }

    private static double[] toArray(List<Double> list) {
        double[] a = new double[list.size()];
        for (int i = 0; i < a.length; i++) {
            a[i] = list.get(i);
        }
        return a;
    }
}
