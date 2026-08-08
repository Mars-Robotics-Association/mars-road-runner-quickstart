package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.TwoDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.opmodes.base.MarsLinearOpMode;
import org.firstinspires.ftc.teamcode.utils.CsvLogger;

import java.util.ArrayList;
import java.util.List;

/**
 * Automatic on-robot tuning of the trajectory-follower feedback gains — the automated counterpart
 * of eyeballing {@code ManualFeedbackTuner}.
 *
 * <p>The identified feedforward already contains a plant model: with voltage-compensated
 * feedforward, each axis closes a loop whose error dynamics are second order with motor time
 * constant {@code tau = kA / kV}. For a chosen bandwidth {@code omega_n} and damping {@code zeta},
 * the gains follow directly: {@code posGain = omega_n^2 * tau} and {@code velGain = 2 * zeta *
 * omega_n * tau - 1} (clamped at 0). So instead of guessing gains, the tuner searches over a single
 * knob — bandwidth.
 *
 * <p>For each axis (heading, then axial, then lateral on mecanum; turn only on tank) it runs
 * profiled bump tests: a smooth cosine reference moves the target to an offset pose (and back), the
 * holonomic controller tracks it through the production feedforward path, and the response is
 * scored on overshoot, error-sign reversals, velocity chatter, and settling. Starting from a
 * conservative bandwidth, it raises {@code omega_n} by {@link #GROWTH} each time the response stays
 * clean, and stops at the last clean setting once overshoot or oscillation appears — the classic
 * "turn it up until it rings, then back off," automated. Each iteration steps out and back, so the
 * robot finishes where it started.
 *
 * <p>During a measurement bump only the axis under test has feedback gains; other axes stay at zero
 * so the leg is free motion (full multi-axis HOLD confounded isolation). Exception: lateral bumps
 * use a soft {@linkplain #HOLD_HEADING_POS heading hold} as a measurement fixture — open-loop
 * strafe curls on this chassis (asterisk-shaped tracks) and pollutes the lateral score; same idea
 * as {@code LateralFeedforwardTuner}. {@linkplain #HOLD_POS_GAIN Hold-level} translation gains are
 * used only for the between-phase {@linkplain #recenterToHome recenter} (after heading, after
 * axial).
 *
 * <p>A pure static setpoint step is deliberately avoided: commanding through {@code kS · sign(v)}
 * with a fixed target spends the whole test in the low-speed relay region and produces a
 * high-frequency "chihuahua" shake that does not reflect trajectory tracking (where the reference
 * velocity keeps the sign consistent until the profile ends). The profiled reference matches how
 * the gains are used on real paths.
 *
 * <p>Prerequisites: localization and feedforward must be tuned first, including {@code kA} (run
 * {@code AxialFeedforwardTuner}; {@code kA} sets the time constant the synthesis relies on). Clear
 * about 2 ft around the robot in every direction it will step. The step tests drive through {@link
 * MecanumDrive#setDriveCommand} / {@link TankDrive#setDriveCommand}, so the exact production
 * feedforward path (anisotropic constants, yaw coupling, voltage compensation) is in the loop.
 *
 * <p>When it finishes, the gains are written into the live {@code PARAMS} statics (same as the
 * other automatic tuners) so they are active for the rest of this RC process, and shown on
 * telemetry to paste into source for persistence. Verify with {@code ManualFeedbackTuner} and
 * {@code SplineTest}.
 *
 * <p>Each run writes two CSVs under {@code /sdcard/FIRST/} (via {@link CsvLogger}):
 *
 * <ul>
 *   <li>{@code fbgain_samples_&lt;stamp&gt;.csv} — every control loop sample (pose, ref, error,
 *       velocity, command, chatter counters)
 *   <li>{@code fbgain_summary_&lt;stamp&gt;.csv} — one row per leg with verdict metrics and
 *       hold-window FFT peaks ({@code fft_err_hz}, {@code fft_err_ratio}, {@code fft_cmd_*}, {@code
 *       fft_period_s}, {@code fft_err_rms})
 * </ul>
 *
 * Pull them with {@code telemetry/pull.sh} (same pattern as the Curiosity repo).
 *
 * <p>Oscillation detection uses a real DFT on the hold window ({@code progress ≥ 1}, at least
 * {@link #FFT_MIN_HOLD_SEC}). Peak/median alone is a poor detector on quiet holds (sensor noise
 * looks like a huge ratio), so peaks are gated by hold-error RMS vs the settle band and by a
 * minimum frequency that excludes the single settle-tail cycle of the hold window. A strong
 * spectral line that also appears in the axis command, with meaningful error amplitude, is treated
 * as closed-loop ring (not mere localizer glitter).
 */
@Config
public final class FeedbackGainTuner extends MarsLinearOpMode {
    /** When false, skip writing CSVs (useful if the hub disk is full). */
    public static boolean LOG_CSV = true;

    /** Flush the sample CSV to disk after this many buffered rows. */
    public static int LOG_FLUSH_EVERY = 256;

    /** Translational step size for the axial/lateral tests, in inches. */
    public static double STEP_INCHES = 8.0;

    /** Heading step size for the heading/turn tests, in degrees. */
    public static double STEP_DEGREES = 40.0;

    /** Target damping ratio for the synthesized gains (1 = critically damped). */
    public static double ZETA = 1.0;

    /**
     * Initial bandwidth, as a fraction of the plant pole 1/tau. Above 0.5 so the first synthesized
     * {@code velGain = 2 ζ ω τ − 1} is positive (critical damping needs plant + derivative).
     */
    public static double START_BANDWIDTH = 0.6;

    /** Absolute ceiling on starting {@code omega_n} (rad/s), regardless of tau. */
    public static double MAX_START_OMEGA = 3.0;

    /**
     * Position gain for between-phase {@link #recenterToHome} (and gentle post-reject returns). Not
     * applied to non-tested translation axes during measurement bumps.
     */
    public static double HOLD_POS_GAIN = 2.5;

    /** Velocity gain paired with {@link #HOLD_POS_GAIN} for recenter / gentle return. */
    public static double HOLD_VEL_GAIN = 0.3;

    /**
     * Soft heading position gain: between-phase recenter, and as a fixture during lateral
     * measurement bumps (keeps strafe from curling into an asterisk).
     */
    public static double HOLD_HEADING_POS = 0.7;

    /** Soft heading velocity gain paired with {@link #HOLD_HEADING_POS}. */
    public static double HOLD_HEADING_VEL = 0.45;

    /**
     * Ceiling on heading (and tank-turn) position gain. Translation can go to {@link
     * #MAX_POS_GAIN}; heading above ~12 with free chassis scrub tends to thrash even when the yaw
     * error is small.
     */
    public static double MAX_HEADING_POS_GAIN = 10.0;

    /** Bandwidth multiplier applied after a clean (or sluggish) step response. */
    public static double GROWTH = 1.4;

    /** Bandwidth multiplier applied after a bad response before any clean one is found. */
    public static double BACKOFF = 0.6;

    /** Maximum step-test iterations per axis. */
    public static int MAX_ITERS = 6;

    /** Position gain ceiling; the search stops growing once synthesis would exceed it. */
    public static double MAX_POS_GAIN = 20.0;

    /**
     * Overshoot (fraction of the step) treated as "too hot to grow further." A single overshoot of
     * ~15–25% that still settles is normal on a real chassis; only reject hard when combined with
     * multi-cycle ring or when overshoot exceeds {@link #REJECT_OVERSHOOT_FRAC}.
     */
    public static double MAX_OVERSHOOT_FRAC = 0.28;

    /**
     * Overshoot above this fraction fails the setting even if the robot eventually settles (one
     * huge lunge is not a usable gain).
     */
    public static double REJECT_OVERSHOOT_FRAC = 0.45;

    /**
     * Error sign reversals (beyond the settle band) above which a response is rejected as ringing.
     */
    public static int MAX_REVERSALS = 2;

    /**
     * Axis-velocity sign flips (above a deadband) during one step above which a response is
     * rejected — catches kS limit-cycle "chihuahua" shake that barely moves position.
     */
    public static int MAX_VEL_FLIPS = 12;

    /** |velocity| below this does not count toward {@link #MAX_VEL_FLIPS}. */
    public static double VEL_FLIP_DEADBAND = 0.5;

    /** Settle band for translational steps, in inches. */
    public static double SETTLE_TOL_IN = 0.6;

    /** Settle band for heading steps, in degrees. */
    public static double SETTLE_TOL_DEG = 2.5;

    /** Time the error must stay inside the settle band, in seconds. */
    public static double SETTLE_HOLD_SEC = 0.35;

    /** Seconds for the smooth reference to travel from start pose to the step target. */
    public static double MOVE_SEC = 1.2;

    /** Per-step timeout, in seconds (move + FFT hold window). */
    public static double STEP_TIMEOUT_SEC = 5.0;

    /**
     * After the reference has arrived ({@code progress ≥ 1}), if |error| stays above this fraction
     * of the step with |axis velocity| below {@link #STALL_VEL} for {@link #STALL_HOLD_SEC}, the
     * leg is treated as stuck (wall / immovable) and aborted.
     */
    public static double STALL_ERR_FRAC = 0.35;

    /** |axis velocity| below this counts as stalled when error is still large. */
    public static double STALL_VEL = 1.0;

    /** Seconds of simultaneous large error + low velocity (after the move) before stall abort. */
    public static double STALL_HOLD_SEC = 0.45;

    /** Skip between-phase recenter when already this close to field home (inches). */
    public static double RECENTER_TOL_IN = 0.75;

    /** Skip between-phase recenter when heading is already this close to field home (degrees). */
    public static double RECENTER_TOL_DEG = 4.0;

    /** Timeout for a between-phase recenter to field home (seconds). */
    public static double RECENTER_TIMEOUT_SEC = 3.0;

    /**
     * Minimum time to remain in the hold phase ({@code progress ≥ 1}) before ending a leg, so the
     * hold-window FFT has enough samples. Settle can flag earlier, but the loop keeps running.
     */
    public static double FFT_MIN_HOLD_SEC = 0.85;

    /**
     * Lowest frequency (Hz) considered for oscillatory peaks. Must be well above {@code 1 /
     * FFT_MIN_HOLD_SEC} so a single settle-tail half-cycle (≈1.1 Hz on an 0.85 s hold) is not
     * mistaken for sustained ring. Real drivetrain thrash from this chassis has been ~5–8 Hz.
     */
    public static double FFT_FMIN_HZ = 2.5;

    /** Highest frequency (Hz) considered (drivetrain ring is usually well below this). */
    public static double FFT_FMAX_HZ = 15.0;

    /**
     * Hold-window error spectrum peak/median ratio above this → "at edge" (keep gain, stop
     * growing), <em>only if</em> {@link #FFT_ERR_RMS_EDGE_MUL} is also met. Quiet holds have tiny
     * medians, so ratio alone false-triggers.
     */
    public static double FFT_PEAK_RATIO_EDGE = 8.0;

    /**
     * Error peak/median above this <em>and</em> a matching command peak <em>and</em> meaningful
     * hold-error RMS → hard-reject (ringing in the loop, not just a soft settle tail).
     */
    public static double FFT_PEAK_RATIO_REJECT = 12.0;

    /** Command spectrum must reach at least this peak/median to confirm a loop ring with error. */
    public static double FFT_CMD_RATIO_MIN = 4.0;

    /** |f_err − f_cmd| / f_err below this counts as the same spectral line. */
    public static double FFT_FREQ_MATCH_FRAC = 0.35;

    /**
     * Hold-error RMS (mean-removed) must be at least this multiple of the settle band before an
     * error spectral peak can stop growth ({@link StepResponse#fftEdge()}). Below this the hold is
     * "quiet enough" that peak/median is just noise shape. Keep ≥1 so a mild overshoot settle tail
     * does not freeze the search on iter 1.
     */
    public static double FFT_ERR_RMS_EDGE_MUL = 1.25;

    /**
     * Hold-error RMS must be at least this multiple of the settle band before FFT can hard-reject
     * as loop ring. Higher than the edge mul so mild residual hunting only soft-stops.
     */
    public static double FFT_ERR_RMS_REJECT_MUL = 2.0;

    /**
     * Consecutive hard-reject iterations (with no clean best yet) before giving up an axis. Avoids
     * six full timeout thrash cycles when lateral kS is hunting at every bandwidth.
     */
    public static int MAX_HARD_REJECT_STREAK = 2;

    /** Minimum uniformly resampled hold samples before trusting the FFT. */
    public static int FFT_MIN_SAMPLES = 24;

    /** Metrics accumulated over one step response. */
    private static final class StepResponse {
        double e0 = Double.NaN;
        double eLast = Double.NaN;
        double minNorm = 1.0;
        int reversals = 0;
        int lastRegion = 0;
        int velFlips = 0;
        int lastVelSign = 0;
        double settledAt = Double.NaN;
        boolean settled = false;
        double stallAt = Double.NaN;
        boolean stalled = false;
        double holdStart = Double.NaN;

        // Hold-window FFT (filled by {@link #finalizeFft}).
        double fftErrHz = Double.NaN;
        double fftErrRatio = Double.NaN;
        double fftCmdHz = Double.NaN;
        double fftCmdRatio = Double.NaN;
        double fftPeriodSec = Double.NaN;

        /** Mean-removed RMS of hold-window error (axis units). */
        double fftErrRms = Double.NaN;

        boolean fftOk = false;

        /** Settle band used for this leg (for amplitude-gating the FFT). */
        double settleTol = Double.NaN;

        private final List<Double> holdT = new ArrayList<>();
        private final List<Double> holdErr = new ArrayList<>();
        private final List<Double> holdCmd = new ArrayList<>();

        /**
         * Ingests one sample. Returns true when the leg may end: stalled, or settled with a long
         * enough hold window for FFT. {@code axisCmd} is the commanded velocity along the test
         * axis.
         */
        boolean update(
                double t, double e, double axisVel, double axisCmd, double tol, double progress) {
            if (Double.isNaN(e0)) e0 = e;
            if (Double.isNaN(settleTol)) settleTol = tol;
            eLast = e;
            double mag0 = Math.max(Math.abs(e0), 1e-6);
            double n = e / (e0 == 0 ? mag0 : e0);
            minNorm = Math.min(minNorm, n);
            double band = tol / mag0;
            int region = n > band ? 1 : (n < -band ? -1 : 0);
            if (region != 0) {
                if (lastRegion != 0 && region != lastRegion) reversals++;
                lastRegion = region;
            }
            int velSign = axisVel > VEL_FLIP_DEADBAND ? 1 : (axisVel < -VEL_FLIP_DEADBAND ? -1 : 0);
            if (velSign != 0) {
                if (lastVelSign != 0 && velSign != lastVelSign) velFlips++;
                lastVelSign = velSign;
            }
            // Wall / immovable: reference is done, error still large, almost no axis motion.
            if (progress >= 1.0
                    && Math.abs(e) > STALL_ERR_FRAC * mag0
                    && Math.abs(axisVel) < STALL_VEL) {
                if (Double.isNaN(stallAt)) stallAt = t;
                if (t - stallAt >= STALL_HOLD_SEC) {
                    stalled = true;
                    return true;
                }
            } else {
                stallAt = Double.NaN;
            }
            if (Math.abs(e) <= tol) {
                if (Double.isNaN(settledAt)) settledAt = t;
                if (t - settledAt >= SETTLE_HOLD_SEC) {
                    settled = true;
                }
            } else {
                settledAt = Double.NaN;
            }
            // Hold-phase buffer for FFT (only after the profile has arrived).
            if (progress >= 1.0) {
                if (Double.isNaN(holdStart)) holdStart = t;
                holdT.add(t);
                holdErr.add(e);
                holdCmd.add(axisCmd);
                // End only once settled *and* we have enough hold time for a trustworthy spectrum.
                if (settled && (t - holdStart) >= FFT_MIN_HOLD_SEC) {
                    return true;
                }
            }
            return false;
        }

        /** Runs the hold-window FFT after the control loop exits (including timeouts). */
        void finalizeFft() {
            fftErrRms = HoldSpectrum.rmsMeanRemoved(holdErr);
            FftPeak errPeak = HoldSpectrum.analyze(holdT, holdErr);
            FftPeak cmdPeak = HoldSpectrum.analyze(holdT, holdCmd);
            if (errPeak != null) {
                fftErrHz = errPeak.hz;
                fftErrRatio = errPeak.peakOverMedian;
                fftPeriodSec = errPeak.hz > 1e-6 ? 1.0 / errPeak.hz : Double.NaN;
                fftOk = true;
            }
            if (cmdPeak != null) {
                fftCmdHz = cmdPeak.hz;
                fftCmdRatio = cmdPeak.peakOverMedian;
            }
        }

        double overshootFrac() {
            return Math.max(0, -minNorm);
        }

        /** Fraction of the initial error closed by the end of the step. */
        double movedFrac() {
            if (Double.isNaN(e0) || Math.abs(e0) < 1e-6) return 0;
            return 1 - Math.abs(eLast) / Math.abs(e0);
        }

        /**
         * True when hold-error RMS is large enough vs the settle band that a spectral peak is
         * physically meaningful (not just shape-of-noise on a quiet hold).
         */
        boolean fftAmplitudeOk(double mulOfTol) {
            if (Double.isNaN(fftErrRms) || Double.isNaN(settleTol) || settleTol <= 0) {
                return false;
            }
            return fftErrRms >= mulOfTol * settleTol;
        }

        /** Error and command share a strong spectral line in the ring band, with real amplitude. */
        boolean fftLoopRing() {
            if (!fftOk
                    || !fftAmplitudeOk(FFT_ERR_RMS_REJECT_MUL)
                    || Double.isNaN(fftErrRatio)
                    || Double.isNaN(fftCmdRatio)
                    || Double.isNaN(fftErrHz)
                    || Double.isNaN(fftCmdHz)) {
                return false;
            }
            if (fftErrRatio < FFT_PEAK_RATIO_REJECT || fftCmdRatio < FFT_CMD_RATIO_MIN) {
                return false;
            }
            double rel = Math.abs(fftErrHz - fftCmdHz) / Math.max(fftErrHz, 1e-6);
            return rel <= FFT_FREQ_MATCH_FRAC;
        }

        /**
         * Error spectrum is peaky enough to stop growing bandwidth — only when the residual hold
         * error is still large vs the settle band (quiet holds must not freeze the search).
         */
        boolean fftEdge() {
            return fftOk
                    && fftAmplitudeOk(FFT_ERR_RMS_EDGE_MUL)
                    && !Double.isNaN(fftErrRatio)
                    && fftErrRatio >= FFT_PEAK_RATIO_EDGE;
        }

        /**
         * Hard reject: multi-cycle ring, velocity chatter, massive overshoot, or FFT-confirmed loop
         * oscillation. A single mild overshoot that settles is <em>not</em> hard-fail.
         */
        boolean hardReject() {
            return velFlips > MAX_VEL_FLIPS
                    || reversals > MAX_REVERSALS
                    || overshootFrac() > REJECT_OVERSHOOT_FRAC
                    || fftLoopRing();
        }

        /** Soft edge: do not grow ω further (overshoot, multi-reversal, or peaky hold spectrum). */
        boolean atEdge() {
            return overshootFrac() > MAX_OVERSHOOT_FRAC || reversals > 1 || fftEdge();
        }
    }

    /** One spectral peak from a hold-window real DFT. */
    private static final class FftPeak {
        final double hz;
        final double peakOverMedian;

        FftPeak(double hz, double peakOverMedian) {
            this.hz = hz;
            this.peakOverMedian = peakOverMedian;
        }
    }

    /**
     * Hold-window spectrum: resample to uniform {@code dt}, linear-detrend, Hann window, direct
     * real DFT magnitudes, then pick the strongest bin in {@link #FFT_FMIN_HZ}…{@link
     * #FFT_FMAX_HZ}.
     */
    private static final class HoldSpectrum {
        private HoldSpectrum() {}

        /** Mean-removed RMS of a raw (non-uniform) hold series; NaN if empty. */
        static double rmsMeanRemoved(List<Double> values) {
            int n = values.size();
            if (n == 0) return Double.NaN;
            double mean = 0;
            for (int i = 0; i < n; i++) mean += values.get(i);
            mean /= n;
            double acc = 0;
            for (int i = 0; i < n; i++) {
                double d = values.get(i) - mean;
                acc += d * d;
            }
            return Math.sqrt(acc / n);
        }

        static FftPeak analyze(List<Double> tSec, List<Double> values) {
            int n = Math.min(tSec.size(), values.size());
            if (n < FFT_MIN_SAMPLES) return null;

            double[] t = new double[n];
            double[] v = new double[n];
            for (int i = 0; i < n; i++) {
                t[i] = tSec.get(i);
                v[i] = values.get(i);
            }
            // Median sample interval from the raw hold stream.
            double[] dts = new double[n - 1];
            for (int i = 0; i < n - 1; i++) {
                dts[i] = t[i + 1] - t[i];
            }
            java.util.Arrays.sort(dts);
            double dt = dts[dts.length / 2];
            if (!(dt > 1e-4) || !(dt < 0.1)) return null;

            double t0 = t[0];
            double t1 = t[n - 1];
            int nu = (int) Math.floor((t1 - t0) / dt) + 1;
            if (nu < FFT_MIN_SAMPLES) return null;
            // Cap length so a slow loop cannot blow the DFT cost (rare).
            if (nu > 512) {
                dt = (t1 - t0) / 511.0;
                nu = 512;
            }

            double[] u = new double[nu];
            int j = 0;
            for (int i = 0; i < nu; i++) {
                double ti = t0 + i * dt;
                while (j < n - 2 && t[j + 1] < ti) j++;
                double tL = t[j];
                double tR = t[Math.min(j + 1, n - 1)];
                double vL = v[j];
                double vR = v[Math.min(j + 1, n - 1)];
                double a = tR > tL ? (ti - tL) / (tR - tL) : 0;
                u[i] = vL + a * (vR - vL);
            }

            // Linear detrend (not just mean): a settle approach looks like a ramp/decay that
            // otherwise dumps energy into the lowest frequency bins of a short hold window.
            linearDetrendInPlace(u);

            // Hann window
            for (int i = 0; i < nu; i++) {
                double w = 0.5 * (1.0 - Math.cos(2.0 * Math.PI * i / (nu - 1)));
                u[i] *= w;
            }

            int nBins = nu / 2 + 1;
            double[] mag = new double[nBins];
            for (int k = 0; k < nBins; k++) {
                double re = 0;
                double im = 0;
                for (int i = 0; i < nu; i++) {
                    double ang = -2.0 * Math.PI * k * i / nu;
                    re += u[i] * Math.cos(ang);
                    im += u[i] * Math.sin(ang);
                }
                mag[k] = Math.hypot(re, im);
            }

            // Also require fMin above ~1.5 cycles / window so a single transient cannot dominate.
            double windowSec = Math.max(t1 - t0, nu * dt);
            double fMin = Math.max(FFT_FMIN_HZ, 1.5 / Math.max(windowSec, 1e-3));
            double fMax = Math.min(FFT_FMAX_HZ, 0.5 / dt - 1e-6);
            if (fMax <= fMin) return null;

            int kLo = Math.max(1, (int) Math.ceil(fMin * nu * dt));
            int kHi = Math.min(nBins - 1, (int) Math.floor(fMax * nu * dt));
            if (kHi < kLo) return null;

            double peak = 0;
            int kPeak = kLo;
            int count = 0;
            for (int k = kLo; k <= kHi; k++) {
                count++;
                if (mag[k] > peak) {
                    peak = mag[k];
                    kPeak = k;
                }
            }
            if (count == 0 || peak <= 0) return null;
            double median = bandMedian(mag, kLo, kHi);
            if (median < 1e-15) median = 1e-15;
            double hz = kPeak / (nu * dt);
            return new FftPeak(hz, peak / median);
        }

        /** Remove best-fit line so slow settle does not look like a low-Hz oscillator. */
        private static void linearDetrendInPlace(double[] u) {
            int n = u.length;
            if (n < 2) return;
            double sumX = 0, sumY = 0, sumXX = 0, sumXY = 0;
            for (int i = 0; i < n; i++) {
                double x = i;
                sumX += x;
                sumY += u[i];
                sumXX += x * x;
                sumXY += x * u[i];
            }
            double denom = n * sumXX - sumX * sumX;
            if (Math.abs(denom) < 1e-18) {
                double mean = sumY / n;
                for (int i = 0; i < n; i++) u[i] -= mean;
                return;
            }
            double slope = (n * sumXY - sumX * sumY) / denom;
            double intercept = (sumY - slope * sumX) / n;
            for (int i = 0; i < n; i++) {
                u[i] -= intercept + slope * i;
            }
        }

        private static double bandMedian(double[] mag, int kLo, int kHi) {
            int n = kHi - kLo + 1;
            double[] tmp = new double[n];
            System.arraycopy(mag, kLo, tmp, 0, n);
            java.util.Arrays.sort(tmp);
            if ((n & 1) == 1) return tmp[n / 2];
            return 0.5 * (tmp[n / 2 - 1] + tmp[n / 2]);
        }
    }

    /** Runs one step test on a specific axis: outward from the start pose or back to it. */
    private interface StepTest {
        StepResponse run(boolean outward, double posGain, double velGain);
    }

    private static final class AxisResult {
        final boolean ok;
        final double posGain;
        final double velGain;
        final String note;

        AxisResult(boolean ok, double posGain, double velGain, String note) {
            this.ok = ok;
            this.posGain = posGain;
            this.velGain = velGain;
            this.note = note;
        }
    }

    private static final String SAMPLE_HEADER =
            "t_s,axis,iter,outward,posGain,velGain,"
                    + "g_ax,g_lat,g_h,g_axv,g_latv,g_hv,"
                    + "pose_x,pose_y,pose_h,ref_x,ref_y,ref_h,"
                    + "err_axis,vel_x,vel_y,vel_h,cmd_vx,cmd_vy,cmd_w,"
                    + "progress,vel_flips,reversals";

    private static final String SUMMARY_HEADER =
            "axis,iter,outward,posGain,velGain,tau,omega_n,"
                    + "e0,e_last,moved_frac,overshoot_frac,reversals,vel_flips,"
                    + "settled,stalled,hard_reject,"
                    + "fft_err_hz,fft_err_ratio,fft_cmd_hz,fft_cmd_ratio,fft_period_s,fft_err_rms,"
                    + "duration_s,verdict";

    private MultipleTelemetry telem;
    private CsvLogger sampleLog;
    private CsvLogger summaryLog;

    /** Axis name for the active search (written into every sample row). */
    private String logAxis = "";

    /** Iteration number for the active search (1-based). */
    private int logIter = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        telem = (MultipleTelemetry) telemetry;

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            runMecanum();
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            runTank();
        } else {
            throw new RuntimeException("Unknown DRIVE_CLASS");
        }
    }

    private void openLogs(String driveKind) {
        if (!LOG_CSV) return;
        String stamp =
                new java.text.SimpleDateFormat("yyyyMMdd_HHmmss", java.util.Locale.US)
                        .format(new java.util.Date());
        // Shared stamp so sample + summary files pair after telemetry/pull.sh.
        sampleLog = new CsvLogger("fbgain_samples_" + stamp + ".csv", SAMPLE_HEADER);
        summaryLog = new CsvLogger("fbgain_summary_" + stamp + ".csv", SUMMARY_HEADER);
        // One meta row (axis=meta) with plant params packed into the verdict column.
        String meta;
        double tau;
        if ("mecanum".equals(driveKind)) {
            MecanumDrive.Params p = MecanumDrive.PARAMS;
            tau = p.kA / p.kV;
            meta =
                    String.format(
                            java.util.Locale.US,
                            "drive=mecanum;kS=%.5f;kV=%.6e;kA=%.6e;latKS=%.5f;latKV=%.6e;latKA=%.6e;"
                                + "twTicks=%.1f;aniso=%s;inPerTick=%.6e;START_BANDWIDTH=%.3f;"
                                + "MOVE_SEC=%.2f;STEP_IN=%.1f;STEP_DEG=%.1f",
                            p.kS,
                            p.kV,
                            p.kA,
                            p.lateralKS,
                            p.lateralKV,
                            p.lateralKA,
                            p.trackWidthTicks,
                            p.useAnisotropicFeedforward,
                            p.inPerTick,
                            START_BANDWIDTH,
                            MOVE_SEC,
                            STEP_INCHES,
                            STEP_DEGREES);
        } else {
            TankDrive.Params p = TankDrive.PARAMS;
            tau = p.kA / p.kV;
            meta =
                    String.format(
                            java.util.Locale.US,
                            "drive=tank;kS=%.5f;kV=%.6e;kA=%.6e;twTicks=%.1f;inPerTick=%.6e;"
                                    + "START_BANDWIDTH=%.3f;MOVE_SEC=%.2f;STEP_DEG=%.1f",
                            p.kS,
                            p.kV,
                            p.kA,
                            p.trackWidthTicks,
                            p.inPerTick,
                            START_BANDWIDTH,
                            MOVE_SEC,
                            STEP_DEGREES);
        }
        // Meta packs plant params into verdict; numeric FFT fields left empty.
        summaryLog.row(
                "meta",
                0,
                "params",
                0,
                0,
                tau,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                false,
                false,
                false,
                Double.NaN,
                Double.NaN,
                Double.NaN,
                Double.NaN,
                Double.NaN,
                0,
                meta);
        summaryLog.flush();
    }

    /**
     * Builds the 6-gain vector for one measurement bump: the axis under test is closed; other axes
     * are zero except lateral, which gets a soft heading fixture ({@link #HOLD_HEADING_POS} /
     * {@link #HOLD_HEADING_VEL}) so open-loop curl does not dominate the scored lateral response.
     */
    private static double[] controllerGains(int axis, double posGain, double velGain) {
        double[] g = {0, 0, 0, 0, 0, 0};
        g[axis] = posGain;
        g[axis + 3] = velGain;
        if (axis == 1) {
            g[2] = HOLD_HEADING_POS;
            g[5] = HOLD_HEADING_VEL;
        }
        return g;
    }

    /** Synthesized (pos, vel) at the search start bandwidth for seeding / display. */
    private static double[] synthesizeStart(double tau) {
        double omega = Math.min(START_BANDWIDTH / tau, MAX_START_OMEGA);
        double kp = Math.min(omega * omega * tau, MAX_POS_GAIN);
        double kd = Math.max(0, 2 * ZETA * omega * tau - 1);
        return new double[] {kp, kd};
    }

    private void closeLogs() {
        if (sampleLog != null) {
            sampleLog.close();
            sampleLog = null;
        }
        if (summaryLog != null) {
            summaryLog.close();
            summaryLog = null;
        }
    }

    private void maybeFlushSamples() {
        if (sampleLog != null && sampleLog.bufferedRows() >= LOG_FLUSH_EVERY) {
            sampleLog.flush();
        }
    }

    private void runMecanum() {
        MecanumDrive drive =
                new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0), this::batteryVoltage);
        checkDeadWheels(drive.localizer);
        requirePositive(MecanumDrive.PARAMS.kV, MecanumDrive.PARAMS.kA);

        double tauAxial = MecanumDrive.PARAMS.kA / MecanumDrive.PARAMS.kV;
        double tauLateral = tauAxial;
        if (MecanumDrive.PARAMS.useAnisotropicFeedforward
                && MecanumDrive.PARAMS.lateralKV > 0
                && MecanumDrive.PARAMS.lateralKA > 0) {
            tauLateral = MecanumDrive.PARAMS.lateralKA / MecanumDrive.PARAMS.lateralKV;
        }

        openLogs("mecanum");

        telem.addLine("Mecanum feedback-gain tuner.");
        telem.addLine("Clear ~2 ft around the robot. It will rotate in place, then step");
        telem.addLine("forward/back, then sideways, adjusting gains after each step pair.");
        double[] startAx = synthesizeStart(tauAxial);
        double[] startLat = synthesizeStart(tauLateral);
        telem.addData("tau axial (s)", "%.3f", tauAxial);
        telem.addData("tau lateral (s)", "%.3f", tauLateral);
        telem.addData("first axial pos/vel", "%.2f / %.2f", startAx[0], startAx[1]);
        telem.addData("recenter pos/vel", "%.2f / %.2f", HOLD_POS_GAIN, HOLD_VEL_GAIN);
        if (sampleLog != null) {
            telem.addData("sample log", sampleLog.fileName());
            telem.addData("summary log", summaryLog.fileName());
            telem.addLine("Pull with: telemetry/pull.sh");
        }
        if (tauAxial < 0.05) {
            telem.addLine(
                    "WARNING: tau is very small (kA low vs kV) — starting gains may be high.");
            telem.addLine("Re-check AxialFeedforwardTuner kA before trusting results.");
        }
        telem.addLine("Clear ~2 ft; do not run against a wall — stall aborts but wastes the run.");
        telem.update();
        waitForStart();
        if (isStopRequested()) {
            closeLogs();
            return;
        }

        // Accumulated gains written to PARAMS at the end. Seed from live PARAMS (or hold-level
        // floor) so an untuned axis still has a usable value if its search fails.
        // Heading first; translation axes do not use these values during measurement (off-axis
        // gains are zero — see controllerGains).
        double[] gains = {
            Math.max(MecanumDrive.PARAMS.axialGain, HOLD_POS_GAIN),
            Math.max(MecanumDrive.PARAMS.lateralGain, HOLD_POS_GAIN),
            Math.max(MecanumDrive.PARAMS.headingGain, HOLD_HEADING_POS),
            Math.max(MecanumDrive.PARAMS.axialVelGain, HOLD_VEL_GAIN),
            Math.max(MecanumDrive.PARAMS.lateralVelGain, HOLD_VEL_GAIN),
            Math.max(MecanumDrive.PARAMS.headingVelGain, HOLD_HEADING_VEL),
        };

        // Field home for between-phase recenter only (not per-leg within an axis).
        drive.updatePoseEstimate();
        Pose2d fieldHome = drive.localizer.getPose();

        AxisResult heading =
                searchAxis(
                        "heading",
                        tauAxial,
                        mecanumStepTest(drive, 2, new Pose2d(0, 0, Math.toRadians(STEP_DEGREES))),
                        MAX_HEADING_POS_GAIN);
        if (heading.ok) {
            gains[2] = heading.posGain;
            gains[5] = heading.velGain;
        }

        recenterToHome(drive, fieldHome);

        AxisResult axial =
                searchAxis(
                        "axial",
                        tauAxial,
                        mecanumStepTest(drive, 0, new Pose2d(STEP_INCHES, 0, 0)));
        if (axial.ok) {
            gains[0] = axial.posGain;
            gains[3] = axial.velGain;
        }

        recenterToHome(drive, fieldHome);

        AxisResult lateral =
                searchAxis(
                        "lateral",
                        tauLateral,
                        mecanumStepTest(drive, 1, new Pose2d(0, STEP_INCHES, 0)));
        if (lateral.ok) {
            gains[1] = lateral.posGain;
            gains[4] = lateral.velGain;
        }

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        closeLogs();

        // Live for the rest of the session; followers read PARAMS gains every loop.
        MecanumDrive.PARAMS.axialGain = gains[0];
        MecanumDrive.PARAMS.lateralGain = gains[1];
        MecanumDrive.PARAMS.headingGain = gains[2];
        MecanumDrive.PARAMS.axialVelGain = gains[3];
        MecanumDrive.PARAMS.lateralVelGain = gains[4];
        MecanumDrive.PARAMS.headingVelGain = gains[5];

        while (nextFrame()) {
            telem.addLine("=== Written to live MecanumDrive.PARAMS ===");
            telem.addData("axialGain", "%.2f", gains[0]);
            telem.addData("lateralGain", "%.2f", gains[1]);
            telem.addData("headingGain", "%.2f", gains[2]);
            telem.addData("axialVelGain", "%.2f", gains[3]);
            telem.addData("lateralVelGain", "%.2f", gains[4]);
            telem.addData("headingVelGain", "%.2f", gains[5]);
            telem.addLine();
            telem.addData("axial", axial.note);
            telem.addData("lateral", lateral.note);
            telem.addData("heading", heading.note);
            if (LOG_CSV) {
                telem.addLine("CSVs on hub under /sdcard/FIRST/ — pull with telemetry/pull.sh");
            }
            telem.addLine("Live for later OpModes this session. Paste into source to keep.");
            telem.addLine("Verify with ManualFeedbackTuner / SplineTest.");
        }
    }

    private void runTank() {
        TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0), this::batteryVoltage);
        checkDeadWheels(drive.localizer);
        requirePositive(TankDrive.PARAMS.kV, TankDrive.PARAMS.kA);

        double tau = TankDrive.PARAMS.kA / TankDrive.PARAMS.kV;

        openLogs("tank");

        telem.addLine("Tank feedback-gain tuner (turnGain / turnVelGain).");
        telem.addLine("The robot will rotate in place; make sure it can spin freely.");
        telem.addLine("Path following uses Ramsete, whose defaults rarely need tuning.");
        if (sampleLog != null) {
            telem.addData("sample log", sampleLog.fileName());
            telem.addData("summary log", summaryLog.fileName());
            telem.addLine("Pull with: telemetry/pull.sh");
        }
        telem.update();
        waitForStart();
        if (isStopRequested()) {
            closeLogs();
            return;
        }

        AxisResult turn =
                searchAxis(
                        "turn",
                        tau,
                        tankTurnStepTest(drive, Math.toRadians(STEP_DEGREES)),
                        MAX_HEADING_POS_GAIN);

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        closeLogs();

        if (turn.ok) {
            TankDrive.PARAMS.turnGain = turn.posGain;
            TankDrive.PARAMS.turnVelGain = turn.velGain;
        }

        while (nextFrame()) {
            telem.addLine("=== Written to live TankDrive.PARAMS ===");
            telem.addData("turnGain", "%.2f", TankDrive.PARAMS.turnGain);
            telem.addData("turnVelGain", "%.2f", TankDrive.PARAMS.turnVelGain);
            telem.addLine();
            telem.addData("turn", turn.note);
            if (LOG_CSV) {
                telem.addLine("CSVs on hub under /sdcard/FIRST/ — pull with telemetry/pull.sh");
            }
            telem.addLine("Live for later OpModes this session. Paste into source to keep.");
            telem.addLine("Verify with a TurnAction.");
        }
    }

    /**
     * Bandwidth search for one axis. Synthesizes (posGain, velGain) from the candidate bandwidth,
     * measures a step pair (out and back), then grows the bandwidth while the response stays clean
     * and stops at the last clean setting once it turns oscillatory.
     *
     * @param maxPosGain ceiling for this axis ({@link #MAX_POS_GAIN} or {@link
     *     #MAX_HEADING_POS_GAIN})
     */
    private AxisResult searchAxis(String name, double tau, StepTest test, double maxPosGain) {
        double omega = Math.min(START_BANDWIDTH / tau, MAX_START_OMEGA);
        double bestKp = Double.NaN, bestKd = Double.NaN;
        String note = "not tuned";
        logAxis = name;
        int hardRejectStreak = 0;

        for (int iter = 1; iter <= MAX_ITERS && opModeIsActive(); iter++) {
            logIter = iter;
            double kp = omega * omega * tau;
            boolean capped = false;
            if (kp > maxPosGain) {
                kp = maxPosGain;
                omega = Math.sqrt(kp / tau);
                capped = true;
            }
            double kd = Math.max(0, 2 * ZETA * omega * tau - 1);

            telem.addData("axis", name);
            telem.addData("iteration", "%d / %d", iter, MAX_ITERS);
            telem.addData("tau", "%.3f s", tau);
            telem.addData("omega_n", "%.2f rad/s", omega);
            telem.addData("trying posGain", "%.2f", kp);
            telem.addData("trying velGain", "%.2f", kd);
            if (sampleLog != null) {
                telem.addData("log", sampleLog.fileName());
            }
            telem.update();

            StepResponse out = test.run(true, kp, kd);
            logLegSummary(name, iter, true, kp, kd, tau, omega, out, "out");
            if (!opModeIsActive()) break;

            if (out.stalled) {
                StepResponse ret = test.run(false, Math.min(kp, HOLD_POS_GAIN), HOLD_VEL_GAIN);
                logLegSummary(
                        name,
                        iter,
                        false,
                        HOLD_POS_GAIN,
                        HOLD_VEL_GAIN,
                        tau,
                        omega,
                        ret,
                        "stall_return");
                return new AxisResult(
                        false,
                        0,
                        0,
                        "FAILED: stalled mid-step (wall / obstruction?) — clear space and re-run;"
                                + " do not keep thrashing against an obstacle");
            }

            if (iter == 1 && out.movedFrac() < 0.5) {
                // The very first, conservative step should easily close most of the error.
                StepResponse ret = test.run(false, kp, kd); // best effort return to start
                logLegSummary(name, iter, false, kp, kd, tau, omega, ret, "barely_moved_return");
                return new AxisResult(
                        false,
                        0,
                        0,
                        "FAILED: robot barely moved on the first step — check feedforward and"
                                + " localization");
            }

            StepResponse back = test.run(false, kp, kd);
            logLegSummary(name, iter, false, kp, kd, tau, omega, back, "back");
            if (!opModeIsActive()) break;

            if (back.stalled) {
                return new AxisResult(
                        false,
                        Double.isNaN(bestKp) ? 0 : bestKp,
                        Double.isNaN(bestKd) ? 0 : bestKd,
                        "FAILED: stalled on return leg (wall / obstruction?) — clear space and"
                                + " re-run");
            }

            boolean hardReject = out.hardReject() || back.hardReject();
            boolean settled = out.settled && back.settled;
            boolean softEdge = !hardReject && (out.atEdge() || back.atEdge());
            boolean fftRing = out.fftLoopRing() || back.fftLoopRing();
            double worstOver = Math.max(out.overshootFrac(), back.overshootFrac());
            int velFlips = Math.max(out.velFlips, back.velFlips);
            double fftHz = preferFftHz(out, back);
            double fftRatio = preferFftRatio(out, back);

            if (hardReject) {
                // Never promote a hard-rejected gain to "best" — even if it eventually settled.
                // (Bug on 194839: heading kept posGain 20 after 14 vel-flips because settled+reject
                // used to overwrite bestKp.)
                hardRejectStreak++;
                // Best-effort home without another full thrash profile at high gain.
                StepResponse gentle =
                        test.run(
                                false,
                                Math.min(kp * 0.25, HOLD_POS_GAIN),
                                Math.min(kd, HOLD_VEL_GAIN));
                logLegSummary(
                        name,
                        iter,
                        false,
                        Math.min(kp * 0.25, HOLD_POS_GAIN),
                        Math.min(kd, HOLD_VEL_GAIN),
                        tau,
                        omega,
                        gentle,
                        "gentle_return");
                if (!Double.isNaN(bestKp)) {
                    note =
                            String.format(
                                    java.util.Locale.US,
                                    "ok: rang at posGain %.2f, kept last clean %.2f",
                                    kp,
                                    bestKp);
                    break;
                }
                omega *= BACKOFF;
                if (fftRing) {
                    note =
                            String.format(
                                    java.util.Locale.US,
                                    "FAILED: FFT loop ring (~%.1f Hz, err peak/med %.0f×) even at"
                                            + " low bandwidth",
                                    fftHz,
                                    fftRatio);
                } else if (velFlips > MAX_VEL_FLIPS) {
                    note =
                            "FAILED: velocity chatter even at low bandwidth — check kS (esp."
                                    + " lateral), localization lag, or wall contact";
                } else {
                    note = "FAILED: oscillatory even at the lowest bandwidth tried";
                }
                if (hardRejectStreak >= MAX_HARD_REJECT_STREAK) {
                    break;
                }
            } else if (settled) {
                hardRejectStreak = 0;
                bestKp = kp;
                bestKd = kd;
                if (softEdge) {
                    // Mild overshoot / soft FFT edge: keep this setting, do not grow further.
                    if (out.fftEdge() || back.fftEdge()) {
                        note =
                                String.format(
                                        java.util.Locale.US,
                                        "ok: settled at posGain %.2f — FFT edge ~%.1f Hz (%.0f×),"
                                                + " not growing",
                                        kp,
                                        fftHz,
                                        fftRatio);
                    } else {
                        note =
                                String.format(
                                        java.util.Locale.US,
                                        "ok: settled at posGain %.2f (overshoot %.0f%%) — edge, not"
                                                + " growing",
                                        kp,
                                        100 * worstOver);
                    }
                    break;
                }
                note =
                        String.format(
                                java.util.Locale.US,
                                "ok: clean at posGain %.2f (overshoot %.0f%%%s)",
                                kp,
                                100 * worstOver,
                                out.fftOk || back.fftOk
                                        ? String.format(
                                                java.util.Locale.US,
                                                ", FFT ~%.1f Hz ×%.0f",
                                                fftHz,
                                                fftRatio)
                                        : "");
                if (capped) break;
                omega *= GROWTH;
            } else {
                // No hard ring but never settled inside the band: sluggish (stiction / wall graze).
                // Grow carefully; do not record as best.
                hardRejectStreak = 0;
                omega *= GROWTH;
                if (Double.isNaN(bestKp)) {
                    note = "FAILED: never settled — check kS, wall clearance, or widen SETTLE_TOL";
                }
                if (capped) break;
            }
        }

        if (Double.isNaN(bestKp)) {
            return new AxisResult(false, 0, 0, note);
        }
        return new AxisResult(true, bestKp, bestKd, note);
    }

    /** Convenience: translation-axis search with {@link #MAX_POS_GAIN}. */
    private AxisResult searchAxis(String name, double tau, StepTest test) {
        return searchAxis(name, tau, test, MAX_POS_GAIN);
    }

    private void logLegSummary(
            String axis,
            int iter,
            boolean outward,
            double kp,
            double kd,
            double tau,
            double omega,
            StepResponse r,
            String verdict) {
        if (summaryLog == null || r == null) return;
        String tag =
                r.stalled
                        ? ";stalled"
                        : r.fftLoopRing()
                                ? ";fft_ring"
                                : r.hardReject()
                                        ? ";hard_reject"
                                        : r.atEdge()
                                                ? ";edge"
                                                : r.settled ? ";settled" : ";timeout_or_sluggish";
        summaryLog.row(
                axis,
                iter,
                outward,
                kp,
                kd,
                tau,
                omega,
                r.e0,
                r.eLast,
                r.movedFrac(),
                r.overshootFrac(),
                r.reversals,
                r.velFlips,
                r.settled,
                r.stalled,
                r.hardReject(),
                r.fftErrHz,
                r.fftErrRatio,
                r.fftCmdHz,
                r.fftCmdRatio,
                r.fftPeriodSec,
                r.fftErrRms,
                Double.isNaN(r.holdStart)
                        ? STEP_TIMEOUT_SEC
                        : (r.holdT.isEmpty() ? STEP_TIMEOUT_SEC : r.holdT.get(r.holdT.size() - 1)),
                verdict + tag);
        summaryLog.flush();
    }

    private static double preferFftHz(StepResponse a, StepResponse b) {
        double ra = a.fftOk ? a.fftErrRatio : Double.NEGATIVE_INFINITY;
        double rb = b.fftOk ? b.fftErrRatio : Double.NEGATIVE_INFINITY;
        if (ra >= rb && a.fftOk) return a.fftErrHz;
        if (b.fftOk) return b.fftErrHz;
        return Double.NaN;
    }

    private static double preferFftRatio(StepResponse a, StepResponse b) {
        double ra = a.fftOk ? a.fftErrRatio : Double.NEGATIVE_INFINITY;
        double rb = b.fftOk ? b.fftErrRatio : Double.NEGATIVE_INFINITY;
        if (ra >= rb && a.fftOk) return a.fftErrRatio;
        if (b.fftOk) return b.fftErrRatio;
        return Double.NaN;
    }

    /**
     * Profiled bump test for one mecanum axis: 0 = axial, 1 = lateral, 2 = heading. A cosine
     * reference carries the target from the current pose to the offset (or back) so feedforward
     * sees a smooth velocity profile — the same regime the gains face on real trajectories.
     * Endpoints are always relative to wherever the robot is now; field re-homing is only between
     * axes via {@link #recenterToHome}.
     */
    private StepTest mecanumStepTest(MecanumDrive drive, int axis, Pose2d delta) {
        final double tol = axis == 2 ? Math.toRadians(SETTLE_TOL_DEG) : SETTLE_TOL_IN;

        return (outward, posGain, velGain) -> {
            drive.updatePoseEstimate();
            Pose2d here = drive.localizer.getPose();
            Pose2d from = here;
            Pose2d to = outward ? here.times(delta) : here.times(invertDelta(delta));

            // Test axis closed; lateral also gets soft heading fixture (see controllerGains).
            double[] g = controllerGains(axis, posGain, velGain);

            StepResponse resp = new StepResponse();
            ElapsedTime timer = new ElapsedTime();
            while (nextFrame() && timer.seconds() < STEP_TIMEOUT_SEC) {
                PoseVelocity2d vel = drive.updatePoseEstimate();
                Pose2d pose = drive.localizer.getPose();
                // Score against the final pose (overshoot / settle), not the moving reference.
                Pose2d error = to.minusExp(pose);
                double e =
                        axis == 0
                                ? error.position.x
                                : axis == 1 ? error.position.y : error.heading.log();
                double axisVel =
                        axis == 0 ? vel.linearVel.x : axis == 1 ? vel.linearVel.y : vel.angVel;
                double progress = smoothProgress(timer.seconds(), MOVE_SEC).value();

                // Heading: interpolatePose keeps XY fixed at `from` while yaw tracks the profile.
                Pose2dDual<Time> txWorldTarget =
                        interpolatePose(from, to, timer.seconds(), MOVE_SEC);
                PoseVelocity2dDual<Time> command =
                        new HolonomicController(g[0], g[1], g[2], g[3], g[4], g[5])
                                .compute(txWorldTarget, pose, vel);
                double axisCmd =
                        axis == 0
                                ? command.value().linearVel.x
                                : axis == 1 ? command.value().linearVel.y : command.value().angVel;
                boolean done = resp.update(timer.seconds(), e, axisVel, axisCmd, tol, progress);
                if (sampleLog != null) {
                    Pose2d ref = txWorldTarget.value();
                    sampleLog.row(
                            timer.seconds(),
                            logAxis,
                            logIter,
                            outward,
                            posGain,
                            velGain,
                            g[0],
                            g[1],
                            g[2],
                            g[3],
                            g[4],
                            g[5],
                            pose.position.x,
                            pose.position.y,
                            pose.heading.toDouble(),
                            ref.position.x,
                            ref.position.y,
                            ref.heading.toDouble(),
                            e,
                            vel.linearVel.x,
                            vel.linearVel.y,
                            vel.angVel,
                            command.value().linearVel.x,
                            command.value().linearVel.y,
                            command.value().angVel,
                            progress,
                            resp.velFlips,
                            resp.reversals);
                    maybeFlushSamples();
                }
                if (done) break;
                drive.setDriveCommand(command);
            }
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
            resp.finalizeFft();
            if (sampleLog != null) sampleLog.flush();
            return resp;
        };
    }

    /**
     * Profiled bump test for the tank turn controller. Mirrors the mecanum heading test: a smooth
     * heading reference plus the same pos/vel gains used by TurnAction.
     */
    private StepTest tankTurnStepTest(TankDrive drive, double stepRad) {
        final double tol = Math.toRadians(SETTLE_TOL_DEG);

        return (outward, posGain, velGain) -> {
            drive.updatePoseEstimate();
            Rotation2d from = drive.localizer.getPose().heading;
            Rotation2d to = from.plus(outward ? stepRad : -stepRad);

            StepResponse resp = new StepResponse();
            ElapsedTime timer = new ElapsedTime();
            while (nextFrame() && timer.seconds() < STEP_TIMEOUT_SEC) {
                PoseVelocity2d vel = drive.updatePoseEstimate();
                Pose2d pose = drive.localizer.getPose();
                double e = to.minus(pose.heading);
                DualNum<Time> progress = smoothProgress(timer.seconds(), MOVE_SEC);
                double span = to.minus(from);
                // Track the moving heading reference (profile velocity + PD on ref error), not a
                // static setpoint — same idea as the mecanum profiled bump.
                Rotation2d ref = from.plus(progress.value() * span);
                double eRef = ref.minus(pose.heading);
                double omegaRef = progress.get(1) * span;
                double cmdW = omegaRef + posGain * eRef - velGain * vel.angVel;
                PoseVelocity2dDual<Time> command =
                        new PoseVelocity2dDual<>(
                                Vector2dDual.constant(new Vector2d(0, 0), 3),
                                new DualNum<>(new double[] {cmdW, progress.get(2) * span, 0}));
                boolean done =
                        resp.update(timer.seconds(), e, vel.angVel, cmdW, tol, progress.value());
                if (sampleLog != null) {
                    sampleLog.row(
                            timer.seconds(),
                            logAxis,
                            logIter,
                            outward,
                            posGain,
                            velGain,
                            0,
                            0,
                            posGain,
                            0,
                            0,
                            velGain,
                            pose.position.x,
                            pose.position.y,
                            pose.heading.toDouble(),
                            pose.position.x,
                            pose.position.y,
                            ref.toDouble(),
                            e,
                            vel.linearVel.x,
                            vel.linearVel.y,
                            vel.angVel,
                            0,
                            0,
                            cmdW,
                            progress.value(),
                            resp.velFlips,
                            resp.reversals);
                    maybeFlushSamples();
                }
                if (done) break;
                drive.setDriveCommand(command);
            }
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
            resp.finalizeFft();
            if (sampleLog != null) sampleLog.flush();
            return resp;
        };
    }

    /**
     * Relative pose that undoes {@code delta} when composed on the right: {@code p * inv ≈ start}.
     */
    private static Pose2d invertDelta(Pose2d delta) {
        return delta.inverse();
    }

    /**
     * Cosine smoothstep progress on {@code [0, T]}: value / 1st / 2nd time derivatives. C1 at the
     * endpoints (zero velocity at start and end).
     */
    private static DualNum<Time> smoothProgress(double t, double T) {
        if (T <= 1e-6 || t <= 0) {
            return new DualNum<>(new double[] {0, 0, 0});
        }
        if (t >= T) {
            return new DualNum<>(new double[] {1, 0, 0});
        }
        double w = Math.PI / T;
        double s = 0.5 - 0.5 * Math.cos(w * t);
        double sd = 0.5 * w * Math.sin(w * t);
        double sdd = 0.5 * w * w * Math.cos(w * t);
        return new DualNum<>(new double[] {s, sd, sdd});
    }

    /**
     * Interpolates from {@code from} to {@code to} in {@code from}'s body frame with smooth
     * progress {@code s(t)}, returning a time-dual pose for HolonomicController feedforward.
     */
    private static Pose2dDual<Time> interpolatePose(Pose2d from, Pose2d to, double t, double T) {
        DualNum<Time> s = smoothProgress(t, T);
        Pose2d rel = from.inverse().times(to);
        // local(s) = (s·dx, s·dy, s·dθ) — exact for our pure-axis deltas; fine for small mixes.
        DualNum<Time> lx = s.times(rel.position.x);
        DualNum<Time> ly = s.times(rel.position.y);
        DualNum<Time> ltheta = s.times(rel.heading.toDouble());
        // world = from * local: p' = R_from * p_local + p_from, θ' = θ_from + θ_local
        double c = from.heading.real;
        double sn = from.heading.imag;
        DualNum<Time> wx =
                lx.times(c).plus(ly.times(-sn)).plus(DualNum.constant(from.position.x, 3));
        DualNum<Time> wy =
                lx.times(sn).plus(ly.times(c)).plus(DualNum.constant(from.position.y, 3));
        DualNum<Time> heading = ltheta.plus(DualNum.constant(from.heading.toDouble(), 3));
        return new Pose2dDual<>(wx, wy, heading);
    }

    /**
     * Drives back to {@code home} with hold-level gains on all axes. Used between search phases
     * (heading → axial → lateral) so residual drift from one axis is scrubbed before the next.
     * No-ops when already inside {@link #RECENTER_TOL_IN} / {@link #RECENTER_TOL_DEG}.
     */
    private void recenterToHome(MecanumDrive drive, Pose2d home) {
        drive.updatePoseEstimate();
        Pose2d here = drive.localizer.getPose();
        Pose2d err = home.minusExp(here);
        double posErr = Math.hypot(err.position.x, err.position.y);
        double headErr = Math.abs(err.heading.log());
        if (posErr <= RECENTER_TOL_IN && headErr <= Math.toRadians(RECENTER_TOL_DEG)) {
            return;
        }

        double[] g = {
            HOLD_POS_GAIN, HOLD_POS_GAIN, HOLD_HEADING_POS,
            HOLD_VEL_GAIN, HOLD_VEL_GAIN, HOLD_HEADING_VEL,
        };
        Pose2d from = here;
        ElapsedTime timer = new ElapsedTime();
        double insideSince = Double.NaN;
        double moveSec = Math.min(MOVE_SEC, RECENTER_TIMEOUT_SEC);
        while (nextFrame() && timer.seconds() < RECENTER_TIMEOUT_SEC) {
            PoseVelocity2d vel = drive.updatePoseEstimate();
            Pose2d pose = drive.localizer.getPose();
            Pose2d e = home.minusExp(pose);
            double pe = Math.hypot(e.position.x, e.position.y);
            double he = Math.abs(e.heading.log());
            if (pe <= RECENTER_TOL_IN && he <= Math.toRadians(RECENTER_TOL_DEG)) {
                if (Double.isNaN(insideSince)) insideSince = timer.seconds();
                if (timer.seconds() - insideSince >= 0.25) break;
            } else {
                insideSince = Double.NaN;
            }
            Pose2dDual<Time> tx =
                    timer.seconds() >= moveSec
                            ? Pose2dDual.constant(home, 3)
                            : interpolatePose(from, home, timer.seconds(), moveSec);
            PoseVelocity2dDual<Time> command =
                    new HolonomicController(g[0], g[1], g[2], g[3], g[4], g[5])
                            .compute(tx, pose, vel);
            drive.setDriveCommand(command);
        }
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
    }

    private static void requirePositive(double kV, double kA) {
        if (kV <= 0 || kA <= 0) {
            throw new RuntimeException(
                    "FeedbackGainTuner needs positive kV and kA (tau = kA/kV sets the plant model)."
                            + " Run AxialFeedforwardTuner first.");
        }
    }

    private static void checkDeadWheels(Object localizer) {
        if (localizer instanceof TwoDeadWheelLocalizer) {
            if (TwoDeadWheelLocalizer.PARAMS.perpXTicks == 0
                    && TwoDeadWheelLocalizer.PARAMS.parYTicks == 0) {
                throw new RuntimeException(
                        "Odometry wheel locations not set! Run AngularRampLogger to tune them.");
            }
        } else if (localizer instanceof ThreeDeadWheelLocalizer) {
            if (ThreeDeadWheelLocalizer.PARAMS.perpXTicks == 0
                    && ThreeDeadWheelLocalizer.PARAMS.par0YTicks == 0
                    && ThreeDeadWheelLocalizer.PARAMS.par1YTicks == 1) {
                throw new RuntimeException(
                        "Odometry wheel locations not set! Run AngularRampLogger to tune them.");
            }
        }
    }
}
