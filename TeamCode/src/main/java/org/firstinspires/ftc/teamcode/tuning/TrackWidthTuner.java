package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.Localizer;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.opmodes.base.MarsLinearOpMode;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

/**
 * Automatic on-robot correction of {@code trackWidthTicks} — the effective track width used by the
 * kinematics to convert angular velocity into wheel speeds.
 *
 * <p>Intended primarily for Pinpoint / OTOS setups, where {@code AngularRampLogger} cannot measure
 * the track width (the regression needs drive encoders in the tuning view). On any setup it also
 * serves as a quick verification of an already-tuned value.
 *
 * <p>How it works: it spins in place, ramping a commanded angular velocity through the production
 * {@code setDriveCommand} feedforward path — which converts {@code ω} to wheel speeds using the
 * *current* {@code trackWidthTicks}. If that value is wrong by a factor, the actual yaw rate is
 * wrong by the inverse factor: {@code ω_actual = ω_commanded · W_param / W_true}. The tuner ramps
 * counterclockwise then clockwise, regresses actual yaw rate against commanded yaw rate, and
 * reports {@code trackWidthTicks / slope} as the corrected value.
 *
 * <p>Yaw measurement:
 *
 * <ul>
 *   <li><b>Pinpoint</b> (when the drive localizer is a {@link PinpointLocalizer}): uses onboard
 *       heading velocity — independent of Control Hub mount orientation.
 *   <li><b>Hub IMU</b> otherwise: does <em>not</em> assume Z is yaw. Each sample records the full
 *       angular-velocity vector (deg/s → rad/s; see FtcRobotController#1070) and a rate from
 *       differentiating unwrapped IMU yaw. After both ramps, spin axis {@code û ∝ Σ ω_cmd · ω} is
 *       estimated from the data and rates are projected as {@code ω · û}. Differentiated yaw is
 *       also fit; whichever candidate has the more plausible slope and better R² wins. A pure
 *       in-place spin is always about field-vertical — if hub orientation is misconfigured that
 *       appears on X/Y instead of Z, and the projection still recovers the signed rate.
 * </ul>
 *
 * <p>Prerequisites: the drive feedforward ({@code kS}/{@code kV}, ideally {@code kA}) must be tuned
 * — the spin is driven open-loop through it. {@code trackWidthTicks} may be left at 0; the tuner
 * then seeds a nominal geometric track width ({@link #DEFAULT_TRACK_WIDTH_IN} / {@code inPerTick})
 * so the multiplicative fit can run. A tape-measure value is still fine if you have one. On a
 * successful fit, writes the corrected {@code trackWidthTicks} into the live {@code PARAMS} statics
 * so later OpModes in the same RC process can chain without a paste. Re-run to verify: the slope
 * should come out ≈ 1.00. Paste into source to keep values across restart or redeploy.
 */
@Config
public final class TrackWidthTuner extends MarsLinearOpMode {
    /** Peak commanded angular velocity at the end of each ramp, in rad/s. */
    public static double MAX_ANG_VEL = Math.PI;

    /** Seconds spent ramping the commanded angular velocity from 0 to {@link #MAX_ANG_VEL}. */
    public static double RAMP_TIME = 3.0;

    /**
     * Samples with commanded angular velocity below this (rad/s) are ignored as start-up transient.
     */
    public static double MIN_ANG_VEL = 0.5;

    /**
     * Nominal geometric track width in inches used when {@code Params.trackWidthTicks} is unset
     * ({@code <= 0}). Converted to ticks via {@code / inPerTick}. The robot size limit is 18 in; 16
     * in leaves margin under that for a typical wheel-center track. Mecanum effective width often
     * comes out somewhat larger after correction because of roller scrub.
     */
    public static double DEFAULT_TRACK_WIDTH_IN = 16.0;

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        MultipleTelemetry telem = (MultipleTelemetry) telemetry;

        Consumer<PoseVelocity2dDual<Time>> setCommand;
        IMU imu;
        Localizer localizer;
        double trackWidthTicks;
        boolean usedDefaultSeed;
        String paramsClass;
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            requireFeedforward(MecanumDrive.PARAMS.kV);
            usedDefaultSeed = MecanumDrive.PARAMS.trackWidthTicks <= 0;
            // Kinematics bake trackWidthTicks at construction — seed Params first when unset.
            trackWidthTicks =
                    resolveTrackWidthTicks(
                            MecanumDrive.PARAMS.trackWidthTicks, MecanumDrive.PARAMS.inPerTick);
            MecanumDrive.PARAMS.trackWidthTicks = trackWidthTicks;
            MecanumDrive drive =
                    new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0), this::batteryVoltage);
            setCommand = drive::setDriveCommand;
            imu = drive.lazyImu.get();
            localizer = drive.localizer;
            paramsClass = "MecanumDrive.Params";
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            requireFeedforward(TankDrive.PARAMS.kV);
            usedDefaultSeed = TankDrive.PARAMS.trackWidthTicks <= 0;
            trackWidthTicks =
                    resolveTrackWidthTicks(
                            TankDrive.PARAMS.trackWidthTicks, TankDrive.PARAMS.inPerTick);
            TankDrive.PARAMS.trackWidthTicks = trackWidthTicks;
            TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0), this::batteryVoltage);
            setCommand = drive::setDriveCommand;
            imu = drive.lazyImu.get();
            localizer = drive.localizer;
            paramsClass = "TankDrive.Params";
        } else {
            throw new RuntimeException("Unknown DRIVE_CLASS");
        }

        final DoubleSupplier pinpointRate;
        final boolean usePinpoint;
        if (localizer instanceof PinpointLocalizer) {
            PinpointLocalizer pl = (PinpointLocalizer) localizer;
            usePinpoint = true;
            pinpointRate =
                    () -> {
                        pl.driver.update();
                        return pl.driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
                    };
        } else {
            usePinpoint = false;
            pinpointRate = null;
        }

        telem.addLine("Track width tuner.");
        telem.addData(
                "yaw source",
                usePinpoint ? "Pinpoint heading velocity" : "Hub IMU (axis projection + yaw)");
        if (usedDefaultSeed) {
            telem.addData(
                    "seed",
                    "default %.1f in → trackWidthTicks=%.2f",
                    DEFAULT_TRACK_WIDTH_IN,
                    trackWidthTicks);
        } else {
            telem.addData("seed", "Params trackWidthTicks=%.2f", trackWidthTicks);
        }
        telem.addLine(
                "Press START, then the robot spins in place: counterclockwise, then clockwise.");
        telem.addLine("Make sure it can rotate freely.");
        telem.update();
        waitForStart();
        if (isStopRequested()) return;

        List<RawSample> ccwRaw = rampAndSample(setCommand, imu, pinpointRate, +1, telem, "CCW");
        List<RawSample> cwRaw = rampAndSample(setCommand, imu, pinpointRate, -1, telem, "CW");

        FitChoice fit = chooseFit(ccwRaw, cwRaw, usePinpoint);
        double slope = 0.5 * (fit.ccw.slope + fit.cw.slope);
        boolean valid = slope > 0.2 && slope < 5.0;
        double corrected = valid ? trackWidthTicks / slope : Double.NaN;

        // Kinematics bake trackWidthTicks at drive construction, so this OpMode's instance keeps
        // the seed; the next OpMode init picks up the corrected static.
        if (valid) {
            if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
                MecanumDrive.PARAMS.trackWidthTicks = corrected;
            } else {
                TankDrive.PARAMS.trackWidthTicks = corrected;
            }
        }

        while (nextFrame()) {
            if (valid) {
                telemetry.addLine("=== Written to live " + paramsClass + " ===");
                telemetry.addData("trackWidthTicks", "%.2f", corrected);
                telemetry.addLine();
                telemetry.addData(
                        "seed used",
                        "%.2f%s",
                        trackWidthTicks,
                        usedDefaultSeed ? " (default geometric)" : "");
                telemetry.addData("yaw source", fit.label);
                telemetry.addData("slope (actual/commanded)", "%.4f", slope);
                if (fit.axisNote != null) {
                    telemetry.addData("spin axis (robot frame)", fit.axisNote);
                }
                telemetry.addData(
                        "ccw fit",
                        "slope %.4f, R^2 %.4f (%d/%d pts)",
                        fit.ccw.slope,
                        fit.ccw.r2,
                        fit.ccw.used,
                        fit.ccw.total);
                telemetry.addData(
                        "cw fit",
                        "slope %.4f, R^2 %.4f (%d/%d pts)",
                        fit.cw.slope,
                        fit.cw.r2,
                        fit.cw.used,
                        fit.cw.total);
                telemetry.addLine(
                        "Live for later OpModes this session. Paste into source to keep.");
                telemetry.addLine("Re-run to verify: the slope should be ~1.00.");
            } else {
                telemetry.addLine(
                        "FAILED: fitted slope "
                                + String.format("%.3f", slope)
                                + " is not plausible (need 0.2–5.0).");
                telemetry.addLine("Check motor directions and feedforward (kS/kV), then retry.");
                telemetry.addData("yaw source", fit.label);
                if (fit.axisNote != null) {
                    telemetry.addData("spin axis (robot frame)", fit.axisNote);
                }
                telemetry.addData(
                        "seed used",
                        "%.2f%s",
                        trackWidthTicks,
                        usedDefaultSeed ? " (default geometric)" : "");
                telemetry.addData(
                        "ccw fit",
                        "slope %.4f, R^2 %.4f (%d/%d pts)",
                        fit.ccw.slope,
                        fit.ccw.r2,
                        fit.ccw.used,
                        fit.ccw.total);
                telemetry.addData(
                        "cw fit",
                        "slope %.4f, R^2 %.4f (%d/%d pts)",
                        fit.cw.slope,
                        fit.cw.r2,
                        fit.cw.used,
                        fit.cw.total);
                if (slope >= 5.0) {
                    telemetry.addLine(
                            "Slope too high: actual yaw >> commanded. Check units / free spin.");
                } else if (slope <= 0.2) {
                    telemetry.addLine(
                            "Slope too low: robot spun much slower than commanded. Watch"
                                    + " commanded vs actual mid-ramp; check kS/kV and free spin.");
                }
            }
        }
    }

    private static void requireFeedforward(double kV) {
        if (kV <= 0) {
            throw new RuntimeException(
                    "Tune the drive feedforward (kS/kV) before running TrackWidthTuner — the spin"
                            + " is driven through it.");
        }
    }

    /**
     * Returns the seed for the multiplicative fit: the existing Params value when set, otherwise
     * {@link #DEFAULT_TRACK_WIDTH_IN} / {@code inPerTick}.
     */
    private static double resolveTrackWidthTicks(double trackWidthTicks, double inPerTick) {
        if (trackWidthTicks > 0) {
            return trackWidthTicks;
        }
        if (inPerTick <= 0) {
            throw new RuntimeException(
                    "Set inPerTick (ForwardPushTest) before TrackWidthTuner — needed to seed"
                            + " trackWidthTicks from "
                            + DEFAULT_TRACK_WIDTH_IN
                            + " in.");
        }
        return DEFAULT_TRACK_WIDTH_IN / inPerTick;
    }

    /**
     * Ramps commanded angular velocity through the production feedforward path. Each sample stores
     * enough IMU data to reconstruct yaw rate after both ramps (axis projection + differentiated
     * yaw), or a direct Pinpoint rate when {@code pinpointRate} is non-null.
     */
    private List<RawSample> rampAndSample(
            Consumer<PoseVelocity2dDual<Time>> setCommand,
            IMU imu,
            DoubleSupplier pinpointRate,
            int dir,
            MultipleTelemetry telemetry,
            String label) {
        List<RawSample> samples = new ArrayList<>();
        double alpha = dir * MAX_ANG_VEL / RAMP_TIME;
        ElapsedTime timer = new ElapsedTime();
        double lastYaw = Double.NaN;
        double lastT = Double.NaN;

        while (nextFrame() && timer.seconds() < RAMP_TIME) {
            double t = timer.seconds();
            double omegaCmd = alpha * t;
            setCommand.accept(
                    new PoseVelocity2dDual<>(
                            Vector2dDual.constant(new Vector2d(0, 0), 3),
                            new DualNum<>(new double[] {omegaCmd, alpha, 0})));

            RawSample s = new RawSample();
            s.omegaCmd = omegaCmd;

            if (pinpointRate != null) {
                s.directRate = pinpointRate.getAsDouble();
            } else {
                // Angular velocity: read deg/s and convert (SDK may ignore AngleUnit.RADIANS;
                // FtcRobotController#1070). Keep all three axes for orientation-robust projection.
                AngularVelocity avDeg = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
                s.wx = Math.toRadians(avDeg.xRotationRate);
                s.wy = Math.toRadians(avDeg.yRotationRate);
                s.wz = Math.toRadians(avDeg.zRotationRate);

                // Differentiated unwrapped yaw — correct when hub orientation is configured right;
                // may be near zero if orientation is wrong (spin appears as pitch/roll instead).
                double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                if (!Double.isNaN(lastYaw) && !Double.isNaN(lastT)) {
                    double dt = t - lastT;
                    if (dt > 1e-6) {
                        s.yawRate = AngleUnit.normalizeRadians(yaw - lastYaw) / dt;
                    }
                }
                lastYaw = yaw;
                lastT = t;
            }

            if (Math.abs(omegaCmd) > MIN_ANG_VEL) {
                samples.add(s);
            }

            double display =
                    pinpointRate != null
                            ? s.directRate
                            : Math.hypot(s.wx, Math.hypot(s.wy, s.wz)) * Math.signum(omegaCmd);

            telemetry.addData("phase", label);
            telemetry.addData("commanded (rad/s)", "%.2f", omegaCmd);
            telemetry.addData("actual (rad/s)", "%.2f", display);
            if (pinpointRate == null) {
                telemetry.addData("ω xyz (rad/s)", "%.2f, %.2f, %.2f", s.wx, s.wy, s.wz);
                telemetry.addData("d(yaw)/dt (rad/s)", "%.2f", s.yawRate);
            }
            telemetry.addData("samples", samples.size());
        }
        setCommand.accept(
                new PoseVelocity2dDual<>(
                        Vector2dDual.constant(new Vector2d(0, 0), 3), DualNum.constant(0, 3)));
        // brief coast so the next ramp starts from rest
        ElapsedTime settle = new ElapsedTime();
        while (nextFrame() && settle.seconds() < 1.0) {
            telemetry.addData("phase", label + " settling");
        }
        return samples;
    }

    /**
     * Builds (cmd, rate) series from raw samples and picks the best of the available estimators.
     */
    private static FitChoice chooseFit(
            List<RawSample> ccwRaw, List<RawSample> cwRaw, boolean usePinpoint) {
        if (usePinpoint) {
            return new FitChoice(
                    fitDirect(ccwRaw), fitDirect(cwRaw), "Pinpoint heading velocity", null);
        }

        // Spin axis û ∝ Σ ω_cmd · ω over both ramps (sign of cmd flips with spin direction, so
        // products reinforce). Pure field-vertical spin maps to some fixed direction in the
        // (possibly misconfigured) robot frame.
        double sx = 0, sy = 0, sz = 0;
        for (RawSample s : ccwRaw) {
            sx += s.omegaCmd * s.wx;
            sy += s.omegaCmd * s.wy;
            sz += s.omegaCmd * s.wz;
        }
        for (RawSample s : cwRaw) {
            sx += s.omegaCmd * s.wx;
            sy += s.omegaCmd * s.wy;
            sz += s.omegaCmd * s.wz;
        }
        double norm = Math.sqrt(sx * sx + sy * sy + sz * sz);
        final double ux, uy, uz;
        final String axisNote;
        if (norm > 1e-9) {
            ux = sx / norm;
            uy = sy / norm;
            uz = sz / norm;
            axisNote = String.format("û=(%.2f, %.2f, %.2f)", ux, uy, uz);
        } else {
            // No correlation — fall back to Z (legacy behavior).
            ux = 0;
            uy = 0;
            uz = 1;
            axisNote = "û=(0, 0, 1) fallback (no spin correlation)";
        }

        TunerRegression.Result ccwProj = fitProjected(ccwRaw, ux, uy, uz);
        TunerRegression.Result cwProj = fitProjected(cwRaw, ux, uy, uz);
        TunerRegression.Result ccwYaw = fitYawDiff(ccwRaw);
        TunerRegression.Result cwYaw = fitYawDiff(cwRaw);

        double scoreProj = scorePair(ccwProj, cwProj);
        double scoreYaw = scorePair(ccwYaw, cwYaw);

        if (scoreYaw > scoreProj) {
            return new FitChoice(ccwYaw, cwYaw, "Hub IMU d(yaw)/dt", axisNote);
        }
        return new FitChoice(ccwProj, cwProj, "Hub IMU ω·û (orientation-robust)", axisNote);
    }

    /**
     * Higher is better. Prefers slopes near 1 and high R²; strongly penalizes out-of-range slopes.
     */
    private static double scorePair(TunerRegression.Result a, TunerRegression.Result b) {
        if (a == null || b == null) {
            return Double.NEGATIVE_INFINITY;
        }
        double slope = 0.5 * (a.slope + b.slope);
        double r2 = 0.5 * (a.r2 + b.r2);
        if (!(slope > 0.05 && slope < 20)) {
            return Double.NEGATIVE_INFINITY;
        }
        // Plausible band gets a bonus; then reward proximity to 1 and fit quality.
        double inBand = (slope > 0.2 && slope < 5.0) ? 2.0 : 0.0;
        double nearOne = -Math.abs(Math.log(slope));
        return inBand + nearOne + r2;
    }

    private static TunerRegression.Result fitDirect(List<RawSample> raw) {
        List<double[]> pts = new ArrayList<>(raw.size());
        for (RawSample s : raw) {
            pts.add(new double[] {s.omegaCmd, s.directRate});
        }
        return TunerRegression.robustFit(pts);
    }

    private static TunerRegression.Result fitProjected(
            List<RawSample> raw, double ux, double uy, double uz) {
        List<double[]> pts = new ArrayList<>(raw.size());
        for (RawSample s : raw) {
            pts.add(new double[] {s.omegaCmd, s.wx * ux + s.wy * uy + s.wz * uz});
        }
        return TunerRegression.robustFit(pts);
    }

    private static TunerRegression.Result fitYawDiff(List<RawSample> raw) {
        List<double[]> pts = new ArrayList<>(raw.size());
        for (RawSample s : raw) {
            if (Double.isNaN(s.yawRate)) {
                continue;
            }
            pts.add(new double[] {s.omegaCmd, s.yawRate});
        }
        if (pts.size() < 2) {
            // Degenerate — robustFit needs at least 2 points; return a zero-slope placeholder.
            pts.add(new double[] {0, 0});
            pts.add(new double[] {1, 0});
        }
        return TunerRegression.robustFit(pts);
    }

    private static final class RawSample {
        double omegaCmd;
        double wx, wy, wz;
        double yawRate = Double.NaN;
        double directRate = Double.NaN;
    }

    private static final class FitChoice {
        final TunerRegression.Result ccw;
        final TunerRegression.Result cw;
        final String label;
        final String axisNote;

        FitChoice(
                TunerRegression.Result ccw,
                TunerRegression.Result cw,
                String label,
                String axisNote) {
            this.ccw = ccw;
            this.cw = cw;
            this.label = label;
            this.axisNote = axisNote;
        }
    }
}
