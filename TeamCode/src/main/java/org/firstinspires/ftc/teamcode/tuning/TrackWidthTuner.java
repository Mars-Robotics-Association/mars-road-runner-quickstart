package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

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
 * counterclockwise then clockwise, regresses actual yaw rate (from the hub IMU, so the measurement
 * is independent of the localizer and of {@code trackWidthTicks}) against commanded yaw rate, and
 * reports {@code trackWidthTicks / slope} as the corrected value.
 *
 * <p>Prerequisites: the drive feedforward ({@code kS}/{@code kV}, ideally {@code kA}) must be
 * tuned — the spin is driven open-loop through it. {@code trackWidthTicks} may be left at 0; the
 * tuner then seeds a nominal geometric track width ({@link #DEFAULT_TRACK_WIDTH_IN} / {@code
 * inPerTick}) so the multiplicative fit can run. A tape-measure value is still fine if you have
 * one. After pasting the corrected value, re-run to verify: the slope should come out ≈ 1.00.
 */
@Config
public final class TrackWidthTuner extends LinearOpMode {
    /** Peak commanded angular velocity at the end of each ramp, in rad/s. */
    public static double MAX_ANG_VEL = Math.PI;
    /** Seconds spent ramping the commanded angular velocity from 0 to {@link #MAX_ANG_VEL}. */
    public static double RAMP_TIME = 3.0;
    /** Samples with commanded angular velocity below this (rad/s) are ignored as start-up transient. */
    public static double MIN_ANG_VEL = 0.5;
    /**
     * Nominal geometric track width in inches used when {@code Params.trackWidthTicks} is unset
     * ({@code <= 0}). Converted to ticks via {@code / inPerTick}. The robot size limit is 18 in;
     * 16 in leaves margin under that for a typical wheel-center track. Mecanum effective width
     * often comes out somewhat larger after correction because of roller scrub.
     */
    public static double DEFAULT_TRACK_WIDTH_IN = 16.0;

    @Override
    public void runOpMode() throws InterruptedException {
        MultipleTelemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        Consumer<PoseVelocity2dDual<Time>> setCommand;
        IMU imu;
        double trackWidthTicks;
        boolean usedDefaultSeed;
        String paramsClass;
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            requireFeedforward(MecanumDrive.PARAMS.kV);
            usedDefaultSeed = MecanumDrive.PARAMS.trackWidthTicks <= 0;
            // Kinematics bake trackWidthTicks at construction — seed Params first when unset.
            trackWidthTicks = resolveTrackWidthTicks(
                    MecanumDrive.PARAMS.trackWidthTicks, MecanumDrive.PARAMS.inPerTick);
            MecanumDrive.PARAMS.trackWidthTicks = trackWidthTicks;
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
            setCommand = drive::setDriveCommand;
            imu = drive.lazyImu.get();
            paramsClass = "MecanumDrive.Params";
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            requireFeedforward(TankDrive.PARAMS.kV);
            usedDefaultSeed = TankDrive.PARAMS.trackWidthTicks <= 0;
            trackWidthTicks = resolveTrackWidthTicks(
                    TankDrive.PARAMS.trackWidthTicks, TankDrive.PARAMS.inPerTick);
            TankDrive.PARAMS.trackWidthTicks = trackWidthTicks;
            TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));
            setCommand = drive::setDriveCommand;
            imu = drive.lazyImu.get();
            paramsClass = "TankDrive.Params";
        } else {
            throw new RuntimeException("Unknown DRIVE_CLASS");
        }

        telemetry.addLine("Track width tuner.");
        if (usedDefaultSeed) {
            telemetry.addData("seed", "default %.1f in → trackWidthTicks=%.2f",
                    DEFAULT_TRACK_WIDTH_IN, trackWidthTicks);
        } else {
            telemetry.addData("seed", "Params trackWidthTicks=%.2f", trackWidthTicks);
        }
        telemetry.addLine("Press START, then the robot spins in place: counterclockwise, then clockwise.");
        telemetry.addLine("Make sure it can rotate freely.");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        // Fit each direction separately so direction-dependent friction lands in the intercepts
        // instead of biasing a shared one.
        List<double[]> ccwSamples = rampAndSample(setCommand, imu, +1, telemetry, "CCW");
        TunerRegression.Result ccw = TunerRegression.robustFit(ccwSamples);

        List<double[]> cwSamples = rampAndSample(setCommand, imu, -1, telemetry, "CW");
        TunerRegression.Result cw = TunerRegression.robustFit(cwSamples);

        double slope = 0.5 * (ccw.slope + cw.slope);
        boolean valid = slope > 0.2 && slope < 5.0;
        double corrected = valid ? trackWidthTicks / slope : Double.NaN;

        while (opModeIsActive()) {
            if (valid) {
                telemetry.addLine("=== Paste into " + paramsClass + " ===");
                telemetry.addData("trackWidthTicks", "%.2f", corrected);
                telemetry.addLine();
                telemetry.addData("seed used", "%.2f%s", trackWidthTicks,
                        usedDefaultSeed ? " (default geometric)" : "");
                telemetry.addData("slope (actual/commanded)", "%.4f", slope);
                telemetry.addData("ccw fit", "slope %.4f, R^2 %.4f (%d/%d pts)", ccw.slope, ccw.r2, ccw.used, ccw.total);
                telemetry.addData("cw fit", "slope %.4f, R^2 %.4f (%d/%d pts)", cw.slope, cw.r2, cw.used, cw.total);
                telemetry.addLine("Re-run after pasting: the slope should be ~1.00.");
            } else {
                telemetry.addLine("FAILED: fitted slope " + String.format("%.3f", slope) + " is not plausible.");
                telemetry.addLine("Check motor directions and feedforward (kS/kV), then retry.");
                telemetry.addData("seed used", "%.2f%s", trackWidthTicks,
                        usedDefaultSeed ? " (default geometric)" : "");
            }
            telemetry.update();
        }
    }

    private static void requireFeedforward(double kV) {
        if (kV <= 0) {
            throw new RuntimeException(
                    "Tune the drive feedforward (kS/kV) before running TrackWidthTuner — the spin is driven through it.");
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
                    "Set inPerTick (ForwardPushTest) before TrackWidthTuner — needed to seed trackWidthTicks from "
                            + DEFAULT_TRACK_WIDTH_IN + " in.");
        }
        return DEFAULT_TRACK_WIDTH_IN / inPerTick;
    }

    /**
     * Ramps a commanded angular velocity through the production feedforward path and returns
     * (commanded, actual) yaw-rate samples, actual taken from the hub IMU.
     */
    private List<double[]> rampAndSample(
            Consumer<PoseVelocity2dDual<Time>> setCommand, IMU imu, int dir,
            MultipleTelemetry telemetry, String label) {
        List<double[]> samples = new ArrayList<>();
        double alpha = dir * MAX_ANG_VEL / RAMP_TIME;
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < RAMP_TIME) {
            double omegaCmd = alpha * timer.seconds();
            setCommand.accept(new PoseVelocity2dDual<>(
                    Vector2dDual.constant(new Vector2d(0, 0), 3),
                    new DualNum<>(new double[]{omegaCmd, alpha, 0})));

            double omegaActual = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
            if (Math.abs(omegaCmd) > MIN_ANG_VEL) {
                samples.add(new double[]{omegaCmd, omegaActual});
            }

            telemetry.addData("phase", label);
            telemetry.addData("commanded (rad/s)", "%.2f", omegaCmd);
            telemetry.addData("actual (rad/s)", "%.2f", omegaActual);
            telemetry.addData("samples", samples.size());
            telemetry.update();
        }
        setCommand.accept(new PoseVelocity2dDual<>(
                Vector2dDual.constant(new Vector2d(0, 0), 3),
                DualNum.constant(0, 3)));
        // brief coast so the next ramp starts from rest
        ElapsedTime settle = new ElapsedTime();
        while (opModeIsActive() && settle.seconds() < 1.0) {
            idle();
        }
        return samples;
    }
}
