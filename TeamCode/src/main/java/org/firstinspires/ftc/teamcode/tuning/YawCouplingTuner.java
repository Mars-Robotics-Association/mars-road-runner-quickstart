package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
import org.firstinspires.ftc.teamcode.utils.CsvLogger;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

/**
 * Automatic on-robot identification of the {@code yawCoupling*} feedforward constants — the
 * parasitic yaw (curl) a robot picks up when driving straight open-loop.
 *
 * <p>Unlike the stock ramp loggers (which record data for the offline tuning site), this OpMode
 * drives the ramp, fits the constants on the robot, writes them into the live {@code PARAMS}
 * statics, and prints values to paste into source for persistence across restart or redeploy.
 *
 * <p>How it works: it drives an open-loop power ramp (feedforward off, no heading correction) and,
 * for each sample above {@link #MIN_SPEED}, records the chassis velocity (from the localizer)
 * against the yaw rate. It regresses yaw rate {@code ω = a + b·v}, then converts to the yaw-mode
 * voltage that cancels it using the small-signal yaw plant gain {@code kV·trackWidth}: {@code kS =
 * −kV·trackWidth·a}, {@code kV_yaw = −kV·trackWidth·b}. Mecanum additionally runs a strafe ramp for
 * the lateral constants; tank uses the axial pair only.
 *
 * <p>Yaw rate source: Pinpoint heading velocity when available (preferred — independent of hub
 * mount). Otherwise Control Hub IMU angular velocity, always read as deg/s and converted to rad/s
 * (FtcRobotController#1070), using the axis of largest |rate| during the ramp so a misconfigured
 * hub orientation still yields a signed spin rate.
 *
 * <p>Fit note: curl is often a weak signal (small ω over IMU/localizer noise). This tuner uses a
 * plain least-squares fit over samples above {@link #MIN_SPEED} rather than {@link
 * TunerRegression#robustFit}'s 0.95 R² chase, which was designed for strong V-vs-v feedforward
 * ramps and can discard most points without ever getting a high R².
 *
 * <p>Prerequisites: localization, drive feedforward ({@code kS}/{@code kV}/{@code kA}), and a sane
 * {@code trackWidthTicks} must already be tuned. Give the robot a clear straight lane of roughly
 * 6–8 ft in each direction it will ramp (forward, and sideways on mecanum). Press gamepad1 A during
 * a ramp to cut power early (e.g. before a wall); the fit uses whatever samples were collected.
 *
 * <p>Dashboard knobs: raise {@link #MAX_POWER} / {@link #RAMP_TIME} if the robot barely moves or R²
 * stays near zero with a short speed range.
 *
 * <p>Each run writes two CSVs under {@code /sdcard/FIRST/} (via {@link CsvLogger}):
 *
 * <ul>
 *   <li>{@code yaw_coupling_samples_&lt;stamp&gt;.csv} — raw ramp measurements (command, chassis
 *       velocity, yaw-source rate, localizer pose/angVel)
 *   <li>{@code yaw_coupling_meta_&lt;stamp&gt;.csv} — plant scale and OpMode knobs needed to re-fit
 *       offline (not the fitted yawCoupling* values)
 * </ul>
 *
 * Pull them with {@code telemetry/pull.sh}.
 */
@Config
public final class YawCouplingTuner extends MarsLinearOpMode {
    /** When false, skip writing CSVs (useful if the hub disk is full). */
    public static boolean LOG_CSV = true;

    /** Flush the sample CSV to disk after this many buffered rows. */
    public static int LOG_FLUSH_EVERY = 256;

    /**
     * Peak open-loop power at the end of the ramp. 0.4 was too gentle for many mecanum bases (only
     * a couple feet of travel); 0.65 needs ~6–8 ft of clear lane but gives a usable speed span.
     */
    public static double MAX_POWER = 0.65;

    /** Seconds spent ramping power from 0 to {@link #MAX_POWER}. */
    public static double RAMP_TIME = 5.0;

    /** Samples slower than this (in/s) are ignored as start-up transient / noise. */
    public static double MIN_SPEED = 5.0;

    /** R² below this is flagged as low-confidence (curl may be tiny or yaw measurement noisy). */
    public static double WARN_R2 = 0.4;

    /**
     * Raw ramp measurements. {@code omega_src} is the yaw rate used by the fit (Pinpoint or hub
     * IMU); {@code omega_loc} is localizer angVel for a second independent trace. Applied voltage
     * is not part of the curl model — power is logged only as the open-loop command.
     */
    private static final String SAMPLE_HEADER =
            "t_s,phase,power,vel_in_s,omega_src_rad_s,omega_loc_rad_s,pose_x,pose_y,pose_h";

    /**
     * One-row (or few-row) run config for offline re-fit. {@code factor = -kV_wheel * track_width}
     * converts ω=a+b·v into coupling kS/kV — log the plant pieces, not the fit.
     */
    private static final String META_HEADER =
            "drive,yaw_source,kV_wheel,track_width,in_per_tick,kV_params,max_power,ramp_time,min_speed";

    private CsvLogger sampleLog;
    private CsvLogger metaLog;

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        MultipleTelemetry telem = (MultipleTelemetry) telemetry;

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive =
                    new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0), this::batteryVoltage);
            double kVWheel = MecanumDrive.PARAMS.kV / MecanumDrive.PARAMS.inPerTick;
            double trackWidth = drive.kinematics.trackWidth;
            requireCalibrated(kVWheel, trackWidth);

            DoubleSupplier yawRate = makeYawRateSource(drive.localizer, drive.lazyImu.get());
            String yawSourceName = yawSourceLabel(drive.localizer);

            openLogs(
                    "MecanumDrive",
                    yawSourceName,
                    kVWheel,
                    trackWidth,
                    MecanumDrive.PARAMS.inPerTick,
                    MecanumDrive.PARAMS.kV);

            telem.addLine("Mecanum yaw-coupling tuner.");
            telem.addData("yaw source", yawSourceName);
            telem.addData(
                    "ramp",
                    "%.0f%% power over %.1fs (need ~6-8 ft clear)",
                    MAX_POWER * 100,
                    RAMP_TIME);
            telem.addLine("Press START, then the robot ramps FORWARD, stops, then ramps SIDEWAYS.");
            telem.addLine("Press gamepad1 A during a ramp to stop early (e.g. before a wall).");
            telem.addLine("Make sure both lanes are clear.");
            if (sampleLog != null) {
                telem.addData("sample log", sampleLog.fileName());
                telem.addData("meta log", metaLog.fileName());
                telem.addLine("Pull with: telemetry/pull.sh");
            }
            telem.update();
            waitForStart();
            if (isStopRequested()) {
                closeLogs();
                return;
            }

            // --- forward ramp -> axial constants ---
            RampResult forward = rampAndSample(drive, yawRate, true, telem, "FORWARD");
            TunerRegression.Result fwd = fitCurl(forward.samples);

            // --- strafe ramp -> lateral constants ---
            RampResult strafe = rampAndSample(drive, yawRate, false, telem, "SIDEWAYS");
            TunerRegression.Result lat = fitCurl(strafe.samples);

            double factor = -kVWheel * trackWidth;
            double kSAxial = factor * fwd.intercept;
            double kVAxial = factor * fwd.slope;
            double kSLateral = factor * lat.intercept;
            double kVLateral = factor * lat.slope;

            MecanumDrive.PARAMS.yawCouplingKsAxial = kSAxial;
            MecanumDrive.PARAMS.yawCouplingKvAxial = kVAxial;
            MecanumDrive.PARAMS.yawCouplingKsLateral = kSLateral;
            MecanumDrive.PARAMS.yawCouplingKvLateral = kVLateral;

            closeLogs();

            while (nextFrame()) {
                telemetry.addLine("=== Written to live MecanumDrive.PARAMS ===");
                telemetry.addData("yawCouplingKsAxial", "%.5f", kSAxial);
                telemetry.addData("yawCouplingKvAxial", "%.5e", kVAxial);
                telemetry.addData("yawCouplingKsLateral", "%.5f", kSLateral);
                telemetry.addData("yawCouplingKvLateral", "%.5e", kVLateral);
                telemetry.addLine();
                telemetry.addData("yaw source", yawSourceName);
                addFitTelemetry(telem, "forward", fwd, forward);
                addFitTelemetry(telem, "strafe", lat, strafe);
                if (LOG_CSV) {
                    telemetry.addLine(
                            "CSVs on hub under /sdcard/FIRST/ — pull with telemetry/pull.sh");
                }
                telemetry.addLine(
                        "Live for later OpModes this session. Paste into source to keep.");
                telemetry.addLine("If a re-test shows the curl got WORSE, negate that pair.");
                if (fwd.r2 < WARN_R2 || lat.r2 < WARN_R2) {
                    telemetry.addLine();
                    telemetry.addLine(
                            "Low R^2 is common when curl is small (noise >> signal). Near-zero"
                                    + " constants are fine. If curl is visibly large, raise"
                                    + " MAX_POWER/RAMP_TIME or check yaw source.");
                }
            }
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0), this::batteryVoltage);
            double kVWheel = TankDrive.PARAMS.kV / TankDrive.PARAMS.inPerTick;
            double trackWidth = drive.kinematics.trackWidth;
            requireCalibrated(kVWheel, trackWidth);

            DoubleSupplier yawRate = makeYawRateSource(drive.localizer, drive.lazyImu.get());
            String yawSourceName = yawSourceLabel(drive.localizer);

            openLogs(
                    "TankDrive",
                    yawSourceName,
                    kVWheel,
                    trackWidth,
                    TankDrive.PARAMS.inPerTick,
                    TankDrive.PARAMS.kV);

            telem.addLine("Tank yaw-coupling tuner.");
            telem.addData("yaw source", yawSourceName);
            telem.addData("ramp", "%.0f%% power over %.1fs", MAX_POWER * 100, RAMP_TIME);
            telem.addLine(
                    "Press START, then the robot ramps FORWARD. Make sure the lane is clear.");
            telem.addLine("Press gamepad1 A during the ramp to stop early (e.g. before a wall).");
            if (sampleLog != null) {
                telem.addData("sample log", sampleLog.fileName());
                telem.addData("meta log", metaLog.fileName());
                telem.addLine("Pull with: telemetry/pull.sh");
            }
            telem.update();
            waitForStart();
            if (isStopRequested()) {
                closeLogs();
                return;
            }

            RampResult forward = rampAndSampleTank(drive, yawRate, telem);
            TunerRegression.Result fwd = fitCurl(forward.samples);

            double factor = -kVWheel * trackWidth;
            double kSAxial = factor * fwd.intercept;
            double kVAxial = factor * fwd.slope;

            TankDrive.PARAMS.yawCouplingKsAxial = kSAxial;
            TankDrive.PARAMS.yawCouplingKvAxial = kVAxial;

            closeLogs();

            while (nextFrame()) {
                telemetry.addLine("=== Written to live TankDrive.PARAMS ===");
                telemetry.addData("yawCouplingKsAxial", "%.5f", kSAxial);
                telemetry.addData("yawCouplingKvAxial", "%.5e", kVAxial);
                telemetry.addLine();
                telemetry.addData("yaw source", yawSourceName);
                addFitTelemetry(telem, "forward", fwd, forward);
                if (LOG_CSV) {
                    telemetry.addLine(
                            "CSVs on hub under /sdcard/FIRST/ — pull with telemetry/pull.sh");
                }
                telemetry.addLine(
                        "Live for later OpModes this session. Paste into source to keep.");
                telemetry.addLine("If a re-test shows the curl got WORSE, negate the pair.");
                if (fwd.r2 < WARN_R2) {
                    telemetry.addLine();
                    telemetry.addLine(
                            "Low R^2 is common when curl is small. Near-zero constants are fine.");
                }
            }
        } else {
            throw new RuntimeException("Unknown DRIVE_CLASS");
        }
    }

    private void openLogs(
            String drive,
            String yawSource,
            double kVWheel,
            double trackWidth,
            double inPerTick,
            double kVParams) {
        if (!LOG_CSV) {
            return;
        }
        String stamp =
                new java.text.SimpleDateFormat("yyyyMMdd_HHmmss", java.util.Locale.US)
                        .format(new java.util.Date());
        sampleLog = new CsvLogger("yaw_coupling_samples_" + stamp + ".csv", SAMPLE_HEADER);
        metaLog = new CsvLogger("yaw_coupling_meta_" + stamp + ".csv", META_HEADER);
        metaLog.row(
                drive,
                yawSource,
                kVWheel,
                trackWidth,
                inPerTick,
                kVParams,
                MAX_POWER,
                RAMP_TIME,
                MIN_SPEED);
        metaLog.flush();
    }

    private void closeLogs() {
        if (sampleLog != null) {
            sampleLog.close();
            sampleLog = null;
        }
        if (metaLog != null) {
            metaLog.close();
            metaLog = null;
        }
    }

    private void maybeFlushSamples() {
        if (sampleLog != null && sampleLog.bufferedRows() >= LOG_FLUSH_EVERY) {
            sampleLog.flush();
        }
    }

    private static void requireCalibrated(double kVWheel, double trackWidth) {
        if (kVWheel <= 0 || trackWidth <= 0) {
            throw new RuntimeException(
                    "Tune the drive feedforward (kV) and track width before running"
                            + " YawCouplingTuner");
        }
    }

    private static String yawSourceLabel(Localizer localizer) {
        return localizer instanceof PinpointLocalizer
                ? "Pinpoint heading velocity"
                : "Hub IMU (deg/s→rad, dominant axis)";
    }

    /**
     * Prefer Pinpoint heading rate. Else hub IMU with deg→rad conversion; during the ramp the
     * caller still records a single omega via {@link #readHubYawRate}, which picks the dominant
     * axis so hub orientation misconfiguration does not zero out the signal.
     */
    private static DoubleSupplier makeYawRateSource(Localizer localizer, IMU imu) {
        if (localizer instanceof PinpointLocalizer) {
            PinpointLocalizer pl = (PinpointLocalizer) localizer;
            return () -> {
                pl.driver.update();
                return pl.driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
            };
        }
        // Stateful dominant-axis estimate shared across reads in one OpMode run.
        final double[] axis = {0, 0, 1}; // default Z until enough samples accumulate
        final double[] sum = {0, 0, 0}; // Σ |ω_i| to pick dominant; sign from that axis
        final int[] n = {0};
        return () -> readHubYawRate(imu, axis, sum, n);
    }

    /**
     * Hub IMU yaw rate in rad/s. Always converts from degrees (SDK unit bug). After a few samples,
     * locks onto the axis with largest mean |rate| (field-vertical spin under wrong orientation).
     */
    private static double readHubYawRate(IMU imu, double[] axis, double[] sumAbs, int[] n) {
        AngularVelocity avDeg = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        double wx = Math.toRadians(avDeg.xRotationRate);
        double wy = Math.toRadians(avDeg.yRotationRate);
        double wz = Math.toRadians(avDeg.zRotationRate);

        // Early samples: build which axis is active during pure translation (curl is small, so
        // this mainly matters if orientation dumps true yaw onto X/Y when the robot *does* spin).
        // For open-loop straight drive, curl is the signal — use Z unless another axis is clearly
        // larger in magnitude over time.
        sumAbs[0] += Math.abs(wx);
        sumAbs[1] += Math.abs(wy);
        sumAbs[2] += Math.abs(wz);
        n[0]++;
        if (n[0] >= 10) {
            if (sumAbs[0] >= sumAbs[1] && sumAbs[0] >= sumAbs[2]) {
                axis[0] = 1;
                axis[1] = 0;
                axis[2] = 0;
            } else if (sumAbs[1] >= sumAbs[0] && sumAbs[1] >= sumAbs[2]) {
                axis[0] = 0;
                axis[1] = 1;
                axis[2] = 0;
            } else {
                axis[0] = 0;
                axis[1] = 0;
                axis[2] = 1;
            }
        }
        return wx * axis[0] + wy * axis[1] + wz * axis[2];
    }

    /** Plain OLS on (v, ω) — better for weak curl than chasing R² ≥ 0.95. */
    private static TunerRegression.Result fitCurl(List<double[]> samples) {
        if (samples.size() < 2) {
            return new TunerRegression.Result(0, 0, 0, 0, 0, samples.size());
        }
        return TunerRegression.fitAll(samples);
    }

    private static void addFitTelemetry(
            MultipleTelemetry telemetry,
            String label,
            TunerRegression.Result fit,
            RampResult ramp) {
        telemetry.addData(
                label + " fit",
                "R^2 %.3f  a=%.4f  b=%.5f  (%d pts)",
                fit.r2,
                fit.intercept,
                fit.slope,
                fit.used);
        telemetry.addData(
                label + " speed", "peak %.1f in/s, travel ~%.0f in", ramp.peakSpeed, ramp.travelIn);
        if (ramp.peakSpeed < MIN_SPEED * 2) {
            telemetry.addData(label + " warn", "peak speed low — raise MAX_POWER or free the lane");
        }
    }

    /**
     * Ramps the mecanum drive open-loop (forward or sideways) and returns (velocity, yawRate)
     * samples. Press gamepad1 A to end the ramp early; edge-detect so a held A from before START
     * does not fire.
     */
    private RampResult rampAndSample(
            MecanumDrive drive,
            DoubleSupplier yawRate,
            boolean forward,
            MultipleTelemetry telemetry,
            String label) {
        List<double[]> samples = new ArrayList<>();
        ElapsedTime timer = new ElapsedTime();
        boolean stoppedByDriver = false;
        double peakSpeed = 0;
        double travel = 0;
        double lastT = 0;

        // Fresh pose for travel estimate
        drive.localizer.setPose(new Pose2d(0, 0, 0));

        while (nextFrame() && timer.seconds() < RAMP_TIME) {
            if (gamepad1.aWasPressed()) {
                stoppedByDriver = true;
                break;
            }

            double power = MAX_POWER * timer.seconds() / RAMP_TIME;
            if (forward) {
                setMecanumPowers(drive, power, power, power, power);
            } else {
                // strafe pattern (lf, lb, rb, rf) = (-, +, -, +) drives +y (robot-left)
                setMecanumPowers(drive, -power, power, -power, power);
            }

            PoseVelocity2d vel = drive.localizer.update();
            double v = forward ? vel.linearVel.x : vel.linearVel.y;
            double omegaSrc = yawRate.getAsDouble();
            double t = timer.seconds();
            double dt = t - lastT;
            if (dt > 1e-6 && dt < 0.2) {
                travel += Math.abs(v) * dt;
            }
            lastT = t;
            peakSpeed = Math.max(peakSpeed, Math.abs(v));

            if (Math.abs(v) > MIN_SPEED) {
                samples.add(new double[] {v, omegaSrc});
            }
            if (sampleLog != null) {
                Pose2d pose = drive.localizer.getPose();
                sampleLog.row(
                        t,
                        label,
                        power,
                        v,
                        omegaSrc,
                        vel.angVel,
                        pose.position.x,
                        pose.position.y,
                        pose.heading.toDouble());
                maybeFlushSamples();
            }

            telemetry.addData("phase", label);
            telemetry.addData("power", "%.2f", power);
            telemetry.addData("speed (in/s)", "%.1f", v);
            telemetry.addData("yaw rate (rad/s)", "%.4f", omegaSrc);
            telemetry.addData("travel (in)", "%.0f", travel);
            telemetry.addData("samples", samples.size());
            telemetry.addLine("A = stop ramp early");
        }
        setMecanumPowers(drive, 0, 0, 0, 0);
        if (sampleLog != null) {
            sampleLog.flush();
        }
        // brief coast so the next ramp starts from rest
        ElapsedTime settle = new ElapsedTime();
        while (nextFrame() && settle.seconds() < 1.0) {
            drive.localizer.update();
            telemetry.addData(
                    "phase", stoppedByDriver ? label + " (A stop) settling" : label + " settling");
            telemetry.addData("samples kept", samples.size());
            telemetry.addData("peak speed", "%.1f in/s", peakSpeed);
        }
        return new RampResult(samples, peakSpeed, travel);
    }

    private RampResult rampAndSampleTank(
            TankDrive drive, DoubleSupplier yawRate, MultipleTelemetry telemetry) {
        List<double[]> samples = new ArrayList<>();
        ElapsedTime timer = new ElapsedTime();
        double peakSpeed = 0;
        double travel = 0;
        double lastT = 0;
        drive.localizer.setPose(new Pose2d(0, 0, 0));

        while (nextFrame() && timer.seconds() < RAMP_TIME) {
            if (gamepad1.aWasPressed()) {
                break;
            }

            double power = MAX_POWER * timer.seconds() / RAMP_TIME;
            for (DcMotorEx m : drive.leftMotors) m.setPower(power);
            for (DcMotorEx m : drive.rightMotors) m.setPower(power);

            PoseVelocity2d vel = drive.localizer.update();
            double v = vel.linearVel.x;
            double omegaSrc = yawRate.getAsDouble();
            double t = timer.seconds();
            double dt = t - lastT;
            if (dt > 1e-6 && dt < 0.2) {
                travel += Math.abs(v) * dt;
            }
            lastT = t;
            peakSpeed = Math.max(peakSpeed, Math.abs(v));

            if (Math.abs(v) > MIN_SPEED) {
                samples.add(new double[] {v, omegaSrc});
            }
            if (sampleLog != null) {
                Pose2d pose = drive.localizer.getPose();
                sampleLog.row(
                        t,
                        "FORWARD",
                        power,
                        v,
                        omegaSrc,
                        vel.angVel,
                        pose.position.x,
                        pose.position.y,
                        pose.heading.toDouble());
                maybeFlushSamples();
            }

            telemetry.addData("phase", "FORWARD");
            telemetry.addData("power", "%.2f", power);
            telemetry.addData("speed (in/s)", "%.1f", v);
            telemetry.addData("yaw rate (rad/s)", "%.4f", omegaSrc);
            telemetry.addData("travel (in)", "%.0f", travel);
            telemetry.addData("samples", samples.size());
            telemetry.addLine("A = stop ramp early");
        }
        for (DcMotorEx m : drive.leftMotors) m.setPower(0);
        for (DcMotorEx m : drive.rightMotors) m.setPower(0);
        if (sampleLog != null) {
            sampleLog.flush();
        }
        return new RampResult(samples, peakSpeed, travel);
    }

    private static void setMecanumPowers(
            MecanumDrive drive, double lf, double lb, double rb, double rf) {
        drive.leftFront.setPower(lf);
        drive.leftBack.setPower(lb);
        drive.rightBack.setPower(rb);
        drive.rightFront.setPower(rf);
    }

    private static final class RampResult {
        final List<double[]> samples;
        final double peakSpeed;
        final double travelIn;

        RampResult(List<double[]> samples, double peakSpeed, double travelIn) {
            this.samples = samples;
            this.peakSpeed = peakSpeed;
            this.travelIn = travelIn;
        }
    }
}
