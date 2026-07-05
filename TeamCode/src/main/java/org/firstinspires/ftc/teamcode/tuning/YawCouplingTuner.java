package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

import java.util.ArrayList;
import java.util.List;

/**
 * Automatic on-robot identification of the {@code yawCoupling*} feedforward constants — the parasitic
 * yaw (curl) a robot picks up when driving straight open-loop.
 *
 * <p>Unlike the stock ramp loggers (which record data for the offline tuning site), this OpMode drives
 * the ramp, fits the constants on the robot, and prints values you can paste straight into
 * {@code MecanumDrive.Params} / {@code TankDrive.Params}.
 *
 * <p>How it works: it drives an open-loop power ramp (feedforward off, no heading correction) and, for
 * each sample above {@link #MIN_SPEED}, records the chassis velocity (from the localizer) against the
 * yaw rate (from the IMU). It regresses yaw rate {@code ω = a + b·v}, then converts to the yaw-mode
 * voltage that cancels it using the small-signal yaw plant gain {@code kV·trackWidth}:
 * {@code kS = −kV·trackWidth·a}, {@code kV_yaw = −kV·trackWidth·b}. Mecanum additionally runs a strafe
 * ramp for the lateral constants; tank uses the axial pair only.
 *
 * <p>Prerequisites: localization and the drive feedforward ({@code kS}/{@code kV}/{@code kA}) must
 * already be tuned. Give the robot a clear straight lane of roughly 5–6 ft in each direction it will
 * ramp (forward, and sideways on mecanum).
 */
@Config
public final class YawCouplingTuner extends LinearOpMode {
    /** Peak open-loop power reached at the end of the ramp. */
    public static double MAX_POWER = 0.4;
    /** Seconds spent ramping power from 0 to {@link #MAX_POWER}. */
    public static double RAMP_TIME = 2.5;
    /** Samples slower than this (in/s) are ignored as start-up transient / noise. */
    public static double MIN_SPEED = 5.0;

    @Override
    public void runOpMode() throws InterruptedException {
        MultipleTelemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
            double kVWheel = MecanumDrive.PARAMS.kV / MecanumDrive.PARAMS.inPerTick;
            double trackWidth = drive.kinematics.trackWidth;
            requireCalibrated(kVWheel, trackWidth);

            IMU imu = drive.lazyImu.get();
            telemetry.addLine("Mecanum yaw-coupling tuner.");
            telemetry.addLine("Press START, then the robot ramps FORWARD, stops, then ramps SIDEWAYS.");
            telemetry.addLine("Make sure both lanes are clear.");
            telemetry.update();
            waitForStart();
            if (isStopRequested()) return;

            // --- forward ramp -> axial constants ---
            List<double[]> forwardSamples = rampAndSample(drive, imu, true, telemetry, "FORWARD");
            TunerRegression.Result fwd = TunerRegression.robustFit(forwardSamples);

            // --- strafe ramp -> lateral constants ---
            List<double[]> strafeSamples = rampAndSample(drive, imu, false, telemetry, "SIDEWAYS");
            TunerRegression.Result lat = TunerRegression.robustFit(strafeSamples);

            double factor = -kVWheel * trackWidth;
            double kSAxial = factor * fwd.intercept;
            double kVAxial = factor * fwd.slope;
            double kSLateral = factor * lat.intercept;
            double kVLateral = factor * lat.slope;

            while (opModeIsActive()) {
                telemetry.addLine("=== Paste into MecanumDrive.Params ===");
                telemetry.addData("yawCouplingKsAxial", "%.5f", kSAxial);
                telemetry.addData("yawCouplingKvAxial", "%.5f", kVAxial);
                telemetry.addData("yawCouplingKsLateral", "%.5f", kSLateral);
                telemetry.addData("yawCouplingKvLateral", "%.5f", kVLateral);
                telemetry.addLine();
                telemetry.addData("forward fit R^2", "%.4f (%d/%d pts)", fwd.r2, fwd.used, fwd.total);
                telemetry.addData("strafe fit R^2", "%.4f (%d/%d pts)", lat.r2, lat.used, lat.total);
                telemetry.addLine("If a re-test shows the curl got WORSE, negate that pair.");
                telemetry.update();
            }
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));
            double kVWheel = TankDrive.PARAMS.kV / TankDrive.PARAMS.inPerTick;
            double trackWidth = drive.kinematics.trackWidth;
            requireCalibrated(kVWheel, trackWidth);

            IMU imu = drive.lazyImu.get();
            telemetry.addLine("Tank yaw-coupling tuner.");
            telemetry.addLine("Press START, then the robot ramps FORWARD. Make sure the lane is clear.");
            telemetry.update();
            waitForStart();
            if (isStopRequested()) return;

            List<double[]> forwardSamples = rampAndSampleTank(drive, imu, telemetry);
            TunerRegression.Result fwd = TunerRegression.robustFit(forwardSamples);

            double factor = -kVWheel * trackWidth;
            double kSAxial = factor * fwd.intercept;
            double kVAxial = factor * fwd.slope;

            while (opModeIsActive()) {
                telemetry.addLine("=== Paste into TankDrive.Params ===");
                telemetry.addData("yawCouplingKsAxial", "%.5f", kSAxial);
                telemetry.addData("yawCouplingKvAxial", "%.5f", kVAxial);
                telemetry.addLine();
                telemetry.addData("forward fit R^2", "%.4f (%d/%d pts)", fwd.r2, fwd.used, fwd.total);
                telemetry.addLine("If a re-test shows the curl got WORSE, negate the pair.");
                telemetry.update();
            }
        } else {
            throw new RuntimeException("Unknown DRIVE_CLASS");
        }
    }

    private static void requireCalibrated(double kVWheel, double trackWidth) {
        if (kVWheel <= 0 || trackWidth <= 0) {
            throw new RuntimeException(
                    "Tune the drive feedforward (kV) and track width before running YawCouplingTuner");
        }
    }

    /**
     * Ramps the mecanum drive open-loop (forward or sideways) and returns (velocity, yawRate) samples.
     */
    private List<double[]> rampAndSample(
            MecanumDrive drive, IMU imu, boolean forward, MultipleTelemetry telemetry, String label) {
        List<double[]> samples = new ArrayList<>();
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < RAMP_TIME) {
            double power = MAX_POWER * timer.seconds() / RAMP_TIME;
            if (forward) {
                setMecanumPowers(drive, power, power, power, power);
            } else {
                // strafe pattern (lf, lb, rb, rf) = (-, +, -, +) drives +y (robot-left)
                setMecanumPowers(drive, -power, power, -power, power);
            }

            PoseVelocity2d vel = drive.localizer.update();
            double v = forward ? vel.linearVel.x : vel.linearVel.y;
            double omega = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
            if (Math.abs(v) > MIN_SPEED) {
                samples.add(new double[]{v, omega});
            }

            telemetry.addData("phase", label);
            telemetry.addData("speed (in/s)", "%.1f", v);
            telemetry.addData("yaw rate (rad/s)", "%.3f", omega);
            telemetry.addData("samples", samples.size());
            telemetry.update();
        }
        setMecanumPowers(drive, 0, 0, 0, 0);
        // brief coast so the next ramp starts from rest
        ElapsedTime settle = new ElapsedTime();
        while (opModeIsActive() && settle.seconds() < 1.0) {
            drive.localizer.update();
        }
        return samples;
    }

    private List<double[]> rampAndSampleTank(TankDrive drive, IMU imu, MultipleTelemetry telemetry) {
        List<double[]> samples = new ArrayList<>();
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < RAMP_TIME) {
            double power = MAX_POWER * timer.seconds() / RAMP_TIME;
            for (DcMotorEx m : drive.leftMotors) m.setPower(power);
            for (DcMotorEx m : drive.rightMotors) m.setPower(power);

            PoseVelocity2d vel = drive.localizer.update();
            double v = vel.linearVel.x;
            double omega = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
            if (Math.abs(v) > MIN_SPEED) {
                samples.add(new double[]{v, omega});
            }

            telemetry.addData("phase", "FORWARD");
            telemetry.addData("speed (in/s)", "%.1f", v);
            telemetry.addData("yaw rate (rad/s)", "%.3f", omega);
            telemetry.addData("samples", samples.size());
            telemetry.update();
        }
        for (DcMotorEx m : drive.leftMotors) m.setPower(0);
        for (DcMotorEx m : drive.rightMotors) m.setPower(0);
        return samples;
    }

    private static void setMecanumPowers(MecanumDrive drive, double lf, double lb, double rb, double rf) {
        drive.leftFront.setPower(lf);
        drive.leftBack.setPower(lb);
        drive.rightBack.setPower(rb);
        drive.rightFront.setPower(rf);
    }
}
