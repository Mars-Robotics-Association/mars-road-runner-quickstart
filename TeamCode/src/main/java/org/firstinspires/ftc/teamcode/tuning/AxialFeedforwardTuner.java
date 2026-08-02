package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

/**
 * Automatic on-robot identification of the axial feedforward — including {@code kA}, which the
 * stock tuner leaves to eyeballing a target-vs-actual velocity graph.
 *
 * <p>Two phases (see {@link ReversalFeedforwardId}):
 *
 * <ol>
 *   <li>Slow open-loop forward ramp — same model as {@code ForwardRampLogger} → {@code kS}, {@code
 *       kV}. Ends early if the robot stops moving under power (e.g. hits a wall).
 *   <li>Reverse square wave (starts reverse) → residual {@code kA}. Half-cycles use the measured
 *       ramp travel (start → wall) so the kA phase spans the free corridor, not a fixed 1 s.
 * </ol>
 *
 * <p>Prerequisites: localization tuned and sign-correct (drive forward → +x). A wall ahead is OK —
 * the ramp stops on contact and reverse half-cycles run within the distance just driven.
 *
 * <p>On a successful full fit, writes {@code kS}/{@code kV}/{@code kA} into the live {@code PARAMS}
 * statics so later tuning OpModes in the same RC process can chain without a paste. A ramp-only (kA
 * failed) fit still writes {@code kS}/{@code kV}. Paste into source to keep values across restart
 * or redeploy.
 */
@Config
public final class AxialFeedforwardTuner extends LinearOpMode {
    /** Power increase per second during the kS/kV ramp (stock ForwardRampLogger uses 0.1). */
    public static double RAMP_POWER_PER_SEC = 0.1;

    /** Peak power of the ramp phase. */
    public static double RAMP_MAX = 0.9;

    /** |Power| of the reverse square wave used only for kA. */
    public static double KA_POWER = 0.6;

    /**
     * Fallback seconds per reverse direction when ramp travel is too short for a position-based
     * corridor (see {@link #MIN_TRAVEL_IN}).
     */
    public static double HALF_CYCLE = 1.0;

    /** Number of reverse half-cycles (direction holds). 4 ≈ two full round trips. */
    public static int KA_HALF_CYCLES = 4;

    /**
     * Inches kept clear of each end of the measured ramp corridor during position-based reverse
     * half-cycles.
     */
    public static double END_MARGIN_IN = ReversalFeedforwardId.DEFAULT_END_MARGIN_IN;

    /**
     * Minimum |ramp travel| (in) before reverse half-cycles use the full corridor instead of timed
     * {@link #HALF_CYCLE} holds.
     */
    public static double MIN_TRAVEL_IN = ReversalFeedforwardId.DEFAULT_MIN_TRAVEL_IN;

    /** Speeds (in/s) within this of zero use sign=0 in the kA residual. */
    public static double SIGN_DEADBAND = 1.0;

    /**
     * Ramp samples slower than this (encoder-tick units per second) are excluded from the kS/kV
     * fit. Converted with {@code inPerTick}; default 1000 cuts the breakaway knee at low speed.
     */
    public static double MIN_RAMP_TICKS_PER_SEC =
            ReversalFeedforwardId.DEFAULT_MIN_RAMP_TICKS_PER_SEC;

    /**
     * Absolute speed (in/s) treated as stalled after the robot has been moving; see also {@link
     * #STALL_FRAC}.
     */
    public static double STALL_SPEED = ReversalFeedforwardId.DEFAULT_STALL_SPEED;

    /** Seconds velocity must stay collapsed before ending the ramp for a wall/stall. */
    public static double STALL_TIME = ReversalFeedforwardId.DEFAULT_STALL_TIME;

    /** Speed (in/s) that must be reached once before stall detection arms. */
    public static double MOVING_SPEED = ReversalFeedforwardId.DEFAULT_MOVING_SPEED;

    /** Minimum commanded power for a stall to count. */
    public static double STALL_MIN_POWER = ReversalFeedforwardId.DEFAULT_STALL_MIN_POWER;

    /** Fraction of peak ramp speed below which velocity counts as collapsed. */
    public static double STALL_FRAC = ReversalFeedforwardId.DEFAULT_STALL_FRAC;

    /**
     * |Travel| (in) that arms stall detection even if speed never reached {@link #MOVING_SPEED}.
     */
    public static double ARM_TRAVEL_IN = ReversalFeedforwardId.DEFAULT_ARM_TRAVEL_IN;

    /** When armed, |d(pos)/dt| below this (in/s) under power counts as wall contact. */
    public static double POS_STALL_SPEED = ReversalFeedforwardId.DEFAULT_POS_STALL_SPEED;

    @Override
    public void runOpMode() throws InterruptedException {
        MultipleTelemetry telemetry =
                new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        DoubleConsumer setPower;
        DoubleSupplier forwardVel; // signed chassis forward velocity, in/s, from the localizer
        DoubleSupplier axisPos; // forward position (in); read after forwardVel updates localizer
        VoltageSensor voltageSensor;
        double inPerTick;
        String driveName;

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
            setPower =
                    p -> {
                        drive.leftFront.setPower(p);
                        drive.leftBack.setPower(p);
                        drive.rightBack.setPower(p);
                        drive.rightFront.setPower(p);
                    };
            forwardVel = () -> drive.localizer.update().linearVel.x;
            axisPos = () -> drive.localizer.getPose().position.x;
            voltageSensor = drive.voltageSensor;
            inPerTick = MecanumDrive.PARAMS.inPerTick;
            driveName = "MecanumDrive";
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));
            List<DcMotorEx> allMotors = new ArrayList<>(drive.leftMotors);
            allMotors.addAll(drive.rightMotors);
            setPower =
                    p -> {
                        for (DcMotorEx m : allMotors) {
                            m.setPower(p);
                        }
                    };
            forwardVel = () -> drive.localizer.update().linearVel.x;
            axisPos = () -> drive.localizer.getPose().position.x;
            voltageSensor = drive.voltageSensor;
            inPerTick = TankDrive.PARAMS.inPerTick;
            driveName = "TankDrive";
        } else {
            throw new RuntimeException("Unknown DRIVE_CLASS");
        }

        telemetry.addLine("Axial feedforward tuner (identifies kS, kV, kA).");
        telemetry.addLine("Phase 1: ramps FORWARD (+x) → kS, kV (like ForwardRampLogger).");
        telemetry.addLine("  Press gamepad1 A just before wall/mat edge (recommended).");
        telemetry.addLine("  Auto-stops if stalled against a wall (wheels may jam).");
        telemetry.addLine("Phase 2: reverse square wave → kA (first goes BACKWARD).");
        telemetry.addLine("  Half-cycles use the distance driven on the ramp when possible.");
        telemetry.addLine("Press START. Leave room BEHIND the start for reverse.");
        telemetry.addLine("Localization must be sign-correct (forward → +x).");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        ReversalFeedforwardId.Result fit =
                ReversalFeedforwardId.identify(
                        this,
                        telemetry,
                        setPower,
                        forwardVel,
                        axisPos,
                        voltageSensor,
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
                        POS_STALL_SPEED);

        boolean wroteRampOnly = false;
        if (!fit.singular) {
            writeAxialParams(driveName, fit.kS, fit.kV, fit.kA, true);
        } else if (fit.rampSamples > 0
                && Double.isFinite(fit.kS)
                && fit.kS != 0
                && Double.isFinite(fit.kV)
                && fit.kV > 0) {
            // Unblocks TrackWidthTuner / YawCoupling; leave kA untouched if reverse phase failed.
            writeAxialParams(driveName, fit.kS, fit.kV, Double.NaN, false);
            wroteRampOnly = true;
        }

        while (opModeIsActive()) {
            if (fit.singular) {
                telemetry.addLine("Fit failed: " + fit.message);
                telemetry.addData("samples", fit.samples);
                telemetry.addData("ramp samples used", fit.rampSamples);
                if (wroteRampOnly) {
                    telemetry.addLine("Ramp-only kS/kV written to live PARAMS (kA not updated):");
                    telemetry.addData("kS", "%.5f", fit.kS);
                    telemetry.addData("kV", "%.5e", fit.kV);
                    telemetry.addData("ramp R^2", "%.3f", fit.rampR2);
                    telemetry.addLine("Still paste into source to keep after restart.");
                } else if (fit.rampSamples > 0 && Double.isFinite(fit.kS) && fit.kS != 0) {
                    telemetry.addLine("Ramp-only (kA not trusted; not written — kV unusable):");
                    telemetry.addData("kS", "%.5f", fit.kS);
                    telemetry.addData("kV", "%.5e", fit.kV);
                    telemetry.addData("ramp R^2", "%.3f", fit.rampR2);
                }
            } else {
                telemetry.addLine("=== Written to live " + driveName + ".PARAMS ===");
                telemetry.addData("kS", "%.5f", fit.kS);
                telemetry.addData("kV", "%.5e", fit.kV);
                telemetry.addData("kA", "%.5e", fit.kA);
                telemetry.addLine();
                telemetry.addData("samples", fit.samples);
                telemetry.addData("ramp samples used", fit.rampSamples);
                telemetry.addData("ramp R^2", "%.3f", fit.rampR2);
                telemetry.addLine(
                        "Live for later OpModes this session. Paste into source to keep.");
                telemetry.addLine(
                        "kS/kV should match ForwardRampLogger closely; kA is the new piece.");
            }
            telemetry.update();
        }
    }

    /**
     * Writes axial feedforward into the drive's live {@code PARAMS}. When {@code writeKA} is false,
     * only {@code kS}/{@code kV} are updated (partial ramp fit).
     */
    private static void writeAxialParams(
            String driveName, double kS, double kV, double kA, boolean writeKA) {
        if ("MecanumDrive".equals(driveName)) {
            MecanumDrive.PARAMS.kS = kS;
            MecanumDrive.PARAMS.kV = kV;
            if (writeKA) {
                MecanumDrive.PARAMS.kA = kA;
            }
        } else {
            TankDrive.PARAMS.kS = kS;
            TankDrive.PARAMS.kV = kV;
            if (writeKA) {
                TankDrive.PARAMS.kA = kA;
            }
        }
    }
}
