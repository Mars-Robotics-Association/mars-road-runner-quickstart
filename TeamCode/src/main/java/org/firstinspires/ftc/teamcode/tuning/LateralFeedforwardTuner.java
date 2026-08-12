package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.opmodes.base.MarsLinearOpMode;
import org.firstinspires.ftc.teamcode.utils.CsvLogger;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

/**
 * Automatic on-robot identification of the anisotropic (strafe) feedforward constants {@code
 * lateralKS} / {@code lateralKV} / {@code lateralKA} for a mecanum drive.
 *
 * <p>Same two-phase procedure as {@link AxialFeedforwardTuner} (slow ramp → kS/kV, reverse square
 * wave → kA), applied as a pure strafe with a light heading hold so open-loop roller scrub does not
 * accumulate yaw. The ramp drives <b>LEFT</b> (robot +y); it ends early if motion collapses under
 * power (or on gamepad1 A), then reverses RIGHT for kA. See {@link ReversalFeedforwardId}.
 *
 * <p>Prerequisites: localization and axial feedforward already tuned. Leave room on the right for
 * reverse; a wall on the left is fine. Expect lateral constants to exceed their axial counterparts
 * (roller scrub).
 *
 * <p>On a successful full fit, writes {@code lateralKS}/{@code lateralKV}/{@code lateralKA} and
 * sets {@code useAnisotropicFeedforward = true} on the live {@code PARAMS} statics so later OpModes
 * in the same RC process can chain without a paste. A ramp-only fit still writes {@code
 * lateralKS}/{@code lateralKV} and enables anisotropic mode. Paste into source to keep values
 * across restart or redeploy.
 *
 * <p>Each run writes two CSVs under {@code /sdcard/FIRST/} (via {@link CsvLogger}):
 *
 * <ul>
 *   <li>{@code lateral_ff_samples_&lt;stamp&gt;.csv} — raw loop measurements (power, battery,
 *       velocity, pose; see {@link ReversalFeedforwardId#SAMPLE_HEADER})
 *   <li>{@code lateral_ff_meta_&lt;stamp&gt;.csv} — run config / plant scale for offline re-fit
 *       (see {@link ReversalFeedforwardId#META_HEADER}); not the fitted lateralKS/KV/KA
 * </ul>
 *
 * Pull them with {@code telemetry/pull.sh}.
 */
@Config
public final class LateralFeedforwardTuner extends MarsLinearOpMode {
    /** When false, skip writing CSVs (useful if the hub disk is full). */
    public static boolean LOG_CSV = true;

    /** Power increase per second during the lateral ramp. */
    public static double RAMP_POWER_PER_SEC = 0.1;

    /** Peak power of the ramp phase. */
    public static double RAMP_MAX = 0.9;

    /** |Power| of the reverse square wave used only for lateralKA. */
    public static double KA_POWER = 0.6;

    /**
     * Fallback seconds per reverse direction when ramp travel is too short for a position-based
     * corridor (see {@link #MIN_TRAVEL_IN}).
     */
    public static double HALF_CYCLE = 1.0;

    /** Number of reverse half-cycles. */
    public static int KA_HALF_CYCLES = 4;

    /** Inches kept clear of each end of the measured ramp corridor. */
    public static double END_MARGIN_IN = ReversalFeedforwardId.DEFAULT_END_MARGIN_IN;

    /** Minimum |ramp travel| (in) to use position-based reverse half-cycles. */
    public static double MIN_TRAVEL_IN = ReversalFeedforwardId.DEFAULT_MIN_TRAVEL_IN;

    /** Speeds (in/s) within this of zero use sign=0 in the kA residual. */
    public static double SIGN_DEADBAND = 1.0;

    /**
     * Ramp samples slower than this (encoder-tick units per second) are excluded from the kS/kV
     * fit. Converted with {@code inPerTick}; default 1000 cuts the breakaway knee at low speed.
     */
    public static double MIN_RAMP_TICKS_PER_SEC =
            ReversalFeedforwardId.DEFAULT_MIN_RAMP_TICKS_PER_SEC;

    /** Absolute speed (in/s) treated as stalled after the robot has been moving. */
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
     * |Travel| (in) that arms stall detection even if speed never reached {@link #MOVING_SPEED}
     * (lateral ramps often hit a wall before that).
     */
    public static double ARM_TRAVEL_IN = ReversalFeedforwardId.DEFAULT_ARM_TRAVEL_IN;

    /**
     * When armed, |d(pos)/dt| below this (in/s) under power counts as wall contact (helps when
     * mecanum rollers keep velocity slightly nonzero against a wall).
     */
    public static double POS_STALL_SPEED = ReversalFeedforwardId.DEFAULT_POS_STALL_SPEED;

    /**
     * Heading hold gain (power per rad of error). Open-loop strafe otherwise curls; this adds a
     * pure yaw correction so lateral sysid stays roughly straight. 0 disables.
     */
    public static double HEADING_GAIN = 1.2;

    /** Heading-rate damping (power per rad/s). */
    public static double HEADING_VEL_GAIN = 0.08;

    /** Max |yaw power| blended into the strafe command. */
    public static double HEADING_MAX_CORR = 0.35;

    @Override
    public void runOpMode() throws InterruptedException {
        if (!TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            throw new RuntimeException(
                    "LateralFeedforwardTuner is mecanum-only; a "
                            + (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)
                                    ? "tank"
                                    : "non-mecanum")
                            + " drive has no lateral motion.");
        }

        initRobot();
        MecanumDrive drive =
                MecanumDrive.forMarsLinear(hardwareMap, new Pose2d(0, 0, 0), this::batteryVoltage);
        double inPerTick = MecanumDrive.PARAMS.inPerTick;
        double lateralMultiplier = drive.kinematics.lateralMultiplier;

        CsvLogger sampleLog = null;
        CsvLogger metaLog = null;
        if (LOG_CSV) {
            String stamp =
                    new java.text.SimpleDateFormat("yyyyMMdd_HHmmss", java.util.Locale.US)
                            .format(new java.util.Date());
            sampleLog =
                    new CsvLogger(
                            "lateral_ff_samples_" + stamp + ".csv",
                            ReversalFeedforwardId.SAMPLE_HEADER);
            // Shared identify knobs + lateral-only plant/hold config (not fit results).
            String metaHeader =
                    ReversalFeedforwardId.META_HEADER
                            + ",lateral_multiplier,heading_gain,heading_vel_gain,heading_max_corr";
            metaLog = new CsvLogger("lateral_ff_meta_" + stamp + ".csv", metaHeader);
            metaLog.row(
                    "MecanumDrive",
                    "lateral",
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
                    POS_STALL_SPEED,
                    ReversalFeedforwardId.DEFAULT_KA_WINDOW,
                    ReversalFeedforwardId.DEFAULT_KA_STRIDE,
                    lateralMultiplier,
                    HEADING_GAIN,
                    HEADING_VEL_GAIN,
                    HEADING_MAX_CORR);
            metaLog.flush();
        }

        telemetry.addLine("Mecanum lateral (strafe) feedforward tuner (kS, kV, kA).");
        telemetry.addLine("Phase 1: ramps LEFT (robot +y) → lateralKS/KV.");
        telemetry.addLine("  Press gamepad1 A just before wall/mat edge (recommended).");
        telemetry.addLine("  Auto-stops if stalled against a wall (wheels may jam).");
        telemetry.addLine("  Heading hold on (HEADING_GAIN) to limit yaw curl.");
        telemetry.addLine("Phase 2: reverse square wave → lateralKA (first goes RIGHT).");
        telemetry.addLine("  Half-cycles use the distance driven on the ramp when possible.");
        telemetry.addLine("Press START. Leave room on the RIGHT for reverse.");
        if (sampleLog != null) {
            telemetry.addData("sample log", sampleLog.fileName());
            telemetry.addData("meta log", metaLog.fileName());
            telemetry.addLine("Pull with: telemetry/pull.sh");
        }
        telemetry.update();
        waitForStart();
        if (isStopRequested()) {
            closeLogs(sampleLog, metaLog);
            return;
        }

        // Hold the heading at START so open-loop strafe reversals do not walk in yaw.
        drive.localizer.update();
        final Rotation2d headingTarget = drive.localizer.getPose().heading;

        // +power => strafe (+y = robot LEFT) plus a pure-yaw heading correction.
        // Kinematics: strafe p → (-p, +p, -p, +p); +angVel u → (-u, -u, +u, +u).
        // Prefer preserving lateral |p| for the sysid voltage model: shrink yaw correction
        // if |p| + |u| would saturate, rather than scaling all four wheels down.
        DoubleConsumer setPower =
                p -> {
                    PoseVelocity2d tw = drive.localizer.update();
                    double err = headingTarget.minus(drive.localizer.getPose().heading);
                    double u = HEADING_GAIN * err - HEADING_VEL_GAIN * tw.angVel;
                    double uMax = Math.min(HEADING_MAX_CORR, Math.max(0.0, 1.0 - Math.abs(p)));
                    u = Range.clip(u, -uMax, uMax);

                    drive.leftFront.setPower(-p - u);
                    drive.leftBack.setPower(+p - u);
                    drive.rightBack.setPower(-p + u);
                    drive.rightFront.setPower(+p + u);
                };
        // Localizer already updated in setPower; read lateral velocity without a second update
        // when possible. identify() calls setPower then wheelVel each loop — one update in
        // setPower is enough; wheelVel re-reads pose velocity from the last update by calling
        // update() again (Pinpoint is cheap / consistent).
        DoubleSupplier wheelVel = () -> drive.localizer.update().linearVel.y * lateralMultiplier;
        DoubleSupplier axisPos = () -> drive.localizer.getPose().position.y;

        // identify() advances samples via nextFrame under MANUAL bulk.
        ReversalFeedforwardId.Result fit =
                ReversalFeedforwardId.identify(
                        this,
                        telemetry,
                        setPower,
                        wheelVel,
                        axisPos,
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
                        POS_STALL_SPEED,
                        sampleLog);

        boolean wroteRampOnly = false;
        if (!fit.singular) {
            MecanumDrive.PARAMS.lateralKS = fit.kS;
            MecanumDrive.PARAMS.lateralKV = fit.kV;
            MecanumDrive.PARAMS.lateralKA = fit.kA;
            MecanumDrive.PARAMS.useAnisotropicFeedforward = true;
        } else if (fit.rampSamples > 0
                && Double.isFinite(fit.kS)
                && fit.kS != 0
                && Double.isFinite(fit.kV)
                && fit.kV > 0) {
            MecanumDrive.PARAMS.lateralKS = fit.kS;
            MecanumDrive.PARAMS.lateralKV = fit.kV;
            MecanumDrive.PARAMS.useAnisotropicFeedforward = true;
            wroteRampOnly = true;
        }

        closeLogs(sampleLog, metaLog);

        while (nextFrame()) {
            if (fit.singular) {
                telemetry.addLine("Fit failed: " + fit.message);
                telemetry.addData("samples", fit.samples);
                telemetry.addData("ramp samples used", fit.rampSamples);
                // Ramp kS/kV may still be usable when only the reverse/kA phase failed.
                if (wroteRampOnly) {
                    telemetry.addLine(
                            "Ramp-only lateralKS/KV written to live PARAMS (kA not updated):");
                    telemetry.addData("useAnisotropicFeedforward", true);
                    telemetry.addData("lateralKS", "%.5f", fit.kS);
                    telemetry.addData("lateralKV", "%.5e", fit.kV);
                    telemetry.addData("ramp R^2", "%.3f", fit.rampR2);
                    telemetry.addLine("Still paste into source to keep after restart.");
                } else if (fit.rampSamples > 0 && Double.isFinite(fit.kS) && fit.kS != 0) {
                    telemetry.addLine("Ramp-only (kA not trusted; not written — kV unusable):");
                    telemetry.addData("lateralKS", "%.5f", fit.kS);
                    telemetry.addData("lateralKV", "%.5e", fit.kV);
                    telemetry.addData("ramp R^2", "%.3f", fit.rampR2);
                }
            } else {
                telemetry.addLine("=== Written to live MecanumDrive.PARAMS ===");
                telemetry.addData("useAnisotropicFeedforward", true);
                telemetry.addData("lateralKS", "%.5f", fit.kS);
                telemetry.addData("lateralKV", "%.5e", fit.kV);
                telemetry.addData("lateralKA", "%.5e", fit.kA);
                telemetry.addLine();
                telemetry.addData("samples", fit.samples);
                telemetry.addData("ramp samples used", fit.rampSamples);
                telemetry.addData("ramp R^2", "%.3f", fit.rampR2);
                telemetry.addLine(
                        "Live for later OpModes this session. Paste into source to keep.");
                telemetry.addLine("Expect lateralKS ≫ axial kS; kV/kA often only modestly higher.");
            }
            if (LOG_CSV) {
                telemetry.addLine("CSVs on hub under /sdcard/FIRST/ — pull with telemetry/pull.sh");
            }
        }
    }

    private static void closeLogs(CsvLogger sampleLog, CsvLogger metaLog) {
        if (sampleLog != null) {
            sampleLog.close();
        }
        if (metaLog != null) {
            metaLog.close();
        }
    }
}
