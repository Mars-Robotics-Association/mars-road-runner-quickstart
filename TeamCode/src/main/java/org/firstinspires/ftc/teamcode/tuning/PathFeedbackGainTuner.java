package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.TwoDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.opmodes.base.MarsLinearOpMode;
import org.firstinspires.ftc.teamcode.utils.CsvLogger;
import org.firstinspires.ftc.teamcode.utils.GatedTelemetry;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * ManualFeedback-style automatic gain search: long line paths, discrete position-gain ladder, back
 * off when chatter appears.
 *
 * <p>Does what you do by eye with {@link ManualFeedbackTuner}, without short profiled bumps that
 * happily climb to unusable gains of 10–20:
 *
 * <ol>
 *   <li><b>Axial</b> — {@link #DISTANCE} in forward/back along field X (heading 0) with a modest
 *       heading hold. Ladder {@code axialGain}. Targets are absolute field anchors from the start
 *       pose so overshoot cannot walk the out-and-back into the near wall.
 *   <li><b>Heading</b> — same path, axial locked at its best. Ladder {@code headingGain}.
 *   <li><b>Lateral</b> — return near field home, turn to {@link #LATERAL_HEADING_RAD} (default
 *       90°), then out-and-back on the same field-X strip via {@code strafeToConstantHeading} (not
 *       {@code lineToX} — RR lineToX follows path tangent, so at 90° it drives field ±Y = robot
 *       reverse). Ladder {@code lateralGain}.
 * </ol>
 *
 * <p>Reusing the field-X corridor after a 90° turn avoids needing a second long strip of floor:
 * strafe-to keeps heading while the position target moves along field X.
 *
 * <p>Velocity gains stay at {@link #VEL_GAIN} (default 0). Chatter is scored on the command channel
 * of the axis under test only.
 *
 * <p>Prerequisites: localization and feedforward tuned (including lateral FF if anisotropic). Clear
 * a straight corridor of about {@code DISTANCE + 12} inches and room to spin in place at the start.
 * Production feedforward path via {@link MecanumDrive#setDriveCommand}.
 *
 * <p>On finish, writes gains into live {@code MecanumDrive.PARAMS}, then drives back to the OpMode
 * start pose and heading so the next run can begin without re-staging. Optional CSVs: {@code
 * path_fbgain_samples_*} / {@code path_fbgain_summary_*}. Pull with {@code telemetry/pull.sh}.
 */
@Config
public final class PathFeedbackGainTuner extends MarsLinearOpMode {
    /** When false, skip writing CSVs. */
    public static boolean LOG_CSV = true;

    public static int LOG_FLUSH_EVERY = 256;

    /** Out-and-back distance, inches — same default as {@link ManualFeedbackTuner#DISTANCE}. */
    public static double DISTANCE = 64.0;

    /** Complete out-and-back cycles at each gain step before deciding clean vs chatter. */
    public static int CYCLES = 2;

    /** First position gain tried for the axis under test. */
    public static double START_GAIN = 2.0;

    /**
     * Position gain ceiling (inclusive). Field experience on this chassis: axial ~4–5 is the quiet
     * sweet spot before stop-then-lunge; audible thrash starts around ~10. Ceiling is above that so
     * the ladder can find the edge and back off.
     */
    public static double MAX_GAIN = 12.0;

    /** Position gain increment after a clean step. */
    public static double GAIN_STEP = 1.0;

    /**
     * Heading position gain while the axial ladder runs. Soft enough not to thrash, strong enough
     * to keep the line from curling. 2 was too weak on this chassis (line wanders); 4 matches the
     * production default. Replaced by the heading-phase result afterward.
     */
    public static double HOLD_HEADING_GAIN = 4.0;

    /**
     * First heading gain tried in the heading phase. Higher than {@link #START_GAIN}: 2 felt limp
     * for yaw hold on long paths, and a false soft-edge at 2 used to strand the ladder there.
     */
    public static double HEADING_START_GAIN = 3.0;

    /**
     * When true, after axial+heading: home, turn to {@link #LATERAL_HEADING_RAD}, and ladder
     * lateralGain on the same field-X corridor (constant heading = pure strafe).
     */
    public static boolean TUNE_LATERAL = true;

    /**
     * Field heading (rad) for the lateral phase. Default {@code π/2}: robot faces +Y while
     * lineToXConstantHeading still travels field X → robot-frame lateral motion on the same strip.
     */
    public static double LATERAL_HEADING_RAD = Math.PI / 2.0;

    /** Velocity gains for all axes (default 0 — D amplifies Pinpoint noise on long paths). */
    public static double VEL_GAIN = 0.0;

    /**
     * Absolute axial command sign flips (low-speed window only) over a full gain step before hard
     * reject. Quiet gains (2–8) should stay near 0 flips; real thrash near ~10 produces many. Raw
     * command "jerk" is logged only — profile decel makes it huge even when quiet.
     */
    public static int MAX_AXIAL_CMD_FLIPS = 16;

    public static int MAX_LATERAL_CMD_FLIPS = 18;

    public static int MAX_HEADING_CMD_FLIPS = 16;

    /**
     * Soft edge: flip count at or above this fraction of the hard max stops the ladder (even
     * without a hard reject). The edge gain itself is <em>not</em> kept — we back off to the last
     * fully clean step so companion phases (heading/lateral) are not poisoned by a stop-then-lunge
     * axial. Kept high so mild noise at 3–5 does not freeze the ladder early.
     */
    public static double EDGE_FRAC = 0.75;

    /** |cmd| below this does not count toward axial/lateral command sign flips (in/s). */
    public static double TRANS_CMD_FLIP_DEADBAND = 3.0;

    /** |cmd| below this does not count toward heading command sign flips (rad/s). */
    public static double HEADING_CMD_FLIP_DEADBAND = 0.25;

    /**
     * Only score command chatter while the trajectory reference speed is below this (in/s). Cruise
     * and planned accel/decel are ignored; thrash shows up when the profile has nearly stopped.
     */
    public static double CHATTER_REF_SPEED = 6.0;

    /** Same idea for reference angular rate (rad/s) when scoring heading chatter. */
    public static double CHATTER_REF_OMEGA = 0.35;

    /**
     * Terminal hesitation: while still short of the path end, axis speed falls below {@link
     * #HESITATION_V_SLOW} then rises above {@link #HESITATION_V_FAST} again (progress in [{@link
     * #HESITATION_PROGRESS_MIN}, {@link #HESITATION_PROGRESS_MAX}]). That is the high-gain "brake,
     * stop short, then lunge" before reverse (axial ≥6 on this chassis; by-eye sweet spot ~4–5).
     *
     * <p>Arming only when remaining path distance ≥ {@link #HESITATION_REMAINING_MIN} avoids
     * counting low-gain overshoot recovery (past the target) as hesitation. Soft-edge backs off to
     * the last fully clean gain; hard-reject at this many events.
     */
    public static int MAX_HESITATIONS_REJECT = 3;

    /**
     * Soft-edge when hesitation count reaches this. Ladder stops and keeps the <em>previous</em>
     * fully clean gain (not this edge step). 2 allows a single mild re-lunge at the sweet spot (~5)
     * while still rejecting clear stop-then-go at ~6+.
     */
    public static int MAX_HESITATIONS_EDGE = 2;

    /** Fraction of trajectory duration treated as late-path for hesitation scoring. */
    public static double HESITATION_PROGRESS_MIN = 0.55;

    /**
     * Include late terminal re-lunge (stop short then catch residual error before reverse). Was
     * 0.90, which missed re-accels that only peak after ~92% of the leg.
     */
    public static double HESITATION_PROGRESS_MAX = 0.95;

    /** |axis velocity| below this (in/s or rad/s) counts as a near-stop for hesitation. */
    public static double HESITATION_V_SLOW = 5.0;

    /**
     * |axis velocity| above this after a near-stop counts as a re-accel hesitation. 12 was too high
     * — mild stop-then-go at axial ~6 only re-accelerates to ~8–10 in/s and looked clean to the
     * counter while still feeling wrong by eye.
     */
    public static double HESITATION_V_FAST = 8.0;

    /**
     * Only arm a hesitation near-stop when this much path distance remains to the traj endpoint
     * (inches). Positive remaining = still short of the end along the path direction. Filters out
     * low-gain overshoot recovery past the target.
     */
    public static double HESITATION_REMAINING_MIN = 1.5;

    /** Heading uses smaller velocity thresholds (rad/s). */
    public static double HESITATION_V_SLOW_HEADING = 0.15;

    public static double HESITATION_V_FAST_HEADING = 0.35;

    /** Heading remaining (rad) required to arm a heading hesitation near-stop. */
    public static double HESITATION_REMAINING_MIN_HEADING = 0.05;

    /**
     * How often to refresh DS / Dashboard status during long {@code nextSample()} path loops (ms).
     * Those loops close gated telemetry by default, which freezes the last pre-step line for the
     * whole out-and-back (~10 s per gain) unless we open and flush periodically.
     */
    public static int STATUS_MS = 250;

    private enum Axis {
        AXIAL,
        LATERAL,
        HEADING
    }

    /** Path orientation for out-and-back cycles. */
    private enum PathMode {
        /** Heading ~0, lineToX — robot axial along field X. */
        AXIAL_LINE,
        /**
         * Heading ~{@link #LATERAL_HEADING_RAD}, strafeToConstantHeading along field X — robot
         * lateral on the same corridor.
         */
        LATERAL_LINE
    }

    private static final String SAMPLE_HEADER =
            "t_s,axis,gain,cycle,leg,pose_x,pose_y,pose_h,ref_x,ref_y,ref_h,"
                    + "err_x,err_y,err_h,vel_x,vel_y,vel_h,cmd_vx,cmd_vy,cmd_w,"
                    + "ax_flips,lat_flips,h_flips";

    private static final String SUMMARY_HEADER =
            "axis,gain,cycles,duration_s,ax_jerk,lat_jerk,h_jerk,"
                    + "ax_flips_per_s,lat_flips_per_s,h_flips_per_s,"
                    + "err_x_rms,err_y_rms,err_h_rms,hesitations,chatter,edge,verdict";

    private Telemetry telem;
    private CsvLogger sampleLog;
    private CsvLogger summaryLog;
    private String logAxis = "";
    private double logGain;
    private int logCycle;
    private String logLeg = "";

    /** Best clean gain for the active axis (for status lines during the long path). */
    private double logBestSoFar = Double.NaN;

    private long lastStatusNs;

    /** Field pose at OpMode start — absolute out-and-back anchors (not relative to drift). */
    private Pose2d fieldHome = new Pose2d(0, 0, 0);

    private static final class AxisResult {
        final boolean ok;
        final double posGain;
        final String note;

        AxisResult(boolean ok, double posGain, String note) {
            this.ok = ok;
            this.posGain = posGain;
            this.note = note;
        }
    }

    /** Metrics accumulated over one or more trajectory legs at a fixed gain. */
    private static final class PathMetrics {
        final Axis axisUnderTest;

        int samples;
        double durationS;

        /** Time spent in the low-ref-speed chatter window (for flip rates / diagnostics). */
        double chatterWindowS;

        int axialCmdFlips;
        int lateralCmdFlips;
        int headingCmdFlips;
        int lastAxSign;
        int lastLatSign;
        int lastHSign;
        // Diagnostic only (not used for reject): second-diff of command during chatter window.
        double axialJerkSum;
        double lateralJerkSum;
        double headingJerkSum;
        int jerkSamples;
        double errXSqSum;
        double errYSqSum;
        double errHSqSum;
        double prevCmdX = Double.NaN;
        double prevCmdY = Double.NaN;
        double prevCmdH = Double.NaN;
        double prevDCmdX = Double.NaN;
        double prevDCmdY = Double.NaN;
        double prevDCmdH = Double.NaN;

        /** Mid-path stop-then-go events on the axis under test (high gain fighting the profile). */
        int hesitations;

        /** 0 = cruising, 1 = saw near-stop mid-path (waiting for re-accel). */
        int hesitationState;

        PathMetrics(Axis axisUnderTest) {
            this.axisUnderTest = axisUnderTest;
        }

        /**
         * Always counts sample time / tracking RMS and late-path hesitations. Command sign flips
         * only when {@code scoreChatter} — low reference speed (see {@link #CHATTER_REF_SPEED}).
         *
         * @param remainingAlongPath distance still short of the traj end along the path (inches for
         *     translation axes, rad for heading). Near-stops past the target (negative remaining)
         *     do not arm hesitation.
         */
        void observe(
                double dt,
                double errX,
                double errY,
                double errH,
                double cmdX,
                double cmdY,
                double cmdH,
                double axisVel,
                double progress01,
                double remainingAlongPath,
                boolean scoreChatter) {
            samples++;
            durationS += Math.max(dt, 0);
            errXSqSum += errX * errX;
            errYSqSum += errY * errY;
            errHSqSum += errH * errH;

            // Late-path stop-short-then-lunge (independent of low-speed chatter window).
            if (progress01 >= HESITATION_PROGRESS_MIN && progress01 <= HESITATION_PROGRESS_MAX) {
                double vSlow =
                        axisUnderTest == Axis.HEADING
                                ? HESITATION_V_SLOW_HEADING
                                : HESITATION_V_SLOW;
                double vFast =
                        axisUnderTest == Axis.HEADING
                                ? HESITATION_V_FAST_HEADING
                                : HESITATION_V_FAST;
                double remMin =
                        axisUnderTest == Axis.HEADING
                                ? HESITATION_REMAINING_MIN_HEADING
                                : HESITATION_REMAINING_MIN;
                double av = Math.abs(axisVel);
                if (hesitationState == 0 && av < vSlow && remainingAlongPath >= remMin) {
                    hesitationState = 1;
                } else if (hesitationState == 1 && av > vFast) {
                    hesitations++;
                    hesitationState = 0;
                }
            } else {
                hesitationState = 0;
            }

            if (!scoreChatter) {
                // Break derivative chain across cruise → low-speed so planned decel is not thrash.
                prevCmdX = Double.NaN;
                prevCmdY = Double.NaN;
                prevCmdH = Double.NaN;
                prevDCmdX = Double.NaN;
                prevDCmdY = Double.NaN;
                prevDCmdH = Double.NaN;
                lastAxSign = 0;
                lastLatSign = 0;
                lastHSign = 0;
                return;
            }

            chatterWindowS += Math.max(dt, 0);

            int axSign = signWithDeadband(cmdX, TRANS_CMD_FLIP_DEADBAND);
            if (axSign != 0) {
                if (lastAxSign != 0 && axSign != lastAxSign) axialCmdFlips++;
                lastAxSign = axSign;
            }
            int latSign = signWithDeadband(cmdY, TRANS_CMD_FLIP_DEADBAND);
            if (latSign != 0) {
                if (lastLatSign != 0 && latSign != lastLatSign) lateralCmdFlips++;
                lastLatSign = latSign;
            }
            int hSign = signWithDeadband(cmdH, HEADING_CMD_FLIP_DEADBAND);
            if (hSign != 0) {
                if (lastHSign != 0 && hSign != lastHSign) headingCmdFlips++;
                lastHSign = hSign;
            }

            // Diagnostic jerk only (not used for reject).
            if (!Double.isNaN(prevCmdX) && dt > 1e-4) {
                double dX = (cmdX - prevCmdX) / dt;
                double dY = (cmdY - prevCmdY) / dt;
                double dH = (cmdH - prevCmdH) / dt;
                if (!Double.isNaN(prevDCmdX)) {
                    axialJerkSum += Math.abs((dX - prevDCmdX) / dt);
                    lateralJerkSum += Math.abs((dY - prevDCmdY) / dt);
                    headingJerkSum += Math.abs((dH - prevDCmdH) / dt);
                    jerkSamples++;
                }
                prevDCmdX = dX;
                prevDCmdY = dY;
                prevDCmdH = dH;
            }
            prevCmdX = cmdX;
            prevCmdY = cmdY;
            prevCmdH = cmdH;
        }

        private static int signWithDeadband(double v, double db) {
            return v > db ? 1 : (v < -db ? -1 : 0);
        }

        double axialJerk() {
            return jerkSamples == 0 ? 0 : axialJerkSum / jerkSamples;
        }

        double lateralJerk() {
            return jerkSamples == 0 ? 0 : lateralJerkSum / jerkSamples;
        }

        double headingJerk() {
            return jerkSamples == 0 ? 0 : headingJerkSum / jerkSamples;
        }

        double axialFlipsPerSec() {
            return chatterWindowS < 1e-3 ? 0 : axialCmdFlips / chatterWindowS;
        }

        double lateralFlipsPerSec() {
            return chatterWindowS < 1e-3 ? 0 : lateralCmdFlips / chatterWindowS;
        }

        double headingFlipsPerSec() {
            return chatterWindowS < 1e-3 ? 0 : headingCmdFlips / chatterWindowS;
        }

        double errXRms() {
            return samples == 0 ? 0 : Math.sqrt(errXSqSum / samples);
        }

        double errYRms() {
            return samples == 0 ? 0 : Math.sqrt(errYSqSum / samples);
        }

        double errHRms() {
            return samples == 0 ? 0 : Math.sqrt(errHSqSum / samples);
        }

        int activeFlips(Axis axis) {
            switch (axis) {
                case AXIAL:
                    return axialCmdFlips;
                case LATERAL:
                    return lateralCmdFlips;
                case HEADING:
                default:
                    return headingCmdFlips;
            }
        }

        int maxFlips(Axis axis) {
            switch (axis) {
                case AXIAL:
                    return MAX_AXIAL_CMD_FLIPS;
                case LATERAL:
                    return MAX_LATERAL_CMD_FLIPS;
                case HEADING:
                default:
                    return MAX_HEADING_CMD_FLIPS;
            }
        }

        /**
         * Hard reject: too many low-speed command flips, or repeated mid-path stop-then-go
         * (high-gain profile fighting).
         */
        boolean chatter(Axis axis) {
            return activeFlips(axis) > maxFlips(axis) || hesitations >= MAX_HESITATIONS_REJECT;
        }

        boolean edge(Axis axis) {
            return activeFlips(axis) >= (int) Math.ceil(EDGE_FRAC * maxFlips(axis))
                    || hesitations >= MAX_HESITATIONS_EDGE;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        telem = telemetry;

        if (!TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            telem.addLine("PathFeedbackGainTuner is mecanum-only (holonomic path gains).");
            telem.addLine("Use ManualFeedbackTuner for tank.");
            telem.update();
            waitForStart();
            return;
        }

        MecanumDrive drive =
                MecanumDrive.forMarsLinear(hardwareMap, new Pose2d(0, 0, 0), this::batteryVoltage);
        checkDeadWheels(drive.localizer);
        requirePositive(MecanumDrive.PARAMS.kV, MecanumDrive.PARAMS.kA);

        openLogs();

        telem.addLine("Path feedback-gain tuner (ManualFeedback-style).");
        telem.addLine(
                String.format(
                        Locale.US,
                        "Corridor: ~%.0f in on field X. %d cycle(s) per gain.",
                        DISTANCE + 12.0,
                        CYCLES));
        telem.addLine(
                String.format(
                        Locale.US,
                        "1) axial  2) heading  3) home+turn %.0f° + lateral (TUNE_LATERAL=%s)",
                        Math.toDegrees(LATERAL_HEADING_RAD),
                        TUNE_LATERAL));
        telem.addLine(
                String.format(
                        Locale.US,
                        "ladder %.0f..%.0f step %.0f, vel=%.1f — chatter → keep last clean",
                        START_GAIN,
                        MAX_GAIN,
                        GAIN_STEP,
                        VEL_GAIN));
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

        drive.updatePoseEstimate();
        fieldHome = drive.localizer.getPose();

        MecanumDrive.PARAMS.axialVelGain = VEL_GAIN;
        MecanumDrive.PARAMS.lateralVelGain = VEL_GAIN;
        MecanumDrive.PARAMS.headingVelGain = VEL_GAIN;

        // --- Phase 1: axial (lateral=0, heading hold) ---
        AxisResult axial =
                searchAxis(
                        drive,
                        Axis.AXIAL,
                        PathMode.AXIAL_LINE,
                        /* axial */ Double.NaN,
                        /* lateral */ 0.0,
                        /* heading */ HOLD_HEADING_GAIN);
        if (!opModeIsActive()) {
            finish(drive, axial, skipped("skipped"), skipped("skipped"));
            return;
        }

        double axialBest = axial.ok ? axial.posGain : START_GAIN;
        MecanumDrive.PARAMS.axialGain = axialBest;
        MecanumDrive.PARAMS.lateralGain = 0.0;

        // --- Phase 2: heading (axial locked, lateral=0) ---
        AxisResult heading =
                searchAxis(
                        drive,
                        Axis.HEADING,
                        PathMode.AXIAL_LINE,
                        axialBest,
                        0.0,
                        /* heading under test */ Double.NaN);
        if (!opModeIsActive()) {
            finish(drive, axial, skipped("skipped"), heading);
            return;
        }

        double headingBest = heading.ok ? heading.posGain : HOLD_HEADING_GAIN;
        MecanumDrive.PARAMS.headingGain = headingBest;

        // --- Phase 3: lateral on same corridor after home + 90° turn ---
        AxisResult lateral;
        if (TUNE_LATERAL && opModeIsActive()) {
            telem.addLine("Reorienting: return home, turn for lateral corridor…");
            telem.update();
            if (!reorientForLateral(drive, fieldHome)) {
                lateral = new AxisResult(false, 0, "FAILED: reorient home/turn aborted");
            } else {
                lateral =
                        searchAxis(
                                drive,
                                Axis.LATERAL,
                                PathMode.LATERAL_LINE,
                                axialBest,
                                /* lateral under test */ Double.NaN,
                                headingBest);
            }
        } else {
            lateral = new AxisResult(true, 0.0, "skipped (TUNE_LATERAL=false)");
        }

        finish(drive, axial, lateral, heading);
    }

    private static AxisResult skipped(String note) {
        return new AxisResult(false, 0, note);
    }

    private void finish(
            MecanumDrive drive, AxisResult axial, AxisResult lateral, AxisResult heading) {
        // Apply best gains first so the return-home path uses them.
        MecanumDrive.PARAMS.axialGain = axial.ok ? axial.posGain : START_GAIN;
        MecanumDrive.PARAMS.lateralGain = lateral.ok ? lateral.posGain : 0.0;
        MecanumDrive.PARAMS.headingGain = heading.ok ? heading.posGain : HOLD_HEADING_GAIN;
        MecanumDrive.PARAMS.axialVelGain = VEL_GAIN;
        MecanumDrive.PARAMS.lateralVelGain = VEL_GAIN;
        MecanumDrive.PARAMS.headingVelGain = VEL_GAIN;

        // Park at the OpMode start pose/heading so the next run can begin without re-staging.
        if (opModeIsActive()) {
            telem.addLine("Returning to start pose / heading…");
            telem.update();
            returnToStart(drive, fieldHome);
        }

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        closeLogs();

        while (nextFrame()) {
            telem.addLine("=== Written to live MecanumDrive.PARAMS ===");
            telem.addData("axialGain", "%.2f", MecanumDrive.PARAMS.axialGain);
            telem.addData("lateralGain", "%.2f", MecanumDrive.PARAMS.lateralGain);
            telem.addData("headingGain", "%.2f", MecanumDrive.PARAMS.headingGain);
            telem.addData("axialVelGain", "%.2f", MecanumDrive.PARAMS.axialVelGain);
            telem.addData("lateralVelGain", "%.2f", MecanumDrive.PARAMS.lateralVelGain);
            telem.addData("headingVelGain", "%.2f", MecanumDrive.PARAMS.headingVelGain);
            telem.addLine();
            telem.addData("axial", axial.note);
            telem.addData("lateral", lateral.note);
            telem.addData("heading", heading.note);
            if (LOG_CSV) {
                telem.addLine("CSVs on hub under /sdcard/FIRST/ — pull with telemetry/pull.sh");
            }
            telem.addLine("Live for later OpModes this session. Paste into source to keep.");
            telem.addLine("Robot should be back at start pose — ready to re-run.");
            telem.addLine("Verify with ManualFeedbackTuner / SplineTest.");
        }
    }

    /**
     * Drive back near field home (constant heading), then turn to {@link #LATERAL_HEADING_RAD} so
     * the next strafe legs are pure robot-lateral on the same field-X strip.
     */
    private boolean reorientForLateral(MecanumDrive drive, Pose2d home) {
        return goToPose(drive, home.position.x, home.position.y, LATERAL_HEADING_RAD, "reorient");
    }

    /** Return to the OpMode start pose (field home XY + original heading) after tuning finishes. */
    private boolean returnToStart(MecanumDrive drive, Pose2d home) {
        return goToPose(
                drive, home.position.x, home.position.y, home.heading.toDouble(), "return_home");
    }

    /**
     * Strafe to field (x, y) at constant heading, then {@code turnTo} the requested heading.
     * Unscored — used for reorient and end-of-run park.
     */
    private boolean goToPose(
            MecanumDrive drive, double x, double y, double headingRad, String logTag) {
        drive.updatePoseEstimate();
        Pose2d here = drive.localizer.getPose();
        logAxis = logTag;
        logGain = 0;
        logCycle = 0;
        Action path =
                drive.actionBuilder(here)
                        .strafeToConstantHeading(new Vector2d(x, y))
                        .turnTo(headingRad)
                        .build();
        boolean ok = runActionUnscored(drive, path);
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        return ok && opModeIsActive();
    }

    /**
     * Discrete gain ladder for one axis. Pass {@link Double#NaN} for the slot under test; the other
     * two slots are fixed companion gains for that phase.
     */
    private AxisResult searchAxis(
            MecanumDrive drive,
            Axis axis,
            PathMode pathMode,
            double axialGainOrNaN,
            double lateralGainOrNaN,
            double headingGainOrNaN) {
        double bestGain = Double.NaN;
        int bestHes = 0;
        // Highest clean step with zero hesitations — preferred when the top clean still had mild
        // hes (e.g. heading 7 clean-with-hes=1, reject at 8 → keep 6, not 7).
        double bestZeroHesGain = Double.NaN;
        String note = "not tuned";
        logAxis = axis.name().toLowerCase(Locale.US);

        double startGain = axis == Axis.HEADING ? HEADING_START_GAIN : START_GAIN;
        for (double gain = startGain;
                gain <= MAX_GAIN + 1e-9 && opModeIsActive();
                gain += GAIN_STEP) {
            logGain = gain;
            logBestSoFar = bestGain;
            applyGains(axis, gain, axialGainOrNaN, lateralGainOrNaN, headingGainOrNaN);

            // Immediate status before the long nextSample() path (which would otherwise freeze DS).
            lastStatusNs = 0L;
            maybePublishStatus(null, null, Double.NaN);

            PathMetrics metrics = new PathMetrics(axis);
            boolean ok = runGainStep(drive, pathMode, metrics);
            if (!ok || !opModeIsActive()) {
                double picked = quieterBest(bestGain, bestHes, bestZeroHesGain);
                return new AxisResult(
                        !Double.isNaN(picked),
                        Double.isNaN(picked) ? 0 : picked,
                        "aborted (stop or path failure)");
            }

            boolean chatter = metrics.chatter(axis);
            boolean edge = !chatter && metrics.edge(axis);
            String verdict =
                    chatter
                            ? (metrics.hesitations >= MAX_HESITATIONS_REJECT
                                    ? "hesitation"
                                    : "chatter")
                            : edge
                                    ? (metrics.hesitations >= MAX_HESITATIONS_EDGE
                                            ? "edge_hesitation"
                                            : "edge_keep")
                                    : "clean";
            logSummary(axis, gain, metrics, chatter, edge, verdict);

            if (chatter) {
                if (!Double.isNaN(bestGain)) {
                    double picked = quieterBest(bestGain, bestHes, bestZeroHesGain);
                    note =
                            String.format(
                                    Locale.US,
                                    "ok: chatter at %.1f — kept %.1f%s",
                                    gain,
                                    picked,
                                    picked + 1e-9 < bestGain
                                            ? String.format(
                                                    Locale.US,
                                                    " (backed off from last clean %.1f; hes=%d)",
                                                    bestGain,
                                                    bestHes)
                                            : " (last clean)");
                    bestGain = picked;
                } else {
                    double lower = gain - GAIN_STEP;
                    if (lower >= 1.0) {
                        bestGain = lower;
                        note =
                                String.format(
                                        Locale.US,
                                        "ok: chatter at start %.1f — backed off to %.1f (verify by"
                                                + " ear)",
                                        gain,
                                        bestGain);
                    } else {
                        note =
                                String.format(
                                        Locale.US,
                                        "FAILED: chatter at start gain %.1f — check feedforward /"
                                                + " lower START_GAIN",
                                        gain);
                        return new AxisResult(false, 0, note);
                    }
                }
                break;
            }

            // Soft edge: do not keep this gain. The edge step already shows stop-then-lunge or
            // near-chatter; keeping it poisons later phases (heading ladder on an axial path that
            // already hesitates) and picks a gain that feels wrong by eye before reverse.
            if (edge) {
                if (!Double.isNaN(bestGain)) {
                    double picked = quieterBest(bestGain, bestHes, bestZeroHesGain);
                    if (metrics.hesitations >= MAX_HESITATIONS_EDGE) {
                        note =
                                String.format(
                                        Locale.US,
                                        "ok: hesitation edge at %.1f (hes=%d) — kept %.1f%s",
                                        gain,
                                        metrics.hesitations,
                                        picked,
                                        picked + 1e-9 < bestGain
                                                ? String.format(
                                                        Locale.US,
                                                        " (backed off from %.1f; hes=%d)",
                                                        bestGain,
                                                        bestHes)
                                                : " (last clean)");
                    } else {
                        note =
                                String.format(
                                        Locale.US,
                                        "ok: near chatter edge at %.1f — kept %.1f%s",
                                        gain,
                                        picked,
                                        picked + 1e-9 < bestGain
                                                ? String.format(
                                                        Locale.US,
                                                        " (backed off from %.1f; hes=%d)",
                                                        bestGain,
                                                        bestHes)
                                                : " (last clean)");
                    }
                    bestGain = picked;
                } else {
                    // First step is already soft-edge: keep it. Backing off below START would
                    // strand heading/axial at an unusably weak gain (the old "heading=2 feels
                    // limp" failure mode when a false edge fired at the floor).
                    bestGain = gain;
                    note =
                            String.format(
                                    Locale.US,
                                    "ok: edge at start %.1f (hes=%d) — kept (no lower clean step;"
                                            + " verify by eye)",
                                    bestGain,
                                    metrics.hesitations);
                }
                break;
            }

            bestGain = gain;
            bestHes = metrics.hesitations;
            if (metrics.hesitations == 0) {
                bestZeroHesGain = gain;
            }
            note =
                    String.format(
                            Locale.US,
                            "ok: clean at %.1f (flips %d, hes %d)",
                            bestGain,
                            metrics.activeFlips(axis),
                            metrics.hesitations);

            if (gain + GAIN_STEP > MAX_GAIN + 1e-9) {
                double picked = quieterBest(bestGain, bestHes, bestZeroHesGain);
                if (picked + 1e-9 < bestGain) {
                    note =
                            String.format(
                                    Locale.US,
                                    "ok: clean through max — backed off %.1f→%.1f (last zero-hes)",
                                    bestGain,
                                    picked);
                    bestGain = picked;
                } else {
                    note = String.format(Locale.US, "ok: clean through max gain %.1f", bestGain);
                }
            }
        }

        if (Double.isNaN(bestGain)) {
            return new AxisResult(false, 0, note);
        }
        applyGains(axis, bestGain, axialGainOrNaN, lateralGainOrNaN, headingGainOrNaN);
        return new AxisResult(true, bestGain, note);
    }

    /**
     * When the highest clean gain still showed mild hesitation, prefer the last fully quiet
     * (zero-hes) clean step. Matches by-eye "7 looked high — back off to 6" after a hard reject at
     * 8: last clean 7 had hes=1 while 6 was clean with hes=0.
     */
    private static double quieterBest(double bestGain, int bestHes, double bestZeroHesGain) {
        if (Double.isNaN(bestGain)) {
            return bestGain;
        }
        if (bestHes > 0 && !Double.isNaN(bestZeroHesGain) && bestZeroHesGain + 1e-9 < bestGain) {
            return bestZeroHesGain;
        }
        return bestGain;
    }

    /**
     * Writes PARAMS gains for this step. The axis under test gets {@code gain}; NaN companion args
     * mean "this is the axis under test" and are ignored for that slot.
     */
    private static void applyGains(
            Axis axis,
            double gain,
            double axialGainOrNaN,
            double lateralGainOrNaN,
            double headingGainOrNaN) {
        MecanumDrive.PARAMS.axialGain = axis == Axis.AXIAL ? gain : axialGainOrNaN;
        MecanumDrive.PARAMS.lateralGain = axis == Axis.LATERAL ? gain : lateralGainOrNaN;
        MecanumDrive.PARAMS.headingGain = axis == Axis.HEADING ? gain : headingGainOrNaN;
    }

    /**
     * {@link #CYCLES} out-and-back paths in the requested path mode. Endpoints are absolute field
     * anchors from {@link #fieldHome} so relative overshoot cannot march the start pose into a
     * wall.
     */
    private boolean runGainStep(MecanumDrive drive, PathMode pathMode, PathMetrics metrics) {
        final double xNear = fieldHome.position.x;
        final double xFar = fieldHome.position.x + DISTANCE;
        final double yHome = fieldHome.position.y;

        for (int cycle = 1; cycle <= CYCLES && opModeIsActive(); cycle++) {
            logCycle = cycle;
            drive.updatePoseEstimate();
            Pose2d begin = drive.localizer.getPose();

            Action path;
            if (pathMode == PathMode.LATERAL_LINE) {
                // Do NOT use lineToX* here: RR lineToX goes along path tangent until x=target, so
                // with heading ≈ 90° the segment is almost field ±Y (robot reverse), not a strafe.
                // Explicit field-XY endpoints + constant heading = pure robot-lateral on the strip.
                path =
                        drive.actionBuilder(begin)
                                .strafeToConstantHeading(new Vector2d(xFar, yHome))
                                .strafeToConstantHeading(new Vector2d(xNear, yHome))
                                .build();
            } else {
                // Absolute field-X anchors (not begin.x ± DISTANCE).
                path = drive.actionBuilder(begin).lineToX(xFar).lineToX(xNear).build();
            }

            if (!runActionSampling(drive, path, metrics)) {
                return false;
            }
        }
        return opModeIsActive();
    }

    /** Run an action for reorient (turns + short strafe) without chatter scoring. */
    private boolean runActionUnscored(MecanumDrive drive, Action action) {
        for (Action leaf : flatten(action)) {
            if (!opModeIsActive()) return false;
            if (leaf instanceof MecanumDrive.FollowTrajectoryAction) {
                TimeTrajectory traj = ((MecanumDrive.FollowTrajectoryAction) leaf).timeTrajectory;
                if (!followTrajectory(drive, traj, /* metrics */ null)) return false;
            } else if (leaf instanceof MecanumDrive.TurnAction) {
                logLeg = "turn";
                TelemetryPacket packet = new TelemetryPacket();
                while (nextSample() && leaf.run(packet)) {
                    drive.updatePoseEstimate();
                    maybePublishStatus(drive.localizer.getPose(), null, Double.NaN);
                }
                if (isStopRequested()) return false;
            } else {
                TelemetryPacket packet = new TelemetryPacket();
                while (nextSample() && leaf.run(packet)) {
                    drive.updatePoseEstimate();
                    maybePublishStatus(drive.localizer.getPose(), null, Double.NaN);
                }
                if (isStopRequested()) return false;
            }
        }
        return true;
    }

    private boolean runActionSampling(MecanumDrive drive, Action action, PathMetrics metrics) {
        for (Action leaf : flatten(action)) {
            if (!opModeIsActive()) return false;
            if (leaf instanceof MecanumDrive.FollowTrajectoryAction) {
                TimeTrajectory traj = ((MecanumDrive.FollowTrajectoryAction) leaf).timeTrajectory;
                if (!followTrajectory(drive, traj, metrics)) return false;
            } else {
                TelemetryPacket packet = new TelemetryPacket();
                while (nextSample() && leaf.run(packet)) {
                    // stock
                }
                if (isStopRequested()) return false;
            }
        }
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        return true;
    }

    private static List<Action> flatten(Action action) {
        List<Action> out = new ArrayList<>();
        flattenInto(action, out);
        return out;
    }

    private static void flattenInto(Action action, List<Action> out) {
        if (action instanceof SequentialAction) {
            for (Action child : ((SequentialAction) action).getInitialActions()) {
                flattenInto(child, out);
            }
        } else {
            out.add(action);
        }
    }

    /**
     * Same control law as {@link MecanumDrive.FollowTrajectoryAction}, plus optional chatter
     * metrics / CSV. Uses live {@code PARAMS} gains.
     */
    private boolean followTrajectory(MecanumDrive drive, TimeTrajectory traj, PathMetrics metrics) {
        logLeg = "traj";
        ElapsedTime clock = new ElapsedTime();
        double beginTs = -1;
        double lastT = 0;
        double duration = Math.max(traj.duration, 1e-6);
        Pose2d trajBegin = traj.get(0).value();
        Pose2d trajEnd = traj.get(duration).value();
        double pathDx = trajEnd.position.x - trajBegin.position.x;
        double pathDy = trajEnd.position.y - trajBegin.position.y;
        double pathLen = Math.hypot(pathDx, pathDy);

        while (nextSample()) {
            if (beginTs < 0) {
                beginTs = clock.seconds();
            }
            double t = clock.seconds() - beginTs;
            if (t >= traj.duration) {
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                break;
            }

            Pose2dDual<Time> txWorldTarget = traj.get(t);
            PoseVelocity2d robotVel = drive.updatePoseEstimate();
            Pose2d pose = drive.localizer.getPose();

            PoseVelocity2dDual<Time> command =
                    new HolonomicController(
                                    MecanumDrive.PARAMS.axialGain,
                                    MecanumDrive.PARAMS.lateralGain,
                                    MecanumDrive.PARAMS.headingGain,
                                    MecanumDrive.PARAMS.axialVelGain,
                                    MecanumDrive.PARAMS.lateralVelGain,
                                    MecanumDrive.PARAMS.headingVelGain)
                            .compute(txWorldTarget, pose, robotVel);

            drive.setDriveCommand(command);

            Pose2d ref = txWorldTarget.value();
            Pose2d error = ref.minusExp(pose);
            double cmdX = command.value().linearVel.x;
            double cmdY = command.value().linearVel.y;
            double cmdW = command.value().angVel;
            double dt = t - lastT;
            lastT = t;
            // Dual first derivatives = reference velocity (field frame for position).
            double refSpeed =
                    Math.hypot(txWorldTarget.position.x.get(1), txWorldTarget.position.y.get(1));
            // heading.velocity() is already dθ/dt as a DualNum; [0] is the rate value.
            double refOmega = Math.abs(txWorldTarget.heading.velocity().get(0));
            boolean scoreChatter = refSpeed <= CHATTER_REF_SPEED && refOmega <= CHATTER_REF_OMEGA;
            double progress01 = t / duration;
            if (metrics != null && dt > 1e-6) {
                double axisVel =
                        metrics.axisUnderTest == Axis.AXIAL
                                ? robotVel.linearVel.x
                                : metrics.axisUnderTest == Axis.LATERAL
                                        ? robotVel.linearVel.y
                                        : robotVel.angVel;
                // Positive remaining = still short of traj end along the path (or heading).
                double remainingAlongPath;
                if (metrics.axisUnderTest == Axis.HEADING) {
                    // Rotation2d.minus already returns the wrapped log angle (rad).
                    remainingAlongPath = Math.abs(trajEnd.heading.minus(pose.heading));
                } else if (pathLen > 1e-3) {
                    double toEndX = trajEnd.position.x - pose.position.x;
                    double toEndY = trajEnd.position.y - pose.position.y;
                    remainingAlongPath = (toEndX * pathDx + toEndY * pathDy) / pathLen;
                } else {
                    remainingAlongPath = 0;
                }
                metrics.observe(
                        dt,
                        error.position.x,
                        error.position.y,
                        error.heading.log(),
                        cmdX,
                        cmdY,
                        cmdW,
                        axisVel,
                        progress01,
                        remainingAlongPath,
                        scoreChatter);
            }

            maybePublishStatus(pose, ref, t / duration);

            if (sampleLog != null) {
                sampleLog.row(
                        t,
                        logAxis,
                        logGain,
                        logCycle,
                        logLeg,
                        pose.position.x,
                        pose.position.y,
                        pose.heading.toDouble(),
                        ref.position.x,
                        ref.position.y,
                        ref.heading.toDouble(),
                        error.position.x,
                        error.position.y,
                        error.heading.log(),
                        robotVel.linearVel.x,
                        robotVel.linearVel.y,
                        robotVel.angVel,
                        cmdX,
                        cmdY,
                        cmdW,
                        metrics == null ? 0 : metrics.axialCmdFlips,
                        metrics == null ? 0 : metrics.lateralCmdFlips,
                        metrics == null ? 0 : metrics.headingCmdFlips);
                maybeFlushSamples();
            }
        }

        if (sampleLog != null) {
            sampleLog.flush();
        }
        return !isStopRequested();
    }

    /**
     * Refresh DS / Dashboard during {@link #nextSample()} bursts. {@code nextSample} keeps gated
     * telemetry closed, so without this the station freezes on the last line for the whole path.
     */
    private void maybePublishStatus(Pose2d pose, Pose2d ref, double progress01) {
        if (STATUS_MS < 0) {
            return;
        }
        long nowNs = System.nanoTime();
        if (STATUS_MS > 0
                && lastStatusNs != 0L
                && (nowNs - lastStatusNs) < STATUS_MS * 1_000_000L) {
            return;
        }
        lastStatusNs = nowNs;

        GatedTelemetry gated = telem instanceof GatedTelemetry ? (GatedTelemetry) telem : null;
        boolean wasOpen = gated == null || gated.isOpen();
        if (gated != null) {
            gated.setOpen(true);
        }

        telem.addData("phase", logAxis.isEmpty() ? "?" : logAxis);
        telem.addData("trying gain", "%.1f", logGain);
        telem.addData(
                "best so far",
                Double.isNaN(logBestSoFar)
                        ? "none yet"
                        : String.format(Locale.US, "%.1f", logBestSoFar));
        telem.addData("cycle", "%d / %d", logCycle, CYCLES);
        telem.addData("leg", logLeg.isEmpty() ? "-" : logLeg);
        if (!Double.isNaN(progress01)) {
            telem.addData("path %", "%.0f", 100.0 * Math.max(0, Math.min(1, progress01)));
        }
        telem.addData(
                "gains ax/lat/h",
                "%.1f / %.1f / %.1f",
                MecanumDrive.PARAMS.axialGain,
                MecanumDrive.PARAMS.lateralGain,
                MecanumDrive.PARAMS.headingGain);
        if (pose != null) {
            telem.addData(
                    "pose",
                    "x=%.1f y=%.1f h=%.0f°",
                    pose.position.x,
                    pose.position.y,
                    Math.toDegrees(pose.heading.toDouble()));
        }
        if (ref != null) {
            telem.addData(
                    "ref",
                    "x=%.1f y=%.1f h=%.0f°",
                    ref.position.x,
                    ref.position.y,
                    Math.toDegrees(ref.heading.toDouble()));
        }
        if (sampleLog != null) {
            telem.addData("log", sampleLog.fileName());
        }

        if (gated != null) {
            gated.forceUpdate();
            gated.setOpen(wasOpen);
        } else {
            telem.update();
        }
    }

    private void openLogs() {
        if (!LOG_CSV) return;
        String stamp =
                new java.text.SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US)
                        .format(new java.util.Date());
        sampleLog = new CsvLogger("path_fbgain_samples_" + stamp + ".csv", SAMPLE_HEADER);
        summaryLog = new CsvLogger("path_fbgain_summary_" + stamp + ".csv", SUMMARY_HEADER);
        summaryLog.row(
                "meta",
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                false,
                false,
                String.format(
                        Locale.US,
                        "DISTANCE=%.1f;CYCLES=%d;START=%.1f;H_START=%.1f;MAX=%.1f;STEP=%.1f;"
                                + "HOLD_H=%.1f;TUNE_LAT=%s;LAT_HEAD_DEG=%.1f;VEL=%.2f;"
                                + "MAX_AX_FLIPS=%d;MAX_LAT_FLIPS=%d;MAX_H_FLIPS=%d;"
                                + "CHATTER_REF_SPEED=%.1f;MAX_HES_EDGE=%d;MAX_HES_REJ=%d;"
                                + "HES_VFAST=%.1f;HES_PMAX=%.2f;HES_REMAIN=%.1f",
                        DISTANCE,
                        CYCLES,
                        START_GAIN,
                        HEADING_START_GAIN,
                        MAX_GAIN,
                        GAIN_STEP,
                        HOLD_HEADING_GAIN,
                        TUNE_LATERAL,
                        Math.toDegrees(LATERAL_HEADING_RAD),
                        VEL_GAIN,
                        MAX_AXIAL_CMD_FLIPS,
                        MAX_LATERAL_CMD_FLIPS,
                        MAX_HEADING_CMD_FLIPS,
                        CHATTER_REF_SPEED,
                        MAX_HESITATIONS_EDGE,
                        MAX_HESITATIONS_REJECT,
                        HESITATION_V_FAST,
                        HESITATION_PROGRESS_MAX,
                        HESITATION_REMAINING_MIN));
        summaryLog.flush();
    }

    private void logSummary(
            Axis axis, double gain, PathMetrics m, boolean chatter, boolean edge, String verdict) {
        if (summaryLog == null) return;
        summaryLog.row(
                axis.name().toLowerCase(Locale.US),
                gain,
                CYCLES,
                m.durationS,
                m.axialJerk(),
                m.lateralJerk(),
                m.headingJerk(),
                m.axialFlipsPerSec(),
                m.lateralFlipsPerSec(),
                m.headingFlipsPerSec(),
                m.errXRms(),
                m.errYRms(),
                m.errHRms(),
                m.hesitations,
                chatter,
                edge,
                verdict);
        summaryLog.flush();
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

    private static void requirePositive(double kV, double kA) {
        if (kV <= 0 || kA <= 0) {
            throw new RuntimeException(
                    "PathFeedbackGainTuner needs positive kV and kA. Run AxialFeedforwardTuner"
                            + " first.");
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
