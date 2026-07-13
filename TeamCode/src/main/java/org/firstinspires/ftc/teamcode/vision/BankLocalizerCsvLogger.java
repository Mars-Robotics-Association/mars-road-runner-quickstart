package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.utils.CsvLogger;
import org.marsroboticsassociation.controllib.localization.vision.HypothesisBankLocalizer;
import org.marsroboticsassociation.controllib.localization.vision.PoseHypothesisBank;
import org.marsroboticsassociation.controllib.localization.vision.VisionFrame;
import org.marsroboticsassociation.controllib.localization.vision.VisionFrameCsv;
import org.marsroboticsassociation.controllib.localization.vision.VisionSource;

import java.util.LinkedHashMap;

/**
 * Full-loop-rate CSV logger for the multi-hypothesis bank localizer, factored out of the demo
 * opmode so any opmode running a {@link HypothesisBankRoadRunnerLocalizer} can drop in the same
 * log. It writes the <b>replay schema</b>: enough per-loop raw input to re-run the whole
 * PnP&nbsp;&rarr; solver&nbsp;&rarr; bank chain off-robot (see {@code BankReplay} + {@code
 * CsvVisionSource} in ControlLib's tests) and, from the logged corners + intrinsics, to re-solve
 * the PnP with a different method. Buffered and flushed in bursts (see {@link CsvLogger}) so disk
 * I/O never stalls the control loop.
 *
 * <p>Each row is three blocks: the logger's own <b>odometry + timing + motion</b> columns, the
 * {@link VisionFrameCsv} <b>frame block</b> (the raw vision inputs — the reconstruction contract
 * shared with the replay source), and the <b>outputs</b> (fused pose + bank state) for comparison.
 * It is a deliberately trimmed subset of the source project's schema: it drops the EKF/gating, MT2,
 * IMU, and label columns, but keeps every raw input the chain — or a re-solve — consumes.
 */
public final class BankLocalizerCsvLogger implements AutoCloseable {

    private static final String LEAD = "t_ms,loop_ms,odo_x,odo_y,odo_headingDeg,ang_vel_deg_s";
    private static final String TAIL =
            "fused_x,fused_y,fused_headingDeg,"
                    + "committed,dom_weight,bank_size,bank_residPosIn,bank_residHeadDeg,span_px";

    /** The CSV schema: odometry/timing/motion, the raw frame block, then the fused/bank outputs. */
    public static final String HEADER = LEAD + "," + VisionFrameCsv.HEADER + "," + TAIL;

    /** Rows past this buffered count trigger a flush, amortizing disk I/O across the loop. */
    private static final int FLUSH_THRESHOLD = 256;

    private final CsvLogger csv;
    private final long startNanos = System.nanoTime();
    private long prevLoopNanos = startNanos;
    private double prevFrameTs =
            Double.NaN; // last logged vision timestamp, for the fresh-frame flag

    /**
     * @param filePrefix prefix for the run-unique file under {@code FIRST/} (e.g. {@code
     *     "bank_localizer"} → {@code bank_localizer_20260711_141233.csv})
     */
    public BankLocalizerCsvLogger(String filePrefix) {
        csv = new CsvLogger(CsvLogger.timestamped(filePrefix), HEADER);
    }

    /** The file being written, e.g. to surface in telemetry ("logging to ..."). */
    public String fileName() {
        return csv.fileName();
    }

    /**
     * Buffer one full-rate row from the current state of {@code loc}. Timing ({@code t_ms}, {@code
     * loop_ms}) is measured from this logger's construction; flushing is automatic.
     */
    public void log(HypothesisBankRoadRunnerLocalizer loc) {
        long nowNanos = System.nanoTime();
        double tMs = (nowNanos - startNanos) / 1e6;
        double loopMs = (nowNanos - prevLoopNanos) / 1e6;
        prevLoopNanos = nowNanos;

        Pose2d odo = loc.getOdometryPose();
        Pose2d fused = loc.getPose();

        HypothesisBankLocalizer core = loc.core();
        PoseHypothesisBank bank = core.bank();
        VisionFrame frame = core.lastFrame();
        if (frame == null) {
            frame = new VisionFrame();
        }
        VisionSource src = loc.visionSource();
        double[] cam = {src.getCalFx(), src.getCalFy(), src.getCalCx(), src.getCalCy()};

        // Frame block (the reconstruction contract). Override vis_newFrame with the fresh-frame
        // flag:
        // valid AND a new camera timestamp this loop, so replay dedups exactly as the robot does
        // (a valid-but-cached frame is logged as not-new and the replay skips re-processing it).
        LinkedHashMap<String, Double> fm = VisionFrameCsv.toMap(frame, cam, src.getCalDistCoeffs());
        boolean fresh = frame.valid && frame.timestamp != prevFrameTs;
        fm.put("vis_newFrame", fresh ? 1.0 : 0.0);
        if (frame.valid) {
            prevFrameTs = frame.timestamp;
        }

        Object[] lead = {
            tMs,
            loopMs,
            odo.position.x,
            odo.position.y,
            Math.toDegrees(odo.heading.toDouble()),
            Math.toDegrees(core.lastYawRateRadPerSec())
        };
        Object[] tail = {
            fused.position.x,
            fused.position.y,
            Math.toDegrees(fused.heading.toDouble()),
            loc.isCommitted() ? 1 : 0,
            loc.dominantWeight(),
            bank.size(),
            bank.lastResidPosIn(),
            bank.lastResidHeadDeg(),
            bank.lastSpanPx()
        };

        Object[] row = new Object[lead.length + fm.size() + tail.length];
        int k = 0;
        for (Object o : lead) {
            row[k++] = o;
        }
        for (double v : fm.values()) {
            row[k++] = v;
        }
        for (Object o : tail) {
            row[k++] = o;
        }
        csv.row(row);

        if (csv.bufferedRows() > FLUSH_THRESHOLD) {
            csv.flush();
        }
    }

    /** Final flush of any buffered rows. */
    @Override
    public void close() {
        csv.close();
    }
}
