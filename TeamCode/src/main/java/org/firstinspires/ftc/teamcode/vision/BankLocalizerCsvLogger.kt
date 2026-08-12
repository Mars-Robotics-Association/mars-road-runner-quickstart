package org.firstinspires.ftc.teamcode.vision

import com.acmerobotics.roadrunner.Pose2d
import org.firstinspires.ftc.teamcode.utils.CsvLogger
import org.marsroboticsassociation.controllib.localization.vision.HypothesisBankLocalizer
import org.marsroboticsassociation.controllib.localization.vision.PoseHypothesisBank
import org.marsroboticsassociation.controllib.localization.vision.VisionFrame
import org.marsroboticsassociation.controllib.localization.vision.VisionFrameCsv
import org.marsroboticsassociation.controllib.localization.vision.VisionSource
import java.util.LinkedHashMap

/**
 * Full-loop-rate CSV logger for the multi-hypothesis bank localizer, factored out of the demo
 * opmode so any opmode running a [HypothesisBankRoadRunnerLocalizer] can drop in the same
 * log. It writes the **replay schema**: enough per-loop raw input to re-run the whole
 * PnP → solver → bank chain off-robot (see `BankReplay` + `CsvVisionSource` in ControlLib's
 * tests) and, from the logged corners + intrinsics, to re-solve the PnP with a different method.
 * Buffered and flushed in bursts (see [CsvLogger]) so disk I/O never stalls the control loop.
 *
 * Each row is three blocks: the logger's own **odometry + timing + motion** columns, the
 * [VisionFrameCsv] **frame block** (the raw vision inputs — the reconstruction contract shared
 * with the replay source), and the **outputs** (fused pose + bank state) for comparison. It is a
 * deliberately trimmed subset of the source project's schema: it drops the EKF/gating, MT2, IMU,
 * and label columns, but keeps every raw input the chain — or a re-solve — consumes.
 */
class BankLocalizerCsvLogger(filePrefix: String) : AutoCloseable {

    private val csv: CsvLogger
    private val startNanos = System.nanoTime()
    private var prevLoopNanos = startNanos
    private var prevFrameTs = Double.NaN // last logged vision timestamp, for the fresh-frame flag

    init {
        // filePrefix: prefix for the run-unique file under FIRST/ (e.g. "bank_localizer" →
        // bank_localizer_20260711_141233.csv)
        csv = CsvLogger(CsvLogger.timestamped(filePrefix), HEADER)
    }

    /** The file being written, e.g. to surface in telemetry ("logging to ..."). */
    fun fileName(): String = csv.fileName()

    /**
     * Buffer one full-rate row from the current state of [loc]. Timing (`t_ms`, `loop_ms`) is
     * measured from this logger's construction; flushing is automatic.
     */
    fun log(loc: HypothesisBankRoadRunnerLocalizer) {
        val nowNanos = System.nanoTime()
        val tMs = (nowNanos - startNanos) / 1e6
        val loopMs = (nowNanos - prevLoopNanos) / 1e6
        prevLoopNanos = nowNanos

        val odo: Pose2d = loc.getOdometryPose()
        val fused: Pose2d = loc.getPose()

        val core: HypothesisBankLocalizer = loc.core()
        val bank: PoseHypothesisBank = core.bank()
        val frame: VisionFrame = core.lastFrame() ?: VisionFrame()
        val src: VisionSource = loc.visionSource()
        val cam = doubleArrayOf(src.calFx, src.calFy, src.calCx, src.calCy)

        // Frame block (the reconstruction contract). Override vis_newFrame with the fresh-frame
        // flag: valid AND a new camera timestamp this loop, so replay dedups exactly as the robot
        // does (a valid-but-cached frame is logged as not-new and the replay skips re-processing
        // it).
        val fm: LinkedHashMap<String, Double> =
            VisionFrameCsv.toMap(frame, cam, src.calDistCoeffs)
        val fresh = frame.valid && frame.timestamp != prevFrameTs
        fm["vis_newFrame"] = if (fresh) 1.0 else 0.0
        if (frame.valid) {
            prevFrameTs = frame.timestamp
        }

        val lead =
            arrayOf<Any>(
                tMs,
                loopMs,
                odo.position.x,
                odo.position.y,
                Math.toDegrees(odo.heading.toDouble()),
                Math.toDegrees(core.lastYawRateRadPerSec()),
            )
        val tail =
            arrayOf<Any>(
                fused.position.x,
                fused.position.y,
                Math.toDegrees(fused.heading.toDouble()),
                if (loc.isCommitted()) 1 else 0,
                loc.dominantWeight(),
                bank.size(),
                bank.lastResidPosIn(),
                bank.lastResidHeadDeg(),
                bank.lastSpanPx(),
            )

        val row = arrayOfNulls<Any>(lead.size + fm.size + tail.size)
        var k = 0
        for (o in lead) {
            row[k++] = o
        }
        for (v in fm.values) {
            row[k++] = v
        }
        for (o in tail) {
            row[k++] = o
        }
        csv.row(*row)

        if (csv.bufferedRows() > FLUSH_THRESHOLD) {
            csv.flush()
        }
    }

    /** Final flush of any buffered rows. */
    override fun close() {
        csv.close()
    }

    companion object {
        private const val LEAD = "t_ms,loop_ms,odo_x,odo_y,odo_headingDeg,ang_vel_deg_s"
        private const val TAIL =
            "fused_x,fused_y,fused_headingDeg," +
                "committed,dom_weight,bank_size,bank_residPosIn,bank_residHeadDeg,span_px"

        /** The CSV schema: odometry/timing/motion, the raw frame block, then the fused/bank outputs. */
        @JvmField
        val HEADER: String = LEAD + "," + VisionFrameCsv.HEADER + "," + TAIL

        /** Rows past this buffered count trigger a flush, amortizing disk I/O across the loop. */
        private const val FLUSH_THRESHOLD = 256
    }
}
