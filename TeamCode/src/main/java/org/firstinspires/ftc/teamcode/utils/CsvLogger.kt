package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.util.RobotLog
import java.io.File
import java.io.FileWriter
import java.io.IOException
import java.util.Date
import java.util.Locale
import org.firstinspires.ftc.robotcore.internal.system.AppUtil

/**
 * Tiny append-only CSV logger for offline-analysis files under the `FIRST/` folder. Rows are
 * buffered in memory and written in batches: a [FileWriter] is opened once per [flush], never once
 * per [row], so high-rate logging from a control loop pays disk I/O in occasional bursts instead of
 * stalling the loop on every sample. Writing is best-effort — an [IOException] is counted
 * ([failureCount]) and logged, never thrown, so a full disk or a bad path can't crash an OpMode
 * mid-match.
 *
 * <p>The header is written lazily and only when the target file does not yet exist. So a fixed
 * filename used across runs accumulates rows under a single header, while a [timestamped] per-run
 * filename gets a fresh file with its own header.
 *
 * <p>Values passed to [row] are stringified with [String.valueOf]; doubles therefore serialize at
 * full round-trip precision (e.g. a `t_ms` timestamp keeps sub-microsecond resolution), and `NaN`
 * is written literally. This is deliberately better for offline analysis than a fixed `%.3f`-style
 * format, which would silently truncate timestamps.
 *
 * <h3>Continuous (per-loop) use</h3>
 * <pre>
 *   CsvLogger log = new CsvLogger(CsvLogger.timestamped("vis_fusion"), "t_ms,odo_x,odo_y");
 *   while (opModeIsActive()) {
 *       ...
 *       log.row(tMs, odoX, odoY);                       // buffered, no disk I/O
 *       if (log.bufferedRows() > 256) log.flush();      // amortize the write
 *   }
 *   log.close();                                        // final flush
 * </pre>
 *
 * <h3>Batch use</h3>
 * <pre>
 *   CsvLogger log = new CsvLogger("encoder_burst.csv", "burst_index,t_ns,position");
 *   for (...) log.row(idx, tNs, position);
 *   log.flush();                                        // one disk write per burst
 * </pre>
 */
class CsvLogger(filename: String, private val header: String) : AutoCloseable {

    private val file: File = File(AppUtil.FIRST_FOLDER, filename)
    private val buffer = StringBuilder()
    private var bufferedRows = 0
    private var failures = 0L

    /** Buffer one row. Columns are comma-joined via [String.valueOf] (full double precision). */
    fun row(vararg cols: Any?) {
        for (i in cols.indices) {
            if (i > 0) buffer.append(',')
            buffer.append(java.lang.String.valueOf(cols[i]))
        }
        buffer.append('\n')
        bufferedRows++
    }

    /** Rows buffered since the last [flush] — poll to decide when to amortize a write. */
    fun bufferedRows(): Int {
        return bufferedRows
    }

    /**
     * Append all buffered rows to disk, writing the header first if the file is new. Best-effort:
     * on an [IOException] the buffer is still cleared (so a transient failure doesn't wedge the
     * logger) and the failure is counted and logged.
     */
    fun flush() {
        if (bufferedRows == 0) {
            return
        }
        val newFile = !file.exists()
        try {
            FileWriter(file, true).use { w ->
                if (newFile) {
                    w.write(header)
                    w.write('\n'.code)
                }
                w.write(buffer.toString())
            }
        } catch (e: IOException) {
            failures++
            RobotLog.ee(TAG, "CSV write failed (" + file.name + "): " + e.message)
        }
        buffer.setLength(0)
        bufferedRows = 0
    }

    /** Number of flushes that hit an [IOException] (0 = all writes succeeded). */
    fun failureCount(): Long {
        return failures
    }

    /** The file being written, e.g. for telemetry ("logging to ..."). */
    fun fileName(): String {
        return file.name
    }

    /** Flushes any remaining buffered rows. */
    override fun close() {
        flush()
    }

    companion object {
        private const val TAG = "CsvLogger"

        /**
         * A run-unique filename like `prefix_20260602_141233.csv`, so each OpMode run writes a
         * fresh file (with its own header) instead of appending to a previous run's data.
         */
        @JvmStatic
        fun timestamped(prefix: String): String {
            val stamp = java.text.SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(Date())
            return prefix + "_" + stamp + ".csv"
        }
    }
}
