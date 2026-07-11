package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Date;
import java.util.Locale;

/**
 * Tiny append-only CSV logger for offline-analysis files under the {@code FIRST/} folder. Rows are
 * buffered in memory and written in batches: a {@link FileWriter} is opened once per {@link
 * #flush()}, never once per {@link #row}, so high-rate logging from a control loop pays disk I/O in
 * occasional bursts instead of stalling the loop on every sample. Writing is best-effort — an
 * {@link IOException} is counted ({@link #failureCount()}) and logged, never thrown, so a full disk
 * or a bad path can't crash an OpMode mid-match.
 *
 * <p>The header is written lazily and only when the target file does not yet exist. So a fixed
 * filename used across runs accumulates rows under a single header, while a {@link #timestamped}
 * per-run filename gets a fresh file with its own header.
 *
 * <p>Values passed to {@link #row} are stringified with {@link String#valueOf}; doubles therefore
 * serialize at full round-trip precision (e.g. a {@code t_ms} timestamp keeps sub-microsecond
 * resolution), and {@code NaN} is written literally. This is deliberately better for offline
 * analysis than a fixed {@code %.3f}-style format, which would silently truncate timestamps.
 *
 * <h3>Continuous (per-loop) use</h3>
 *
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
 *
 * <pre>
 *   CsvLogger log = new CsvLogger("encoder_burst.csv", "burst_index,t_ns,position");
 *   for (...) log.row(idx, tNs, position);
 *   log.flush();                                        // one disk write per burst
 * </pre>
 */
public final class CsvLogger implements AutoCloseable {

    private static final String TAG = "CsvLogger";

    private final File file;
    private final String header;
    private final StringBuilder buffer = new StringBuilder();
    private int bufferedRows = 0;
    private long failures = 0;

    /**
     * @param filename name of the file under {@code FIRST/} (e.g. {@code "encoder_burst.csv"} or
     *     {@link #timestamped}{@code ("vis_fusion")})
     * @param header CSV header line, comma-separated, with no trailing newline; written only when
     *     the file does not already exist
     */
    public CsvLogger(String filename, String header) {
        this.file = new File(AppUtil.FIRST_FOLDER, filename);
        this.header = header;
    }

    /**
     * A run-unique filename like {@code prefix_20260602_141233.csv}, so each OpMode run writes a
     * fresh file (with its own header) instead of appending to a previous run's data.
     */
    public static String timestamped(String prefix) {
        String stamp =
                new java.text.SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
        return prefix + "_" + stamp + ".csv";
    }

    /**
     * Buffer one row. Columns are comma-joined via {@link String#valueOf} (full double precision).
     */
    public void row(Object... cols) {
        for (int i = 0; i < cols.length; i++) {
            if (i > 0) buffer.append(',');
            buffer.append(String.valueOf(cols[i]));
        }
        buffer.append('\n');
        bufferedRows++;
    }

    /** Rows buffered since the last {@link #flush()} — poll to decide when to amortize a write. */
    public int bufferedRows() {
        return bufferedRows;
    }

    /**
     * Append all buffered rows to disk, writing the header first if the file is new. Best-effort:
     * on an {@link IOException} the buffer is still cleared (so a transient failure doesn't wedge
     * the logger) and the failure is counted and logged.
     */
    public void flush() {
        if (bufferedRows == 0) {
            return;
        }
        boolean newFile = !file.exists();
        try (FileWriter w = new FileWriter(file, true)) {
            if (newFile) {
                w.write(header);
                w.write('\n');
            }
            w.write(buffer.toString());
        } catch (IOException e) {
            failures++;
            RobotLog.ee(TAG, "CSV write failed (" + file.getName() + "): " + e.getMessage());
        }
        buffer.setLength(0);
        bufferedRows = 0;
    }

    /** Number of flushes that hit an {@link IOException} (0 = all writes succeeded). */
    public long failureCount() {
        return failures;
    }

    /** The file being written, e.g. for telemetry ("logging to ..."). */
    public String fileName() {
        return file.getName();
    }

    /** Flushes any remaining buffered rows. */
    @Override
    public void close() {
        flush();
    }
}
