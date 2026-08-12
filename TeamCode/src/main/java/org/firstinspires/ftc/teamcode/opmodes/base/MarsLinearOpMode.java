package org.firstinspires.ftc.teamcode.opmodes.base;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.robot.BulkReads;
import org.firstinspires.ftc.teamcode.utils.DashboardTelemetryPacketAccess;
import org.firstinspires.ftc.teamcode.utils.GatedTelemetry;
import org.marsroboticsassociation.controllib.util.RunningIntervalStats;

/**
 * Linear OpMode base with explicit frame/sample I/O. Does not own a drive; subclasses construct via
 * {@code MecanumDrive.forMarsLinear} / {@code TankDrive.forMarsLinear} (passing {@link
 * #batteryVoltage()}) so AUTO bulk is not re-enabled after {@link #initRobot()}.
 *
 * <p>Call {@link #initRobot()} once at the start of {@code runOpMode()} (after {@code hardwareMap}
 * is live), create any hardware, then {@link #waitForStart()} (requires init), then either:
 *
 * <ul>
 *   <li>{@code while (nextFrame()) { ... }} — interactive control (sensors each loop; telemetry
 *       published about every {@link #FRAME_TELEMETRY_MS})
 *   <li>{@code while (nextSample()) { ... }} — high-rate measurement/control (sensors only; yield
 *       periodically; telemetry writes are closed)
 * </ul>
 *
 * <p>Each {@link #nextFrame()} call:
 *
 * <ol>
 *   <li>Returns false if the OpMode is not in the Run phase ({@link #opModeIsActive()}, which also
 *       yields)
 *   <li>On a telemetry publish frame (see {@link #FRAME_TELEMETRY_MS}): flushes the previous
 *       publish snapshot ({@link GatedTelemetry#forceUpdate()}), opens telemetry writes for this
 *       frame, and queues loop timing into telemetry
 *   <li>On non-publish frames: closes telemetry writes so {@code addData}/{@code addLine} are
 *       no-ops, and {@link #dashboardPacket()} is a throwaway packet (field overlay is not sent)
 *   <li>Clears Lynx bulk caches ({@link BulkReads#readAll()}) so the next hardware reads are fresh
 *   <li>Invalidates the lazy battery sample
 *   <li>On publish frames, captures the live Dashboard {@link TelemetryPacket} for field overlay
 *   <li>Records loop timing into {@link #loopStats()}
 * </ol>
 *
 * <p>{@link #nextSample()} is for dense capture (CSV, FFT, closed-loop steps): stop is polled every
 * call without yielding; cooperative {@link #idle()} runs about every {@link #SAMPLE_YIELD_MS};
 * bulk is refreshed every sample; battery is refreshed at most every {@link
 * #SAMPLE_BATTERY_TTL_MS}; loop timing is recorded; telemetry and Dashboard key/value publishes are
 * not.
 *
 * <p>{@link #batteryVoltage()} performs a hub ADC read only when the cached value is invalid
 * (battery is not in the bulk packet). {@link #nextFrame()} always invalidates so each frame gets a
 * fresh sample; {@link #nextSample()} reuses the cache until {@link #SAMPLE_BATTERY_TTL_MS} elapses
 * so dense loops are not ADC-bound. {@link #loopDt()} is the last measured interval (seconds) for
 * controllers.
 */
public abstract class MarsLinearOpMode extends LinearOpMode {
    /**
     * Max time between cooperative {@link #idle()} calls inside {@link #nextSample()} (ms). Stop is
     * still checked every sample via {@link #isStopRequested()}. {@code 0} disables periodic yield
     * (not recommended for long bursts).
     */
    public static int SAMPLE_YIELD_MS = 500;

    /**
     * Max age of the cached battery voltage during {@link #nextSample()} bursts (ms). Battery is
     * not in the bulk packet; reusing the cache for a short TTL avoids an ADC read on every dense
     * sample when voltage FF runs each tick. {@code 0} invalidates every sample (always-fresh).
     * {@link #nextFrame()} always invalidates regardless of this value.
     */
    public static int SAMPLE_BATTERY_TTL_MS = 100;

    /**
     * Minimum interval between telemetry publish frames in {@link #nextFrame()} (ms). Matches the
     * SDK default DS transmission interval. On publish frames, previous snapshot is flushed and
     * {@code addData} is live; between publishes, {@link GatedTelemetry} no-ops writes and skips
     * {@code update()}. {@code 0} publishes every frame (full rate).
     */
    public static int FRAME_TELEMETRY_MS = 250;

    /**
     * Sliding window (seconds) for loop-interval mean/stdev in {@link #loopStats()}. Applied on
     * each {@link #nextFrame()} / {@link #nextSample()} via {@link
     * RunningIntervalStats#reconfigure(double)}.
     */
    public static double LOOP_STATS_WINDOW_S = 2.0;

    protected BulkReads bulk;

    private VoltageSensor batterySensor;
    private double batteryV = Double.NaN;

    /**
     * NanoTime of the last ADC read into {@link #batteryV}; meaningful only when {@code batteryV}
     * is finite.
     */
    private long batterySampleNs;

    private DashboardTelemetryPacketAccess packetAccess;
    private GatedTelemetry gatedTelemetry;

    /** Live Dashboard packet (publish frames) or a never-sent throwaway (non-publish). */
    private TelemetryPacket packet;

    private boolean robotInitialized;
    private long lastSampleYieldNs;
    private long lastTelemetryPublishNs;
    private RunningIntervalStats loopStats;

    /**
     * Sets up bulk caching, battery sensor, gated Dashboard telemetry, and loop-timing stats. Must
     * run at the start of {@code runOpMode()} — not in a field initializer or constructor ({@code
     * hardwareMap} is null until then). Subclasses may override to construct drive/mechanisms after
     * {@code super.initRobot()}. Telemetry writes stay open through init / {@link #waitForStart()}
     * so pre-start messages still work.
     */
    protected void initRobot() {
        bulk = new BulkReads(hardwareMap);
        batterySensor = hardwareMap.voltageSensor.iterator().next();
        packetAccess = new DashboardTelemetryPacketAccess();
        gatedTelemetry =
                new GatedTelemetry(
                        new MultipleTelemetry(telemetry, packetAccess.dashboardTelemetry));
        gatedTelemetry.setOpen(true);
        if (FRAME_TELEMETRY_MS > 0) {
            gatedTelemetry.setMsTransmissionInterval(FRAME_TELEMETRY_MS);
        }
        telemetry = gatedTelemetry;
        batteryV = Double.NaN;
        batterySampleNs = 0L;
        packet = null;
        lastSampleYieldNs = 0L;
        lastTelemetryPublishNs = 0L;
        loopStats = new RunningIntervalStats(LOOP_STATS_WINDOW_S);
        robotInitialized = true;
    }

    /**
     * Same as {@link LinearOpMode#waitForStart()}, but requires {@link #initRobot()} first so
     * bulk/battery/telemetry are set up before the OpMode blocks for play. Resets loop-timing stats
     * so {@link RunningIntervalStats#getElapsed()} is from START, not from init. Seeds {@code
     * lastSampleYieldNs} so the first {@link #nextSample()} does not always force an {@link
     * #idle()} from a zero timestamp.
     */
    @Override
    public final void waitForStart() {
        requireRobotInitialized();
        super.waitForStart();
        loopStats = new RunningIntervalStats(LOOP_STATS_WINDOW_S);
        lastSampleYieldNs = System.nanoTime();
        lastTelemetryPublishNs = 0L;
        gatedTelemetry.setOpen(true);
    }

    /**
     * Starts a new control frame. Use as the loop condition after {@code waitForStart()}:
     *
     * <pre>{@code
     * while (nextFrame()) {
     *     // use batteryVoltage(), dashboardPacket(), loopDt(), drive, ...
     *     // telemetry.addData may be called every loop; only publish frames pay for it
     * }
     * }</pre>
     *
     * <p>Do not also call {@link #opModeIsActive()} as the loop condition (that would yield twice
     * and skip a clean frame boundary). For mid-frame stop checks without a new bulk/ADC cycle, use
     * {@link #isStopRequested()}.
     *
     * <p>Public so helpers (e.g. {@code ReversalFeedforwardId}) can advance a frame without living
     * in this package.
     *
     * @return true while the OpMode should keep running
     */
    public final boolean nextFrame() {
        requireRobotInitialized();
        if (!opModeIsActive()) {
            return false;
        }

        long nowNs = System.nanoTime();
        boolean publish =
                FRAME_TELEMETRY_MS <= 0
                        || lastTelemetryPublishNs == 0L
                        || (nowNs - lastTelemetryPublishNs) >= FRAME_TELEMETRY_MS * 1_000_000L;

        if (publish) {
            // Flush previous publish snapshot (key/value + live Dashboard packet), then accept
            // writes against a fresh live packet.
            gatedTelemetry.forceUpdate();
            gatedTelemetry.setOpen(true);
            lastTelemetryPublishNs = nowNs;
            packet = packetAccess.getTelemetryPacket();
        } else {
            gatedTelemetry.setOpen(false);
            // Never-sent sink so callers can draw every loop without stacking ops on the live
            // packet (avoids multipose ghost trails at publish). New instance so canvas ops do
            // not grow unbounded across non-publish frames.
            packet = new TelemetryPacket();
        }

        bulk.readAll();
        batteryV = Double.NaN;
        recordLoopTiming();
        // Only formats when open; lands in the next forceUpdate on a later publish frame.
        if (publish) {
            loopStats.writeTelemetry(telemetry::addData);
        }
        return true;
    }

    /**
     * High-rate sample boundary for measurement and closed-loop bursts (CSV logging, FFT holds,
     * gain steps). Prefer {@link #nextFrame()} when the loop should update driver-station /
     * Dashboard telemetry.
     *
     * <p>Each call:
     *
     * <ol>
     *   <li>Returns false if the OpMode has not started or stop was requested ({@link
     *       #isStopRequested()}, no yield)
     *   <li>About every {@link #SAMPLE_YIELD_MS}, calls {@link #idle()} so the SDK is not starved,
     *       then rechecks stop
     *   <li>Clears Lynx bulk caches; invalidates the battery cache if {@link
     *       #SAMPLE_BATTERY_TTL_MS} has elapsed (or always if TTL is {@code 0})
     *   <li>Records loop timing (no telemetry write)
     * </ol>
     *
     * <p>Closes gated telemetry so accidental {@code addData} is free. Does not call {@code
     * telemetry.update()} or open a new Dashboard publish. Public so helpers can advance samples
     * without living in this package.
     *
     * @return true while the OpMode should keep sampling
     */
    public final boolean nextSample() {
        requireRobotInitialized();
        if (!isStarted() || isStopRequested()) {
            return false;
        }
        gatedTelemetry.setOpen(false);
        packet = null;
        long nowNs = System.nanoTime();
        if (SAMPLE_YIELD_MS > 0 && (nowNs - lastSampleYieldNs) >= SAMPLE_YIELD_MS * 1_000_000L) {
            idle();
            lastSampleYieldNs = System.nanoTime();
            if (isStopRequested()) {
                return false;
            }
            nowNs = lastSampleYieldNs;
        }
        bulk.readAll();
        if (SAMPLE_BATTERY_TTL_MS <= 0
                || Double.isNaN(batteryV)
                || (nowNs - batterySampleNs) >= SAMPLE_BATTERY_TTL_MS * 1_000_000L) {
            batteryV = Double.NaN;
        }
        recordLoopTiming();
        return true;
    }

    /**
     * Battery input voltage (volts) for this frame or sample. Reads the hub ADC only when the cache
     * is invalid; later calls reuse the cached value until the next invalidation. {@link
     * #nextFrame()} invalidates every frame; {@link #nextSample()} invalidates when {@link
     * #SAMPLE_BATTERY_TTL_MS} elapses. Pass to the drive factory: {@code
     * MecanumDrive.forMarsLinear(hardwareMap, pose, this::batteryVoltage)}.
     */
    public final double batteryVoltage() {
        requireRobotInitialized();
        if (Double.isNaN(batteryV)) {
            batteryV = batterySensor.getVoltage();
            batterySampleNs = System.nanoTime();
        }
        return batteryV;
    }

    /**
     * Last measured interval between {@link #nextFrame()} / {@link #nextSample()} calls (seconds),
     * or {@link Double#NaN} before the second advance. Same value as {@link
     * RunningIntervalStats#getDt()}.
     */
    public final double loopDt() {
        requireRobotInitialized();
        return loopStats.getDt();
    }

    /**
     * Sliding-window loop interval stats (mean/stdev ms, last dt, elapsed). Recorded automatically
     * by {@link #nextFrame()} and {@link #nextSample()}.
     */
    protected final RunningIntervalStats loopStats() {
        requireRobotInitialized();
        return loopStats;
    }

    /**
     * Dashboard packet for field overlay (and related packet puts). After {@link #nextFrame()}: the
     * live Dashboard packet on publish frames, or a throwaway packet on non-publish frames (draws
     * are discarded and never sent). Null before the first frame and during {@link #nextSample()}
     * bursts. Prefer {@link #isTelemetryPublishFrame()} if draw work itself should be skipped.
     */
    protected final TelemetryPacket dashboardPacket() {
        return packet;
    }

    /**
     * Whether this {@link #nextFrame()} is a telemetry publish frame (string writes open and {@link
     * #dashboardPacket()} is the live Dashboard packet). False on non-publish frames, during {@link
     * #nextSample()}, and before the first frame.
     */
    protected final boolean isTelemetryPublishFrame() {
        return gatedTelemetry != null && gatedTelemetry.isOpen();
    }

    private void recordLoopTiming() {
        loopStats.reconfigure(LOOP_STATS_WINDOW_S);
        loopStats.record();
    }

    private void requireRobotInitialized() {
        if (!robotInitialized) {
            throw new IllegalStateException(
                    "Call initRobot() at the start of runOpMode() before waitForStart(),"
                            + " nextFrame(), nextSample(), or batteryVoltage()");
        }
    }
}
