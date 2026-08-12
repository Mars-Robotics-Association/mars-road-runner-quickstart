package org.firstinspires.ftc.teamcode.opmodes.base

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.teamcode.robot.BulkReads
import org.firstinspires.ftc.teamcode.utils.DashboardTelemetryPacketAccess
import org.firstinspires.ftc.teamcode.utils.GatedTelemetry
import org.marsroboticsassociation.controllib.util.RunningIntervalStats

/**
 * Linear OpMode base with explicit frame/sample I/O. Does not own a drive; subclasses construct via
 * `MecanumDrive.forMarsLinear` / `TankDrive.forMarsLinear` (passing [batteryVoltage]) so AUTO bulk
 * is not re-enabled after [initRobot].
 *
 * <p>Call [initRobot] once at the start of `runOpMode()` (after `hardwareMap` is live), create any
 * hardware, then [waitForStart] (requires init), then either:
 * <ul>
 * <li>`while (nextFrame()) { ... }` — interactive control (sensors each loop; telemetry published
 *   about every [FRAME_TELEMETRY_MS])
 * <li>`while (nextSample()) { ... }` — high-rate measurement/control (sensors only; yield
 *   periodically; telemetry writes are closed)
 * </ul>
 *
 * <p>Each [nextFrame] call:
 * <ol>
 * <li>Returns false if the OpMode is not in the Run phase ([opModeIsActive], which also yields)
 * <li>On a telemetry publish frame (see [FRAME_TELEMETRY_MS]): flushes the previous publish
 *   snapshot ([GatedTelemetry.forceUpdate]), opens telemetry writes for this frame, and queues loop
 *   timing into telemetry
 * <li>On non-publish frames: closes telemetry writes so `addData`/`addLine` are no-ops, and
 *   [dashboardPacket] is a throwaway packet (field overlay is not sent)
 * <li>Clears Lynx bulk caches ([BulkReads.readAll]) so the next hardware reads are fresh
 * <li>Invalidates the lazy battery sample
 * <li>On publish frames, captures the live Dashboard [TelemetryPacket] for field overlay
 * <li>Records loop timing into [loopStats]
 * </ol>
 *
 * <p>[nextSample] is for dense capture (CSV, FFT, closed-loop steps): stop is polled every call
 * without yielding; cooperative [idle] runs about every [SAMPLE_YIELD_MS]; bulk is refreshed every
 * sample; battery is refreshed at most every [SAMPLE_BATTERY_TTL_MS]; loop timing is recorded;
 * telemetry and Dashboard key/value publishes are not.
 *
 * <p>[batteryVoltage] performs a hub ADC read only when the cached value is invalid (battery is not
 * in the bulk packet). [nextFrame] always invalidates so each frame gets a fresh sample;
 * [nextSample] reuses the cache until [SAMPLE_BATTERY_TTL_MS] elapses so dense loops are not
 * ADC-bound. [loopDt] is the last measured interval (seconds) for controllers.
 */
abstract class MarsLinearOpMode : LinearOpMode() {
    companion object {
        /**
         * Max time between cooperative [idle] calls inside [nextSample] (ms). Stop is still checked
         * every sample via [isStopRequested]. `0` disables periodic yield (not recommended for long
         * bursts).
         */
        @JvmField var SAMPLE_YIELD_MS = 500

        /**
         * Max age of the cached battery voltage during [nextSample] bursts (ms). Battery is not in
         * the bulk packet; reusing the cache for a short TTL avoids an ADC read on every dense
         * sample when voltage FF runs each tick. `0` invalidates every sample (always-fresh).
         * [nextFrame] always invalidates regardless of this value.
         */
        @JvmField var SAMPLE_BATTERY_TTL_MS = 100

        /**
         * Minimum interval between telemetry publish frames in [nextFrame] (ms). Matches the SDK
         * default DS transmission interval. On publish frames, previous snapshot is flushed and
         * `addData` is live; between publishes, [GatedTelemetry] no-ops writes and skips
         * `update()`. `0` publishes every frame (full rate).
         */
        @JvmField var FRAME_TELEMETRY_MS = 250

        /**
         * Sliding window (seconds) for loop-interval mean/stdev in [loopStats]. Applied on each
         * [nextFrame] / [nextSample] via [RunningIntervalStats.reconfigure].
         */
        @JvmField var LOOP_STATS_WINDOW_S = 2.0
    }

    protected lateinit var bulk: BulkReads

    private lateinit var batterySensor: VoltageSensor
    private var batteryV = Double.NaN

    /** NanoTime of the last ADC read into [batteryV]; meaningful only when `batteryV` is finite. */
    private var batterySampleNs = 0L

    private lateinit var packetAccess: DashboardTelemetryPacketAccess
    private lateinit var gatedTelemetry: GatedTelemetry

    /** Live Dashboard packet (publish frames) or a never-sent throwaway (non-publish). */
    private var packet: TelemetryPacket? = null

    private var robotInitialized = false
    private var lastSampleYieldNs = 0L
    private var lastTelemetryPublishNs = 0L
    private lateinit var loopIntervalStats: RunningIntervalStats

    /**
     * Sets up bulk caching, battery sensor, gated Dashboard telemetry, and loop-timing stats. Must
     * run at the start of `runOpMode()` — not in a field initializer or constructor (` hardwareMap`
     * is null until then). Subclasses may override to construct drive/mechanisms after
     * `super.initRobot()`. Telemetry writes stay open through init / [waitForStart] so pre-start
     * messages still work.
     */
    protected open fun initRobot() {
        bulk = BulkReads(hardwareMap)
        batterySensor = hardwareMap.voltageSensor.iterator().next()
        packetAccess = DashboardTelemetryPacketAccess()
        gatedTelemetry =
            GatedTelemetry(MultipleTelemetry(telemetry, packetAccess.dashboardTelemetry))
        gatedTelemetry.setOpen(true)
        if (FRAME_TELEMETRY_MS > 0) {
            gatedTelemetry.setMsTransmissionInterval(FRAME_TELEMETRY_MS)
        }
        telemetry = gatedTelemetry
        batteryV = Double.NaN
        batterySampleNs = 0L
        packet = null
        lastSampleYieldNs = 0L
        lastTelemetryPublishNs = 0L
        loopIntervalStats = RunningIntervalStats(LOOP_STATS_WINDOW_S)
        robotInitialized = true
    }

    /**
     * Same as [LinearOpMode.waitForStart], but requires [initRobot] first so bulk/battery/telemetry
     * are set up before the OpMode blocks for play. Resets loop-timing stats so
     * [RunningIntervalStats.getElapsed] is from START, not from init. Seeds ` lastSampleYieldNs` so
     * the first [nextSample] does not always force an [idle] from a zero timestamp.
     */
    final override fun waitForStart() {
        requireRobotInitialized()
        super.waitForStart()
        loopIntervalStats = RunningIntervalStats(LOOP_STATS_WINDOW_S)
        lastSampleYieldNs = System.nanoTime()
        lastTelemetryPublishNs = 0L
        gatedTelemetry.setOpen(true)
    }

    /**
     * Starts a new control frame. Use as the loop condition after `waitForStart()`:
     * <pre>`{@`
     * while (nextFrame()) {
     *     // use batteryVoltage(), dashboardPacket(), loopDt(), drive, ...
     *     // telemetry.addData may be called every loop; only publish frames pay for it
     * }
     * `</pre>
     *
     * <p>Do not also call [opModeIsActive] as the loop condition (that would yield twice and skip a
     * clean frame boundary). For mid-frame stop checks without a new bulk/ADC cycle, use
     * [isStopRequested].
     *
     * <p>Public so helpers (e.g. `ReversalFeedforwardId`) can advance a frame without living in
     * this package.
     *
     * @return true while the OpMode should keep running
     */
    fun nextFrame(): Boolean {
        requireRobotInitialized()
        if (!opModeIsActive()) {
            return false
        }

        val nowNs = System.nanoTime()
        val publish =
            FRAME_TELEMETRY_MS <= 0 ||
                lastTelemetryPublishNs == 0L ||
                (nowNs - lastTelemetryPublishNs) >= FRAME_TELEMETRY_MS * 1_000_000L

        if (publish) {
            // Flush previous publish snapshot (key/value + live Dashboard packet), then accept
            // writes against a fresh live packet.
            gatedTelemetry.forceUpdate()
            gatedTelemetry.setOpen(true)
            lastTelemetryPublishNs = nowNs
            packet = packetAccess.getTelemetryPacket()
        } else {
            gatedTelemetry.setOpen(false)
            // Never-sent sink so callers can draw every loop without stacking ops on the live
            // packet (avoids multipose ghost trails at publish). New instance so canvas ops do
            // not grow unbounded across non-publish frames.
            packet = TelemetryPacket()
        }

        bulk.readAll()
        batteryV = Double.NaN
        recordLoopTiming()
        // Only formats when open; lands in the next forceUpdate on a later publish frame.
        if (publish) {
            loopIntervalStats.writeTelemetry { caption, format, value ->
                telemetry.addData(caption, format, value)
            }
        }
        return true
    }

    /**
     * High-rate sample boundary for measurement and closed-loop bursts (CSV logging, FFT holds,
     * gain steps). Prefer [nextFrame] when the loop should update driver-station / Dashboard
     * telemetry.
     *
     * <p>Each call:
     * <ol>
     * <li>Returns false if the OpMode has not started or stop was requested ( [isStopRequested], no
     *   yield)
     * <li>About every [SAMPLE_YIELD_MS], calls [idle] so the SDK is not starved, then rechecks stop
     * <li>Clears Lynx bulk caches; invalidates the battery cache if [SAMPLE_BATTERY_TTL_MS] has
     *   elapsed (or always if TTL is `0`)
     * <li>Records loop timing (no telemetry write)
     * </ol>
     *
     * <p>Closes gated telemetry so accidental `addData` is free. Does not call `
     * telemetry.update()` or open a new Dashboard publish. Public so helpers can advance samples
     * without living in this package.
     *
     * @return true while the OpMode should keep sampling
     */
    fun nextSample(): Boolean {
        requireRobotInitialized()
        if (!isStarted || isStopRequested) {
            return false
        }
        gatedTelemetry.setOpen(false)
        packet = null
        var nowNs = System.nanoTime()
        if (SAMPLE_YIELD_MS > 0 && (nowNs - lastSampleYieldNs) >= SAMPLE_YIELD_MS * 1_000_000L) {
            idle()
            lastSampleYieldNs = System.nanoTime()
            if (isStopRequested) {
                return false
            }
            nowNs = lastSampleYieldNs
        }
        bulk.readAll()
        if (
            SAMPLE_BATTERY_TTL_MS <= 0 ||
                batteryV.isNaN() ||
                (nowNs - batterySampleNs) >= SAMPLE_BATTERY_TTL_MS * 1_000_000L
        ) {
            batteryV = Double.NaN
        }
        recordLoopTiming()
        return true
    }

    /**
     * Battery input voltage (volts) for this frame or sample. Reads the hub ADC only when the cache
     * is invalid; later calls reuse the cached value until the next invalidation. [nextFrame]
     * invalidates every frame; [nextSample] invalidates when [SAMPLE_BATTERY_TTL_MS] elapses. Pass
     * to the drive factory: ` MecanumDrive.forMarsLinear(hardwareMap, pose, this::batteryVoltage)`.
     */
    fun batteryVoltage(): Double {
        requireRobotInitialized()
        if (batteryV.isNaN()) {
            batteryV = batterySensor.voltage
            batterySampleNs = System.nanoTime()
        }
        return batteryV
    }

    /**
     * Last measured interval between [nextFrame] / [nextSample] calls (seconds), or [Double.NaN]
     * before the second advance. Same value as [RunningIntervalStats.getDt].
     */
    fun loopDt(): Double {
        requireRobotInitialized()
        return loopIntervalStats.dt
    }

    /**
     * Sliding-window loop interval stats (mean/stdev ms, last dt, elapsed). Recorded automatically
     * by [nextFrame] and [nextSample].
     */
    protected fun loopStats(): RunningIntervalStats {
        requireRobotInitialized()
        return loopIntervalStats
    }

    /**
     * Dashboard packet for field overlay (and related packet puts). After [nextFrame]: the live
     * Dashboard packet on publish frames, or a throwaway packet on non-publish frames (draws are
     * discarded and never sent). Null before the first frame and during [nextSample] bursts. Prefer
     * [isTelemetryPublishFrame] if draw work itself should be skipped.
     */
    protected fun dashboardPacket(): TelemetryPacket? = packet

    /**
     * Whether this [nextFrame] is a telemetry publish frame (string writes open and
     * [dashboardPacket] is the live Dashboard packet). False on non-publish frames, during
     * [nextSample], and before the first frame.
     */
    protected fun isTelemetryPublishFrame(): Boolean = robotInitialized && gatedTelemetry.isOpen()

    private fun recordLoopTiming() {
        loopIntervalStats.reconfigure(LOOP_STATS_WINDOW_S)
        loopIntervalStats.record()
    }

    private fun requireRobotInitialized() {
        check(robotInitialized) {
            "Call initRobot() at the start of runOpMode() before waitForStart()," +
                " nextFrame(), nextSample(), or batteryVoltage()"
        }
    }
}
