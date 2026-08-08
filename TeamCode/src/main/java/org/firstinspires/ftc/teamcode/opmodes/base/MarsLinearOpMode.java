package org.firstinspires.ftc.teamcode.opmodes.base;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.robot.BulkReads;
import org.firstinspires.ftc.teamcode.utils.DashboardTelemetryPacketAccess;

/**
 * Linear OpMode base with an explicit control frame via {@link #nextFrame()}. Does not own a drive;
 * subclasses construct {@code MecanumDrive} / {@code TankDrive} (etc.) and may pass {@link
 * #batteryVoltage()} as the feedforward voltage supplier.
 *
 * <p>Call {@link #initRobot()} once at the start of {@code runOpMode()} (after {@code hardwareMap}
 * is live), create any hardware, then {@link #waitForStart()} (requires init), then {@code while
 * (nextFrame()) { ... }}.
 *
 * <p>Each {@link #nextFrame()} call:
 *
 * <ol>
 *   <li>Returns false if the OpMode is not in the Run phase ({@link #opModeIsActive()}, which also
 *       yields)
 *   <li>Flushes the previous frame's telemetry ({@code telemetry.update()})
 *   <li>Clears Lynx bulk caches ({@link BulkReads#readAll()}) so the next hardware reads are fresh
 *   <li>Invalidates the lazy battery sample
 *   <li>Captures the Dashboard {@link TelemetryPacket} for field overlay this frame
 * </ol>
 *
 * <p>{@link #batteryVoltage()} performs at most one hub ADC read per frame (battery is not in the
 * bulk packet).
 */
public abstract class MarsLinearOpMode extends LinearOpMode {
    protected BulkReads bulk;

    private VoltageSensor batterySensor;
    private double batteryV = Double.NaN;
    private DashboardTelemetryPacketAccess packetAccess;
    private TelemetryPacket packet;
    private boolean robotInitialized;

    /**
     * Sets up bulk caching, battery sensor, and Dashboard telemetry. Must run at the start of
     * {@code runOpMode()} — not in a field initializer or constructor ({@code hardwareMap} is null
     * until then). Subclasses may override to construct drive/mechanisms after {@code
     * super.initRobot()}.
     */
    protected void initRobot() {
        bulk = new BulkReads(hardwareMap);
        batterySensor = hardwareMap.voltageSensor.iterator().next();
        packetAccess = new DashboardTelemetryPacketAccess();
        telemetry = new MultipleTelemetry(telemetry, packetAccess.dashboardTelemetry);
        batteryV = Double.NaN;
        packet = null;
        robotInitialized = true;
    }

    /**
     * Same as {@link LinearOpMode#waitForStart()}, but requires {@link #initRobot()} first so
     * bulk/battery/telemetry are set up before the OpMode blocks for play.
     */
    @Override
    public final void waitForStart() {
        requireRobotInitialized();
        super.waitForStart();
    }

    /**
     * Starts a new control frame. Use as the loop condition after {@code waitForStart()}:
     *
     * <pre>{@code
     * while (nextFrame()) {
     *     // use batteryVoltage(), dashboardPacket(), drive, ...
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
        // Flush previous frame, then open a fresh Dashboard packet for this frame's overlay/data.
        telemetry.update();
        bulk.readAll();
        batteryV = Double.NaN;
        packet = packetAccess.getTelemetryPacket();
        return true;
    }

    /**
     * Battery input voltage (volts) for this frame. First call after {@link #nextFrame()} reads the
     * hub ADC; later calls reuse the cached value until the next frame. Safe to pass as {@code
     * MecanumDrive}'s voltage getter: {@code new MecanumDrive(hardwareMap, pose,
     * this::batteryVoltage)}.
     */
    public final double batteryVoltage() {
        requireRobotInitialized();
        if (Double.isNaN(batteryV)) {
            batteryV = batterySensor.getVoltage();
        }
        return batteryV;
    }

    /**
     * Dashboard packet for the current frame (field overlay, etc.). Valid after a successful {@link
     * #nextFrame()}; null before the first frame.
     */
    protected final TelemetryPacket dashboardPacket() {
        return packet;
    }

    private void requireRobotInitialized() {
        if (!robotInitialized) {
            throw new IllegalStateException(
                    "Call initRobot() at the start of runOpMode() before waitForStart(),"
                            + " nextFrame(), or batteryVoltage()");
        }
    }
}
