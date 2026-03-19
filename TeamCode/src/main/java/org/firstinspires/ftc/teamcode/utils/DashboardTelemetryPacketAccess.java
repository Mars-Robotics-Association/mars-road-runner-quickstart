package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.reflect.Field;

/**
 * Provides access to the internal {@link TelemetryPacket} from FTC Dashboard's telemetry adapter.
 *
 * <p>This uses reflection to access the private {@code currentPacket} field in
 * {@code FtcDashboard.TelemetryAdapter}. The {@link Field} reference is cached
 * at construction time to avoid repeated reflection lookups.
 */
public class DashboardTelemetryPacketAccess {

    public final Telemetry dashboardTelemetry;
    private final Field currentPacketField;

    /**
     * Constructs an accessor for the given dashboard telemetry instance.
     *
     * @throws RuntimeException if the field cannot be found (e.g., wrong telemetry type or API change).
     */
    public DashboardTelemetryPacketAccess() {
        this.dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();
        try {
            this.currentPacketField = dashboardTelemetry.getClass().getDeclaredField("currentPacket");
            this.currentPacketField.setAccessible(true);
        } catch (NoSuchFieldException e) {
            throw new RuntimeException("Failed to find currentPacket field. Is this a dashboard Telemetry?", e);
        }
    }

    /**
     * Returns the current {@link TelemetryPacket} being built by the dashboard telemetry.
     *
     * <p>This packet is reset after each {@code telemetry.update()} call, so you should
     * access it each loop iteration before calling update.
     *
     * @return The current TelemetryPacket, or null if access fails.
     */
    public TelemetryPacket getTelemetryPacket() {
        try {
            return (TelemetryPacket) currentPacketField.get(dashboardTelemetry);
        } catch (IllegalAccessException e) {
            return null;
        }
    }
}
