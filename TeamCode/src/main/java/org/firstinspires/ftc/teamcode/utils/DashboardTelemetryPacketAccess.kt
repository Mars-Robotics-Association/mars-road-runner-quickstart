package org.firstinspires.ftc.teamcode.utils

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import org.firstinspires.ftc.robotcore.external.Telemetry
import java.lang.reflect.Field

/**
 * Provides access to the internal [TelemetryPacket] from FTC Dashboard's telemetry adapter.
 *
 * <p>This uses reflection to access the private `currentPacket` field in
 * `FtcDashboard.TelemetryAdapter`. The [Field] reference is cached
 * at construction time to avoid repeated reflection lookups.
 */
class DashboardTelemetryPacketAccess {
    @JvmField
    val dashboardTelemetry: Telemetry = FtcDashboard.getInstance().telemetry
    private val currentPacketField: Field

    /**
     * Constructs an accessor for the given dashboard telemetry instance.
     *
     * @throws RuntimeException if the field cannot be found (e.g., wrong telemetry type or API change).
     */
    init {
        try {
            this.currentPacketField = dashboardTelemetry.javaClass.getDeclaredField("currentPacket")
            this.currentPacketField.isAccessible = true
        } catch (e: NoSuchFieldException) {
            throw RuntimeException("Failed to find currentPacket field. Is this a dashboard Telemetry?", e)
        }
    }

    /**
     * Returns the current [TelemetryPacket] being built by the dashboard telemetry.
     *
     * <p>This packet is reset after each `telemetry.update()` call, so you should
     * access it each loop iteration before calling update.
     *
     * @return The current TelemetryPacket, or null if access fails.
     */
    fun getTelemetryPacket(): TelemetryPacket? {
        return try {
            currentPacketField.get(dashboardTelemetry) as TelemetryPacket?
        } catch (e: IllegalAccessException) {
            null
        }
    }
}
