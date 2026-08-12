package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.HardwareMap

class BulkReads(hardwareMap: HardwareMap) {
    private val lynxModules: List<LynxModule> = hardwareMap.getAll(LynxModule::class.java)

    init {
        for (hub in lynxModules) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }
    }

    fun readAll() {
        for (hub in lynxModules) {
            hub.clearBulkCache()
        }
    }
}
