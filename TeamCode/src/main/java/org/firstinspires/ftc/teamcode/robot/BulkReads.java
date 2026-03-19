package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class BulkReads {
    private final List<LynxModule> lynxModules;

    public BulkReads(HardwareMap hardwareMap) {
        lynxModules = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : lynxModules) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    public void readAll() {
        for (LynxModule hub : lynxModules) {
            hub.clearBulkCache();
        }
    }
}
