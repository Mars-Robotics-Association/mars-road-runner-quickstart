package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Thin Java facade around a few Road Runner FTC helpers whose Kotlin facades do not resolve cleanly
 * from TeamCode Kotlin sources (same bytecode is visible to javac).
 */
public final class FtcRoadRunnerCompat {
    private FtcRoadRunnerCompat() {}

    public static void throwIfModulesAreOutdated(HardwareMap hardwareMap) {
        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);
    }

    public static void runBlocking(Action action) {
        Actions.runBlocking(action);
    }
}
