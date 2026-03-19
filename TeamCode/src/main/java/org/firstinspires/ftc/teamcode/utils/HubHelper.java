package org.firstinspires.ftc.teamcode.utils;

import android.util.Log;

import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.lang.reflect.Method;
import java.util.List;

public class HubHelper {

    /**
     * Attempt to get the hub module the motor is actually attached to.
     * Uses reflection to call private LynxDcMotorController.getModule().
     * Falls back to the first module in hardwareMap if anything goes wrong.
     */
    public static LynxModule getHubForMotor(DcMotorEx motor, HardwareMap hardwareMap) {
        try {
            Object controller = motor.getController();

            if (controller instanceof LynxDcMotorController lynxController) {
                // getModule() may be declared on a superclass (e.g. LynxController),
                // so walk the hierarchy to find it.
                Method getModuleMethod = null;
                for (Class<?> cls = LynxDcMotorController.class; cls != null; cls = cls.getSuperclass()) {
                    try {
                        getModuleMethod = cls.getDeclaredMethod("getModule");
                        break;
                    } catch (NoSuchMethodException ignored) {}
                }
                if (getModuleMethod == null) throw new NoSuchMethodException("getModule");
                getModuleMethod.setAccessible(true);

                Object module = getModuleMethod.invoke(lynxController);
                if (module instanceof LynxModule lynxModule) {
                    return lynxModule;
                }
            }

            // If any check fails, fall through to fallback
        } catch (Exception e) {
            Log.e("HubHelper", "Failed to get hub for motor");
            // after logging, fallback
        }

        // Fallback: return the first hub in hardware map
        var hubs = hardwareMap.getAll(LynxModule.class);
        if (!hubs.isEmpty()) {
            return hubs.get(0);
        }

        throw new IllegalStateException("No LynxModules found in hardwareMap");
    }
}
