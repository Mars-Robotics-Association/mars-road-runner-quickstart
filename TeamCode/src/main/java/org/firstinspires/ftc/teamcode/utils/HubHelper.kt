package org.firstinspires.ftc.teamcode.utils

import android.util.Log
import com.qualcomm.hardware.lynx.LynxDcMotorController
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import java.lang.reflect.Method

class HubHelper {
    companion object {
        /**
         * Attempt to get the hub module the motor is actually attached to. Uses reflection to call
         * private LynxDcMotorController.getModule(). Falls back to the first module in hardwareMap
         * if anything goes wrong.
         */
        @JvmStatic
        fun getHubForMotor(motor: DcMotorEx, hardwareMap: HardwareMap): LynxModule {
            try {
                val controller = motor.controller

                if (controller is LynxDcMotorController) {
                    // getModule() may be declared on a superclass (e.g. LynxController),
                    // so walk the hierarchy to find it.
                    var getModuleMethod: Method? = null
                    var cls: Class<*>? = LynxDcMotorController::class.java
                    while (cls != null) {
                        try {
                            getModuleMethod = cls.getDeclaredMethod("getModule")
                            break
                        } catch (_: NoSuchMethodException) {}
                        cls = cls.superclass
                    }
                    if (getModuleMethod == null) throw NoSuchMethodException("getModule")
                    getModuleMethod.isAccessible = true

                    val module = getModuleMethod.invoke(controller)
                    if (module is LynxModule) {
                        return module
                    }
                }

                // If any check fails, fall through to fallback
            } catch (e: Exception) {
                Log.e("HubHelper", "Failed to get hub for motor")
                // after logging, fallback
            }

            // Fallback: return the first hub in hardware map
            val hubs = hardwareMap.getAll(LynxModule::class.java)
            if (hubs.isNotEmpty()) {
                return hubs[0]
            }

            throw IllegalStateException("No LynxModules found in hardwareMap")
        }
    }
}
