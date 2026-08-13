package org.firstinspires.ftc.teamcode.robot

import android.graphics.Color
import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.marsroboticsassociation.controllib.util.LUT
import org.marsroboticsassociation.controllib.util.SetOnChange

class ColorDistanceSensor(revV3: Boolean, hardwareMap: HardwareMap, name: String) {
    private val colorSensorV3: RevColorSensorV3?
    private val colorSensor: ColorSensor
    private val distanceSensor: DistanceSensor?
    private val ledEnabled: SetOnChange<Boolean>
    private var argb: Int

    init {
        if (revV3) {
            colorSensorV3 = hardwareMap.colorSensor.get(name) as RevColorSensorV3
            colorSensor = colorSensorV3
            distanceSensor = null
        } else {
            colorSensorV3 = null
            colorSensor = hardwareMap.colorSensor.get(name)
            distanceSensor = hardwareMap.get(DistanceSensor::class.java, name)
        }
        ledEnabled = SetOnChange.of(true, colorSensor::enableLed)
        argb = colorSensor.argb()
        ledEnabled.set(false)
    }

    fun currentHSV(): FloatArray {
        val hsv = FloatArray(3)
        Color.colorToHSV(this.argb, hsv)
        return hsv
    }

    fun currentColorName(): String {
        return if (currentHSV()[1] > 0.6f) hueToColorName(currentHSV()[0]) else ""
    }

    fun currentARGB(): IntArray {
        val argb = IntArray(4)
        argb[0] = Color.alpha(this.argb)
        argb[1] = Color.red(this.argb)
        argb[2] = Color.green(this.argb)
        argb[3] = Color.blue(this.argb)
        return argb
    }

    fun next() {
        this.argb = if (ledEnabled.get()) colorSensor.argb() else 0
    }

    fun getDistance(unit: DistanceUnit): Double {
        return if (colorSensorV3 != null) {
            colorSensorV3.getDistance(unit)
        } else {
            distanceSensor!!.getDistance(unit)
        }
    }

    fun enableLed(enable: Boolean) {
        if (!enable) this.argb = 0
        ledEnabled.set(enable)
    }

    fun getLedEnabled(): Boolean {
        return ledEnabled.get()
    }

    companion object {
        private val colorNames: LUT<Float, String> =
            LUT<Float, String>().apply {
                add(0f, "Red")
                add(30f, "Orange")
                add(60f, "Yellow")
                add(90f, "Chartruese")
                add(120f, "Green")
                add(150f, "Spring")
                add(180f, "Cyan")
                add(210f, "Azure")
                add(240f, "Blue")
                add(270f, "Violet")
                add(300f, "Magenta")
                add(330f, "Rose")
            }

        @JvmStatic
        fun hueToColorName(hue: Float): String {
            return colorNames.getClosest(hue) ?: ""
        }
    }
}
