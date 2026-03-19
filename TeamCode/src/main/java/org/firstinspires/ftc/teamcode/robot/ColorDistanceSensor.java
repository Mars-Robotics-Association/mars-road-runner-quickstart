package org.firstinspires.ftc.teamcode.robot;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.marsroboticsassociation.controllib.util.SetOnChange;
import org.marsroboticsassociation.controllib.util.LUT;

public class ColorDistanceSensor {
    private final RevColorSensorV3 colorSensorV3;
    private final ColorSensor colorSensor;
    private final DistanceSensor distanceSensor;
    private final SetOnChange<Boolean> ledEnabled;
    private int argb;

    private static final LUT<Float, String> colorNames;

    static {
        colorNames = new LUT<>();
        colorNames.add(0f, "Red");
        colorNames.add(30f, "Orange");
        colorNames.add(60f, "Yellow");
        colorNames.add(90f, "Chartruese");
        colorNames.add(120f, "Green");
        colorNames.add(150f, "Spring");
        colorNames.add(180f, "Cyan");
        colorNames.add(210f, "Azure");
        colorNames.add(240f, "Blue");
        colorNames.add(270f, "Violet");
        colorNames.add(300f, "Magenta");
        colorNames.add(330f, "Rose");
    }

    public ColorDistanceSensor(boolean revV3, HardwareMap hardwareMap, String name) {
        if (revV3) {
            colorSensorV3 = (RevColorSensorV3) hardwareMap.colorSensor.get(name);
            colorSensor = colorSensorV3;
            distanceSensor = null;
        } else {
            colorSensorV3 = null;
            colorSensor = hardwareMap.colorSensor.get(name);
            distanceSensor = hardwareMap.get(DistanceSensor.class, name);
        }
        ledEnabled = SetOnChange.of(true, colorSensor::enableLed);
        argb = colorSensor.argb();
        ledEnabled.set(false);
    }

    public float[] currentHSV() {
        float[] hsv = new float[3];
        Color.colorToHSV(this.argb, hsv);
        return hsv;
    }

    public String currentColorName() {
        return currentHSV()[1] > 0.6f ? hueToColorName(currentHSV()[0]) : "";
    }

    public static String hueToColorName(float hue) {
        return colorNames.getClosest(hue);
    }

    public int[] currentARGB() {
        int[] argb = new int[4];
        argb[0] = Color.alpha(this.argb);
        argb[1] = Color.red(this.argb);
        argb[2] = Color.green(this.argb);
        argb[3] = Color.blue(this.argb);
        return argb;
    }

    public void next() {
        this.argb = ledEnabled.get() ? colorSensor.argb() : 0;
    }

    public double getDistance(DistanceUnit unit) {
        if (colorSensorV3 != null) {
            return colorSensorV3.getDistance(unit);
        } else {
            return distanceSensor.getDistance(unit);
        }
    }

    public void enableLed(boolean enable) {
        if (!enable) this.argb = 0;
        ledEnabled.set(enable);
    }

    public boolean getLedEnabled() {
        return ledEnabled.get();
    }
}
