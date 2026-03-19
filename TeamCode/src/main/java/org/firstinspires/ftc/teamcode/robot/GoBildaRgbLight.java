package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/** goBILDA RGB LED light on a servo channel. Wires {@link org.marsroboticsassociation.controllib.hardware.GoBildaRgbLight} to hardware. */
public class GoBildaRgbLight extends org.marsroboticsassociation.controllib.hardware.GoBildaRgbLight {

    public GoBildaRgbLight(@NonNull HardwareMap hardwareMap) {
        super(hardwareMap.get(Servo.class, "rgb")::setPosition);
    }
}
