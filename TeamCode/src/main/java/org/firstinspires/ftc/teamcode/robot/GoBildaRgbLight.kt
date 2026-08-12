package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

/** goBILDA RGB LED light on a servo channel. Wires [org.marsroboticsassociation.controllib.hardware.GoBildaRgbLight] to hardware. */
class GoBildaRgbLight(hardwareMap: HardwareMap) :
    org.marsroboticsassociation.controllib.hardware.GoBildaRgbLight(
        hardwareMap.get(Servo::class.java, "rgb")::setPosition
    )
