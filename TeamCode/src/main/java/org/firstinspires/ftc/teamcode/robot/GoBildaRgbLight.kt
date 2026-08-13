package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.marsroboticsassociation.controllib.hardware.GoBildaRgbLight as ControlLibRgbLight

/**
 * Wires ControlLib's goBILDA RGB LED to the `rgb` servo. The library class is `final`, so this is a
 * factory rather than a subclass.
 */
fun GoBildaRgbLight(hardwareMap: HardwareMap): ControlLibRgbLight =
    ControlLibRgbLight(hardwareMap.get(Servo::class.java, "rgb")::setPosition)
