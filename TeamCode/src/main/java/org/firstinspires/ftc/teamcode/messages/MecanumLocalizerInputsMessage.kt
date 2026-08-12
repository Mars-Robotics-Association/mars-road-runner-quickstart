package org.firstinspires.ftc.teamcode.messages

import com.acmerobotics.roadrunner.ftc.PositionVelocityPair
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles

class MecanumLocalizerInputsMessage(
    leftFront: PositionVelocityPair,
    leftBack: PositionVelocityPair,
    rightBack: PositionVelocityPair,
    rightFront: PositionVelocityPair,
    angles: YawPitchRollAngles,
) {
    @JvmField var timestamp: Long = System.nanoTime()
    @JvmField var leftFront: PositionVelocityPair = leftFront
    @JvmField var leftBack: PositionVelocityPair = leftBack
    @JvmField var rightBack: PositionVelocityPair = rightBack
    @JvmField var rightFront: PositionVelocityPair = rightFront
    @JvmField var yaw: Double
    @JvmField var pitch: Double
    @JvmField var roll: Double

    init {
        this.yaw = angles.getYaw(AngleUnit.RADIANS)
        this.pitch = angles.getPitch(AngleUnit.RADIANS)
        this.roll = angles.getRoll(AngleUnit.RADIANS)
    }
}
