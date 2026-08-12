package org.firstinspires.ftc.teamcode.messages

import com.acmerobotics.roadrunner.ftc.PositionVelocityPair
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles

class TwoDeadWheelInputsMessage(
    par: PositionVelocityPair,
    perp: PositionVelocityPair,
    angles: YawPitchRollAngles,
    angularVelocity: AngularVelocity,
) {
    @JvmField var timestamp: Long = System.nanoTime()
    @JvmField var par: PositionVelocityPair = par
    @JvmField var perp: PositionVelocityPair = perp
    @JvmField var yaw: Double
    @JvmField var pitch: Double
    @JvmField var roll: Double
    @JvmField var xRotationRate: Double
    @JvmField var yRotationRate: Double
    @JvmField var zRotationRate: Double

    init {
        this.yaw = angles.getYaw(AngleUnit.RADIANS)
        this.pitch = angles.getPitch(AngleUnit.RADIANS)
        this.roll = angles.getRoll(AngleUnit.RADIANS)
        this.xRotationRate = angularVelocity.xRotationRate.toDouble()
        this.yRotationRate = angularVelocity.yRotationRate.toDouble()
        this.zRotationRate = angularVelocity.zRotationRate.toDouble()
    }
}
