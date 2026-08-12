package org.firstinspires.ftc.teamcode.messages

import com.acmerobotics.roadrunner.ftc.PositionVelocityPair

class ThreeDeadWheelInputsMessage(
    par0: PositionVelocityPair,
    par1: PositionVelocityPair,
    perp: PositionVelocityPair,
) {
    @JvmField var timestamp: Long = System.nanoTime()
    @JvmField var par0: PositionVelocityPair = par0
    @JvmField var par1: PositionVelocityPair = par1
    @JvmField var perp: PositionVelocityPair = perp
}
