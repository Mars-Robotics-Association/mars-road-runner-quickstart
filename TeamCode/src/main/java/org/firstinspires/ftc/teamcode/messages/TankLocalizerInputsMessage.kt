package org.firstinspires.ftc.teamcode.messages

import com.acmerobotics.roadrunner.ftc.PositionVelocityPair

class TankLocalizerInputsMessage(
    left: List<PositionVelocityPair>,
    right: List<PositionVelocityPair>,
) {
    @JvmField var timestamp: Long = System.nanoTime()
    @JvmField var left: Array<PositionVelocityPair> = left.toTypedArray()
    @JvmField var right: Array<PositionVelocityPair> = right.toTypedArray()
}
