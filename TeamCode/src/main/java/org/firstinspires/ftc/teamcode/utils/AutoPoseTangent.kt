package org.firstinspires.ftc.teamcode.utils

class AutoPoseTangent(
    x: Double,
    y: Double,
    hd: Double,
    @JvmField var td: Double,
) : AutoPose(x, y, hd) {
    fun getTangentRadians(): Double {
        return Math.toRadians(td)
    }
}
