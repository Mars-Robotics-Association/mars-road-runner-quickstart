package org.firstinspires.ftc.teamcode.messages

class MecanumCommandMessage(
    voltage: Double,
    leftFrontPower: Double,
    leftBackPower: Double,
    rightBackPower: Double,
    rightFrontPower: Double,
) {
    @JvmField var timestamp: Long = System.nanoTime()
    @JvmField var voltage: Double = voltage
    @JvmField var leftFrontPower: Double = leftFrontPower
    @JvmField var leftBackPower: Double = leftBackPower
    @JvmField var rightBackPower: Double = rightBackPower
    @JvmField var rightFrontPower: Double = rightFrontPower
}
