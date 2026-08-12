package org.firstinspires.ftc.teamcode.messages

class TankCommandMessage(voltage: Double, leftPower: Double, rightPower: Double) {
    @JvmField var timestamp: Long = System.nanoTime()
    @JvmField var voltage: Double = voltage
    @JvmField var leftPower: Double = leftPower
    @JvmField var rightPower: Double = rightPower
}
