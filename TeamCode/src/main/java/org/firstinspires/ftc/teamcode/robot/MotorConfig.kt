package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.robotcore.hardware.DcMotorSimple

/** Pairs a hardware map motor name with its rotation direction. */
data class MotorConfig(val name: String, val direction: DcMotorSimple.Direction)
