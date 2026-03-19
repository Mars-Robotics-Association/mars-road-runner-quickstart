package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

/** Pairs a hardware map motor name with its rotation direction. */
public record MotorConfig(String name, DcMotorSimple.Direction direction) {}
