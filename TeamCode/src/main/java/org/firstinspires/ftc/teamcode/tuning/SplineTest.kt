package org.firstinspires.ftc.teamcode.tuning

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import kotlin.math.PI
import org.firstinspires.ftc.teamcode.FtcRoadRunnerCompat
import org.firstinspires.ftc.teamcode.MecanumDrive
import org.firstinspires.ftc.teamcode.TankDrive

class SplineTest : LinearOpMode() {
    override fun runOpMode() {
        val beginPose = Pose2d(0.0, 0.0, 0.0)
        when (TuningOpModes.DRIVE_CLASS) {
            MecanumDrive::class.java -> {
                val drive = MecanumDrive(hardwareMap, beginPose)
                waitForStart()
                FtcRoadRunnerCompat.runBlocking(
                    drive
                        .actionBuilder(beginPose)
                        .splineTo(Vector2d(30.0, 30.0), PI / 2)
                        .splineTo(Vector2d(0.0, 60.0), PI)
                        .build()
                )
            }
            TankDrive::class.java -> {
                val drive = TankDrive(hardwareMap, beginPose)
                waitForStart()
                FtcRoadRunnerCompat.runBlocking(
                    drive
                        .actionBuilder(beginPose)
                        .splineTo(Vector2d(30.0, 30.0), PI / 2)
                        .splineTo(Vector2d(0.0, 60.0), PI)
                        .build()
                )
            }
            else -> error("Unknown TuningOpModes.DRIVE_CLASS: ${TuningOpModes.DRIVE_CLASS}")
        }
    }
}
