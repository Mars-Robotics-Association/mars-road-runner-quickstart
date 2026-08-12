package org.firstinspires.ftc.teamcode.tuning

import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.FtcRoadRunnerCompat
import org.firstinspires.ftc.teamcode.Localizer
import org.firstinspires.ftc.teamcode.MecanumDrive
import org.firstinspires.ftc.teamcode.TankDrive
import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer
import org.firstinspires.ftc.teamcode.TwoDeadWheelLocalizer

class ManualFeedbackTuner : LinearOpMode() {
    companion object {
        @JvmField var DISTANCE = 64.0
    }

    override fun runOpMode() {
        when (TuningOpModes.DRIVE_CLASS) {
            MecanumDrive::class.java -> {
                val drive = MecanumDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0))
                requireDeadWheelOffsets(drive.localizer)
                waitForStart()
                while (opModeIsActive()) {
                    FtcRoadRunnerCompat.runBlocking(
                        drive
                            .actionBuilder(Pose2d(0.0, 0.0, 0.0))
                            .lineToX(DISTANCE)
                            .lineToX(0.0)
                            .build()
                    )
                }
            }
            TankDrive::class.java -> {
                val drive = TankDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0))
                requireDeadWheelOffsets(drive.localizer)
                waitForStart()
                while (opModeIsActive()) {
                    FtcRoadRunnerCompat.runBlocking(
                        drive
                            .actionBuilder(Pose2d(0.0, 0.0, 0.0))
                            .lineToX(DISTANCE)
                            .lineToX(0.0)
                            .build()
                    )
                }
            }
            else -> error("Unknown TuningOpModes.DRIVE_CLASS: ${TuningOpModes.DRIVE_CLASS}")
        }
    }

    private fun requireDeadWheelOffsets(localizer: Localizer) {
        when (localizer) {
            is TwoDeadWheelLocalizer -> {
                if (
                    TwoDeadWheelLocalizer.PARAMS.perpXTicks == 0.0 &&
                        TwoDeadWheelLocalizer.PARAMS.parYTicks == 0.0
                ) {
                    error("Odometry wheel locations not set! Run AngularRampLogger to tune them.")
                }
            }
            is ThreeDeadWheelLocalizer -> {
                if (
                    ThreeDeadWheelLocalizer.PARAMS.perpXTicks == 0.0 &&
                        ThreeDeadWheelLocalizer.PARAMS.par0YTicks == 0.0 &&
                        ThreeDeadWheelLocalizer.PARAMS.par1YTicks == 1.0
                ) {
                    error("Odometry wheel locations not set! Run AngularRampLogger to tune them.")
                }
            }
        }
    }
}
