package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.toOTOSPose
import com.acmerobotics.roadrunner.ftc.toRRPose
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

@Config
class OTOSLocalizer(
    hardwareMap: HardwareMap,
    initialPose: Pose2d,
) : Localizer {
    class Params {
        @JvmField var angularScalar = 1.0
        @JvmField var linearScalar = 1.0

        // Note: units are in inches and radians
        @JvmField var offset = SparkFunOTOS.Pose2D(0.0, 0.0, 0.0)
    }

    companion object {
        @JvmField
        var PARAMS = Params()
    }

    @JvmField
    val otos: SparkFunOTOS

    private var currentPose: Pose2d

    init {
        // TODO: make sure your config has an OTOS device with this name
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        otos = hardwareMap.get(SparkFunOTOS::class.java, "sensor_otos")
        currentPose = initialPose
        otos.setPosition(currentPose.toOTOSPose())
        otos.setLinearUnit(DistanceUnit.INCH)
        otos.setAngularUnit(AngleUnit.RADIANS)

        otos.calibrateImu()
        otos.setLinearScalar(PARAMS.linearScalar)
        otos.setAngularScalar(PARAMS.angularScalar)
        otos.setOffset(PARAMS.offset)
    }

    override fun getPose(): Pose2d {
        return currentPose
    }

    override fun setPose(pose: Pose2d) {
        currentPose = pose
        otos.setPosition(currentPose.toOTOSPose())
    }

    override fun update(): PoseVelocity2d {
        val otosPose = SparkFunOTOS.Pose2D()
        val otosVel = SparkFunOTOS.Pose2D()
        val otosAcc = SparkFunOTOS.Pose2D()
        otos.getPosVelAcc(otosPose, otosVel, otosAcc)

        currentPose = otosPose.toRRPose()
        val fieldVel = Vector2d(otosVel.x, otosVel.y)
        val robotVel = Rotation2d.exp(otosPose.h).inverse().times(fieldVel)
        return PoseVelocity2d(robotVel, otosVel.h)
    }
}
