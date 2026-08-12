package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit
import java.util.Objects

@Config
class PinpointLocalizer(
    hardwareMap: HardwareMap,
    inPerTick: Double,
    initialPose: Pose2d,
) : Localizer {
    class Params {
        @JvmField var parYTicks = 0.0 // y position of the parallel encoder (in tick units)
        @JvmField var perpXTicks = 0.0 // x position of the perpendicular encoder (in tick units)
    }

    companion object {
        @JvmField
        var PARAMS = Params()
    }

    @JvmField
    val driver: GoBildaPinpointDriver

    @JvmField
    val initialParDirection: GoBildaPinpointDriver.EncoderDirection

    @JvmField
    val initialPerpDirection: GoBildaPinpointDriver.EncoderDirection

    private var txWorldPinpoint: Pose2d
    private var txPinpointRobot = Pose2d(0.0, 0.0, 0.0)

    init {
        // TODO: make sure your config has a Pinpoint device with this name
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        driver = hardwareMap.get(GoBildaPinpointDriver::class.java, "pinpoint")

        val mmPerTick = inPerTick * 25.4
        driver.setEncoderResolution(1 / mmPerTick, DistanceUnit.MM)
        driver.setOffsets(mmPerTick * PARAMS.parYTicks, mmPerTick * PARAMS.perpXTicks, DistanceUnit.MM)

        // TODO: reverse encoder directions if needed
        initialParDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD
        initialPerpDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD

        driver.setEncoderDirections(initialParDirection, initialPerpDirection)

        driver.resetPosAndIMU()

        txWorldPinpoint = initialPose
    }

    override fun setPose(pose: Pose2d) {
        txWorldPinpoint = pose.times(txPinpointRobot.inverse())
    }

    override fun getPose(): Pose2d {
        return txWorldPinpoint.times(txPinpointRobot)
    }

    override fun update(): PoseVelocity2d {
        driver.update()
        if (Objects.requireNonNull(driver.deviceStatus) == GoBildaPinpointDriver.DeviceStatus.READY) {
            txPinpointRobot = Pose2d(
                driver.getPosX(DistanceUnit.INCH),
                driver.getPosY(DistanceUnit.INCH),
                driver.getHeading(UnnormalizedAngleUnit.RADIANS),
            )
            val worldVelocity = Vector2d(driver.getVelX(DistanceUnit.INCH), driver.getVelY(DistanceUnit.INCH))
            val robotVelocity = Rotation2d.fromDouble(-txPinpointRobot.heading.log()).times(worldVelocity)

            return PoseVelocity2d(robotVelocity, driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS))
        }
        return PoseVelocity2d(Vector2d(0.0, 0.0), 0.0)
    }
}
