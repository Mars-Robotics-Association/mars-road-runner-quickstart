package org.firstinspires.ftc.teamcode.utils

import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.hardware.Gamepad

class RoadRunnerTeleOpDrive {
    companion object {
        /**
         * @param fieldCentric true means keep the field as the frame of reference, even as the robot turns. AKA "headless" mode
         * @param lateralSpeed also known as strafe speed. Range [-1,1]. Positive is right
         * @param axialSpeed   also known as forward speed. Range [-1,1]. Negative is backwards
         * @param turnSpeed    Range [-1,1]. Positive is turn right (clockwise)
         * @param speedFactor  A scaling factor applied to all the other speeds. Useful for slowing down to drive more precisely, like "crouch" in a FPS game
         * @param heading      the heading from the RR localizer
         * @return a PoseVelocity2d that represents velocities in the weird RR coordinate system. Ready to pass directly into setDrivePowers
         */
        @JvmStatic
        fun toDrivePowers(
            fieldCentric: Boolean,
            lateralSpeed: Double,
            axialSpeed: Double,
            turnSpeed: Double,
            speedFactor: Double,
            heading: Rotation2d,
        ): PoseVelocity2d {
            var input = Vector2d(
                -axialSpeed,
                -lateralSpeed,
            ).times(speedFactor)
            if (fieldCentric) {
                // https://github.com/acmerobotics/road-runner-quickstart/issues/246
                input = heading.inverse().times(input)
            }

            return PoseVelocity2d(
                input,
                -turnSpeed * speedFactor,
            )
        }

        /**
         * @param fieldCentric true means keep the field as the frame of reference, even as the robot turns. AKA "headless" mode
         * @param gamepad      this applies a standard Halo style mapping to the gamepad. Pass in whichever gamepad the chassis operator will be using.
         * @param crouchSpeed  a speed scaling factor that will be applied when left stick button is pressed
         * @param heading      the heading from the RR localizer
         * @return a PoseVelocity2d that represents velocities in the weird RR coordinate system. Ready to pass directly into setDrivePowers
         */
        @JvmStatic
        fun toDrivePowers(
            fieldCentric: Boolean,
            gamepad: Gamepad,
            crouchSpeed: Double,
            heading: Rotation2d,
            forceCrouch: Boolean,
        ): PoseVelocity2d {
            return toDrivePowers(
                fieldCentric,
                gamepad.left_stick_x.toDouble(),
                gamepad.left_stick_y.toDouble(),
                gamepad.right_stick_x.toDouble(),
                if (gamepad.left_stick_button || forceCrouch) crouchSpeed else 1.0, // analogous to "crouch" in Halo
                heading,
            )
        }
    }
}
