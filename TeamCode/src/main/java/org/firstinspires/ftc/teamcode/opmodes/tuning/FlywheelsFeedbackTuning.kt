package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.Range
import kotlin.math.abs

/**
 * Interactive tuning opmode for flywheel kP and IIR low-pass filter alpha values.
 *
 * <p>Paste kS and kV values from [FlywheelsFeedforwardTuning] into the static fields below
 * (or edit them live via FTC Dashboard). Then adjust [Params.targetTPS], [Params.targetAlpha],
 * [Params.velocityAlpha], and [Params.kP] while watching the telemetry graphs to find values that
 * track the target velocity quickly without excessive oscillation.
 *
 * <p>Set [Params.useLeftMotor] to `true` for the left motor or `false` for the right
 * motor. The feedforward model is `voltage = kS + kV * smoothedTarget`, and the feedback term
 * is `kP * (smoothedTarget - smoothedVelocity)`.
 */
@Config
@TeleOp(name = "Flywheels Feedback Tuning", group = "Tuning")
class FlywheelsFeedbackTuning : FlywheelsTuningBase() {
    class Params {
        @JvmField var useLeftMotor = true
        @JvmField var targetTPS = 0.0
        @JvmField var targetAlpha = 0.02
        @JvmField var velocityAlpha = 0.05
        @JvmField var kP = 0.002

        @JvmField var leftKS = 1.5109
        @JvmField var leftKV = 0.005178
        @JvmField var rightKS = 1.3725
        @JvmField var rightKV = 0.004908
    }

    companion object {
        @JvmField
        var PARAMS = Params()
    }

    override fun runOpModeInternal() {
        telemetry.addLine("Ready. Press START to begin.")
        telemetry.addLine("Adjust values via FTC Dashboard.")
        telemetry.update()

        waitForStart()
        if (isStopRequested) return

        var smoothedTarget = 0.0
        var smoothedVelocity = 0.0

        while (nextFrame()) {
            // With a single motor, always use index 0. With two motors,
            // useLeftMotor selects which one to tune (switchable via Dashboard).
            val activeIndex = if (PARAMS.useLeftMotor || motors.size == 1) 0 else 1
            val activeMotor = motors[activeIndex]
            val kS = if (PARAMS.useLeftMotor) PARAMS.leftKS else PARAMS.rightKS
            val kV = if (PARAMS.useLeftMotor) PARAMS.leftKV else PARAMS.rightKV

            // Stop any non-active motors. This ensures the previously-active motor
            // is stopped when useLeftMotor is toggled mid-run via Dashboard.
            for (i in motors.indices) {
                if (i != activeIndex) motors[i].power = 0.0
            }

            val rawVelocity = abs(activeMotor.velocity)

            smoothedTarget =
                PARAMS.targetAlpha * PARAMS.targetTPS +
                    (1 - PARAMS.targetAlpha) * smoothedTarget
            // An IIR filter approaches its target asymptotically (never truly arrives).
            // Snap to the exact target once we're within 1% to avoid lingering error.
            if (PARAMS.targetTPS != 0.0 &&
                abs(smoothedTarget - PARAMS.targetTPS) / PARAMS.targetTPS < 0.01
            ) {
                smoothedTarget = PARAMS.targetTPS
            }
            smoothedVelocity =
                PARAMS.velocityAlpha * rawVelocity +
                    (1 - PARAMS.velocityAlpha) * smoothedVelocity

            if (PARAMS.targetTPS == 0.0) {
                activeMotor.power = 0.0
                smoothedTarget = 0.0
                smoothedVelocity = 0.0
            } else {
                val feedforward = kS + kV * smoothedTarget
                val feedback = PARAMS.kP * (smoothedTarget - smoothedVelocity)
                val voltage = feedforward + feedback
                val battV = batteryVoltage()
                // Match ArmSysId: reject near-zero ADC so power is never Inf/NaN.
                activeMotor.power =
                    if (battV > 0.5) Range.clip(voltage / battV, -1.0, 1.0) else 0.0
            }

            telemetry.addData("Motor", if (PARAMS.useLeftMotor) "Left" else "Right")
            telemetry.addData("Raw Target", "%.1f", PARAMS.targetTPS)
            telemetry.addData("Smoothed Target", "%.1f", smoothedTarget)
            telemetry.addData("Raw Velocity", "%.1f", rawVelocity)
            telemetry.addData("Smoothed Velocity", "%.1f", smoothedVelocity)
        }
    }
}
