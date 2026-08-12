package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

/**
 * Interactive tuning opmode for flywheel kP and IIR low-pass filter alpha values.
 *
 * <p>Paste kS and kV values from {@link FlywheelsFeedforwardTuning} into the static fields below
 * (or edit them live via FTC Dashboard). Then adjust {@link #targetTPS}, {@link #targetAlpha},
 * {@link #velocityAlpha}, and {@link #kP} while watching the telemetry graphs to find values that
 * track the target velocity quickly without excessive oscillation.
 *
 * <p>Set {@link #useLeftMotor} to {@code true} for the left motor or {@code false} for the right
 * motor. The feedforward model is {@code voltage = kS + kV * smoothedTarget}, and the feedback term
 * is {@code kP * (smoothedTarget - smoothedVelocity)}.
 */
@Config
@TeleOp(name = "Flywheels Feedback Tuning", group = "Tuning")
public class FlywheelsFeedbackTuning extends FlywheelsTuningBase {

    public static class Params {
        public boolean useLeftMotor = true;
        public double targetTPS = 0;
        public double targetAlpha = 0.02;
        public double velocityAlpha = 0.05;
        public double kP = 0.002;

        public double leftKS = 1.5109;
        public double leftKV = 0.005178;
        public double rightKS = 1.3725;
        public double rightKV = 0.004908;
    }

    public static Params PARAMS = new Params();

    @Override
    protected void runOpModeInternal() throws InterruptedException {
        telemetry.addLine("Ready. Press START to begin.");
        telemetry.addLine("Adjust values via FTC Dashboard.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        double smoothedTarget = 0;
        double smoothedVelocity = 0;

        while (nextFrame()) {
            // With a single motor, always use index 0. With two motors,
            // useLeftMotor selects which one to tune (switchable via Dashboard).
            int activeIndex = (PARAMS.useLeftMotor || motors.length == 1) ? 0 : 1;
            DcMotorEx activeMotor = motors[activeIndex];
            double kS = PARAMS.useLeftMotor ? PARAMS.leftKS : PARAMS.rightKS;
            double kV = PARAMS.useLeftMotor ? PARAMS.leftKV : PARAMS.rightKV;

            // Stop any non-active motors. This ensures the previously-active motor
            // is stopped when useLeftMotor is toggled mid-run via Dashboard.
            for (int i = 0; i < motors.length; i++) {
                if (i != activeIndex) motors[i].setPower(0);
            }

            double rawVelocity = Math.abs(activeMotor.getVelocity());

            smoothedTarget =
                    PARAMS.targetAlpha * PARAMS.targetTPS
                            + (1 - PARAMS.targetAlpha) * smoothedTarget;
            // An IIR filter approaches its target asymptotically (never truly arrives).
            // Snap to the exact target once we're within 1% to avoid lingering error.
            if (PARAMS.targetTPS != 0
                    && Math.abs(smoothedTarget - PARAMS.targetTPS) / PARAMS.targetTPS < 0.01) {
                smoothedTarget = PARAMS.targetTPS;
            }
            smoothedVelocity =
                    PARAMS.velocityAlpha * rawVelocity
                            + (1 - PARAMS.velocityAlpha) * smoothedVelocity;

            if (PARAMS.targetTPS == 0) {
                activeMotor.setPower(0);
                smoothedTarget = 0;
                smoothedVelocity = 0;
            } else {
                double feedforward = kS + kV * smoothedTarget;
                double feedback = PARAMS.kP * (smoothedTarget - smoothedVelocity);
                double voltage = feedforward + feedback;
                double battV = batteryVoltage();
                // Match ArmSysId: reject near-zero ADC so power is never Inf/NaN.
                activeMotor.setPower(battV > 0.5 ? Range.clip(voltage / battV, -1.0, 1.0) : 0.0);
            }

            telemetry.addData("Motor", PARAMS.useLeftMotor ? "Left" : "Right");
            telemetry.addData("Raw Target", "%.1f", PARAMS.targetTPS);
            telemetry.addData("Smoothed Target", "%.1f", smoothedTarget);
            telemetry.addData("Raw Velocity", "%.1f", rawVelocity);
            telemetry.addData("Smoothed Velocity", "%.1f", smoothedVelocity);
        }
    }
}
