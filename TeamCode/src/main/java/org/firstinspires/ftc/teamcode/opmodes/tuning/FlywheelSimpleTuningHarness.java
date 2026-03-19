package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.robot.BulkReads;
import org.firstinspires.ftc.teamcode.robot.LinkedMotorGroup;
import org.firstinspires.ftc.teamcode.robot.MotorConfig;
import org.firstinspires.ftc.teamcode.robot.QuantizedPowerMotor;
import org.firstinspires.ftc.teamcode.utils.DashboardTelemetryPacketAccess;
import org.marsroboticsassociation.controllib.control.FlywheelSimple;

/**
 * Teleop test harness for tuning {@link FlywheelSimple}.
 *
 * <p>All {@link FlywheelSimple.Params} (kS, kV, kA, kP, velLpfCutoffHz, readyThreshold, etc.)
 * are exposed live in FTC Dashboard under "FlywheelSimple". Change them while the flywheel is
 * spinning and watch the effect on the Dashboard telemetry graphs.
 *
 * <p>Controls:
 * <ul>
 *   <li>Right bumper — spin at {@link #TARGET_TPS}</li>
 *   <li>Left bumper  — coast (stop)</li>
 * </ul>
 *
 * <p>Tuning procedure:
 * <ol>
 *   <li>Run {@code FlywheelsFeedforwardTuning} first to measure kS, kV, and kA, then paste
 *       those values into {@link FlywheelSimple#PARAMS}.</li>
 *   <li>Set {@link #TARGET_TPS} to a target speed in the Dashboard.</li>
 *   <li>Press right bumper. Watch "velocity (smooth)" converge to "setpoint".</li>
 *   <li>If there is steady-state error, increase kP. If the velocity oscillates, decrease kP.</li>
 *   <li>Check {@code isReady}: it should go true within a second or two of spin-up at a good kP.</li>
 * </ol>
 */
@Config
@TeleOp(name = "FlywheelSimple Tuning Harness", group = "Tuning")
public class FlywheelSimpleTuningHarness extends LinearOpMode {

    /** Target flywheel speed in ticks per second. Adjust via FTC Dashboard. */
    public static double TARGET_TPS = 2000.0;

    @Override
    public void runOpMode() {
        // Mirror telemetry to FTC Dashboard for live graph view.
        DashboardTelemetryPacketAccess packetAccess = new DashboardTelemetryPacketAccess();
        telemetry = new MultipleTelemetry(telemetry, packetAccess.dashboardTelemetry);

        var bulkReads = new BulkReads(hardwareMap);

        LinkedMotorGroup group = new LinkedMotorGroup(hardwareMap,
                new MotorConfig("flywheel",  DcMotorSimple.Direction.REVERSE),
                new MotorConfig("flywheel2", DcMotorSimple.Direction.REVERSE));
        group.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        group.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        QuantizedPowerMotor flywheelMotor = new QuantizedPowerMotor(group, 0.01);
        FlywheelSimple flywheel = new FlywheelSimple(telemetry::addData, flywheelMotor);

        telemetry.addLine("Right bumper → spin at TARGET_TPS");
        telemetry.addLine("Left bumper  → coast");
        telemetry.addLine("Edit TARGET_TPS and FlywheelSimple params in FTC Dashboard.");
        telemetry.update();
        waitForStart();

        boolean spinning = false;

        while (opModeIsActive()) {
            bulkReads.readAll();

            if (gamepad1.rightBumperWasPressed()) spinning = true;
            if (gamepad1.leftBumperWasPressed())  spinning = false;

            flywheel.setTps(spinning ? TARGET_TPS : 0);
            flywheel.update();

            flywheel.writeTelemetry();
            telemetry.addData("Spinning",      spinning);
            telemetry.addData("isPowerTooLow", flywheel.isPowerTooLowForTargetVelocity());
            telemetry.update();
        }
    }
}
