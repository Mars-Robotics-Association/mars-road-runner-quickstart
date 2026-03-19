package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.robot.BulkReads;
import org.firstinspires.ftc.teamcode.utils.DashboardTelemetryPacketAccess;
import org.firstinspires.ftc.teamcode.utils.HubHelper;

@Config
public abstract class FlywheelsTuningBase extends LinearOpMode {
    public static class HardwareConfig {
        public boolean MOTORS_COUPLED = true;
        public String[] MOTOR_NAMES = {"flywheel", "flywheel2"};
        public DcMotorSimple.Direction[] MOTOR_DIRECTIONS = {
                DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.REVERSE
        };
    }
    public static HardwareConfig HARDWARE = new HardwareConfig();

    protected DcMotorEx[] motors;
    protected DashboardTelemetryPacketAccess packetAccess;
    protected LynxModule module;
    protected BulkReads bulkReads;

    protected void initHardware() {
        motors = new DcMotorEx[HARDWARE.MOTOR_NAMES.length];
        for (int i = 0; i < HARDWARE.MOTOR_NAMES.length; i++) {
            motors[i] = hardwareMap.get(DcMotorEx.class, HARDWARE.MOTOR_NAMES[i]);
            motors[i].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            motors[i].setDirection(HARDWARE.MOTOR_DIRECTIONS[i]);
            motors[i].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        }

        packetAccess = new DashboardTelemetryPacketAccess();
        telemetry = new MultipleTelemetry(telemetry, packetAccess.dashboardTelemetry);

        module = HubHelper.getHubForMotor(motors[0], hardwareMap);
        bulkReads = new BulkReads(hardwareMap);
    }

    public void stopMotors() {
        for (DcMotorEx m : motors) m.setPower(0);
    }

    @Override
    public final void runOpMode() throws InterruptedException {
        initHardware();
        runOpModeInternal();
    }

    protected abstract void runOpModeInternal() throws InterruptedException;
}
