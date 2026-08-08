package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.opmodes.base.MarsLinearOpMode;
import org.firstinspires.ftc.teamcode.robot.BulkReads;
import org.firstinspires.ftc.teamcode.utils.HubHelper;

@Config
public abstract class FlywheelsTuningBase extends MarsLinearOpMode {
    public static class HardwareConfig {
        public boolean MOTORS_COUPLED = true;
        public String[] MOTOR_NAMES = {"flywheel", "flywheel2"};
        public DcMotorSimple.Direction[] MOTOR_DIRECTIONS = {
            DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.REVERSE
        };
    }

    public static HardwareConfig HARDWARE = new HardwareConfig();

    protected DcMotorEx[] motors;
    protected LynxModule module;

    /** Same instance as {@link #bulk}; kept for existing subclass call sites. */
    protected BulkReads bulkReads;

    protected void initHardware() {
        initRobot();
        bulkReads = bulk;

        motors = new DcMotorEx[HARDWARE.MOTOR_NAMES.length];
        for (int i = 0; i < HARDWARE.MOTOR_NAMES.length; i++) {
            motors[i] = hardwareMap.get(DcMotorEx.class, HARDWARE.MOTOR_NAMES[i]);
            motors[i].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            motors[i].setDirection(HARDWARE.MOTOR_DIRECTIONS[i]);
            motors[i].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        }

        module = HubHelper.getHubForMotor(motors[0], hardwareMap);
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
