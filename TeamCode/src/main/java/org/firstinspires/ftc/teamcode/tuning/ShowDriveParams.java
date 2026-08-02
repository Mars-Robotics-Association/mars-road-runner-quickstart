package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.OTOSLocalizer;
import org.firstinspires.ftc.teamcode.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.TwoDeadWheelLocalizer;

/**
 * Read-only dump of the live drive / localizer {@code PARAMS} statics — one place to copy values
 * after chaining automatic tuners in a single RC process.
 *
 * <p>Does not touch hardware. Values are whatever is in memory now: source defaults after a cold
 * start, or session writes from the automatic tuners if you ran them without restarting. Paste into
 * the corresponding {@code Params} classes before restart or redeploy to keep them.
 *
 * <p>Registered under the Driver Station <b>Utility</b> menu (SDK 11.2+). Drive class follows
 * {@link TuningOpModes#DRIVE_CLASS}; localizer offsets for Pinpoint / dead wheels / OTOS are always
 * shown so mixed setups still have one copy page.
 */
@Utility(
        name = "Show Drive Params",
        description = "Dump live PARAMS statics for pasting into source after sysid")
public final class ShowDriveParams extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addLine("Live PARAMS (this RC process). Paste into source to keep.");
            telemetry.addLine("kV / kA shown as scientific notation for tick-unit pasting.");
            telemetry.addLine();

            if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
                dumpMecanum();
            } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
                dumpTank();
            } else {
                telemetry.addLine(
                        "Unknown TuningOpModes.DRIVE_CLASS: " + TuningOpModes.DRIVE_CLASS);
            }

            telemetry.addLine();
            telemetry.addLine("--- Localizer offsets (always shown) ---");
            dumpLocalizers();

            telemetry.update();
        }
    }

    private void dumpMecanum() {
        MecanumDrive.Params p = MecanumDrive.PARAMS;
        telemetry.addLine("=== MecanumDrive.Params ===");
        addFixed("inPerTick", p.inPerTick);
        addFixed("lateralInPerTick", p.lateralInPerTick);
        addFixed("trackWidthTicks", p.trackWidthTicks);
        telemetry.addLine();
        telemetry.addLine("-- axial feedforward (tick units) --");
        addFixed("kS", p.kS);
        addSci("kV", p.kV);
        addSci("kA", p.kA);
        telemetry.addLine();
        telemetry.addLine("-- anisotropic lateral --");
        telemetry.addData("useAnisotropicFeedforward", p.useAnisotropicFeedforward);
        addFixed("lateralKS", p.lateralKS);
        addSci("lateralKV", p.lateralKV);
        addSci("lateralKA", p.lateralKA);
        telemetry.addLine();
        telemetry.addLine("-- yaw coupling (V, V/(in/s)) --");
        addFixed("yawCouplingKsAxial", p.yawCouplingKsAxial);
        addSci("yawCouplingKvAxial", p.yawCouplingKvAxial);
        addFixed("yawCouplingKsLateral", p.yawCouplingKsLateral);
        addSci("yawCouplingKvLateral", p.yawCouplingKvLateral);
        telemetry.addLine();
        telemetry.addLine("-- feedback gains --");
        addGain("axialGain", p.axialGain);
        addGain("lateralGain", p.lateralGain);
        addGain("headingGain", p.headingGain);
        addGain("axialVelGain", p.axialVelGain);
        addGain("lateralVelGain", p.lateralVelGain);
        addGain("headingVelGain", p.headingVelGain);
        telemetry.addLine();
        telemetry.addLine("-- planning (not from sysid; for reference) --");
        telemetry.addData("useWheelVoltageConstraint", p.useWheelVoltageConstraint);
        addFixed("maxVoltageForPlanning", p.maxVoltageForPlanning);
        addFixed("cruiseFraction", p.cruiseFraction);
        addFixed("maxCentripetalAccel", p.maxCentripetalAccel);
        addFixed("maxWheelVel", p.maxWheelVel);
    }

    private void dumpTank() {
        TankDrive.Params p = TankDrive.PARAMS;
        telemetry.addLine("=== TankDrive.Params ===");
        addFixed("inPerTick", p.inPerTick);
        addFixed("trackWidthTicks", p.trackWidthTicks);
        telemetry.addLine();
        telemetry.addLine("-- feedforward (tick units) --");
        addFixed("kS", p.kS);
        addSci("kV", p.kV);
        addSci("kA", p.kA);
        telemetry.addLine();
        telemetry.addLine("-- yaw coupling (V, V/(in/s)) --");
        addFixed("yawCouplingKsAxial", p.yawCouplingKsAxial);
        addSci("yawCouplingKvAxial", p.yawCouplingKvAxial);
        telemetry.addLine();
        telemetry.addLine("-- turn gains --");
        addGain("turnGain", p.turnGain);
        addGain("turnVelGain", p.turnVelGain);
        telemetry.addLine();
        telemetry.addLine("-- planning (not from sysid; for reference) --");
        telemetry.addData("useWheelVoltageConstraint", p.useWheelVoltageConstraint);
        addFixed("maxVoltageForPlanning", p.maxVoltageForPlanning);
        addFixed("cruiseFraction", p.cruiseFraction);
        addFixed("maxCentripetalAccel", p.maxCentripetalAccel);
        addFixed("maxWheelVel", p.maxWheelVel);
        addGain("ramseteZeta", p.ramseteZeta);
        addGain("ramseteBBar", p.ramseteBBar);
    }

    private void dumpLocalizers() {
        PinpointLocalizer.Params pin = PinpointLocalizer.PARAMS;
        telemetry.addLine("PinpointLocalizer.Params");
        addFixed("parYTicks", pin.parYTicks);
        addFixed("perpXTicks", pin.perpXTicks);

        TwoDeadWheelLocalizer.Params two = TwoDeadWheelLocalizer.PARAMS;
        telemetry.addLine("TwoDeadWheelLocalizer.Params");
        addFixed("parYTicks", two.parYTicks);
        addFixed("perpXTicks", two.perpXTicks);

        ThreeDeadWheelLocalizer.Params three = ThreeDeadWheelLocalizer.PARAMS;
        telemetry.addLine("ThreeDeadWheelLocalizer.Params");
        addFixed("par0YTicks", three.par0YTicks);
        addFixed("par1YTicks", three.par1YTicks);
        addFixed("perpXTicks", three.perpXTicks);

        OTOSLocalizer.Params otos = OTOSLocalizer.PARAMS;
        telemetry.addLine("OTOSLocalizer.Params");
        addFixed("angularScalar", otos.angularScalar);
        addFixed("linearScalar", otos.linearScalar);
    }

    private void addFixed(String name, double v) {
        telemetry.addData(name, "%.5f", v);
    }

    private void addSci(String name, double v) {
        telemetry.addData(name, "%.5e", v);
    }

    private void addGain(String name, double v) {
        telemetry.addData(name, "%.2f", v);
    }
}
