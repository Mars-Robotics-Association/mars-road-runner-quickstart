package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LazyHardwareMapImu;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.messages.DriveCommandMessage;
import org.firstinspires.ftc.teamcode.messages.MecanumCommandMessage;
import org.firstinspires.ftc.teamcode.messages.MecanumLocalizerInputsMessage;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

@Config
public final class MecanumDrive {
    public static class Params {
        // IMU orientation
        // TODO: fill in these values based on
        //   see
        // https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html?highlight=imu#physical-hub-mounting
        public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        // drive model parameters
        // Linear scale for the drive model (kV/kA, trackWidthTicks, kinematics).
        // Also the odometry scale passed into Pinpoint / dead-wheel localizers.
        // Starting values by localizer:
        //   Pinpoint / dead wheels (goBILDA 48mm pods, 2000 ticks/rev):
        //     (48 / 25.4 * Math.PI) / 2000.0
        //   OTOS (reports inches natively): 1.0
        //   Drive encoders: ForwardPushTest (or compute from wheel diameter / gearing)
        public double inPerTick = 1;
        public double lateralInPerTick = inPerTick;
        // Tape track width (in) / inPerTick. TrackWidthTuner seeds/refines this when 0.
        public double trackWidthTicks = 0;

        // feedforward parameters (in tick units)
        public double kS = 0;
        public double kV = 0;
        public double kA = 0;

        // Anisotropic (axial vs. lateral) feedforward. Mecanum rollers scrub when strafing, which
        // raises the effective kS/kV/kA relative to forward/rotational motion. Leave
        // useAnisotropicFeedforward false for stock (isotropic) behavior; when true, the lateral*
        // constants (in tick units, like kS/kV/kA) are applied to the strafe component of each
        // wheel.
        public boolean useAnisotropicFeedforward = false;
        public double lateralKS = 0;
        public double lateralKV = 0;
        public double lateralKA = 0;

        // Yaw-coupling feedforward. Cancels the parasitic yaw moment (curl) produced by chassis
        // translation under open-loop driving. Axial constants come from a forward ramp, lateral
        // from a strafe ramp. Units are volts (kS*) and volts per inch/s (kV*). Zero = disabled.
        public double yawCouplingKsAxial = 0;
        public double yawCouplingKvAxial = 0;
        public double yawCouplingKsLateral = 0;
        public double yawCouplingKvLateral = 0;

        // path profile parameters (in inches)
        public double maxWheelVel = 50;
        public double minProfileAccel = -30;
        public double maxProfileAccel = 50;

        // Voltage-budget path constraint (back-EMF/traction aware). When enabled, path velocity and
        // acceleration are limited by keeping every wheel's feedforward voltage within the budget
        // instead of by the fixed maxWheelVel / profile-accel caps above. Requires calibrated kV,
        // kA.
        public boolean useWheelVoltageConstraint = false;
        public double maxVoltageForPlanning = 11.0; // plan below the 12 V nominal to leave headroom
        public double cruiseFraction = 0.95; // budget spent cruising; remainder reserved for accel

        // Centripetal (cornering) acceleration limit, in in/s^2. Caps speed through curves to keep
        // the wheels from slipping sideways. Zero = disabled.
        public double maxCentripetalAccel = 0;

        // turn profile parameters (in radians)
        public double maxAngVel = java.lang.Math.PI; // shared with path
        public double maxAngAccel = java.lang.Math.PI;

        // path controller gains
        public double axialGain = 0.0;
        public double lateralGain = 0.0;
        public double headingGain = 0.0; // shared with turn

        public double axialVelGain = 0.0;
        public double lateralVelGain = 0.0;
        public double headingVelGain = 0.0; // shared with turn
    }

    public static Params PARAMS = new Params();

    public final MecanumKinematics kinematics =
            new MecanumKinematics(
                    PARAMS.inPerTick * PARAMS.trackWidthTicks,
                    PARAMS.inPerTick / PARAMS.lateralInPerTick);

    public final TurnConstraints defaultTurnConstraints =
            new TurnConstraints(PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);
    // A single WheelVoltageConstraint instance serves as both the velocity and the acceleration
    // constraint when PARAMS.useWheelVoltageConstraint is set; null otherwise. It must be declared
    // (and initialized) before the two constraint fields that reference it below.
    private final MecanumKinematics.WheelVoltageConstraint wheelVoltageConstraint =
            PARAMS.useWheelVoltageConstraint ? makeWheelVoltageConstraint() : null;

    public final VelConstraint defaultVelConstraint = makeDefaultVelConstraint();
    public final AccelConstraint defaultAccelConstraint =
            wheelVoltageConstraint != null
                    ? wheelVoltageConstraint
                    : new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);

    private MecanumKinematics.WheelVoltageConstraint makeWheelVoltageConstraint() {
        MotorFeedforward axial =
                new MotorFeedforward(
                        PARAMS.kS, PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
        MotorFeedforward lateral =
                PARAMS.useAnisotropicFeedforward
                        ? new MotorFeedforward(
                                PARAMS.lateralKS,
                                PARAMS.lateralKV / PARAMS.inPerTick,
                                PARAMS.lateralKA / PARAMS.inPerTick)
                        : axial;
        YawCouplingFeedforward yawCoupling =
                new YawCouplingFeedforward(
                        PARAMS.yawCouplingKsAxial, PARAMS.yawCouplingKvAxial,
                        PARAMS.yawCouplingKsLateral, PARAMS.yawCouplingKvLateral);
        return kinematics
        .new WheelVoltageConstraint(
                new AnisotropicMotorFeedforward(axial, lateral),
                yawCoupling,
                PARAMS.maxVoltageForPlanning,
                PARAMS.cruiseFraction);
    }

    private VelConstraint makeDefaultVelConstraint() {
        List<VelConstraint> constraints = new ArrayList<>();
        if (wheelVoltageConstraint != null) {
            // the voltage constraint's velocity limit subsumes the fixed wheel-velocity cap
            constraints.add(wheelVoltageConstraint);
        } else {
            constraints.add(kinematics.new WheelVelConstraint(PARAMS.maxWheelVel));
        }
        constraints.add(new AngularVelConstraint(PARAMS.maxAngVel));
        if (PARAMS.maxCentripetalAccel > 0) {
            constraints.add(new CentripetalAccelVelConstraint(PARAMS.maxCentripetalAccel));
        }
        return new MinVelConstraint(constraints);
    }

    public final DcMotorEx leftFront, leftBack, rightBack, rightFront;

    public final VoltageSensor voltageSensor;

    public final LazyImu lazyImu;

    public final Localizer localizer;
    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    private final DownsampledWriter estimatedPoseWriter =
            new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    private final DownsampledWriter targetPoseWriter =
            new DownsampledWriter("TARGET_POSE", 50_000_000);
    private final DownsampledWriter driveCommandWriter =
            new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
    private final DownsampledWriter mecanumCommandWriter =
            new DownsampledWriter("MECANUM_COMMAND", 50_000_000);

    public class DriveLocalizer implements Localizer {
        public final Encoder leftFront, leftBack, rightBack, rightFront;
        public final IMU imu;

        private int lastLeftFrontPos, lastLeftBackPos, lastRightBackPos, lastRightFrontPos;
        private Rotation2d lastHeading;
        private boolean initialized;
        private Pose2d pose;

        public DriveLocalizer(Pose2d pose) {
            leftFront = new OverflowEncoder(new RawEncoder(MecanumDrive.this.leftFront));
            leftBack = new OverflowEncoder(new RawEncoder(MecanumDrive.this.leftBack));
            rightBack = new OverflowEncoder(new RawEncoder(MecanumDrive.this.rightBack));
            rightFront = new OverflowEncoder(new RawEncoder(MecanumDrive.this.rightFront));

            imu = lazyImu.get();

            // TODO: reverse encoders if needed
            //   leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

            this.pose = pose;
        }

        @Override
        public void setPose(Pose2d pose) {
            this.pose = pose;
        }

        @Override
        public Pose2d getPose() {
            return pose;
        }

        @Override
        public PoseVelocity2d update() {
            PositionVelocityPair leftFrontPosVel = leftFront.getPositionAndVelocity();
            PositionVelocityPair leftBackPosVel = leftBack.getPositionAndVelocity();
            PositionVelocityPair rightBackPosVel = rightBack.getPositionAndVelocity();
            PositionVelocityPair rightFrontPosVel = rightFront.getPositionAndVelocity();

            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();

            FlightRecorder.write(
                    "MECANUM_LOCALIZER_INPUTS",
                    new MecanumLocalizerInputsMessage(
                            leftFrontPosVel,
                            leftBackPosVel,
                            rightBackPosVel,
                            rightFrontPosVel,
                            angles));

            Rotation2d heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS));

            if (!initialized) {
                initialized = true;

                lastLeftFrontPos = leftFrontPosVel.position;
                lastLeftBackPos = leftBackPosVel.position;
                lastRightBackPos = rightBackPosVel.position;
                lastRightFrontPos = rightFrontPosVel.position;

                lastHeading = heading;

                return new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0);
            }

            double headingDelta = heading.minus(lastHeading);
            Twist2dDual<Time> twist =
                    kinematics.forward(
                            new MecanumKinematics.WheelIncrements<>(
                                    new DualNum<Time>(
                                                    new double[] {
                                                        (leftFrontPosVel.position
                                                                - lastLeftFrontPos),
                                                        leftFrontPosVel.velocity,
                                                    })
                                            .times(PARAMS.inPerTick),
                                    new DualNum<Time>(
                                                    new double[] {
                                                        (leftBackPosVel.position - lastLeftBackPos),
                                                        leftBackPosVel.velocity,
                                                    })
                                            .times(PARAMS.inPerTick),
                                    new DualNum<Time>(
                                                    new double[] {
                                                        (rightBackPosVel.position
                                                                - lastRightBackPos),
                                                        rightBackPosVel.velocity,
                                                    })
                                            .times(PARAMS.inPerTick),
                                    new DualNum<Time>(
                                                    new double[] {
                                                        (rightFrontPosVel.position
                                                                - lastRightFrontPos),
                                                        rightFrontPosVel.velocity,
                                                    })
                                            .times(PARAMS.inPerTick)));

            lastLeftFrontPos = leftFrontPosVel.position;
            lastLeftBackPos = leftBackPosVel.position;
            lastRightBackPos = rightBackPosVel.position;
            lastRightFrontPos = rightFrontPosVel.position;

            lastHeading = heading;

            pose = pose.plus(new Twist2d(twist.line.value(), headingDelta));

            return twist.velocity().value();
        }
    }

    public MecanumDrive(HardwareMap hardwareMap, Pose2d pose) {
        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: make sure your config has motors with these names (or change them)
        //   see
        // https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TODO: reverse motor directions if needed
        //   leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO: make sure your config has an IMU with this name (can be BNO or BHI)
        //   see
        // https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        lazyImu =
                new LazyHardwareMapImu(
                        hardwareMap,
                        "imu",
                        new RevHubOrientationOnRobot(
                                PARAMS.logoFacingDirection, PARAMS.usbFacingDirection));

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        localizer = new DriveLocalizer(pose);

        FlightRecorder.write("MECANUM_PARAMS", PARAMS);
    }

    public void setDrivePowers(PoseVelocity2d powers) {
        MecanumKinematics.WheelVelocities<Time> wheelVels =
                new MecanumKinematics(1).inverse(PoseVelocity2dDual.constant(powers, 1));

        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = java.lang.Math.max(maxPowerMag, power.value());
        }

        leftFront.setPower(wheelVels.leftFront.get(0) / maxPowerMag);
        leftBack.setPower(wheelVels.leftBack.get(0) / maxPowerMag);
        rightBack.setPower(wheelVels.rightBack.get(0) / maxPowerMag);
        rightFront.setPower(wheelVels.rightFront.get(0) / maxPowerMag);
    }

    /**
     * Applies a follower velocity/acceleration command to the wheels through the full production
     * feedforward path: anisotropic constants (when enabled), yaw-coupling voltages, and battery
     * voltage compensation. Shared by {@link FollowTrajectoryAction} and the feedback-gain tuner so
     * gain tests exercise exactly the voltages the follower applies.
     */
    public void setDriveCommand(PoseVelocity2dDual<Time> command) {
        MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
        double voltage = voltageSensor.getVoltage();

        // Base feedforward voltage per wheel. With useAnisotropicFeedforward, each wheel's strafe
        // (lateral) velocity component is fed the separately-calibrated lateral constants;
        // otherwise
        // one set of constants is applied to the total wheel velocity (stock quickstart behavior).
        double leftFrontFF, leftBackFF, rightBackFF, rightFrontFF;
        if (PARAMS.useAnisotropicFeedforward) {
            AnisotropicMotorFeedforward feedforward =
                    new AnisotropicMotorFeedforward(
                            new MotorFeedforward(
                                    PARAMS.kS,
                                    PARAMS.kV / PARAMS.inPerTick,
                                    PARAMS.kA / PARAMS.inPerTick),
                            new MotorFeedforward(
                                    PARAMS.lateralKS,
                                    PARAMS.lateralKV / PARAMS.inPerTick,
                                    PARAMS.lateralKA / PARAMS.inPerTick));
            MecanumKinematics.WheelVelocityComponents<Time> components =
                    kinematics.inverseComponents(command);
            leftFrontFF =
                    feedforward.compute(components.axial.leftFront, components.lateral.leftFront);
            leftBackFF =
                    feedforward.compute(components.axial.leftBack, components.lateral.leftBack);
            rightBackFF =
                    feedforward.compute(components.axial.rightBack, components.lateral.rightBack);
            rightFrontFF =
                    feedforward.compute(components.axial.rightFront, components.lateral.rightFront);
        } else {
            final MotorFeedforward feedforward =
                    new MotorFeedforward(
                            PARAMS.kS, PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            leftFrontFF = feedforward.compute(wheelVels.leftFront);
            leftBackFF = feedforward.compute(wheelVels.leftBack);
            rightBackFF = feedforward.compute(wheelVels.rightBack);
            rightFrontFF = feedforward.compute(wheelVels.rightFront);
        }

        // Yaw-coupling feedforward: a per-wheel voltage that cancels the parasitic yaw from chassis
        // translation. Entries follow wheel order (leftFront, leftBack, rightBack, rightFront).
        // Zero constants (the default) make this a no-op.
        YawCouplingFeedforward yawCoupling =
                new YawCouplingFeedforward(
                        PARAMS.yawCouplingKsAxial, PARAMS.yawCouplingKvAxial,
                        PARAMS.yawCouplingKsLateral, PARAMS.yawCouplingKvLateral);
        List<Double> yawCouplingVoltages =
                kinematics.yawCouplingVoltages(yawCoupling, command.value());

        double leftFrontPower = (leftFrontFF + yawCouplingVoltages.get(0)) / voltage;
        double leftBackPower = (leftBackFF + yawCouplingVoltages.get(1)) / voltage;
        double rightBackPower = (rightBackFF + yawCouplingVoltages.get(2)) / voltage;
        double rightFrontPower = (rightFrontFF + yawCouplingVoltages.get(3)) / voltage;
        mecanumCommandWriter.write(
                new MecanumCommandMessage(
                        voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower));

        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
        rightFront.setPower(rightFrontPower);
    }

    public final class FollowTrajectoryAction implements Action {
        public final TimeTrajectory timeTrajectory;
        private double beginTs = -1;

        private final double[] xPoints, yPoints;

        public FollowTrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;

            List<Double> disps =
                    com.acmerobotics.roadrunner.Math.range(
                            0,
                            t.path.length(),
                            java.lang.Math.max(2, (int) java.lang.Math.ceil(t.path.length() / 2)));
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= timeTrajectory.duration) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);

                return false;
            }

            Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command =
                    new HolonomicController(
                                    PARAMS.axialGain,
                                    PARAMS.lateralGain,
                                    PARAMS.headingGain,
                                    PARAMS.axialVelGain,
                                    PARAMS.lateralVelGain,
                                    PARAMS.headingVelGain)
                            .compute(txWorldTarget, localizer.getPose(), robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            setDriveCommand(command);

            p.put("x", localizer.getPose().position.x);
            p.put("y", localizer.getPose().position.y);
            p.put(
                    "heading (deg)",
                    java.lang.Math.toDegrees(localizer.getPose().heading.toDouble()));

            Pose2d error = txWorldTarget.value().minusExp(localizer.getPose());
            p.put("xError", error.position.x);
            p.put("yError", error.position.y);
            p.put("headingError (deg)", java.lang.Math.toDegrees(error.heading.toDouble()));

            // only draw when active; only one drive action should be active at a time
            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, localizer.getPose());

            c.setStroke("#4CAF50FF");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }
    }

    public final class TurnAction implements Action {
        private final TimeTurn turn;

        private double beginTs = -1;

        public TurnAction(TimeTurn turn) {
            this.turn = turn;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= turn.duration) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);

                return false;
            }

            Pose2dDual<Time> txWorldTarget = turn.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command =
                    new HolonomicController(
                                    PARAMS.axialGain,
                                    PARAMS.lateralGain,
                                    PARAMS.headingGain,
                                    PARAMS.axialVelGain,
                                    PARAMS.lateralVelGain,
                                    PARAMS.headingVelGain)
                            .compute(txWorldTarget, localizer.getPose(), robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            final MotorFeedforward feedforward =
                    new MotorFeedforward(
                            PARAMS.kS, PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(
                    new MecanumCommandMessage(
                            voltage,
                            leftFrontPower,
                            leftBackPower,
                            rightBackPower,
                            rightFrontPower));

            leftFront.setPower(feedforward.compute(wheelVels.leftFront) / voltage);
            leftBack.setPower(feedforward.compute(wheelVels.leftBack) / voltage);
            rightBack.setPower(feedforward.compute(wheelVels.rightBack) / voltage);
            rightFront.setPower(feedforward.compute(wheelVels.rightFront) / voltage);

            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, localizer.getPose());

            c.setStroke("#7C4DFFFF");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#7C4DFF7A");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }

    public PoseVelocity2d updatePoseEstimate() {
        PoseVelocity2d vel = localizer.update();
        poseHistory.add(localizer.getPose());

        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        estimatedPoseWriter.write(new PoseMessage(localizer.getPose()));

        return vel;
    }

    private void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        int i = 0;
        for (Pose2d t : poseHistory) {
            xPoints[i] = t.position.x;
            yPoints[i] = t.position.y;

            i++;
        }

        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                new TrajectoryBuilderParams(1e-6, new ProfileParams(0.25, 0.1, 1e-2)),
                beginPose,
                0.0,
                defaultTurnConstraints,
                defaultVelConstraint,
                defaultAccelConstraint);
    }
}
