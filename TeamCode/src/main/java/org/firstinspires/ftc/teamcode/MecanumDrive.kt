package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.AccelConstraint
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.AngularVelConstraint
import com.acmerobotics.roadrunner.AnisotropicMotorFeedforward
import com.acmerobotics.roadrunner.CentripetalAccelVelConstraint
import com.acmerobotics.roadrunner.DualNum
import com.acmerobotics.roadrunner.HolonomicController
import com.acmerobotics.roadrunner.MecanumKinematics
import com.acmerobotics.roadrunner.MinVelConstraint
import com.acmerobotics.roadrunner.MotorFeedforward
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Pose2dDual
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.PoseVelocity2dDual
import com.acmerobotics.roadrunner.ProfileAccelConstraint
import com.acmerobotics.roadrunner.ProfileParams
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.Time
import com.acmerobotics.roadrunner.TimeTrajectory
import com.acmerobotics.roadrunner.TimeTurn
import com.acmerobotics.roadrunner.TrajectoryActionBuilder
import com.acmerobotics.roadrunner.TrajectoryBuilderParams
import com.acmerobotics.roadrunner.TurnConstraints
import com.acmerobotics.roadrunner.Twist2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.VelConstraint
import com.acmerobotics.roadrunner.YawCouplingFeedforward
import com.acmerobotics.roadrunner.ftc.DownsampledWriter
import com.acmerobotics.roadrunner.ftc.Encoder
import com.acmerobotics.roadrunner.ftc.FlightRecorder
import com.acmerobotics.roadrunner.ftc.LazyHardwareMapImu
import com.acmerobotics.roadrunner.ftc.LazyImu

import com.acmerobotics.roadrunner.ftc.OverflowEncoder
import com.acmerobotics.roadrunner.ftc.RawEncoder
import com.acmerobotics.roadrunner.now
import com.acmerobotics.roadrunner.range
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.messages.DriveCommandMessage
import org.firstinspires.ftc.teamcode.messages.MecanumCommandMessage
import org.firstinspires.ftc.teamcode.messages.MecanumLocalizerInputsMessage
import org.firstinspires.ftc.teamcode.messages.PoseMessage
import org.firstinspires.ftc.teamcode.opmodes.base.MarsLinearOpMode
import java.util.LinkedList
import java.util.Objects
import java.util.function.DoubleSupplier
import kotlin.math.PI
import kotlin.math.ceil
import kotlin.math.max

@Config
class MecanumDrive private constructor(
    hardwareMap: HardwareMap,
    pose: Pose2d,
    voltageGetter: DoubleSupplier?,
    autoBulk: Boolean,
) {
    class Params {
        // IMU orientation
        // TODO: fill in these values based on
        //   see
        // https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html?highlight=imu#physical-hub-mounting
        @JvmField
        var logoFacingDirection: RevHubOrientationOnRobot.LogoFacingDirection =
            RevHubOrientationOnRobot.LogoFacingDirection.UP
        @JvmField
        var usbFacingDirection: RevHubOrientationOnRobot.UsbFacingDirection =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD

        // drive model parameters
        // Linear scale for the drive model (kV/kA, trackWidthTicks, kinematics).
        // Also the odometry scale passed into Pinpoint / dead-wheel localizers.
        // Starting values by localizer:
        //   Pinpoint / dead wheels (goBILDA 48mm pods, 2000 ticks/rev):
        //     (48 / 25.4 * Math.PI) / 2000.0
        //   OTOS (reports inches natively): 1.0
        //   Drive encoders: ForwardPushTest (or compute from wheel diameter / gearing)
        @JvmField var inPerTick = 1.0
        @JvmField var lateralInPerTick = inPerTick
        // Tape track width (in) / inPerTick. TrackWidthTuner seeds/refines this when 0.
        @JvmField var trackWidthTicks = 0.0

        // feedforward parameters (in tick units)
        @JvmField var kS = 0.0
        @JvmField var kV = 0.0
        @JvmField var kA = 0.0

        // Anisotropic (axial vs. lateral) feedforward. Mecanum rollers scrub when strafing, which
        // raises the effective kS/kV/kA relative to forward/rotational motion. Leave
        // useAnisotropicFeedforward false for stock (isotropic) behavior; when true, the lateral*
        // constants (in tick units, like kS/kV/kA) are applied to the strafe component of each
        // wheel.
        @JvmField var useAnisotropicFeedforward = false
        @JvmField var lateralKS = 0.0
        @JvmField var lateralKV = 0.0
        @JvmField var lateralKA = 0.0

        // Yaw-coupling feedforward. Cancels the parasitic yaw moment (curl) produced by chassis
        // translation under open-loop driving. Axial constants come from a forward ramp, lateral
        // from a strafe ramp. Units are volts (kS*) and volts per inch/s (kV*). Zero = disabled.
        @JvmField var yawCouplingKsAxial = 0.0
        @JvmField var yawCouplingKvAxial = 0.0
        @JvmField var yawCouplingKsLateral = 0.0
        @JvmField var yawCouplingKvLateral = 0.0

        // path profile parameters (in inches)
        @JvmField var maxWheelVel = 50.0
        @JvmField var minProfileAccel = -30.0
        @JvmField var maxProfileAccel = 50.0

        // Voltage-budget path constraint (back-EMF/traction aware). When enabled, path velocity and
        // acceleration are limited by keeping every wheel's feedforward voltage within the budget
        // instead of by the fixed maxWheelVel / profile-accel caps above. Requires calibrated kV,
        // kA.
        @JvmField var useWheelVoltageConstraint = false
        @JvmField var maxVoltageForPlanning = 11.0 // plan below the 12 V nominal to leave headroom
        @JvmField var cruiseFraction = 0.95 // budget spent cruising; remainder reserved for accel

        // Centripetal (cornering) acceleration limit, in in/s^2. Caps speed through curves to keep
        // the wheels from slipping sideways. Zero = disabled.
        @JvmField var maxCentripetalAccel = 0.0

        // turn profile parameters (in radians)
        @JvmField var maxAngVel = PI // shared with path
        @JvmField var maxAngAccel = PI

        // path controller gains
        @JvmField var axialGain = 0.0
        @JvmField var lateralGain = 0.0
        @JvmField var headingGain = 0.0 // shared with turn

        @JvmField var axialVelGain = 0.0
        @JvmField var lateralVelGain = 0.0
        @JvmField var headingVelGain = 0.0 // shared with turn
    }

    companion object {
        @JvmField
        var PARAMS = Params()

        /**
         * Frame-owned drive for [MarsLinearOpMode]: uses `batteryVoltage` for FF and does
         * not change bulk caching (caller already set MANUAL via `initRobot()` / BulkReads).
         *
         * @param batteryVoltage typically `this::batteryVoltage` after `initRobot()`
         */
        @JvmStatic
        fun forMarsLinear(
            hardwareMap: HardwareMap,
            pose: Pose2d,
            batteryVoltage: DoubleSupplier,
        ): MecanumDrive {
            Objects.requireNonNull(batteryVoltage, "batteryVoltage")
            return MecanumDrive(hardwareMap, pose, batteryVoltage, false)
        }
    }

    /**
     * Stock / RR path: hub [VoltageSensor] for each FF sample, and enables
     * [LynxModule.BulkCachingMode.AUTO] on all hubs. Prefer [forMarsLinear] under
     * [MarsLinearOpMode] so AUTO does not undo MANUAL bulk from `initRobot()`.
     */
    constructor(hardwareMap: HardwareMap, pose: Pose2d) : this(hardwareMap, pose, null, true)

    @JvmField
    val kinematics = MecanumKinematics(
        PARAMS.inPerTick * PARAMS.trackWidthTicks,
        PARAMS.inPerTick / PARAMS.lateralInPerTick,
    )

    @JvmField
    val defaultTurnConstraints = TurnConstraints(
        PARAMS.maxAngVel,
        -PARAMS.maxAngAccel,
        PARAMS.maxAngAccel,
    )

    // A single WheelVoltageConstraint instance serves as both the velocity and the acceleration
    // constraint when PARAMS.useWheelVoltageConstraint is set; null otherwise. It must be declared
    // (and initialized) before the two constraint fields that reference it below.
    private val wheelVoltageConstraint: MecanumKinematics.WheelVoltageConstraint? =
        if (PARAMS.useWheelVoltageConstraint) makeWheelVoltageConstraint() else null

    @JvmField
    val defaultVelConstraint: VelConstraint = makeDefaultVelConstraint()

    @JvmField
    val defaultAccelConstraint: AccelConstraint =
        wheelVoltageConstraint
            ?: ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel)

    @JvmField
    val leftFront: DcMotorEx

    @JvmField
    val leftBack: DcMotorEx

    @JvmField
    val rightBack: DcMotorEx

    @JvmField
    val rightFront: DcMotorEx

    @JvmField
    val voltageSensor: VoltageSensor

    /**
     * Battery voltage (volts) for feedforward compensation. Stock constructor uses
     * [VoltageSensor.getVoltage] each sample; [forMarsLinear] uses the OpMode frame cache
     * (e.g. [MarsLinearOpMode.batteryVoltage]).
     */
    private val voltageGetter: DoubleSupplier

    @JvmField
    val lazyImu: LazyImu

    @JvmField
    val localizer: Localizer

    private val poseHistory = LinkedList<Pose2d>()

    private val estimatedPoseWriter = DownsampledWriter("ESTIMATED_POSE", 50_000_000)
    private val targetPoseWriter = DownsampledWriter("TARGET_POSE", 50_000_000)
    private val driveCommandWriter = DownsampledWriter("DRIVE_COMMAND", 50_000_000)
    private val mecanumCommandWriter = DownsampledWriter("MECANUM_COMMAND", 50_000_000)

    private fun makeWheelVoltageConstraint(): MecanumKinematics.WheelVoltageConstraint {
        val axial = MotorFeedforward(
            PARAMS.kS,
            PARAMS.kV / PARAMS.inPerTick,
            PARAMS.kA / PARAMS.inPerTick,
        )
        val lateral =
            if (PARAMS.useAnisotropicFeedforward) {
                MotorFeedforward(
                    PARAMS.lateralKS,
                    PARAMS.lateralKV / PARAMS.inPerTick,
                    PARAMS.lateralKA / PARAMS.inPerTick,
                )
            } else {
                axial
            }
        val yawCoupling = YawCouplingFeedforward(
            PARAMS.yawCouplingKsAxial,
            PARAMS.yawCouplingKvAxial,
            PARAMS.yawCouplingKsLateral,
            PARAMS.yawCouplingKvLateral,
        )
        return kinematics.WheelVoltageConstraint(
            AnisotropicMotorFeedforward(axial, lateral),
            yawCoupling,
            PARAMS.maxVoltageForPlanning,
            PARAMS.cruiseFraction,
        )
    }

    private fun makeDefaultVelConstraint(): VelConstraint {
        val constraints = ArrayList<VelConstraint>()
        if (wheelVoltageConstraint != null) {
            // the voltage constraint's velocity limit subsumes the fixed wheel-velocity cap
            constraints.add(wheelVoltageConstraint)
        } else {
            constraints.add(kinematics.WheelVelConstraint(PARAMS.maxWheelVel))
        }
        constraints.add(AngularVelConstraint(PARAMS.maxAngVel))
        if (PARAMS.maxCentripetalAccel > 0) {
            constraints.add(CentripetalAccelVelConstraint(PARAMS.maxCentripetalAccel))
        }
        return MinVelConstraint(constraints)
    }

    init {
        FtcRoadRunnerCompat.throwIfModulesAreOutdated(hardwareMap)

        if (autoBulk) {
            for (module in hardwareMap.getAll(LynxModule::class.java)) {
                module.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
            }
        }

        // TODO: make sure your config has motors with these names (or change them)
        //   see
        // https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        leftFront = hardwareMap.get(DcMotorEx::class.java, "leftFront")
        leftBack = hardwareMap.get(DcMotorEx::class.java, "leftBack")
        rightBack = hardwareMap.get(DcMotorEx::class.java, "rightBack")
        rightFront = hardwareMap.get(DcMotorEx::class.java, "rightFront")

        leftFront.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        leftBack.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightBack.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightFront.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        // TODO: reverse motor directions if needed
        //   leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO: make sure your config has an IMU with this name (can be BNO or BHI)
        //   see
        // https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        lazyImu = LazyHardwareMapImu(
            hardwareMap,
            "imu",
            RevHubOrientationOnRobot(PARAMS.logoFacingDirection, PARAMS.usbFacingDirection),
        )

        voltageSensor = hardwareMap.voltageSensor.iterator().next()
        this.voltageGetter = voltageGetter ?: DoubleSupplier { voltageSensor.voltage }

        // Pinpoint: pass PARAMS.inPerTick (pod scale; 48mm/2000 default above).
        // OTOS: localizer = new OTOSLocalizer(hardwareMap, pose); and set PARAMS.inPerTick = 1.0
        localizer = DriveLocalizer(pose)

        FlightRecorder.write("MECANUM_PARAMS", PARAMS)
    }

    inner class DriveLocalizer(pose: Pose2d) : Localizer {
        @JvmField
        val leftFront: Encoder = OverflowEncoder(RawEncoder(this@MecanumDrive.leftFront))

        @JvmField
        val leftBack: Encoder = OverflowEncoder(RawEncoder(this@MecanumDrive.leftBack))

        @JvmField
        val rightBack: Encoder = OverflowEncoder(RawEncoder(this@MecanumDrive.rightBack))

        @JvmField
        val rightFront: Encoder = OverflowEncoder(RawEncoder(this@MecanumDrive.rightFront))

        @JvmField
        val imu: IMU = lazyImu.get()

        private var lastLeftFrontPos = 0
        private var lastLeftBackPos = 0
        private var lastRightBackPos = 0
        private var lastRightFrontPos = 0
        private lateinit var lastHeading: Rotation2d
        private var initialized = false
        private var pose: Pose2d = pose

        init {
            // TODO: reverse encoders if needed
            //   leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        override fun setPose(pose: Pose2d) {
            this.pose = pose
        }

        override fun getPose(): Pose2d {
            return pose
        }

        override fun update(): PoseVelocity2d {
            val leftFrontPosVel = leftFront.getPositionAndVelocity()
            val leftBackPosVel = leftBack.getPositionAndVelocity()
            val rightBackPosVel = rightBack.getPositionAndVelocity()
            val rightFrontPosVel = rightFront.getPositionAndVelocity()

            val angles = imu.robotYawPitchRollAngles

            FlightRecorder.write(
                "MECANUM_LOCALIZER_INPUTS",
                MecanumLocalizerInputsMessage(
                    leftFrontPosVel,
                    leftBackPosVel,
                    rightBackPosVel,
                    rightFrontPosVel,
                    angles,
                ),
            )

            val heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS))

            if (!initialized) {
                initialized = true

                lastLeftFrontPos = leftFrontPosVel.position
                lastLeftBackPos = leftBackPosVel.position
                lastRightBackPos = rightBackPosVel.position
                lastRightFrontPos = rightFrontPosVel.position

                lastHeading = heading

                return PoseVelocity2d(Vector2d(0.0, 0.0), 0.0)
            }

            val headingDelta = heading.minus(lastHeading)
            val twist = kinematics.forward(
                MecanumKinematics.WheelIncrements(
                    DualNum<Time>(
                        doubleArrayOf(
                            (leftFrontPosVel.position - lastLeftFrontPos).toDouble(),
                            leftFrontPosVel.velocity!!.toDouble(),
                        ),
                    ).times(PARAMS.inPerTick),
                    DualNum<Time>(
                        doubleArrayOf(
                            (leftBackPosVel.position - lastLeftBackPos).toDouble(),
                            leftBackPosVel.velocity!!.toDouble(),
                        ),
                    ).times(PARAMS.inPerTick),
                    DualNum<Time>(
                        doubleArrayOf(
                            (rightBackPosVel.position - lastRightBackPos).toDouble(),
                            rightBackPosVel.velocity!!.toDouble(),
                        ),
                    ).times(PARAMS.inPerTick),
                    DualNum<Time>(
                        doubleArrayOf(
                            (rightFrontPosVel.position - lastRightFrontPos).toDouble(),
                            rightFrontPosVel.velocity!!.toDouble(),
                        ),
                    ).times(PARAMS.inPerTick),
                ),
            )

            lastLeftFrontPos = leftFrontPosVel.position
            lastLeftBackPos = leftBackPosVel.position
            lastRightBackPos = rightBackPosVel.position
            lastRightFrontPos = rightFrontPosVel.position

            lastHeading = heading

            pose = pose.plus(Twist2d(twist.line.value(), headingDelta))

            return twist.velocity().value()
        }
    }

    /** Battery voltage used for feedforward power scaling (via the construction-time getter). */
    fun getBatteryVoltage(): Double {
        return voltageGetter.asDouble
    }

    fun setDrivePowers(powers: PoseVelocity2d) {
        val wheelVels = MecanumKinematics(1.0).inverse(PoseVelocity2dDual.constant<Time>(powers, 1))

        var maxPowerMag = 1.0
        for (power in wheelVels.all()) {
            maxPowerMag = max(maxPowerMag, power.value())
        }

        leftFront.power = wheelVels.leftFront[0] / maxPowerMag
        leftBack.power = wheelVels.leftBack[0] / maxPowerMag
        rightBack.power = wheelVels.rightBack[0] / maxPowerMag
        rightFront.power = wheelVels.rightFront[0] / maxPowerMag
    }

    /**
     * Applies a follower velocity/acceleration command to the wheels through the full production
     * feedforward path: anisotropic constants (when enabled), yaw-coupling voltages, and battery
     * voltage compensation. Shared by [FollowTrajectoryAction] and the feedback-gain tuner so
     * gain tests exercise exactly the voltages the follower applies.
     */
    fun setDriveCommand(command: PoseVelocity2dDual<Time>) {
        val wheelVels = kinematics.inverse(command)
        val voltage = getBatteryVoltage()

        // Base feedforward voltage per wheel. With useAnisotropicFeedforward, each wheel's strafe
        // (lateral) velocity component is fed the separately-calibrated lateral constants;
        // otherwise
        // one set of constants is applied to the total wheel velocity (stock quickstart behavior).
        val leftFrontFF: Double
        val leftBackFF: Double
        val rightBackFF: Double
        val rightFrontFF: Double
        if (PARAMS.useAnisotropicFeedforward) {
            val feedforward = AnisotropicMotorFeedforward(
                MotorFeedforward(
                    PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick,
                    PARAMS.kA / PARAMS.inPerTick,
                ),
                MotorFeedforward(
                    PARAMS.lateralKS,
                    PARAMS.lateralKV / PARAMS.inPerTick,
                    PARAMS.lateralKA / PARAMS.inPerTick,
                ),
            )
            val components = kinematics.inverseComponents(command)
            leftFrontFF = feedforward.compute(components.axial.leftFront, components.lateral.leftFront)
            leftBackFF = feedforward.compute(components.axial.leftBack, components.lateral.leftBack)
            rightBackFF = feedforward.compute(components.axial.rightBack, components.lateral.rightBack)
            rightFrontFF = feedforward.compute(components.axial.rightFront, components.lateral.rightFront)
        } else {
            val feedforward = MotorFeedforward(
                PARAMS.kS,
                PARAMS.kV / PARAMS.inPerTick,
                PARAMS.kA / PARAMS.inPerTick,
            )
            leftFrontFF = feedforward.compute(wheelVels.leftFront)
            leftBackFF = feedforward.compute(wheelVels.leftBack)
            rightBackFF = feedforward.compute(wheelVels.rightBack)
            rightFrontFF = feedforward.compute(wheelVels.rightFront)
        }

        // Yaw-coupling feedforward: a per-wheel voltage that cancels the parasitic yaw from chassis
        // translation. Entries follow wheel order (leftFront, leftBack, rightBack, rightFront).
        // Zero constants (the default) make this a no-op.
        val yawCoupling = YawCouplingFeedforward(
            PARAMS.yawCouplingKsAxial,
            PARAMS.yawCouplingKvAxial,
            PARAMS.yawCouplingKsLateral,
            PARAMS.yawCouplingKvLateral,
        )
        val yawCouplingVoltages = kinematics.yawCouplingVoltages(yawCoupling, command.value())

        val leftFrontPower = (leftFrontFF + yawCouplingVoltages[0]) / voltage
        val leftBackPower = (leftBackFF + yawCouplingVoltages[1]) / voltage
        val rightBackPower = (rightBackFF + yawCouplingVoltages[2]) / voltage
        val rightFrontPower = (rightFrontFF + yawCouplingVoltages[3]) / voltage
        mecanumCommandWriter.write(
            MecanumCommandMessage(
                voltage,
                leftFrontPower,
                leftBackPower,
                rightBackPower,
                rightFrontPower,
            ),
        )

        leftFront.power = leftFrontPower
        leftBack.power = leftBackPower
        rightBack.power = rightBackPower
        rightFront.power = rightFrontPower
    }

    inner class FollowTrajectoryAction(t: TimeTrajectory) : Action {
        @JvmField
        val timeTrajectory: TimeTrajectory = t

        private var beginTs = -1.0

        private val xPoints: DoubleArray
        private val yPoints: DoubleArray

        init {
            val disps = range(
                0.0,
                t.path.length(),
                max(2, ceil(t.path.length() / 2).toInt()),
            )
            xPoints = DoubleArray(disps.size)
            yPoints = DoubleArray(disps.size)
            for (i in disps.indices) {
                val p = t.path[disps[i], 1].value()
                xPoints[i] = p.position.x
                yPoints[i] = p.position.y
            }
        }

        override fun run(p: TelemetryPacket): Boolean {
            val t: Double
            if (beginTs < 0) {
                beginTs = now()
                t = 0.0
            } else {
                t = now() - beginTs
            }

            if (t >= timeTrajectory.duration) {
                leftFront.power = 0.0
                leftBack.power = 0.0
                rightBack.power = 0.0
                rightFront.power = 0.0

                return false
            }

            val txWorldTarget = timeTrajectory[t]
            targetPoseWriter.write(PoseMessage(txWorldTarget.value()))

            val robotVelRobot = updatePoseEstimate()

            val command = HolonomicController(
                PARAMS.axialGain,
                PARAMS.lateralGain,
                PARAMS.headingGain,
                PARAMS.axialVelGain,
                PARAMS.lateralVelGain,
                PARAMS.headingVelGain,
            ).compute(txWorldTarget, localizer.getPose(), robotVelRobot)
            driveCommandWriter.write(DriveCommandMessage(command))

            setDriveCommand(command)

            p.put("x", localizer.getPose().position.x)
            p.put("y", localizer.getPose().position.y)
            p.put("heading (deg)", Math.toDegrees(localizer.getPose().heading.toDouble()))

            val error = txWorldTarget.value().minusExp(localizer.getPose())
            p.put("xError", error.position.x)
            p.put("yError", error.position.y)
            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()))

            // only draw when active; only one drive action should be active at a time
            val c = p.fieldOverlay()
            drawPoseHistory(c)

            c.setStroke("#4CAF50")
            Drawing.drawRobot(c, txWorldTarget.value())

            c.setStroke("#3F51B5")
            Drawing.drawRobot(c, localizer.getPose())

            c.setStroke("#4CAF50FF")
            c.setStrokeWidth(1)
            c.strokePolyline(xPoints, yPoints)

            return true
        }

        override fun preview(c: Canvas) {
            c.setStroke("#4CAF507A")
            c.setStrokeWidth(1)
            c.strokePolyline(xPoints, yPoints)
        }
    }

    inner class TurnAction(private val turn: TimeTurn) : Action {
        private var beginTs = -1.0

        override fun run(p: TelemetryPacket): Boolean {
            val t: Double
            if (beginTs < 0) {
                beginTs = now()
                t = 0.0
            } else {
                t = now() - beginTs
            }

            if (t >= turn.duration) {
                leftFront.power = 0.0
                leftBack.power = 0.0
                rightBack.power = 0.0
                rightFront.power = 0.0

                return false
            }

            val txWorldTarget = turn[t]
            targetPoseWriter.write(PoseMessage(txWorldTarget.value()))

            val robotVelRobot = updatePoseEstimate()

            val command = HolonomicController(
                PARAMS.axialGain,
                PARAMS.lateralGain,
                PARAMS.headingGain,
                PARAMS.axialVelGain,
                PARAMS.lateralVelGain,
                PARAMS.headingVelGain,
            ).compute(txWorldTarget, localizer.getPose(), robotVelRobot)
            driveCommandWriter.write(DriveCommandMessage(command))

            val wheelVels = kinematics.inverse(command)
            val voltage = getBatteryVoltage()
            val feedforward = MotorFeedforward(
                PARAMS.kS,
                PARAMS.kV / PARAMS.inPerTick,
                PARAMS.kA / PARAMS.inPerTick,
            )
            val leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage
            val leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage
            val rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage
            val rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage
            mecanumCommandWriter.write(
                MecanumCommandMessage(
                    voltage,
                    leftFrontPower,
                    leftBackPower,
                    rightBackPower,
                    rightFrontPower,
                ),
            )

            leftFront.power = feedforward.compute(wheelVels.leftFront) / voltage
            leftBack.power = feedforward.compute(wheelVels.leftBack) / voltage
            rightBack.power = feedforward.compute(wheelVels.rightBack) / voltage
            rightFront.power = feedforward.compute(wheelVels.rightFront) / voltage

            val c = p.fieldOverlay()
            drawPoseHistory(c)

            c.setStroke("#4CAF50")
            Drawing.drawRobot(c, txWorldTarget.value())

            c.setStroke("#3F51B5")
            Drawing.drawRobot(c, localizer.getPose())

            c.setStroke("#7C4DFFFF")
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2.0)

            return true
        }

        override fun preview(c: Canvas) {
            c.setStroke("#7C4DFF7A")
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2.0)
        }
    }

    fun updatePoseEstimate(): PoseVelocity2d {
        val vel = localizer.update()
        poseHistory.add(localizer.getPose())

        while (poseHistory.size > 100) {
            poseHistory.removeFirst()
        }

        estimatedPoseWriter.write(PoseMessage(localizer.getPose()))

        return vel
    }

    private fun drawPoseHistory(c: Canvas) {
        val xPoints = DoubleArray(poseHistory.size)
        val yPoints = DoubleArray(poseHistory.size)

        var i = 0
        for (t in poseHistory) {
            xPoints[i] = t.position.x
            yPoints[i] = t.position.y

            i++
        }

        c.setStrokeWidth(1)
        c.setStroke("#3F51B5")
        c.strokePolyline(xPoints, yPoints)
    }

    fun actionBuilder(beginPose: Pose2d): TrajectoryActionBuilder {
        return TrajectoryActionBuilder(
            { TurnAction(it) },
            { FollowTrajectoryAction(it) },
            TrajectoryBuilderParams(1e-6, ProfileParams(0.25, 0.1, 1e-2)),
            beginPose,
            0.0,
            defaultTurnConstraints,
            defaultVelConstraint,
            defaultAccelConstraint,
        )
    }
}
