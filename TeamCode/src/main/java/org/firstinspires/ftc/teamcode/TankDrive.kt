package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.AccelConstraint
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.AngularVelConstraint
import com.acmerobotics.roadrunner.Arclength
import com.acmerobotics.roadrunner.CentripetalAccelVelConstraint
import com.acmerobotics.roadrunner.DualNum
import com.acmerobotics.roadrunner.MinVelConstraint
import com.acmerobotics.roadrunner.MotorFeedforward
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Pose2dDual
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.PoseVelocity2dDual
import com.acmerobotics.roadrunner.ProfileAccelConstraint
import com.acmerobotics.roadrunner.ProfileParams
import com.acmerobotics.roadrunner.RamseteController
import com.acmerobotics.roadrunner.TankKinematics
import com.acmerobotics.roadrunner.Time
import com.acmerobotics.roadrunner.TimeTrajectory
import com.acmerobotics.roadrunner.TimeTurn
import com.acmerobotics.roadrunner.TrajectoryActionBuilder
import com.acmerobotics.roadrunner.TrajectoryBuilderParams
import com.acmerobotics.roadrunner.TurnConstraints
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.Vector2dDual
import com.acmerobotics.roadrunner.VelConstraint
import com.acmerobotics.roadrunner.YawCouplingFeedforward
import com.acmerobotics.roadrunner.ftc.DownsampledWriter
import com.acmerobotics.roadrunner.ftc.Encoder
import com.acmerobotics.roadrunner.ftc.FlightRecorder
import com.acmerobotics.roadrunner.ftc.LazyHardwareMapImu
import com.acmerobotics.roadrunner.ftc.LazyImu
import com.acmerobotics.roadrunner.ftc.OverflowEncoder
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair
import com.acmerobotics.roadrunner.ftc.RawEncoder
import com.acmerobotics.roadrunner.now
import com.acmerobotics.roadrunner.range
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor
import java.util.Collections
import java.util.LinkedList
import java.util.Objects
import java.util.function.DoubleSupplier
import kotlin.math.PI
import kotlin.math.ceil
import kotlin.math.max
import org.firstinspires.ftc.teamcode.messages.DriveCommandMessage
import org.firstinspires.ftc.teamcode.messages.PoseMessage
import org.firstinspires.ftc.teamcode.messages.TankCommandMessage
import org.firstinspires.ftc.teamcode.messages.TankLocalizerInputsMessage
import org.firstinspires.ftc.teamcode.opmodes.base.MarsLinearOpMode

@Config
class TankDrive
private constructor(
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
        @JvmField var inPerTick = 0.0
        @JvmField var trackWidthTicks = 0.0

        // feedforward parameters (in tick units)
        @JvmField var kS = 0.0
        @JvmField var kV = 0.0
        @JvmField var kA = 0.0

        // Yaw-coupling feedforward. Cancels the parasitic yaw (curl) produced by forward/back
        // translation under open-loop driving. Only the axial constants apply — a tank drive has no
        // lateral motion. Units are volts (kS) and volts per inch/s (kV). Zero = disabled.
        @JvmField var yawCouplingKsAxial = 0.0
        @JvmField var yawCouplingKvAxial = 0.0

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
        @JvmField var ramseteZeta = 0.7 // in the range (0, 1)
        @JvmField var ramseteBBar = 2.0 // positive

        // turn controller gains
        @JvmField var turnGain = 0.0
        @JvmField var turnVelGain = 0.0
    }

    companion object {
        @JvmField var PARAMS = Params()

        /**
         * Frame-owned drive for [MarsLinearOpMode]: uses `batteryVoltage` for FF and does not
         * change bulk caching (caller already set MANUAL via `initRobot()` / BulkReads).
         *
         * @param batteryVoltage typically `this::batteryVoltage` after `initRobot()`
         */
        @JvmStatic
        fun forMarsLinear(
            hardwareMap: HardwareMap,
            pose: Pose2d,
            batteryVoltage: DoubleSupplier,
        ): TankDrive {
            Objects.requireNonNull(batteryVoltage, "batteryVoltage")
            return TankDrive(hardwareMap, pose, batteryVoltage, false)
        }
    }

    /**
     * Stock / RR path: hub [VoltageSensor] for each FF sample, and enables
     * [LynxModule.BulkCachingMode.AUTO] on all hubs. Prefer [forMarsLinear] under
     * [MarsLinearOpMode] so AUTO does not undo MANUAL bulk from `initRobot()`.
     */
    constructor(hardwareMap: HardwareMap, pose: Pose2d) : this(hardwareMap, pose, null, true)

    @JvmField val kinematics = TankKinematics(PARAMS.inPerTick * PARAMS.trackWidthTicks)

    @JvmField
    val defaultTurnConstraints =
        TurnConstraints(
            PARAMS.maxAngVel,
            -PARAMS.maxAngAccel,
            PARAMS.maxAngAccel,
        )

    // A single WheelVoltageConstraint instance serves as both the velocity and the acceleration
    // constraint when PARAMS.useWheelVoltageConstraint is set; null otherwise. It must be declared
    // (and initialized) before the two constraint fields that reference it below.
    private val wheelVoltageConstraint: TankKinematics.WheelVoltageConstraint? =
        if (PARAMS.useWheelVoltageConstraint) makeWheelVoltageConstraint() else null

    @JvmField val defaultVelConstraint: VelConstraint = makeDefaultVelConstraint()

    @JvmField
    val defaultAccelConstraint: AccelConstraint =
        wheelVoltageConstraint
            ?: ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel)

    @JvmField val leftMotors: List<DcMotorEx>

    @JvmField val rightMotors: List<DcMotorEx>

    @JvmField val lazyImu: LazyImu

    @JvmField val voltageSensor: VoltageSensor

    /**
     * Battery voltage (volts) for feedforward compensation. Stock constructor uses
     * [VoltageSensor.getVoltage] each sample; [forMarsLinear] uses the OpMode frame cache (e.g.
     * [MarsLinearOpMode.batteryVoltage]).
     */
    private val voltageGetter: DoubleSupplier

    @JvmField val localizer: Localizer

    private val poseHistory = LinkedList<Pose2d>()

    private val estimatedPoseWriter = DownsampledWriter("ESTIMATED_POSE", 50_000_000)
    private val targetPoseWriter = DownsampledWriter("TARGET_POSE", 50_000_000)
    private val driveCommandWriter = DownsampledWriter("DRIVE_COMMAND", 50_000_000)

    private val tankCommandWriter = DownsampledWriter("TANK_COMMAND", 50_000_000)

    private fun makeWheelVoltageConstraint(): TankKinematics.WheelVoltageConstraint {
        val feedforward =
            MotorFeedforward(
                PARAMS.kS,
                PARAMS.kV / PARAMS.inPerTick,
                PARAMS.kA / PARAMS.inPerTick,
            )
        val yawCoupling =
            YawCouplingFeedforward(
                PARAMS.yawCouplingKsAxial,
                PARAMS.yawCouplingKvAxial,
            )
        return kinematics.WheelVoltageConstraint(
            feedforward,
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
        //   add additional motors on each side if you have them
        //   see
        // https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        leftMotors = listOf(hardwareMap.get(DcMotorEx::class.java, "left"))
        rightMotors = listOf(hardwareMap.get(DcMotorEx::class.java, "right"))

        for (m in leftMotors) {
            m.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
        for (m in rightMotors) {
            m.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }

        // TODO: reverse motor directions if needed
        //   leftMotors.get(0).setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO: make sure your config has an IMU with this name (can be BNO or BHI)
        //   see
        // https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        lazyImu =
            LazyHardwareMapImu(
                hardwareMap,
                "imu",
                RevHubOrientationOnRobot(PARAMS.logoFacingDirection, PARAMS.usbFacingDirection),
            )

        voltageSensor = hardwareMap.voltageSensor.iterator().next()
        this.voltageGetter = voltageGetter ?: DoubleSupplier { voltageSensor.voltage }

        // Drive encoders (default). When switching localizers, also set PARAMS.inPerTick:
        //   Pinpoint: new PinpointLocalizer(hardwareMap, PARAMS.inPerTick, pose)
        //     with pod scale (e.g. (48/25.4*PI)/2000 for goBILDA 48mm, 2000 ticks/rev)
        //   OTOS: new OTOSLocalizer(hardwareMap, pose) and PARAMS.inPerTick = 1.0
        localizer = DriveLocalizer(pose)

        FlightRecorder.write("TANK_PARAMS", PARAMS)
    }

    inner class DriveLocalizer(pose: Pose2d) : Localizer {
        @JvmField
        val leftEncs: List<Encoder> = run {
            val encs = ArrayList<Encoder>()
            for (m in leftMotors) {
                encs.add(OverflowEncoder(RawEncoder(m)))
            }
            Collections.unmodifiableList(encs)
        }

        @JvmField
        val rightEncs: List<Encoder> = run {
            val encs = ArrayList<Encoder>()
            for (m in rightMotors) {
                encs.add(OverflowEncoder(RawEncoder(m)))
            }
            Collections.unmodifiableList(encs)
        }

        private var pose: Pose2d = pose

        private var lastLeftPos = 0.0
        private var lastRightPos = 0.0
        private var initialized = false

        init {
            // TODO: reverse encoder directions if needed
            //   leftEncs.get(0).setDirection(DcMotorSimple.Direction.REVERSE);
        }

        override fun setPose(pose: Pose2d) {
            this.pose = pose
        }

        override fun getPose(): Pose2d {
            return pose
        }

        override fun update(): PoseVelocity2d {
            val leftReadings = ArrayList<PositionVelocityPair>()
            val rightReadings = ArrayList<PositionVelocityPair>()
            var meanLeftPos = 0.0
            var meanLeftVel = 0.0
            for (e in leftEncs) {
                val p = e.getPositionAndVelocity()
                meanLeftPos += p.position
                meanLeftVel += p.velocity!!
                leftReadings.add(p)
            }
            meanLeftPos /= leftEncs.size
            meanLeftVel /= leftEncs.size

            var meanRightPos = 0.0
            var meanRightVel = 0.0
            for (e in rightEncs) {
                val p = e.getPositionAndVelocity()
                meanRightPos += p.position
                meanRightVel += p.velocity!!
                rightReadings.add(p)
            }
            meanRightPos /= rightEncs.size
            meanRightVel /= rightEncs.size

            FlightRecorder.write(
                "TANK_LOCALIZER_INPUTS",
                TankLocalizerInputsMessage(leftReadings, rightReadings),
            )

            if (!initialized) {
                initialized = true

                lastLeftPos = meanLeftPos
                lastRightPos = meanRightPos

                return PoseVelocity2d(Vector2d(0.0, 0.0), 0.0)
            }

            val twist =
                kinematics.forward(
                    TankKinematics.WheelIncrements(
                        DualNum<Time>(
                                doubleArrayOf(
                                    meanLeftPos - lastLeftPos,
                                    meanLeftVel,
                                )
                            )
                            .times(PARAMS.inPerTick),
                        DualNum<Time>(
                                doubleArrayOf(
                                    meanRightPos - lastRightPos,
                                    meanRightVel,
                                )
                            )
                            .times(PARAMS.inPerTick),
                    )
                )

            lastLeftPos = meanLeftPos
            lastRightPos = meanRightPos

            pose = pose.plus(twist.value())

            return twist.velocity().value()
        }
    }

    /** Battery voltage used for feedforward power scaling (via the construction-time getter). */
    fun getBatteryVoltage(): Double {
        return voltageGetter.asDouble
    }

    fun setDrivePowers(powers: PoseVelocity2d) {
        val wheelVels = TankKinematics(2.0).inverse(PoseVelocity2dDual.constant<Time>(powers, 1))

        var maxPowerMag = 1.0
        for (power in wheelVels.all()) {
            maxPowerMag = max(maxPowerMag, power.value())
        }

        for (m in leftMotors) {
            m.power = wheelVels.left[0] / maxPowerMag
        }
        for (m in rightMotors) {
            m.power = wheelVels.right[0] / maxPowerMag
        }
    }

    /**
     * Applies a follower velocity/acceleration command to the wheels through the full production
     * feedforward path: drive constants, yaw-coupling voltages, and battery voltage compensation.
     * Shared by [FollowTrajectoryAction] and the feedback-gain tuner so gain tests exercise exactly
     * the voltages the follower applies.
     */
    fun setDriveCommand(command: PoseVelocity2dDual<Time>) {
        val wheelVels = kinematics.inverse(command)
        val voltage = getBatteryVoltage()
        val feedforward =
            MotorFeedforward(
                PARAMS.kS,
                PARAMS.kV / PARAMS.inPerTick,
                PARAMS.kA / PARAMS.inPerTick,
            )

        // Yaw-coupling feedforward: a per-wheel voltage that cancels the parasitic yaw from
        // forward/back translation. Entries follow wheel order (left, right). Zero constants
        // (the default) make this a no-op.
        val yawCoupling =
            YawCouplingFeedforward(PARAMS.yawCouplingKsAxial, PARAMS.yawCouplingKvAxial)
        val yawCouplingVoltages = kinematics.yawCouplingVoltages(yawCoupling, command.value())

        val leftPower = (feedforward.compute(wheelVels.left) + yawCouplingVoltages[0]) / voltage
        val rightPower = (feedforward.compute(wheelVels.right) + yawCouplingVoltages[1]) / voltage
        tankCommandWriter.write(TankCommandMessage(voltage, leftPower, rightPower))

        for (m in leftMotors) {
            m.power = leftPower
        }
        for (m in rightMotors) {
            m.power = rightPower
        }
    }

    inner class FollowTrajectoryAction(t: TimeTrajectory) : Action {
        @JvmField val timeTrajectory: TimeTrajectory = t

        private var beginTs = -1.0

        private val xPoints: DoubleArray
        private val yPoints: DoubleArray

        init {
            val disps =
                range(
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
                for (m in leftMotors) {
                    m.power = 0.0
                }
                for (m in rightMotors) {
                    m.power = 0.0
                }

                return false
            }

            val x = timeTrajectory.profile[t]

            val txWorldTarget: Pose2dDual<Arclength> = timeTrajectory.path[x.value(), 3]
            targetPoseWriter.write(PoseMessage(txWorldTarget.value()))

            updatePoseEstimate()

            val command =
                RamseteController(
                        kinematics.trackWidth,
                        PARAMS.ramseteZeta,
                        PARAMS.ramseteBBar,
                    )
                    .compute(x, txWorldTarget, localizer.getPose())
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
                for (m in leftMotors) {
                    m.power = 0.0
                }
                for (m in rightMotors) {
                    m.power = 0.0
                }

                return false
            }

            val txWorldTarget = turn[t]
            targetPoseWriter.write(PoseMessage(txWorldTarget.value()))

            val robotVelRobot = updatePoseEstimate()

            // Positive gains act on (target - actual), matching HolonomicController's convention.
            // (The stock quickstart has the operands flipped, which makes positive gains unstable.)
            val command =
                PoseVelocity2dDual(
                    Vector2dDual.constant(Vector2d(0.0, 0.0), 3),
                    txWorldTarget.heading
                        .velocity()
                        .plus(
                            PARAMS.turnGain *
                                txWorldTarget.heading.value().minus(localizer.getPose().heading) +
                                PARAMS.turnVelGain *
                                    (txWorldTarget.heading.velocity().value() -
                                        robotVelRobot.angVel)
                        ),
                )
            driveCommandWriter.write(DriveCommandMessage(command))

            val wheelVels = kinematics.inverse(command)
            val voltage = getBatteryVoltage()
            val feedforward =
                MotorFeedforward(
                    PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick,
                    PARAMS.kA / PARAMS.inPerTick,
                )
            val leftPower = feedforward.compute(wheelVels.left) / voltage
            val rightPower = feedforward.compute(wheelVels.right) / voltage
            tankCommandWriter.write(TankCommandMessage(voltage, leftPower, rightPower))

            for (m in leftMotors) {
                m.power = leftPower
            }
            for (m in rightMotors) {
                m.power = rightPower
            }

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
