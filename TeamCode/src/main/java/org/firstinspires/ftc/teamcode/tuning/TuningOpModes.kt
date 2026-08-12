package org.firstinspires.ftc.teamcode.tuning

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig
import com.acmerobotics.roadrunner.MotorFeedforward
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.ftc.AngularRampLogger
import com.acmerobotics.roadrunner.ftc.DeadWheelDirectionDebugger
import com.acmerobotics.roadrunner.ftc.DriveType
import com.acmerobotics.roadrunner.ftc.DriveView
import com.acmerobotics.roadrunner.ftc.DriveViewFactory
import com.acmerobotics.roadrunner.ftc.Encoder
import com.acmerobotics.roadrunner.ftc.EncoderGroup
import com.acmerobotics.roadrunner.ftc.EncoderRef
import com.acmerobotics.roadrunner.ftc.ForwardPushTest
import com.acmerobotics.roadrunner.ftc.ForwardRampLogger
import com.acmerobotics.roadrunner.ftc.LateralPushTest
import com.acmerobotics.roadrunner.ftc.LateralRampLogger
import com.acmerobotics.roadrunner.ftc.LazyImu
import com.acmerobotics.roadrunner.ftc.LynxQuadratureEncoderGroup
import com.acmerobotics.roadrunner.ftc.ManualFeedforwardTuner
import com.acmerobotics.roadrunner.ftc.MecanumMotorDirectionDebugger
import com.acmerobotics.roadrunner.ftc.OTOSAngularScalarTuner
import com.acmerobotics.roadrunner.ftc.OTOSEncoderGroup
import com.acmerobotics.roadrunner.ftc.OTOSHeadingOffsetTuner
import com.acmerobotics.roadrunner.ftc.OTOSIMU
import com.acmerobotics.roadrunner.ftc.OTOSLinearScalarTuner
import com.acmerobotics.roadrunner.ftc.OTOSPositionOffsetTuner
import com.acmerobotics.roadrunner.ftc.PinpointEncoderGroup
import com.acmerobotics.roadrunner.ftc.PinpointIMU
import com.acmerobotics.roadrunner.ftc.PinpointView
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta
import org.firstinspires.ftc.teamcode.MecanumDrive
import org.firstinspires.ftc.teamcode.OTOSLocalizer
import org.firstinspires.ftc.teamcode.PinpointLocalizer
import org.firstinspires.ftc.teamcode.TankDrive
import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer
import org.firstinspires.ftc.teamcode.TwoDeadWheelLocalizer

class TuningOpModes private constructor() {
    companion object {
        // TODO: change this to TankDrive::class.java if you're using tank
        @JvmField
        val DRIVE_CLASS: Class<*> = MecanumDrive::class.java

        @JvmField
        val GROUP = "quickstart"

        @JvmField
        val DISABLED = false

        private fun metaForClass(cls: Class<out OpMode>): OpModeMeta {
            return OpModeMeta.Builder()
                .setName(cls.simpleName)
                .setGroup(GROUP)
                .setFlavor(OpModeMeta.Flavor.TELEOP)
                .build()
        }

        private fun makePinpointView(pl: PinpointLocalizer): PinpointView {
            return object : PinpointView {
                private var parEncDirection = pl.initialParDirection
                private var perpEncDirection = pl.initialPerpDirection

                override fun update() {
                    pl.driver.update()
                }

                override fun getParEncoderPosition(): Int {
                    return pl.driver.encoderX
                }

                override fun getPerpEncoderPosition(): Int {
                    return pl.driver.encoderY
                }

                override fun getHeadingVelocity(unit: UnnormalizedAngleUnit): Float {
                    return pl.driver.getHeadingVelocity(unit).toFloat()
                }

                // PinpointView is a Kotlin interface: bean getters/setters surface as properties.
                override var parDirection: DcMotorSimple.Direction
                    get() =
                        if (parEncDirection == GoBildaPinpointDriver.EncoderDirection.FORWARD) {
                            DcMotorSimple.Direction.FORWARD
                        } else {
                            DcMotorSimple.Direction.REVERSE
                        }
                    set(direction) {
                        parEncDirection =
                            if (direction == DcMotorSimple.Direction.FORWARD) {
                                GoBildaPinpointDriver.EncoderDirection.FORWARD
                            } else {
                                GoBildaPinpointDriver.EncoderDirection.REVERSED
                            }
                        pl.driver.setEncoderDirections(parEncDirection, perpEncDirection)
                    }

                override var perpDirection: DcMotorSimple.Direction
                    get() =
                        if (perpEncDirection == GoBildaPinpointDriver.EncoderDirection.FORWARD) {
                            DcMotorSimple.Direction.FORWARD
                        } else {
                            DcMotorSimple.Direction.REVERSE
                        }
                    set(direction) {
                        perpEncDirection =
                            if (direction == DcMotorSimple.Direction.FORWARD) {
                                GoBildaPinpointDriver.EncoderDirection.FORWARD
                            } else {
                                GoBildaPinpointDriver.EncoderDirection.REVERSED
                            }
                        pl.driver.setEncoderDirections(parEncDirection, perpEncDirection)
                    }
            }
        }

        @JvmStatic
        @OpModeRegistrar
        fun register(manager: OpModeManager) {
            if (DISABLED) return

            val dvf: DriveViewFactory
            if (DRIVE_CLASS == MecanumDrive::class.java) {
                dvf =
                    object : DriveViewFactory {
                        override fun make(hardwareMap: HardwareMap): DriveView {
                        val md = MecanumDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0))
                        var lazyImu: LazyImu = md.lazyImu

                        val encoderGroups = ArrayList<EncoderGroup>()
                        val leftEncs = ArrayList<EncoderRef>()
                        val rightEncs = ArrayList<EncoderRef>()
                        val parEncs = ArrayList<EncoderRef>()
                        val perpEncs = ArrayList<EncoderRef>()
                        when (val localizer = md.localizer) {
                            is MecanumDrive.DriveLocalizer -> {
                                encoderGroups.add(
                                    LynxQuadratureEncoderGroup(
                                        hardwareMap.getAll(LynxModule::class.java),
                                        listOf(
                                            localizer.leftFront,
                                            localizer.leftBack,
                                            localizer.rightFront,
                                            localizer.rightBack,
                                        ),
                                    ),
                                )
                                leftEncs.add(EncoderRef(0, 0))
                                leftEncs.add(EncoderRef(0, 1))
                                rightEncs.add(EncoderRef(0, 2))
                                rightEncs.add(EncoderRef(0, 3))
                            }
                            is ThreeDeadWheelLocalizer -> {
                                encoderGroups.add(
                                    LynxQuadratureEncoderGroup(
                                        hardwareMap.getAll(LynxModule::class.java),
                                        listOf(localizer.par0, localizer.par1, localizer.perp),
                                    ),
                                )
                                parEncs.add(EncoderRef(0, 0))
                                parEncs.add(EncoderRef(0, 1))
                                perpEncs.add(EncoderRef(0, 2))
                            }
                            is TwoDeadWheelLocalizer -> {
                                encoderGroups.add(
                                    LynxQuadratureEncoderGroup(
                                        hardwareMap.getAll(LynxModule::class.java),
                                        listOf(localizer.par, localizer.perp),
                                    ),
                                )
                                parEncs.add(EncoderRef(0, 0))
                                perpEncs.add(EncoderRef(0, 1))
                            }
                            is OTOSLocalizer -> {
                                encoderGroups.add(OTOSEncoderGroup(localizer.otos))
                                parEncs.add(EncoderRef(0, 0))
                                perpEncs.add(EncoderRef(0, 1))
                                lazyImu = OTOSIMU(localizer.otos)
                            }
                            is PinpointLocalizer -> {
                                val pv = makePinpointView(localizer)
                                encoderGroups.add(PinpointEncoderGroup(pv))
                                parEncs.add(EncoderRef(0, 0))
                                perpEncs.add(EncoderRef(0, 1))
                                lazyImu = PinpointIMU(pv)
                            }
                            else -> {
                                throw RuntimeException(
                                    "unknown localizer: " + md.localizer.javaClass.name,
                                )
                            }
                        }

                        return DriveView(
                            DriveType.MECANUM,
                            MecanumDrive.PARAMS.inPerTick,
                            MecanumDrive.PARAMS.maxWheelVel,
                            MecanumDrive.PARAMS.minProfileAccel,
                            MecanumDrive.PARAMS.maxProfileAccel,
                            encoderGroups,
                            listOf(md.leftFront, md.leftBack),
                            listOf(md.rightFront, md.rightBack),
                            leftEncs,
                            rightEncs,
                            parEncs,
                            perpEncs,
                            lazyImu,
                            md.voltageSensor,
                            {
                                MotorFeedforward(
                                    MecanumDrive.PARAMS.kS,
                                    MecanumDrive.PARAMS.kV / MecanumDrive.PARAMS.inPerTick,
                                    MecanumDrive.PARAMS.kA / MecanumDrive.PARAMS.inPerTick,
                                )
                            },
                            0,
                        )
                        }
                    }
            } else if (DRIVE_CLASS == TankDrive::class.java) {
                dvf =
                    object : DriveViewFactory {
                        override fun make(hardwareMap: HardwareMap): DriveView {
                        val td = TankDrive(hardwareMap, Pose2d(0.0, 0.0, 0.0))
                        var lazyImu: LazyImu = td.lazyImu

                        val encoderGroups = ArrayList<EncoderGroup>()
                        val leftEncs = ArrayList<EncoderRef>()
                        val rightEncs = ArrayList<EncoderRef>()
                        val parEncs = ArrayList<EncoderRef>()
                        val perpEncs = ArrayList<EncoderRef>()
                        when (val localizer = td.localizer) {
                            is TankDrive.DriveLocalizer -> {
                                val allEncoders = ArrayList<Encoder>()
                                allEncoders.addAll(localizer.leftEncs)
                                allEncoders.addAll(localizer.rightEncs)
                                encoderGroups.add(
                                    LynxQuadratureEncoderGroup(
                                        hardwareMap.getAll(LynxModule::class.java),
                                        allEncoders,
                                    ),
                                )
                                for (i in localizer.leftEncs.indices) {
                                    leftEncs.add(EncoderRef(0, i))
                                }
                                for (i in localizer.rightEncs.indices) {
                                    rightEncs.add(EncoderRef(0, localizer.leftEncs.size + i))
                                }
                            }
                            is ThreeDeadWheelLocalizer -> {
                                encoderGroups.add(
                                    LynxQuadratureEncoderGroup(
                                        hardwareMap.getAll(LynxModule::class.java),
                                        listOf(localizer.par0, localizer.par1, localizer.perp),
                                    ),
                                )
                                parEncs.add(EncoderRef(0, 0))
                                parEncs.add(EncoderRef(0, 1))
                                perpEncs.add(EncoderRef(0, 2))
                            }
                            is TwoDeadWheelLocalizer -> {
                                encoderGroups.add(
                                    LynxQuadratureEncoderGroup(
                                        hardwareMap.getAll(LynxModule::class.java),
                                        listOf(localizer.par, localizer.perp),
                                    ),
                                )
                                parEncs.add(EncoderRef(0, 0))
                                perpEncs.add(EncoderRef(0, 1))
                            }
                            is PinpointLocalizer -> {
                                val pv = makePinpointView(localizer)
                                encoderGroups.add(PinpointEncoderGroup(pv))
                                parEncs.add(EncoderRef(0, 0))
                                perpEncs.add(EncoderRef(0, 1))
                                lazyImu = PinpointIMU(pv)
                            }
                            is OTOSLocalizer -> {
                                encoderGroups.add(OTOSEncoderGroup(localizer.otos))
                                parEncs.add(EncoderRef(0, 0))
                                perpEncs.add(EncoderRef(0, 1))
                                lazyImu = OTOSIMU(localizer.otos)
                            }
                            else -> {
                                throw RuntimeException(
                                    "unknown localizer: " + td.localizer.javaClass.name,
                                )
                            }
                        }

                        return DriveView(
                            DriveType.TANK,
                            TankDrive.PARAMS.inPerTick,
                            TankDrive.PARAMS.maxWheelVel,
                            TankDrive.PARAMS.minProfileAccel,
                            TankDrive.PARAMS.maxProfileAccel,
                            encoderGroups,
                            td.leftMotors,
                            td.rightMotors,
                            leftEncs,
                            rightEncs,
                            parEncs,
                            perpEncs,
                            lazyImu,
                            td.voltageSensor,
                            {
                                MotorFeedforward(
                                    TankDrive.PARAMS.kS,
                                    TankDrive.PARAMS.kV / TankDrive.PARAMS.inPerTick,
                                    TankDrive.PARAMS.kA / TankDrive.PARAMS.inPerTick,
                                )
                            },
                            0,
                        )
                        }
                    }
            } else {
                throw RuntimeException()
            }

            manager.register(metaForClass(AngularRampLogger::class.java), AngularRampLogger(dvf))
            manager.register(metaForClass(ForwardPushTest::class.java), ForwardPushTest(dvf))
            manager.register(metaForClass(ForwardRampLogger::class.java), ForwardRampLogger(dvf))
            manager.register(metaForClass(LateralPushTest::class.java), LateralPushTest(dvf))
            manager.register(metaForClass(LateralRampLogger::class.java), LateralRampLogger(dvf))
            manager.register(
                metaForClass(ManualFeedforwardTuner::class.java),
                ManualFeedforwardTuner(dvf),
            )
            manager.register(
                metaForClass(MecanumMotorDirectionDebugger::class.java),
                MecanumMotorDirectionDebugger(dvf),
            )
            manager.register(
                metaForClass(DeadWheelDirectionDebugger::class.java),
                DeadWheelDirectionDebugger(dvf),
            )

            manager.register(metaForClass(ManualFeedbackTuner::class.java), ManualFeedbackTuner::class.java)
            manager.register(metaForClass(SplineTest::class.java), SplineTest::class.java)
            manager.register(metaForClass(LocalizationTest::class.java), LocalizationTest::class.java)

            // MARS extensions: automatic on-robot tuners for the feedforward constants and feedback
            // gains
            manager.register(metaForClass(YawCouplingTuner::class.java), YawCouplingTuner::class.java)
            manager.register(metaForClass(AxialFeedforwardTuner::class.java), AxialFeedforwardTuner::class.java)
            manager.register(metaForClass(TrackWidthTuner::class.java), TrackWidthTuner::class.java)
            manager.register(metaForClass(PathFeedbackGainTuner::class.java), PathFeedbackGainTuner::class.java)
            if (DRIVE_CLASS == MecanumDrive::class.java) {
                manager.register(
                    metaForClass(LateralFeedforwardTuner::class.java),
                    LateralFeedforwardTuner::class.java,
                )
            }

            manager.register(
                metaForClass(OTOSAngularScalarTuner::class.java),
                OTOSAngularScalarTuner(dvf),
            )
            manager.register(metaForClass(OTOSLinearScalarTuner::class.java), OTOSLinearScalarTuner(dvf))
            manager.register(
                metaForClass(OTOSHeadingOffsetTuner::class.java),
                OTOSHeadingOffsetTuner(dvf),
            )
            manager.register(
                metaForClass(OTOSPositionOffsetTuner::class.java),
                OTOSPositionOffsetTuner(dvf),
            )

            FtcDashboard.getInstance()
                .withConfigRoot { configRoot ->
                    for (c in listOf(
                        AngularRampLogger::class.java,
                        ForwardRampLogger::class.java,
                        LateralRampLogger::class.java,
                        ManualFeedforwardTuner::class.java,
                        MecanumMotorDirectionDebugger::class.java,
                        ManualFeedbackTuner::class.java,
                        YawCouplingTuner::class.java,
                        AxialFeedforwardTuner::class.java,
                        LateralFeedforwardTuner::class.java,
                        TrackWidthTuner::class.java,
                        PathFeedbackGainTuner::class.java,
                        TunerRegression::class.java,
                    )) {
                        configRoot.putVariable(
                            c.simpleName,
                            ReflectionConfig.createVariableFromClass(c),
                        )
                    }
                }
        }
    }
}
