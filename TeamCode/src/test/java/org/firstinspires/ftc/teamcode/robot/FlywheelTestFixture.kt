package org.firstinspires.ftc.teamcode.robot

import java.util.Random
import org.marsroboticsassociation.controllib.control.FlywheelSimple
import org.marsroboticsassociation.controllib.sim.FlywheelMotorSim

/**
 * Shared simulation infrastructure for flywheel controller unit tests.
 *
 * Holds the [SimMotorAdapter] stub, the nominal hub voltage, and a [makeSim] factory so each test
 * class only configures what's specific to the controller under test (noise level, disturbance
 * offset, etc.).
 */
internal object FlywheelTestFixture {

    /** Nominal hub voltage used across all flywheel sim tests. */
    const val HUB_VOLTAGE = 13.75

    /** RNG seed shared by all flywheel tests for reproducible noise sequences. */
    const val SEED = 42L

    /** Approximate velocity noise std-dev in TPS for tests that inject measurements directly. */
    const val NOISE_STD_TPS = 10.0

    /** Create a new seeded RNG for test-local use. */
    fun makeRng(): Random = Random(SEED)

    /**
     * DcMotorEx stub that bridges [FlywheelMotorSim] to the controller under test.
     *
     * Only `getVelocity()`, `getCurrentPosition()`, and `setPower()` are called by flywheel
     * controllers; all other DcMotorEx methods delegate to the null inner motor and are never
     * reached during tests.
     */
    class SimMotorAdapter(private val sim: FlywheelMotorSim) : EncapsulatedDcMotorEx(null) {
        var lastPower = 0.0

        override val encoderVelocity: Double
            get() = sim.getVelocityTps()

        override fun getCurrentPosition(): Int = sim.getPositionTicks()

        override fun setPower(power: Double) {
            lastPower = power
        }

        override val hubVoltage: Double
            get() = HUB_VOLTAGE
    }

    /**
     * Create a flywheel plant simulation with the standard noise settings.
     *
     * Uses the TPS-unit characterization constants from `FlywheelSimple.PARAMS`, which are
     * unit-equivalent to the SI constants in `FlywheelStateSpace.PARAMS` and represent the same
     * physical plant.
     */
    fun makeSim(): FlywheelMotorSim {
        return FlywheelMotorSim(FlywheelSimple.PARAMS.kV, FlywheelSimple.PARAMS.kA)
    }
}
