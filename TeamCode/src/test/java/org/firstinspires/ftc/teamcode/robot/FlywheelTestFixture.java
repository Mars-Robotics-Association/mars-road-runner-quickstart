package org.firstinspires.ftc.teamcode.robot;

import org.marsroboticsassociation.controllib.control.FlywheelSimple;
import org.marsroboticsassociation.controllib.sim.FlywheelMotorSim;

import java.util.Random;

/**
 * Shared simulation infrastructure for flywheel controller unit tests.
 *
 * <p>Holds the {@link SimMotorAdapter} stub, the nominal hub voltage, and a
 * {@link #makeSim} factory so each test class only configures what's specific
 * to the controller under test (noise level, disturbance offset, etc.).
 */
class FlywheelTestFixture {

    /** Nominal hub voltage used across all flywheel sim tests. */
    static final double HUB_VOLTAGE = 13.75;

    /** RNG seed shared by all flywheel tests for reproducible noise sequences. */
    static final long SEED = 42L;

    /** Approximate velocity noise std-dev in TPS for tests that inject measurements directly. */
    static final double NOISE_STD_TPS = 10.0;

    /** Create a new seeded RNG for test-local use. */
    static Random makeRng() { return new Random(SEED); }

    /**
     * DcMotorEx stub that bridges {@link FlywheelMotorSim} to the controller under test.
     *
     * <p>Only {@code getVelocity()}, {@code getCurrentPosition()}, and {@code setPower()}
     * are called by flywheel controllers; all other DcMotorEx methods delegate to the
     * null inner motor and are never reached during tests.
     */
    static class SimMotorAdapter extends EncapsulatedDcMotorEx {
        private final FlywheelMotorSim sim;
        double lastPower = 0.0;

        SimMotorAdapter(FlywheelMotorSim sim) {
            super(null);
            this.sim = sim;
        }

        @Override public double getVelocity()          { return sim.getVelocityTps(); }
        @Override public int    getCurrentPosition()   { return sim.getPositionTicks(); }
        @Override public void   setPower(double power) { lastPower = power; }
        @Override public double getHubVoltage()        { return HUB_VOLTAGE; }
    }

    /**
     * Create a flywheel plant simulation with the standard noise settings.
     *
     * <p>Uses the TPS-unit characterization constants from {@code FlywheelSimple.PARAMS},
     * which are unit-equivalent to the SI constants in {@code FlywheelStateSpace.PARAMS}
     * and represent the same physical plant.
     */
    static FlywheelMotorSim makeSim() {
        return new FlywheelMotorSim(
                FlywheelSimple.PARAMS.kV,
                FlywheelSimple.PARAMS.kA);
    }
}
