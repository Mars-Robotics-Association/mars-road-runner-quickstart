package org.firstinspires.ftc.teamcode.tuning;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/** Simulated-plant checks for the ramp (kS/kV) + reverse (kA) identifier. */
public class ReversalFeedforwardIdTest {

    @Test
    void rampAndReverseRecoverNearTruth() {
        double trueKS = 1.07;
        double trueKV = 0.15;
        double trueKA = 0.025;
        double inPerTick = 0.003;
        double battery = 12.0;
        double dt = 0.005;

        RampSeries ramp = simulateRamp(trueKS, trueKV, trueKA, 0.1, 0.9, battery, dt);
        RevSeries rev =
                simulateReverse(trueKS, trueKV, trueKA, 0.6, 1.0, 4, battery, dt, ramp.vEnd);

        ReversalFeedforwardId.Result fit =
                ReversalFeedforwardId.fitRampAndReverse(
                        ramp.t,
                        ramp.voltage,
                        ramp.vel,
                        rev.t,
                        rev.voltage,
                        rev.vel,
                        inPerTick,
                        1.0,
                        0.12,
                        0.04,
                        1000);

        assertFalse(fit.singular, fit.message);
        assertTrue(fit.rampSamples > 20, "ramp samples=" + fit.rampSamples);
        assertTrue(fit.rampR2 > 0.9, "ramp R2=" + fit.rampR2);

        // kS intercept is the sensitive one; accel-gated ramp should land near truth.
        assertEquals(trueKS, fit.kS, 0.15, "kS got " + fit.kS);
        assertEquals(trueKV * inPerTick, fit.kV, trueKV * inPerTick * 0.15, "kV");
        assertEquals(trueKA * inPerTick, fit.kA, trueKA * inPerTick * 0.45, "kA");
    }

    @Test
    void negativeKaIsRejectedAsSingular() {
        // Reverse series with voltage/velocity anti-correlated → kA fit ≤ 0.
        double inPerTick = 0.003;
        double[] t = new double[40];
        double[] voltage = new double[40];
        double[] vel = new double[40];
        for (int i = 0; i < 40; i++) {
            t[i] = i * 0.02;
            // Constant high positive V while velocity falls (wrong-way residual for +kA).
            voltage[i] = 8.0;
            vel[i] = 30.0 - i * 0.8;
        }
        // Need a valid ramp first so fit reaches kA.
        RampSeries ramp = simulateRamp(1.1, 0.15, 0.02, 0.1, 0.9, 12.0, 0.01);
        ReversalFeedforwardId.Result fit =
                ReversalFeedforwardId.fitRampAndReverse(
                        ramp.t,
                        ramp.voltage,
                        ramp.vel,
                        t,
                        voltage,
                        vel,
                        inPerTick,
                        1.0,
                        0.12,
                        0.04,
                        1000);
        // Either singular on kA or (if noise luck) not — but must never accept kA ≤ 0 as success.
        if (!fit.singular) {
            assertTrue(fit.kA > 0, "accepted non-positive kA " + fit.kA);
        } else {
            assertTrue(
                    fit.message.contains("kA")
                            || fit.message.contains("reverse")
                            || fit.message.contains("identifiable"),
                    fit.message);
        }
    }

    @Test
    void rampFitDropsBreakawayKneeBelowMinTicksPerSec() {
        // Synthetic: breakaway cloud in the 1–3 in/s band (kept by SIGN_DEADBAND=1, dropped by
        // 1000 ticks/s × inPerTick≈3), then linear V = kS + kV·v.
        double kS = 1.1;
        double kV = 0.15;
        double inPerTick = 0.003; // 1000 ticks/s → 3 in/s
        int nBreak = 40;
        int nLinear = 80;
        double[] t = new double[nBreak + nLinear];
        double[] voltage = new double[t.length];
        double[] vel = new double[t.length];
        for (int i = 0; i < nBreak; i++) {
            t[i] = i * 0.01;
            // Curved/low V at moderate speed — the knee that biases a joint intercept low.
            double frac = i / (nBreak - 1.0);
            vel[i] = 1.1 + 1.7 * frac; // 1.1 … 2.8 in/s
            voltage[i] = 0.4 + 0.5 * frac; // well below kS + kV·v
        }
        for (int i = 0; i < nLinear; i++) {
            int j = nBreak + i;
            t[j] = 1.0 + i * 0.02;
            double v = 4.0 + i * 0.4; // all above 3 in/s
            vel[j] = v;
            voltage[j] = kS + kV * v;
        }

        ReversalFeedforwardId.RampFit withCut =
                ReversalFeedforwardId.fitRamp(voltage, vel, t, 1000 * inPerTick);
        ReversalFeedforwardId.RampFit withoutCut =
                ReversalFeedforwardId.fitRamp(voltage, vel, t, 1.0);

        assertFalse(withCut.singular, withCut.message);
        assertFalse(withoutCut.singular, withoutCut.message);
        assertEquals(kS, withCut.kS, 0.05, "kS with cut");
        assertEquals(kV, withCut.kVInch, 0.01, "kV with cut");
        // Knee still in the uncut fit pulls the intercept down toward the low-V cloud.
        assertTrue(
                withoutCut.kS < withCut.kS - 0.05,
                "uncut kS=" + withoutCut.kS + " should be below cut kS=" + withCut.kS);
    }

    @Test
    void highSpeedOnlyCruiseBiasesKsDownWhenKvSlightlyHigh() {
        // Documents the failure mode the user hit: multi-level cruise at high |v| only.
        // A 4% kV overestimate drops kS by ~0.3 V at v≈40 in/s.
        double trueKS = 1.07;
        double trueKV = 0.15;
        double v = 40.0;
        double V = trueKS + trueKV * v; // 7.07
        double kvHi = trueKV * 1.04;
        double ksBiased = V - kvHi * v;
        assertTrue(ksBiased < 0.85, "expected ~0.71, got " + ksBiased);
        assertTrue(Math.abs(ksBiased - 0.706) < 0.05 || ksBiased < trueKS - 0.2);
    }

    @Test
    void corridorFromRampUsesTravelWithMargins() {
        ReversalFeedforwardId.Corridor c = ReversalFeedforwardId.corridorFromRamp(0, 80, 6, 18);
        assertTrue(c.usable);
        assertEquals(80, c.travelIn, 1e-9);
        assertEquals(1.0, c.travelSign, 0);
        assertEquals(6, c.nearStart, 1e-9);
        assertEquals(74, c.nearEnd, 1e-9);
        assertTrue(ReversalFeedforwardId.reverseReached(5.0, c));
        assertFalse(ReversalFeedforwardId.reverseReached(10.0, c));
        assertTrue(ReversalFeedforwardId.forwardReached(75.0, c));
        assertFalse(ReversalFeedforwardId.forwardReached(70.0, c));
    }

    @Test
    void corridorTooShortFallsBackToTimed() {
        ReversalFeedforwardId.Corridor c = ReversalFeedforwardId.corridorFromRamp(0, 10, 6, 18);
        assertFalse(c.usable);
    }

    @Test
    void stallDetectionIgnoresBreakawayAndArmsAfterMoving() {
        double stallSpeed = 2.0;
        double stallFrac = 0.25;
        double stallMinPower = 0.15;

        // Still at rest: not armed.
        assertFalse(ReversalFeedforwardId.isStallArmed(false, true, 0.0, 4.0));
        assertFalse(
                ReversalFeedforwardId.isVelocityCollapsed(
                        false, 0.5, 0.0, 0.0, stallSpeed, stallFrac, stallMinPower));

        // Slow lateral: never hit MOVING_SPEED, but traveled far enough to arm.
        assertTrue(ReversalFeedforwardId.isStallArmed(false, true, 5.0, 4.0));

        // Moving freely near peak: not collapsed.
        assertFalse(
                ReversalFeedforwardId.isVelocityCollapsed(
                        true, 0.6, 38.0, 40.0, stallSpeed, stallFrac, stallMinPower));

        // Wall hit: velocity collapsed vs peak, under power.
        assertTrue(
                ReversalFeedforwardId.isVelocityCollapsed(
                        true, 0.6, 1.0, 40.0, stallSpeed, stallFrac, stallMinPower));

        // Low power (coasting): do not treat as wall.
        assertFalse(
                ReversalFeedforwardId.isVelocityCollapsed(
                        true, 0.05, 0.5, 40.0, stallSpeed, stallFrac, stallMinPower));

        // Position freeze against wall (noisy residual velocity) after arming via travel.
        assertTrue(ReversalFeedforwardId.isPositionFrozen(true, 0.6, 0.3, stallMinPower, 1.5));
        assertFalse(ReversalFeedforwardId.isPositionFrozen(true, 0.6, 8.0, stallMinPower, 1.5));
        assertFalse(ReversalFeedforwardId.isPositionFrozen(false, 0.6, 0.1, stallMinPower, 1.5));
    }

    @Test
    void trimPostPeakCollapseDropsTrailingCrashSamples() {
        java.util.List<Double> t = new java.util.ArrayList<>();
        java.util.List<Double> v = new java.util.ArrayList<>();
        java.util.List<Double> vel = new java.util.ArrayList<>();
        // Steady climb then crash into a wall (high V, collapsing vel).
        for (int i = 0; i < 20; i++) {
            t.add(i * 0.05);
            v.add(2.0 + i * 0.3);
            vel.add(5.0 + i * 1.5); // peaks near 33.5
        }
        double peak = vel.get(vel.size() - 1);
        // 5 crash samples
        for (int i = 0; i < 5; i++) {
            t.add(1.0 + i * 0.05);
            v.add(8.0);
            vel.add(peak * (0.7 - i * 0.15)); // below 0.85 * peak
        }
        int before = t.size();
        ReversalFeedforwardId.trimPostPeakCollapse(t, v, vel, peak);
        assertTrue(t.size() < before, "expected trim, size " + t.size());
        assertTrue(vel.get(vel.size() - 1) >= peak * 0.85 - 1e-9);
        assertEquals(t.size(), v.size());
        assertEquals(t.size(), vel.size());
    }

    private static final class RampSeries {
        final double[] t, voltage, vel;
        final double vEnd;

        RampSeries(double[] t, double[] voltage, double[] vel, double vEnd) {
            this.t = t;
            this.voltage = voltage;
            this.vel = vel;
            this.vEnd = vEnd;
        }
    }

    private static final class RevSeries {
        final double[] t, voltage, vel;

        RevSeries(double[] t, double[] voltage, double[] vel) {
            this.t = t;
            this.voltage = voltage;
            this.vel = vel;
        }
    }

    private static RampSeries simulateRamp(
            double kS,
            double kV,
            double kA,
            double powerPerSec,
            double powerMax,
            double battery,
            double dt) {
        double duration = powerMax / powerPerSec + 0.4;
        int n = (int) Math.round(duration / dt);
        double[] t = new double[n];
        double[] voltage = new double[n];
        double[] vel = new double[n];
        double v = 0;
        for (int i = 0; i < n; i++) {
            double ti = i * dt;
            double power = Math.min(powerPerSec * ti, powerMax);
            double volt = power * battery;
            v = step(v, volt, kS, kV, kA, dt);
            t[i] = ti;
            voltage[i] = volt;
            vel[i] = v;
        }
        return new RampSeries(t, voltage, vel, v);
    }

    private static RevSeries simulateReverse(
            double kS,
            double kV,
            double kA,
            double kaPower,
            double halfCycle,
            int halfCycles,
            double battery,
            double dt,
            double v0) {
        double T = halfCycles * halfCycle;
        int n = (int) Math.round(T / dt);
        double[] t = new double[n];
        double[] voltage = new double[n];
        double[] vel = new double[n];
        double v = v0;
        for (int i = 0; i < n; i++) {
            double ti = i * dt;
            int halfIdx = Math.min(halfCycles - 1, (int) (ti / halfCycle));
            double power = (halfIdx % 2 == 0) ? -kaPower : kaPower;
            double volt = power * battery;
            v = step(v, volt, kS, kV, kA, dt);
            t[i] = ti;
            voltage[i] = volt;
            vel[i] = v;
        }
        return new RevSeries(t, voltage, vel);
    }

    private static double step(double v, double volt, double kS, double kV, double kA, double dt) {
        if (Math.abs(v) < 0.05) {
            if (Math.abs(volt) <= kS) {
                return 0;
            }
            return v + dt * (volt - kS * Math.signum(volt) - kV * v) / kA;
        }
        return v + dt * (volt - kS * Math.signum(v) - kV * v) / kA;
    }
}
