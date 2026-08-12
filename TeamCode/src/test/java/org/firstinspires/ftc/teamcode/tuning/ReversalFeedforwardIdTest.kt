package org.firstinspires.ftc.teamcode.tuning

import kotlin.math.abs
import kotlin.math.round
import kotlin.math.sign
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/** Simulated-plant checks for the ramp (kS/kV) + reverse (kA) identifier. */
class ReversalFeedforwardIdTest {

    @Test
    fun rampAndReverseRecoverNearTruth() {
        val trueKS = 1.07
        val trueKV = 0.15
        val trueKA = 0.025
        val inPerTick = 0.003
        val battery = 12.0
        val dt = 0.005

        val ramp = simulateRamp(trueKS, trueKV, trueKA, 0.1, 0.9, battery, dt)
        val rev = simulateReverse(trueKS, trueKV, trueKA, 0.6, 1.0, 4, battery, dt, ramp.vEnd)

        val fit =
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
                1000.0,
            )

        assertFalse(fit.singular, fit.message)
        assertTrue(fit.rampSamples > 20, "ramp samples=" + fit.rampSamples)
        assertTrue(fit.rampR2 > 0.9, "ramp R2=" + fit.rampR2)

        // kS intercept is the sensitive one; accel-gated ramp should land near truth.
        assertEquals(trueKS, fit.kS, 0.15, "kS got " + fit.kS)
        assertEquals(trueKV * inPerTick, fit.kV, trueKV * inPerTick * 0.15, "kV")
        assertEquals(trueKA * inPerTick, fit.kA, trueKA * inPerTick * 0.45, "kA")
    }

    @Test
    fun negativeKaIsRejectedAsSingular() {
        // Reverse series with voltage/velocity anti-correlated → kA fit ≤ 0.
        val inPerTick = 0.003
        val t = DoubleArray(40)
        val voltage = DoubleArray(40)
        val vel = DoubleArray(40)
        for (i in 0 until 40) {
            t[i] = i * 0.02
            // Constant high positive V while velocity falls (wrong-way residual for +kA).
            voltage[i] = 8.0
            vel[i] = 30.0 - i * 0.8
        }
        // Need a valid ramp first so fit reaches kA.
        val ramp = simulateRamp(1.1, 0.15, 0.02, 0.1, 0.9, 12.0, 0.01)
        val fit =
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
                1000.0,
            )
        // Either singular on kA or (if noise luck) not — but must never accept kA ≤ 0 as success.
        if (!fit.singular) {
            assertTrue(fit.kA > 0, "accepted non-positive kA " + fit.kA)
        } else {
            assertTrue(
                fit.message.contains("kA") ||
                    fit.message.contains("reverse") ||
                    fit.message.contains("identifiable"),
                fit.message,
            )
        }
    }

    @Test
    fun rampFitDropsBreakawayKneeBelowMinTicksPerSec() {
        // Synthetic: breakaway cloud in the 1–3 in/s band (kept by SIGN_DEADBAND=1, dropped by
        // 1000 ticks/s × inPerTick≈3), then linear V = kS + kV·v.
        val kS = 1.1
        val kV = 0.15
        val inPerTick = 0.003 // 1000 ticks/s → 3 in/s
        val nBreak = 40
        val nLinear = 80
        val t = DoubleArray(nBreak + nLinear)
        val voltage = DoubleArray(t.size)
        val vel = DoubleArray(t.size)
        for (i in 0 until nBreak) {
            t[i] = i * 0.01
            // Curved/low V at moderate speed — the knee that biases a joint intercept low.
            val frac = i / (nBreak - 1.0)
            vel[i] = 1.1 + 1.7 * frac // 1.1 … 2.8 in/s
            voltage[i] = 0.4 + 0.5 * frac // well below kS + kV·v
        }
        for (i in 0 until nLinear) {
            val j = nBreak + i
            t[j] = 1.0 + i * 0.02
            val v = 4.0 + i * 0.4 // all above 3 in/s
            vel[j] = v
            voltage[j] = kS + kV * v
        }

        val withCut = ReversalFeedforwardId.fitRamp(voltage, vel, t, 1000 * inPerTick)
        val withoutCut = ReversalFeedforwardId.fitRamp(voltage, vel, t, 1.0)

        assertFalse(withCut.singular, withCut.message)
        assertFalse(withoutCut.singular, withoutCut.message)
        assertEquals(kS, withCut.kS, 0.05, "kS with cut")
        assertEquals(kV, withCut.kVInch, 0.01, "kV with cut")
        // Knee still in the uncut fit pulls the intercept down toward the low-V cloud.
        assertTrue(
            withoutCut.kS < withCut.kS - 0.05,
            "uncut kS=" + withoutCut.kS + " should be below cut kS=" + withCut.kS,
        )
    }

    @Test
    fun highSpeedOnlyCruiseBiasesKsDownWhenKvSlightlyHigh() {
        // Documents the failure mode the user hit: multi-level cruise at high |v| only.
        // A 4% kV overestimate drops kS by ~0.3 V at v≈40 in/s.
        val trueKS = 1.07
        val trueKV = 0.15
        val v = 40.0
        val V = trueKS + trueKV * v // 7.07
        val kvHi = trueKV * 1.04
        val ksBiased = V - kvHi * v
        assertTrue(ksBiased < 0.85, "expected ~0.71, got $ksBiased")
        assertTrue(abs(ksBiased - 0.706) < 0.05 || ksBiased < trueKS - 0.2)
    }

    @Test
    fun corridorFromRampUsesTravelWithMargins() {
        val c = ReversalFeedforwardId.corridorFromRamp(0.0, 80.0, 6.0, 18.0)
        assertTrue(c.usable)
        assertEquals(80.0, c.travelIn, 1e-9)
        assertEquals(1.0, c.travelSign, 0.0)
        assertEquals(6.0, c.nearStart, 1e-9)
        assertEquals(74.0, c.nearEnd, 1e-9)
        assertTrue(ReversalFeedforwardId.reverseReached(5.0, c))
        assertFalse(ReversalFeedforwardId.reverseReached(10.0, c))
        assertTrue(ReversalFeedforwardId.forwardReached(75.0, c))
        assertFalse(ReversalFeedforwardId.forwardReached(70.0, c))
    }

    @Test
    fun corridorTooShortFallsBackToTimed() {
        val c = ReversalFeedforwardId.corridorFromRamp(0.0, 10.0, 6.0, 18.0)
        assertFalse(c.usable)
    }

    @Test
    fun stallDetectionIgnoresBreakawayAndArmsAfterMoving() {
        val stallSpeed = 2.0
        val stallFrac = 0.25
        val stallMinPower = 0.15

        // Still at rest: not armed.
        assertFalse(ReversalFeedforwardId.isStallArmed(false, true, 0.0, 4.0))
        assertFalse(
            ReversalFeedforwardId.isVelocityCollapsed(
                false,
                0.5,
                0.0,
                0.0,
                stallSpeed,
                stallFrac,
                stallMinPower,
            )
        )

        // Slow lateral: never hit MOVING_SPEED, but traveled far enough to arm.
        assertTrue(ReversalFeedforwardId.isStallArmed(false, true, 5.0, 4.0))

        // Moving freely near peak: not collapsed.
        assertFalse(
            ReversalFeedforwardId.isVelocityCollapsed(
                true,
                0.6,
                38.0,
                40.0,
                stallSpeed,
                stallFrac,
                stallMinPower,
            )
        )

        // Wall hit: velocity collapsed vs peak, under power.
        assertTrue(
            ReversalFeedforwardId.isVelocityCollapsed(
                true,
                0.6,
                1.0,
                40.0,
                stallSpeed,
                stallFrac,
                stallMinPower,
            )
        )

        // Low power (coasting): do not treat as wall.
        assertFalse(
            ReversalFeedforwardId.isVelocityCollapsed(
                true,
                0.05,
                0.5,
                40.0,
                stallSpeed,
                stallFrac,
                stallMinPower,
            )
        )

        // Position freeze against wall (noisy residual velocity) after arming via travel.
        assertTrue(ReversalFeedforwardId.isPositionFrozen(true, 0.6, 0.3, stallMinPower, 1.5))
        assertFalse(ReversalFeedforwardId.isPositionFrozen(true, 0.6, 8.0, stallMinPower, 1.5))
        assertFalse(ReversalFeedforwardId.isPositionFrozen(false, 0.6, 0.1, stallMinPower, 1.5))
    }

    @Test
    fun trimPostPeakCollapseDropsTrailingCrashSamples() {
        val t = ArrayList<Double>()
        val v = ArrayList<Double>()
        val vel = ArrayList<Double>()
        // Steady climb then crash into a wall (high V, collapsing vel).
        for (i in 0 until 20) {
            t.add(i * 0.05)
            v.add(2.0 + i * 0.3)
            vel.add(5.0 + i * 1.5) // peaks near 33.5
        }
        val peak = vel[vel.size - 1]
        // 5 crash samples
        for (i in 0 until 5) {
            t.add(1.0 + i * 0.05)
            v.add(8.0)
            vel.add(peak * (0.7 - i * 0.15)) // below 0.85 * peak
        }
        val before = t.size
        ReversalFeedforwardId.trimPostPeakCollapse(t, v, vel, peak)
        assertTrue(t.size < before, "expected trim, size " + t.size)
        assertTrue(vel[vel.size - 1] >= peak * 0.85 - 1e-9)
        assertEquals(t.size, v.size)
        assertEquals(t.size, vel.size)
    }

    private class RampSeries(
        val t: DoubleArray,
        val voltage: DoubleArray,
        val vel: DoubleArray,
        val vEnd: Double,
    )

    private class RevSeries(val t: DoubleArray, val voltage: DoubleArray, val vel: DoubleArray)

    companion object {
        private fun simulateRamp(
            kS: Double,
            kV: Double,
            kA: Double,
            powerPerSec: Double,
            powerMax: Double,
            battery: Double,
            dt: Double,
        ): RampSeries {
            val duration = powerMax / powerPerSec + 0.4
            val n = round(duration / dt).toInt()
            val t = DoubleArray(n)
            val voltage = DoubleArray(n)
            val vel = DoubleArray(n)
            var v = 0.0
            for (i in 0 until n) {
                val ti = i * dt
                val power = minOf(powerPerSec * ti, powerMax)
                val volt = power * battery
                v = step(v, volt, kS, kV, kA, dt)
                t[i] = ti
                voltage[i] = volt
                vel[i] = v
            }
            return RampSeries(t, voltage, vel, v)
        }

        private fun simulateReverse(
            kS: Double,
            kV: Double,
            kA: Double,
            kaPower: Double,
            halfCycle: Double,
            halfCycles: Int,
            battery: Double,
            dt: Double,
            v0: Double,
        ): RevSeries {
            val T = halfCycles * halfCycle
            val n = round(T / dt).toInt()
            val t = DoubleArray(n)
            val voltage = DoubleArray(n)
            val vel = DoubleArray(n)
            var v = v0
            for (i in 0 until n) {
                val ti = i * dt
                val halfIdx = minOf(halfCycles - 1, (ti / halfCycle).toInt())
                val power = if (halfIdx % 2 == 0) -kaPower else kaPower
                val volt = power * battery
                v = step(v, volt, kS, kV, kA, dt)
                t[i] = ti
                voltage[i] = volt
                vel[i] = v
            }
            return RevSeries(t, voltage, vel)
        }

        private fun step(
            v: Double,
            volt: Double,
            kS: Double,
            kV: Double,
            kA: Double,
            dt: Double,
        ): Double {
            if (abs(v) < 0.05) {
                if (abs(volt) <= kS) {
                    return 0.0
                }
                return v + dt * (volt - kS * sign(volt) - kV * v) / kA
            }
            return v + dt * (volt - kS * sign(v) - kV * v) / kA
        }
    }
}
