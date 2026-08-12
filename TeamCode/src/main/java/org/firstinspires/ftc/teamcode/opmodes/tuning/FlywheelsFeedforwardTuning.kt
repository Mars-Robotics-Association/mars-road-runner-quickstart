package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import kotlin.math.abs
import kotlin.math.ln
import kotlin.math.max

/**
 * Automated feedforward tuner for the dual flywheel shooter.
 *
 * When MOTORS_COUPLED is true, all motors are driven together and velocity is read from motors[0],
 * producing one set of gains. When false, each motor is tuned independently, producing separate kS,
 * kV, kA per motor.
 *
 * Procedure (per motor or motor group):
 * 1. Ramps power slowly until the motor(s) overcome stiction (start spinning).
 * 2. Steps from stiction power to full power in NUM_STEPS equal steps.
 * 3. At each step, waits SETTLE_TIME_S for the speed to stabilize, then takes NUM_SAMPLES readings
 *    of velocity and applied voltage.
 * 4. Fits a line voltage = kS + kV * velocity via least-squares regression.
 * 5. Applies a step input and records the velocity rise curve. Fits ln(1 - w/w_final) vs t to
 *    extract the time constant tau, then computes kA = tau * kV. Repeats for STEP_RESPONSE_TRIALS
 *    trials.
 * 6. Displays kS, kV, kA, R², and all data points until the OpMode is stopped.
 */
@Config
@TeleOp(name = "FlywheelsFeedforwardTuning", group = "Tuning")
class FlywheelsFeedforwardTuning : FlywheelsTuningBase() {
    class Params {
        @JvmField var NUM_STEPS = 6
        @JvmField var SETTLE_TIME_S = 5.0
        @JvmField var NUM_SAMPLES = 25
        @JvmField var STICTION_RAMP_RATE = 0.03 // power per second
        @JvmField var STICTION_THRESHOLD_TPS = 50.0

        @JvmField var STEP_RESPONSE_POWER = 0.80
        @JvmField var COAST_STOP_THRESHOLD_TPS = 10.0
        @JvmField var COAST_TIMEOUT_S = 10.0
        @JvmField var STEP_RESPONSE_TRIALS = 3
        @JvmField var STEP_RESPONSE_MAX_TIME_S = 4.0
        @JvmField var OMEGA_LOWER_FRACTION = 0.05
        @JvmField var OMEGA_UPPER_FRACTION = 0.95
    }

    companion object {
        @JvmField var PARAMS = Params()

        private data class LinRegResult(
            val slope: Double,
            val intercept: Double,
            val rSquared: Double,
        )

        private fun linearRegression(x: DoubleArray, y: DoubleArray, n: Int): LinRegResult {
            var sumX = 0.0
            var sumY = 0.0
            var sumXY = 0.0
            var sumX2 = 0.0
            for (i in 0 until n) {
                sumX += x[i]
                sumY += y[i]
                sumXY += x[i] * y[i]
                sumX2 += x[i] * x[i]
            }
            val denom = n * sumX2 - sumX * sumX
            val slope = (n * sumXY - sumX * sumY) / denom
            val intercept = (sumY - slope * sumX) / n

            val meanY = sumY / n
            var ssRes = 0.0
            var ssTot = 0.0
            for (i in 0 until n) {
                val predicted = intercept + slope * x[i]
                ssRes += (y[i] - predicted) * (y[i] - predicted)
                ssTot += (y[i] - meanY) * (y[i] - meanY)
            }
            val rSquared = if (ssTot == 0.0) 1.0 else 1.0 - ssRes / ssTot
            return LinRegResult(slope, intercept, rSquared)
        }
    }

    private data class TuneResult(
        val label: String,
        var kS: Double = 0.0,
        var kV: Double = 0.0,
        var kA: Double = 0.0,
        var rSquared: Double = 0.0,
        var avgStepR2: Double = 0.0,
        var avgTau: Double = 0.0,
        var validTrials: Int = 0,
        var stictionPower: Double = 0.0,
        var stictionVoltage: Double = 0.0,
        var avgVelocities: DoubleArray = DoubleArray(0),
        var avgVoltages: DoubleArray = DoubleArray(0),
    )

    /**
     * Runs a full tuning pass (stiction, steady-state, regression, step response).
     *
     * @param label display name for telemetry
     * @param powerMotors motors to drive during this pass
     * @param encoder motor to read velocity from
     * @param passIndex 0-based pass number (for telemetry)
     * @param totalPasses total number of passes (for telemetry)
     * @return results, or null if the OpMode is stopped or the motor never starts
     */
    private fun runTuningPass(
        label: String,
        powerMotors: Array<DcMotorEx>,
        encoder: DcMotorEx,
        passIndex: Int,
        totalPasses: Int,
    ): TuneResult? {
        val result = TuneResult(label = label)
        val steps = max(PARAMS.NUM_STEPS, 2)

        // ── Phase 1: find stiction ───────────────────────────────────────────
        var power = 0.0
        var lastTime = runtime

        while (nextFrame()) {
            val now = runtime
            val dt = now - lastTime
            lastTime = now
            if (dt < 1e-6) continue

            power = minOf(power + PARAMS.STICTION_RAMP_RATE * dt, 1.0)
            powerMotors.forEach { it.power = power }

            val velocity = encoder.velocity

            telemetry.addData("Pass", "%s (%d/%d)", label, passIndex + 1, totalPasses)
            telemetry.addData("Phase", "Finding stiction")
            telemetry.addData("Power", "%.4f", power)
            telemetry.addData("Velocity (tps)", "%.1f", velocity)

            if (abs(velocity) > PARAMS.STICTION_THRESHOLD_TPS) {
                result.stictionPower = power
                break
            }

            if (power >= 1.0) {
                if (!isStopRequested) {
                    powerMotors.forEach { it.power = 0.0 }
                }
                while (nextFrame()) {
                    telemetry.addData("ERROR", "Motor never started. Check connections.")
                }
                return null
            }
        }

        if (isStopRequested) return null

        result.stictionVoltage = result.stictionPower * batteryVoltage()

        // ── Phase 2: step through powers and collect data ────────────────────
        result.avgVelocities = DoubleArray(steps)
        result.avgVoltages = DoubleArray(steps)

        for (step in 0 until steps) {
            if (isStopRequested) return null

            val stepPower = result.stictionPower + (1.0 - result.stictionPower) * step / (steps - 1)

            powerMotors.forEach { it.power = stepPower }

            // settle (deliberate wall-clock wait; live telemetry via nextFrame)
            val settleStart = runtime
            while (nextFrame() && (runtime - settleStart) < PARAMS.SETTLE_TIME_S) {
                telemetry.addData("Pass", "%s (%d/%d)", label, passIndex + 1, totalPasses)
                telemetry.addData("Phase", "Step %d/%d – Settling", step + 1, steps)
                telemetry.addData("Power", "%.4f", stepPower)
                telemetry.addData("Velocity (tps)", "%.1f", encoder.velocity)
                telemetry.addData(
                    "Time left",
                    "%.1f s",
                    PARAMS.SETTLE_TIME_S - (runtime - settleStart),
                )
            }

            if (isStopRequested) return null

            // sample (pace between samples for sysid; nextFrame refreshes bulk/battery)
            var totalVel = 0.0
            var totalV = 0.0
            val samples = max(PARAMS.NUM_SAMPLES, 1)
            repeat(samples) { s ->
                if (!nextFrame()) return null
                val vel = encoder.velocity
                val battV = batteryVoltage()
                totalVel += vel
                totalV += stepPower * battV

                telemetry.addData("Pass", "%s (%d/%d)", label, passIndex + 1, totalPasses)
                telemetry.addData(
                    "Phase",
                    "Step %d/%d – Sampling %d/%d",
                    step + 1,
                    steps,
                    s + 1,
                    samples,
                )
                telemetry.addData("Power", "%.4f", stepPower)
                telemetry.addData("Velocity (tps)", "%.1f", vel)
                telemetry.addData("Voltage (V)", "%.3f", stepPower * battV)

                sleep(20)
            }

            result.avgVelocities[step] = totalVel / samples
            result.avgVoltages[step] = totalV / samples
        }

        if (isStopRequested) return null

        powerMotors.forEach { it.power = 0.0 }

        // ── Phase 3: least-squares fit  voltage = kS + kV · velocity ─────────
        val fit = linearRegression(result.avgVelocities, result.avgVoltages, steps)
        result.kV = fit.slope
        result.kS = fit.intercept
        result.rSquared = fit.rSquared

        // ── Phase 4: step response for kA ────────────────────────────────────
        val stepVoltage = PARAMS.STEP_RESPONSE_POWER * batteryVoltage()
        val wFinal = (stepVoltage - result.kS) / result.kV

        val tauValues = mutableListOf<Double>()
        val stepRSquaredValues = mutableListOf<Double>()

        if (wFinal > 0) {
            for (trial in 0 until PARAMS.STEP_RESPONSE_TRIALS) {
                if (isStopRequested) break

                // Coast to stop
                powerMotors.forEach { it.power = 0.0 }
                val coastStart = runtime
                while (nextFrame()) {
                    val vel = abs(encoder.velocity)
                    telemetry.addData("Pass", "%s (%d/%d)", label, passIndex + 1, totalPasses)
                    telemetry.addData(
                        "Phase",
                        "Step Response – Coasting (trial %d/%d)",
                        trial + 1,
                        PARAMS.STEP_RESPONSE_TRIALS,
                    )
                    telemetry.addData("Velocity (tps)", "%.1f", vel)
                    if (vel < PARAMS.COAST_STOP_THRESHOLD_TPS) break
                    if (runtime - coastStart > PARAMS.COAST_TIMEOUT_S) break
                    sleep(10)
                }
                if (isStopRequested) break
                sleep(200) // brief pause at rest

                // Apply step and sample
                val sampleTimes = mutableListOf<Double>()
                val sampleVels = mutableListOf<Double>()
                val stepStart = runtime
                powerMotors.forEach { it.power = PARAMS.STEP_RESPONSE_POWER }

                while (nextFrame() && (runtime - stepStart) < PARAMS.STEP_RESPONSE_MAX_TIME_S) {
                    val t = runtime - stepStart
                    val vel = encoder.velocity
                    sampleTimes.add(t)
                    sampleVels.add(vel)

                    telemetry.addData("Pass", "%s (%d/%d)", label, passIndex + 1, totalPasses)
                    telemetry.addData(
                        "Phase",
                        "Step Response – Sampling (trial %d/%d)",
                        trial + 1,
                        PARAMS.STEP_RESPONSE_TRIALS,
                    )
                    telemetry.addData("Time", "%.3f s", t)
                    telemetry.addData("Velocity (tps)", "%.1f", vel)
                    telemetry.addData("w_final (tps)", "%.1f", wFinal)
                }

                if (isStopRequested) break
                powerMotors.forEach { it.power = 0.0 }

                // Linearized regression: ln(1 - w/w_final) = -t/tau
                val regT = mutableListOf<Double>()
                val regY = mutableListOf<Double>()

                for (i in sampleTimes.indices) {
                    val ratio = sampleVels[i] / wFinal
                    if (
                        ratio < PARAMS.OMEGA_LOWER_FRACTION || ratio > PARAMS.OMEGA_UPPER_FRACTION
                    ) {
                        continue
                    }
                    regT.add(sampleTimes[i])
                    regY.add(ln(1.0 - ratio))
                }

                if (regT.size >= 3) {
                    val tArr = regT.toDoubleArray()
                    val yArr = regY.toDoubleArray()
                    val stepFit = linearRegression(tArr, yArr, regT.size)
                    if (stepFit.slope < 0) {
                        tauValues.add(-1.0 / stepFit.slope)
                        stepRSquaredValues.add(stepFit.rSquared)
                    }
                }
            }
        }

        // Compute kA from averaged tau
        result.validTrials = tauValues.size
        if (result.validTrials > 0) {
            result.avgTau = tauValues.average()
            result.avgStepR2 = stepRSquaredValues.average()
            result.kA = result.avgTau * result.kV
        }

        return result
    }

    override fun runOpModeInternal() {
        telemetry.addData("Status", "Ready. Press Start to begin tuning.")
        telemetry.update()

        waitForStart()
        if (isStopRequested) return

        val numPasses = if (HARDWARE.MOTORS_COUPLED) 1 else motors.size
        val results = ArrayList<TuneResult>(numPasses)

        for (pass in 0 until numPasses) {
            if (isStopRequested) return

            val (label, powerMotors, encoder) =
                if (HARDWARE.MOTORS_COUPLED) {
                    Triple("All motors", motors, motors[0])
                } else {
                    Triple(HARDWARE.MOTOR_NAMES[pass], arrayOf(motors[pass]), motors[pass])
                }

            val result = runTuningPass(label, powerMotors, encoder, pass, numPasses) ?: return
            results.add(result)
        }

        // ── Display results until stopped ────────────────────────────────────
        while (nextFrame()) {
            for (r in results) {
                val pfx = if (results.size > 1) "${r.label} " else ""
                telemetry.addData("── ${r.label} RESULTS ──", "")
                telemetry.addData("${pfx}kS", "%.4f V", r.kS)
                telemetry.addData("${pfx}kV", "%.6f V·s/tick", r.kV)
                if (r.validTrials > 0) {
                    telemetry.addData("${pfx}kA", "%.6f V·s²/tick", r.kA)
                    telemetry.addData("${pfx}tau", "%.4f s", r.avgTau)
                    telemetry.addData("${pfx}kA R²", "%.6f", r.avgStepR2)
                    telemetry.addData(
                        "${pfx}kA valid trials",
                        "%d / %d",
                        r.validTrials,
                        PARAMS.STEP_RESPONSE_TRIALS,
                    )
                } else {
                    telemetry.addData("${pfx}kA", "FAILED – no valid trials")
                }
                telemetry.addData("${pfx}kS/kV R²", "%.6f", r.rSquared)
                telemetry.addData("${pfx}stiction power", "%.4f", r.stictionPower)
                telemetry.addData("${pfx}stiction voltage", "%.3f V", r.stictionVoltage)
                telemetry.addLine("")
                telemetry.addData("── ${r.label} PASTE ──", "")
                telemetry.addData("${pfx}  kS =", "%.4f", r.kS)
                telemetry.addData("${pfx}  kV =", "%.6f", r.kV)
                telemetry.addData("${pfx}  kA =", "%.6f", r.kA)
                telemetry.addLine("")
                telemetry.addData("── ${r.label} DATA POINTS ──", "")
                for (i in r.avgVelocities.indices) {
                    telemetry.addData(
                        "${pfx}Step ${i + 1}",
                        "%.1f tps @ %.3f V",
                        r.avgVelocities[i],
                        r.avgVoltages[i],
                    )
                }
                telemetry.addLine("")
            }
        }
    }
}
