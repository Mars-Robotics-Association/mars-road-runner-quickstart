package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.ln
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sign
import kotlin.math.sin
import kotlin.math.sqrt
import org.firstinspires.ftc.teamcode.opmodes.base.MarsLinearOpMode

/**
 * Automated feedforward tuner for a single arm using ArmFeedforward (WPILib port).
 *
 * Produces kS, kG, kV, kA in the units ArmFeedforward expects: V, V, V·s/rad, V·s²/rad.
 *
 * The sin/cos dual-regressor model `V = kS·sign(ω) + c1·cos(θ) + c2·sin(θ) + kV·ω` absorbs both
 * encoder-zero offset and any CM angular offset (L-shaped arm, offset attachment), so the encoder
 * does not need to be zeroed at horizontal. After regression: `kG = sqrt(c1²+c2²)`, `φ = atan2(-c2,
 * c1)` = angle where arm is horizontal in encoder ticks.
 *
 * Procedure:
 * 1. Position arm at one end of its range, press Start, then press A.
 * 2. Quasistatic forward sweep: power ramps up slowly while recording samples.
 * 3. Move arm back toward start, press A.
 * 4. Quasistatic backward sweep: power ramps negative while recording samples.
 * 5. OLS regression yields kS, kG (=√(c1²+c2²)), kV, φ, and R².
 * 6. Step response (STEP_TRIALS trials): gravity-subtracted first-order fit → kA.
 * 7. Results displayed until OpMode is stopped.
 */
@Config
@TeleOp(name = "ArmFeedforwardTuning", group = "Tuning")
class ArmFeedforwardTuning : MarsLinearOpMode() {
    class Params {
        /** Hardware map name of the arm motor. */
        @JvmField var MOTOR_NAME = "arm"

        /**
         * Encoder ticks for one full arm revolution (geared). = motor_encoder_ppr ×
         * external_gear_ratio. Incorrect value shifts kV/kA but does not affect kS or kG.
         */
        @JvmField var TICKS_PER_ARM_REV = 537.7

        /** Stop forward sweep at this many encoder ticks past the zeroed start position. */
        @JvmField var SOFT_LIMIT_FWD = 2000

        /** Stop backward sweep at this encoder tick offset (0 = back to start). */
        @JvmField var SOFT_LIMIT_BACK = 0

        /** Power ramp rate during quasistatic sweeps, in power/second. */
        @JvmField var QUASISTATIC_RAMP = 0.03

        /** Minimum arm angular velocity in rad/s; slower samples are ignored. */
        @JvmField var MIN_VELOCITY_RAD = 0.05

        /** Fraction of battery voltage applied during step response trials. */
        @JvmField var STEP_FRACTION = 0.50

        /** Number of step-response trials. */
        @JvmField var STEP_TRIALS = 3

        /** Maximum duration of each step-response trial in seconds. */
        @JvmField var STEP_MAX_TIME_S = 3.0

        /** Arm is considered "at rest" when |velocity| < this threshold in ticks/s. */
        @JvmField var COAST_THRESHOLD_TPS = 20.0

        /** Maximum time to wait for arm to come to rest before starting a step trial. */
        @JvmField var COAST_TIMEOUT_S = 8.0

        /** Step-fit lower exclusion: skip points where ω/ω_∞ < this fraction. */
        @JvmField var OMEGA_LOWER_FRACTION = 0.05

        /** Step-fit upper exclusion: skip points where ω/ω_∞ > this fraction. */
        @JvmField var OMEGA_UPPER_FRACTION = 0.95
    }

    companion object {
        @JvmField var PARAMS = Params()

        // ── Gaussian elimination with partial pivoting (4×4) ─────────────────────

        /**
         * Solves the 4×4 linear system A·x = b via Gaussian elimination with partial pivoting. A
         * and b are modified in-place; pass copies if they need to be preserved.
         *
         * @return solution vector, or null if A is (near-)singular
         */
        private fun solveGaussian(A: Array<DoubleArray>, b: DoubleArray): DoubleArray? {
            val n = 4
            for (col in 0 until n) {
                // Find row with largest pivot magnitude in column col.
                var maxRow = col
                for (row in col + 1 until n) {
                    if (abs(A[row][col]) > abs(A[maxRow][col])) maxRow = row
                }
                // Swap rows.
                val tmpRow = A[col]
                A[col] = A[maxRow]
                A[maxRow] = tmpRow
                val tmpB = b[col]
                b[col] = b[maxRow]
                b[maxRow] = tmpB

                if (abs(A[col][col]) < 1e-12) return null // singular

                // Eliminate below pivot.
                for (row in col + 1 until n) {
                    val factor = A[row][col] / A[col][col]
                    for (j in col until n) A[row][j] -= factor * A[col][j]
                    b[row] -= factor * b[col]
                }
            }
            // Back-substitution.
            val x = DoubleArray(n)
            for (k in n - 1 downTo 0) {
                x[k] = b[k]
                for (j in k + 1 until n) x[k] -= A[k][j] * x[j]
                x[k] /= A[k][k]
            }
            return x
        }

        // ── v^T · M · v ──────────────────────────────────────────────────────────

        private fun quadForm(v: DoubleArray, M: Array<DoubleArray>): Double {
            var result = 0.0
            for (i in v.indices) {
                for (j in v.indices) result += v[i] * M[i][j] * v[j]
            }
            return result
        }

        // ── Dot product ──────────────────────────────────────────────────────────

        private fun dot(a: DoubleArray, b: DoubleArray): Double {
            var s = 0.0
            for (i in a.indices) s += a[i] * b[i]
            return s
        }

        // ── Simple OLS linear regression (for step-response fit) ─────────────────

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
            if (abs(denom) < 1e-12) {
                return LinRegResult(slope = 0.0, intercept = sumY / n, rSquared = 0.0)
            }
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

        private data class LinRegResult(
            val slope: Double,
            val intercept: Double,
            val rSquared: Double,
        )
    }

    // ── Online normal-equation accumulators ───────────────────────────────────
    // Model: V = kS·sign(ω) + c1·cos(θ) + c2·sin(θ) + kV·ω
    // Feature vector x = [sign(ω), cos(θ), sin(θ), ω], indices 0–3.
    private val XtX = Array(4) { DoubleArray(4) }
    private val XtY = DoubleArray(4)
    private var qsSumY = 0.0
    private var qsSumY2 = 0.0
    private var qsN = 0

    override fun runOpMode() {
        initRobot()

        // ── Hardware init ────────────────────────────────────────────────────
        val motor = hardwareMap.get(DcMotorEx::class.java, PARAMS.MOTOR_NAME)
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        val ticksToRad = 2.0 * PI / PARAMS.TICKS_PER_ARM_REV

        // ── Wait for start ───────────────────────────────────────────────────
        telemetry.addData("Status", "Move arm to one end of range, then press Start.")
        telemetry.update()
        waitForStart()
        if (isStopRequested) return

        // ── Phase 1: Quasistatic forward sweep ───────────────────────────────
        if (
            !waitForA(
                "Phase 1/5 – Move arm to one end of range, then press A to begin forward sweep."
            )
        ) {
            return
        }

        // waitForA ends on a nextFrame (fresh bulk); position after operator move is current.
        val tickOffset = motor.currentPosition
        val samples1 =
            runQuasistaticSweep(motor, ticksToRad, tickOffset, true, "1/5 – Forward sweep")
        if (isStopRequested) return
        motor.power = 0.0

        // ── Phase 2: Quasistatic backward sweep ──────────────────────────────
        if (
            !waitForA(
                "Phase 2/5 – Forward sweep done ($samples1 samples). Move arm back, then press A" +
                    " for reverse sweep."
            )
        ) {
            return
        }

        val samples2 =
            runQuasistaticSweep(motor, ticksToRad, tickOffset, false, "2/5 – Reverse sweep")
        if (isStopRequested) return
        motor.power = 0.0

        // ── Phase 3: OLS regression ──────────────────────────────────────────
        var kS = 0.0
        var c1 = 0.0
        var c2 = 0.0
        var kV = 0.0
        var kG = 0.0
        var phiDeg = 0.0
        var rSquared = 0.0
        var beta: DoubleArray? = null

        if (qsN >= 5) {
            // Deep-copy so Gaussian elimination doesn't clobber the accumulators.
            val A = Array(4) { i -> XtX[i].copyOf() }
            val b = XtY.copyOf()
            beta = solveGaussian(A, b)
        }

        beta?.let { b ->
            kS = b[0]
            c1 = b[1]
            c2 = b[2]
            kV = b[3]
            kG = sqrt(c1 * c1 + c2 * c2)
            phiDeg = Math.toDegrees(atan2(-c2, c1))

            // R² using online accumulators (no individual samples stored).
            // SST = sumY² − sumY²/n
            // SSE = sumY2 − 2·βᵀ·XᵀY + βᵀ·XᵀX·β
            val sst = qsSumY2 - qsSumY * qsSumY / qsN
            val sse = qsSumY2 - 2.0 * dot(b, XtY) + quadForm(b, XtX)
            rSquared = if (sst == 0.0) 1.0 else 1.0 - sse / sst
        }

        // ── Phase 4: Step response for kA ────────────────────────────────────
        val tauList = mutableListOf<Double>()
        val stepR2List = mutableListOf<Double>()

        if (beta != null && kV > 0) {
            for (trial in 0 until PARAMS.STEP_TRIALS) {
                if (isStopRequested) break

                // Coast to rest
                motor.power = 0.0
                val coastStart = runtime
                while (nextFrame()) {
                    val velTps = abs(motor.velocity)
                    telemetry.addData(
                        "Phase",
                        "4/5 – Step response: coasting to rest (trial %d/%d)",
                        trial + 1,
                        PARAMS.STEP_TRIALS,
                    )
                    telemetry.addData("Velocity (tps)", "%.1f", velTps)
                    if (velTps < PARAMS.COAST_THRESHOLD_TPS) break
                    if (runtime - coastStart > PARAMS.COAST_TIMEOUT_S) break
                }
                if (isStopRequested) break

                // Prompt user to position arm near vertical (gravity ≈ 0 → best kA accuracy).
                if (
                    !waitForA(
                        "Trial ${trial + 1}/${PARAMS.STEP_TRIALS} – Position arm near vertical" +
                            " (gravity ≈ 0), then press A."
                    )
                ) {
                    return
                }

                // Snapshot after waitForA's nextFrame (fresh bulk + battery epoch).
                val theta0 = (motor.currentPosition - tickOffset) * ticksToRad
                val battV0 = batteryVoltage()

                // Gravity-subtracted net voltage and predicted terminal velocity.
                val vNet = PARAMS.STEP_FRACTION * battV0 - c1 * cos(theta0) - c2 * sin(theta0)
                val wInf = (vNet - kS) / kV

                if (wInf <= 0) {
                    telemetry.addData(
                        "Phase",
                        "Trial %d/%d – wInf ≤ 0 (arm orientation problem). Skipping.",
                        trial + 1,
                        PARAMS.STEP_TRIALS,
                    )
                    continue
                }

                // Apply step and record (t, ω).
                val sampleT = mutableListOf<Double>()
                val sampleW = mutableListOf<Double>()

                val stepStart = runtime
                motor.power = PARAMS.STEP_FRACTION

                while (nextFrame() && (runtime - stepStart) < PARAMS.STEP_MAX_TIME_S) {
                    val t = runtime - stepStart
                    val omega = motor.velocity * ticksToRad
                    sampleT.add(t)
                    sampleW.add(omega)

                    telemetry.addData(
                        "Phase",
                        "4/5 – Step response: sampling (trial %d/%d)",
                        trial + 1,
                        PARAMS.STEP_TRIALS,
                    )
                    telemetry.addData("Time", "%.3f s", t)
                    telemetry.addData("ω (rad/s)", "%.3f", omega)
                    telemetry.addData("ω_∞ (rad/s)", "%.3f", wInf)
                }
                if (isStopRequested) break
                motor.power = 0.0

                // Linearized fit: ln(1 − ω/ω_∞) vs t → slope = −1/τ.
                val regT = mutableListOf<Double>()
                val regY = mutableListOf<Double>()
                for (i in sampleT.indices) {
                    val ratio = sampleW[i] / wInf
                    if (
                        ratio < PARAMS.OMEGA_LOWER_FRACTION || ratio > PARAMS.OMEGA_UPPER_FRACTION
                    ) {
                        continue
                    }
                    regT.add(sampleT[i])
                    regY.add(ln(1.0 - ratio))
                }

                if (regT.size >= 3) {
                    val fit =
                        linearRegression(regT.toDoubleArray(), regY.toDoubleArray(), regT.size)
                    if (fit.slope < 0) {
                        tauList.add(-1.0 / fit.slope)
                        stepR2List.add(fit.rSquared)
                    }
                }
            }
        }

        if (!isStopRequested) {
            motor.power = 0.0
        }

        // Average τ across valid trials → kA.
        val validTrials = tauList.size
        val avgTau = if (validTrials > 0) tauList.average() else 0.0
        val avgStepR2 = if (validTrials > 0) stepR2List.average() else 0.0
        val kA = if (beta != null && validTrials > 0) avgTau * kV else 0.0

        // ── Phase 5: Results display until stopped ────────────────────────────
        while (nextFrame()) {
            telemetry.addLine("── RESULTS ──")
            if (beta != null) {
                telemetry.addData("kS", "%.4f V", kS)
                telemetry.addData("kG", "%.4f V", kG)
                telemetry.addData("kV", "%.4f V·s/rad", kV)
                if (validTrials > 0) telemetry.addData("kA", "%.4f V·s²/rad", kA)
                else telemetry.addData("kA", "FAILED – no valid trials")
                telemetry.addData("φ", "%.1f°  (encoder zero offset from horizontal)", phiDeg)
                telemetry.addData("R²", "%.4f  (quasistatic fit)", rSquared)
                telemetry.addData("samples", "%d fwd + %d rev = %d total", samples1, samples2, qsN)
            } else {
                telemetry.addData(
                    "Regression",
                    "FAILED – insufficient samples (n=%d, need ≥ 5)",
                    qsN,
                )
            }

            if (validTrials > 0) {
                telemetry.addData("kA R²", "%.4f  (step response fit, avg)", avgStepR2)
                telemetry.addData("τ (avg)", "%.4f s", avgTau)
                telemetry.addData("kA valid trials", "%d / %d", validTrials, PARAMS.STEP_TRIALS)
                for (i in tauList.indices) {
                    telemetry.addData(
                        "  Trial ${i + 1}",
                        "τ=%.4f s  R²=%.4f",
                        tauList[i],
                        stepR2List[i],
                    )
                }
            }

            telemetry.addLine("")
            telemetry.addLine("── PASTE INTO ArmFeedforward CONSTRUCTOR ──")
            if (beta != null) {
                telemetry.addData("  kS =", "%.4f", kS)
                telemetry.addData("  kG =", "%.4f", kG)
                telemetry.addData("  kV =", "%.4f", kV)
                telemetry.addData("  kA =", "%.4f", kA)
            }
        }
    }

    // ── Quasistatic sweep ─────────────────────────────────────────────────────

    /**
     * Ramps motor power while recording (V_applied, θ, ω) samples into the online normal equations.
     * Returns when a soft limit is hit or opMode ends.
     *
     * @param forward true for positive power ramp (forward sweep), false for negative
     * @return number of samples recorded during this sweep
     */
    private fun runQuasistaticSweep(
        motor: DcMotorEx,
        ticksToRad: Double,
        tickOffset: Int,
        forward: Boolean,
        phaseLabel: String,
    ): Int {
        var count = 0
        var power = 0.0

        while (nextFrame()) {
            val dt = loopDt()
            if (!(dt > 1e-6) || dt > 0.5) {
                // NaN before second advance, duplicate frame, or long gap after a wait.
                continue
            }

            power += (if (forward) 1.0 else -1.0) * PARAMS.QUASISTATIC_RAMP * dt
            power = max(-1.0, min(1.0, power))
            motor.power = power

            val ticks = motor.currentPosition
            val theta = (ticks - tickOffset) * ticksToRad
            val omega = motor.velocity * ticksToRad
            val battV = batteryVoltage()
            val vApplied = power * battV

            if (abs(omega) > PARAMS.MIN_VELOCITY_RAD) {
                accumulateSample(vApplied, theta, omega)
                count++
            }

            telemetry.addData("Phase", phaseLabel)
            telemetry.addData("Power", "%.4f", power)
            telemetry.addData("θ (rad)", "%.3f", theta)
            telemetry.addData("ω (rad/s)", "%.3f", omega)
            telemetry.addData("V applied (V)", "%.3f", vApplied)
            telemetry.addData("Samples", count)

            val relTicks = ticks - tickOffset
            if (forward && relTicks >= PARAMS.SOFT_LIMIT_FWD) break
            if (!forward && relTicks <= PARAMS.SOFT_LIMIT_BACK) break
        }

        return count
    }

    // ── Online normal-equation accumulation ──────────────────────────────────

    private fun accumulateSample(vApplied: Double, theta: Double, omega: Double) {
        val x = doubleArrayOf(sign(omega), cos(theta), sin(theta), omega)
        for (i in 0 until 4) {
            for (j in 0 until 4) XtX[i][j] += x[i] * x[j]
            XtY[i] += x[i] * vApplied
        }
        qsSumY += vApplied
        qsSumY2 += vApplied * vApplied
        qsN++
    }

    // ── Gamepad-A waiter ─────────────────────────────────────────────────────

    /**
     * Loops showing `message` until the A button is pressed (rising edge). Uses [nextFrame] so bulk
     * is fresh on the frame that returns — safe to read encoders immediately after (operator may
     * have moved the arm during the wait).
     */
    private fun waitForA(message: String): Boolean {
        while (nextFrame()) {
            telemetry.addData("Waiting", message)
            if (gamepad1.aWasPressed()) {
                return true
            }
        }
        return false
    }
}
