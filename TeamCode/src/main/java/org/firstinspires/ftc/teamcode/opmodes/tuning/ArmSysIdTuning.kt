package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.qualcomm.robotcore.hardware.DcMotor

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.opmodes.base.MarsLinearOpMode
import org.marsroboticsassociation.controllib.mechanism.ArmSysId
import kotlin.math.PI

/**
 * On-robot arm system ID using ControlLab's integrated equation-of-motion method
 * ([ArmSysId]).
 *
 * <p>Recovers the five-parameter model used by `ArmModel` / mechanism controllers:
 *
 * <pre>
 *   V = kS·sign(ω) + kV·ω + kA·α + kCos·cos(θ) + kSin·sin(θ)
 * </pre>
 *
 * by logging encoder position (not hub velocity) and regressing on integrated intervals so
 * acceleration is never differentiated from noisy velocity.
 *
 * <p><b>Two-stage fit.</b> Constant-velocity <b>holds</b> (Y / right bumper) pin `kS`, `kV`,
 * and gravity quasi-statically — at a held speed α ≈ 0, so a lashy/flexy drivetrain cannot
 * inflate `kS` (the direction-flipping flex/lash deflection would otherwise land in the
 * `sign(ω)` term), and the fit's direction-split gravity columns absorb the ±half-lash
 * encoder offset (reported back as a backlash estimate). Constant-voltage <b>runs</b> (A / X)
 * excite `Δω` and pin `kA`; run at several speeds in both directions so the hold-side
 * `kV` is trusted. [ArmSysId.solveTwoStage] combines them and cross-checks the
 * hold-side `kV` against the run-side one — disagreement flags flex contamination. An
 * over-estimated `kS` is an anti-braking feedforward term that brings back arrival overshoot,
 * so the holds matter on a real arm.
 *
 * <p><b>Flex-aware runs.</b> If the arm visibly rings, capture a ring-down (<b>left bumper</b>:
 * power drops, flick the arm, hold still) or set [Params.FLEX_HZ]; the fit then spans its run
 * intervals over whole flex periods so the ring cancels out of `kA`. Raw logs are kept and
 * accumulated at solve time, so a ring-down captured after some runs still benefits them.
 *
 * <p>Each control loop reads the hub battery voltage and commands `power = clamp(V_cmd /
 * V_batt)` so the applied voltage tracks the requested volts under battery sag. Samples are taken
 * every loop with wall-clock timestamps — no sleep, no paced sample rate. The fit uses measured
 * applied voltage and actual loop `dt`.
 *
 * <p><b>Fixed range of motion</b> — set [Params.MIN_ANGLE_DEG] and
 * [Params.MAX_ANGLE_DEG] (deg from horizontal) to your mechanical hard stops. Open-loop runs
 * soft-stop [Params.RUN_STOP_MARGIN_DEG] inside that range; the OLS fit also discards samples
 * near the stops so contact forces do not pollute kS / gravity.
 *
 * <p><b>Procedure</b>
 *
 * <ol>
 *   <li>Set range, motor name, and `TICKS_PER_ARM_REV` on Dashboard (or in code).
 *   <li>Press Start. Idle loop shows θ vs the configured range and live battery voltage.
 *   <li><b>Holds (kS, kV, gravity, backlash):</b> park the arm near one hard stop, then press
 *       <b>Y</b> (sweep up) or <b>right bumper</b> (sweep down) to hold `HOLD_SPEED` across
 *       the range. Collect a few speeds in both directions (change `HOLD_SPEED` between
 *       holds) — the speed spread is what lets the holds pin `kV`.
 *   <li><b>Runs (kA):</b> press <b>A</b> for a +voltage run, <b>X</b> for a −voltage run; change
 *       `RUN_VOLTAGE` between runs. Near-horizontal starts help.
 *   <li><b>Flex (optional):</b> press <b>left bumper</b>, flick the arm, and let it ring to measure
 *       the flex period (or set `FLEX_HZ`).
 *   <li>Press <b>B</b> to solve (two-stage) and show results until stop.
 * </ol>
 *
 * <p>Unlike [ArmFeedforwardTuning] (quasistatic + separate step for kA), this recovers all
 * five coefficients from the same encoder-log battery as ControlLab's "Run SysID".
 */
@Config
@TeleOp(name = "ArmSysIdTuning", group = "Tuning")
class ArmSysIdTuning : MarsLinearOpMode() {
    class Params {
        /** Hardware map name of the arm motor. */
        @JvmField var MOTOR_NAME = "arm"

        /**
         * Encoder ticks for one full arm revolution (geared). = motor encoder PPR × external gear
         * ratio (e.g. 28 × 19.2 ≈ 537.6).
         */
        @JvmField var TICKS_PER_ARM_REV = 537.7

        /**
         * Angle from horizontal (deg) when the encoder reads 0. If unknown, leave 0 — kSin absorbs
         * residual phase; use [ArmSysId.Result.phiRad] to correct later.
         */
        @JvmField var ENCODER_ZERO_OFFSET_DEG = 0.0

        /**
         * Fixed range of motion: lower hard stop (deg from horizontal). Soft-stop and OLS both
         * respect this bound. Example: −45 = 45° below horizontal in front.
         */
        @JvmField var MIN_ANGLE_DEG = -45.0

        /**
         * Fixed range of motion: upper hard stop (deg from horizontal). Must be greater than
         * [MIN_ANGLE_DEG]. Example: 225 for a 270° over-the-top workspace from −45°.
         */
        @JvmField var MAX_ANGLE_DEG = 225.0

        /**
         * Soft-stop buffer (deg) inside min…max so open-loop runs never drive into the hard stops.
         * The fit uses a slightly smaller margin (~4°) of its own.
         */
        @JvmField var RUN_STOP_MARGIN_DEG = 8.0

        /**
         * Commanded voltage magnitude (volts) for A/X runs. Power is recomputed each loop as `
         * V_cmd / V_batt` from the live hub reading.
         */
        @JvmField var RUN_VOLTAGE = 6.0

        /** Duration of each run (seconds). ControlLab uses 0.9 s. */
        @JvmField var RUN_DURATION_S = 0.9

        /**
         * Target speed magnitude (rad/s) for a Y / right-bumper constant-velocity hold. Collect
         * several holds at a few speeds in both directions: the holds pin kS and gravity
         * quasi-statically (α ≈ 0), which keeps a lashy/flexy drivetrain from inflating kS. Change
         * this between holds for a wider speed spread (helps kV).
         */
        @JvmField var HOLD_SPEED = 1.5

        /** Velocity-hold P gain, volts per rad/s (saturates warmup, then holds the speed). */
        @JvmField var HOLD_KP = 8.0

        /** Velocity-hold I gain, volts per rad. */
        @JvmField var HOLD_KI = 20.0

        /** Max duration of a single hold sweep (seconds) before it gives up. */
        @JvmField var HOLD_MAX_S = 5.0

        /**
         * Structural-flex frequency (Hz) if known; 0 = none. A left-bumper ring-down capture
         * overrides this. When set, run intervals span whole flex periods so the ring cancels out
         * of the kA fit.
         */
        @JvmField var FLEX_HZ = 0.0

        /** Duration of a left-bumper ring-down capture (seconds). */
        @JvmField var RINGDOWN_S = 2.5
    }

    companion object {
        @JvmField
        var PARAMS = Params()
    }

    private lateinit var motor: DcMotorEx
    private var ticksToRad = 0.0

    // Raw logs ({theta, volts, time} per capture), accumulated into fit rows at solve time so a
    // flex period learned late (ring-down or Dashboard edit) still applies to every run.
    private val runLogs = ArrayList<Array<DoubleArray>>() // stage 2 (kA): A/X runs
    private val holdLogs = ArrayList<Array<DoubleArray>>() // stage 1: Y/RB holds
    private var flexPeriodMeasured = 0.0 // seconds; from a left-bumper ring-down capture
    private var runsCompleted = 0
    private var holdsCompleted = 0
    private var totalSamples = 0
    private var holdRowCount = 0 // set at solve time
    private var movingRowCount = 0

    override fun runOpMode() {
        initRobot()

        motor = hardwareMap.get(DcMotorEx::class.java, PARAMS.MOTOR_NAME)
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        ticksToRad = 2.0 * PI / PARAMS.TICKS_PER_ARM_REV

        telemetry.addLine("ArmSysId — two-stage (holds for kS/kV/gravity, runs for kA)")
        telemetry.addLine("Set MIN/MAX_ANGLE_DEG to your hard stops (Dashboard).")
        telemetry.addLine("A = +V run, X = −V run, Y = +speed hold, RB = −speed hold,")
        telemetry.addLine("LB = flex ring-down (flick the arm), B = solve.")
        addRangeTelemetry()
        telemetry.update()
        waitForStart()
        if (isStopRequested) {
            return
        }

        // Collect runs until B.
        while (nextFrame()) {
            cutPower()

            telemetry.addLine("── collect ──")
            telemetry.addData("A / X", "±%.1f V run (kA)", PARAMS.RUN_VOLTAGE)
            telemetry.addData("Y / RB", "±%.1f rad/s hold (kS, kV, gravity)", PARAMS.HOLD_SPEED)
            telemetry.addData(
                "LB",
                "flex ring-down (%.1f s; flick the arm first)",
                PARAMS.RINGDOWN_S,
            )
            telemetry.addData("B", "solve (two-stage)")
            addRangeTelemetry()
            telemetry.addData("V_batt (live)", "%.2f V", batteryVoltage())
            telemetry.addData(
                "RUN_VOLTAGE / HOLD_SPEED",
                "%.2f V / %.2f rad/s",
                PARAMS.RUN_VOLTAGE,
                PARAMS.HOLD_SPEED,
            )
            telemetry.addData("runs / holds", "%d / %d", runsCompleted, holdsCompleted)
            telemetry.addData("raw samples", totalSamples)
            telemetry.addData("flex period", flexPeriodTelemetry())

            if (gamepad1.aWasPressed()) {
                if (!runVoltageCommand(+PARAMS.RUN_VOLTAGE, "+V")) {
                    return
                }
            } else if (gamepad1.xWasPressed()) {
                if (!runVoltageCommand(-PARAMS.RUN_VOLTAGE, "−V")) {
                    return
                }
            } else if (gamepad1.yWasPressed()) {
                if (!runVelocityHold(+PARAMS.HOLD_SPEED, "+hold")) {
                    return
                }
            } else if (gamepad1.rightBumperWasPressed()) {
                if (!runVelocityHold(-PARAMS.HOLD_SPEED, "−hold")) {
                    return
                }
            } else if (gamepad1.leftBumperWasPressed()) {
                if (!captureRingDown()) {
                    return
                }
            } else if (gamepad1.bWasPressed()) {
                break
            }
        }

        cutPower()
        if (!opModeIsActive()) {
            return
        }

        showResults(solveFromLogs())
    }

    /**
     * Accumulate every stored log with the final fit parameters (including any flex period learned
     * along the way), then run the two-stage solve.
     */
    private fun solveFromLogs(): ArmSysId.Result {
        val fit = ArmSysId.FitParams()
        fit.flexPeriodSec = flexPeriodSec()

        val movingRows = ArrayList<DoubleArray>()
        val movingRhs = ArrayList<Double>()
        for (log in runLogs) {
            ArmSysId.accumulateRun(
                log[0],
                log[1],
                log[2],
                minAngleRad(),
                maxAngleRad(),
                fit,
                movingRows,
                movingRhs,
            )
        }
        val holdRows = ArrayList<DoubleArray>()
        val holdRhs = ArrayList<Double>()
        for (log in holdLogs) {
            ArmSysId.accumulateHold(
                log[0], log[1], log[2], minAngleRad(), maxAngleRad(), fit, holdRows, holdRhs,
            )
        }
        holdRowCount = holdRows.size
        movingRowCount = movingRows.size
        return ArmSysId.solveTwoStage(fit, holdRows, holdRhs, movingRows, movingRhs)
    }

    /** The flex period to fit with: a ring-down measurement wins over the configured FLEX_HZ. */
    private fun flexPeriodSec(): Double {
        if (flexPeriodMeasured > 0) {
            return flexPeriodMeasured
        }
        return if (PARAMS.FLEX_HZ > 0) 1.0 / PARAMS.FLEX_HZ else 0.0
    }

    private fun flexPeriodTelemetry(): String {
        val period = flexPeriodSec()
        if (period <= 0) {
            return "none (LB to measure, or set FLEX_HZ)"
        }
        return String.format(
            "%.3f s (%.1f Hz, %s)",
            period,
            1.0 / period,
            if (flexPeriodMeasured > 0) "measured" else "FLEX_HZ",
        )
    }

    /**
     * Open-loop run targeting `voltageCmd` volts for [Params.RUN_DURATION_S]. Every
     * loop: bulk-read, measure hub voltage, set `power = clamp(V_cmd / V_batt)`, log θ / V /
     * wall time at the natural loop rate. Soft-stops at the fixed range margins.
     *
     * @return false if the OpMode stopped
     */
    private fun runVoltageCommand(voltageCmd: Double, label: String): Boolean {
        val thetaList = ArrayList<Double>()
        val voltList = ArrayList<Double>()
        val timeList = ArrayList<Double>()
        val run = ElapsedTime()
        val loSoft = minAngleRad() + stopMarginRad()
        val hiSoft = maxAngleRad() - stopMarginRad()
        var lastT = 0.0

        run.reset()

        while (nextFrame() && run.seconds() < PARAMS.RUN_DURATION_S) {
            val t = run.seconds()
            val dt = t - lastT
            lastT = t
            if (dt < 1e-6) {
                continue
            }

            val battV = batteryVoltage()
            // Guard divide-by-zero / brownout; recompute power every loop from live V_batt.
            val power = if (battV > 0.5) Range.clip(voltageCmd / battV, -1.0, 1.0) else 0.0
            motor.power = power
            val vApplied = power * battV

            val theta = readThetaRad()

            // Soft stop only in the direction of commanded voltage (respects fixed ROM).
            if ((voltageCmd > 0 && theta >= hiSoft) || (voltageCmd < 0 && theta <= loSoft)) {
                telemetry.addData(
                    "Run",
                    "%s — soft stop at θ=%.1f° (range %.0f…%.0f°)",
                    label,
                    Math.toDegrees(theta),
                    PARAMS.MIN_ANGLE_DEG,
                    PARAMS.MAX_ANGLE_DEG,
                )
                break
            }

            // One sample per loop; wall-clock time carries the actual dt into the fit.
            thetaList.add(theta)
            voltList.add(vApplied)
            timeList.add(t)

            telemetry.addData("Run", "%s  V_cmd=%.2f", label, voltageCmd)
            telemetry.addData("t", "%.2f / %.2f s", t, PARAMS.RUN_DURATION_S)
            telemetry.addData("loop dt", "%.1f ms", dt * 1000.0)
            telemetry.addData("V_batt (live)", "%.2f V", battV)
            telemetry.addData("power (cmd)", "%.3f", power)
            telemetry.addData("V applied", "%.2f V", vApplied)
            addRangeTelemetry()
            telemetry.addData("samples this run", thetaList.size)
            telemetry.addData("runs so far", runsCompleted)
        }

        if (isStopRequested) {
            return false
        }
        cutPower()

        val n = thetaList.size
        if (n < 10) {
            telemetry.addData("Run", "%s — too few samples (%d), skipped", label, n)
            return true
        }

        runLogs.add(toLog(thetaList, voltList, timeList))
        totalSamples += n
        runsCompleted++

        val meanDtMs = 1000.0 * (timeList[n - 1] - timeList[0]) / (n - 1)
        telemetry.addData(
            "Run",
            "%s done: %d samples mean dt=%.1f ms (rows built at solve)",
            label,
            n,
            meanDtMs,
        )
        return true
    }

    /**
     * Constant-velocity hold targeting `targetVel` rad/s: a PI loop on velocity error drives
     * the arm across the range at a held speed, logging θ / applied V / wall time each loop. The
     * held speed keeps acceleration ≈ 0, so [ArmSysId.accumulateHold] recovers a quasi-static
     * `kS` and gravity that a lashy/flexy drivetrain would otherwise corrupt. Start the arm
     * near the opposite hard stop so the hold sweeps a wide angle band. Soft-stops at the fixed
     * range margins.
     *
     * @return false if the OpMode stopped
     */
    private fun runVelocityHold(targetVel: Double, label: String): Boolean {
        val thetaList = ArrayList<Double>()
        val voltList = ArrayList<Double>()
        val timeList = ArrayList<Double>()
        val run = ElapsedTime()
        val loSoft = minAngleRad() + stopMarginRad()
        val hiSoft = maxAngleRad() - stopMarginRad()
        var integralV = 0.0

        run.reset()

        while (nextFrame() && run.seconds() < PARAMS.HOLD_MAX_S) {
            val t = run.seconds()
            val dt = loopDt()
            if (!(dt > 1e-6) || dt > 0.5) {
                // NaN before second advance, duplicate frame, or long gap after idle collect.
                continue
            }

            val battV = batteryVoltage()
            val vel = motor.velocity * ticksToRad // encoder velocity, rad/s
            val errV = targetVel - vel
            integralV += errV * dt
            val vCmd = PARAMS.HOLD_KP * errV + PARAMS.HOLD_KI * integralV
            val power = if (battV > 0.5) Range.clip(vCmd / battV, -1.0, 1.0) else 0.0
            motor.power = power
            val vApplied = power * battV

            val theta = readThetaRad()

            // Sweep done once we reach the far margin in the direction of travel.
            if ((targetVel > 0 && theta >= hiSoft) || (targetVel < 0 && theta <= loSoft)) {
                if (thetaList.isNotEmpty()) {
                    break
                }
            } else {
                // Log while inside the range; accumulateHold's α gate drops the warmup samples.
                thetaList.add(theta)
                voltList.add(vApplied)
                timeList.add(t)
            }

            telemetry.addData("Hold", "%s  target=%.2f rad/s", label, targetVel)
            telemetry.addData("t", "%.2f / %.2f s", t, PARAMS.HOLD_MAX_S)
            telemetry.addData("ω (meas)", "%.2f rad/s", vel)
            telemetry.addData("V applied", "%.2f V", vApplied)
            addRangeTelemetry()
            telemetry.addData("samples this hold", thetaList.size)
            telemetry.addData("holds so far", holdsCompleted)
        }

        if (isStopRequested) {
            return false
        }
        cutPower()

        val n = thetaList.size
        if (n < 12) {
            telemetry.addData(
                "Hold",
                "%s — too few samples (%d), skipped. " + "Start near the opposite stop.",
                label,
                n,
            )
            return true
        }

        holdLogs.add(toLog(thetaList, voltList, timeList))
        totalSamples += n
        holdsCompleted++

        telemetry.addData("Hold", "%s done: %d samples (rows built at solve)", label, n)
        return true
    }

    /**
     * Ring-down capture for the flex period: power drops to float, the driver flicks the arm, and
     * the free oscillation is logged for [Params.RINGDOWN_S].
     * [ArmSysId.estimateFlexPeriod] turns the log into a period for the solve-time fit.
     *
     * @return false if the OpMode stopped
     */
    private fun captureRingDown(): Boolean {
        val thetaList = ArrayList<Double>()
        val timeList = ArrayList<Double>()
        val run = ElapsedTime()
        // Float, not brake: a shorted motor damps the very oscillation being measured.
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        motor.power = 0.0
        run.reset()

        while (nextFrame() && run.seconds() < PARAMS.RINGDOWN_S) {
            thetaList.add(readThetaRad())
            timeList.add(run.seconds())
            telemetry.addData("Ring-down", "flick the arm, hold the base still")
            telemetry.addData("t", "%.2f / %.2f s", run.seconds(), PARAMS.RINGDOWN_S)
        }
        if (isStopRequested) {
            return false
        }
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        val n = thetaList.size
        val theta = DoubleArray(n)
        val timeSec = DoubleArray(n)
        for (i in 0 until n) {
            theta[i] = thetaList[i]
            timeSec[i] = timeList[i]
        }
        val period = ArmSysId.estimateFlexPeriod(theta, timeSec)
        if (period > 0) {
            flexPeriodMeasured = period
            telemetry.addData("Ring-down", "flex period %.3f s (%.1f Hz)", period, 1.0 / period)
        } else {
            telemetry.addData("Ring-down", "no clear oscillation — flick harder or set FLEX_HZ")
        }
        return true
    }

    /** Pack parallel sample lists into a `{theta, volts, time}` log. */
    private fun toLog(
        theta: List<Double>,
        volts: List<Double>,
        time: List<Double>,
    ): Array<DoubleArray> {
        val n = theta.size
        val log = Array(3) { DoubleArray(n) }
        for (i in 0 until n) {
            log[0][i] = theta[i]
            log[1][i] = volts[i]
            log[2][i] = time[i]
        }
        return log
    }

    private fun showResults(r: ArmSysId.Result) {
        while (nextFrame()) {
            telemetry.addLine("── ArmSysId RESULTS (two-stage) ──")
            if (r.samples < 5) {
                telemetry.addData(
                    "Fit",
                    "FAILED – only %d rows (need ≥ 5). Add Y/RB holds (kS, kV, gravity) and A/X" +
                        " runs (kA) at several speeds and angles.",
                    r.samples,
                )
            } else {
                telemetry.addData("kS", "%.4f V", r.kS)
                telemetry.addData("kV", "%.4f V·s/rad", r.kV)
                telemetry.addData("kA", "%.4f V·s²/rad", r.kA)
                telemetry.addData("kCos", "%.4f V", r.kCos)
                telemetry.addData("kSin", "%.4f V", r.kSin)
                telemetry.addData("R²", "%.4f", r.rSquared)
                telemetry.addData("rows (hold+moving)", "%d + %d", holdRowCount, movingRowCount)
                telemetry.addData(
                    "runs / holds / samples",
                    "%d / %d / %d",
                    runsCompleted,
                    holdsCompleted,
                    totalSamples,
                )
                telemetry.addLine("")
                telemetry.addLine("── cross-checks ──")
                if (r.kVHold.isNaN()) {
                    telemetry.addData(
                        "kV (hold)",
                        "n/a — holds lack speed spread or a direction; " +
                            "run-side kV shipped",
                    )
                } else {
                    telemetry.addData(
                        "kV hold / run",
                        "%.4f / %.4f  (hold ships)",
                        r.kVHold,
                        r.kVRun,
                    )
                    if (r.kVDisagreement() > 0.10) {
                        telemetry.addLine(
                            "WARNING: hold/run kV disagree >10% — moving runs look " +
                                "flex/lash contaminated; trust the hold-side value.",
                        )
                    }
                }
                if (!r.halfLashRad.isNaN()) {
                    telemetry.addData(
                        "backlash (est.)",
                        "%.2f° total (±%.2f° half-lash)",
                        2 * Math.toDegrees(r.halfLashRad),
                        Math.toDegrees(r.halfLashRad),
                    )
                }
                telemetry.addData("flex period", flexPeriodTelemetry())
                telemetry.addLine("")
                telemetry.addLine("── derived (ArmFeedforward form) ──")
                telemetry.addData("kG", "%.4f V  (= √(kCos²+kSin²))", r.kG())
                telemetry.addData(
                    "φ",
                    "%.1f°  (encoder zero from horizontal)",
                    Math.toDegrees(r.phiRad()),
                )
                telemetry.addLine("")
                telemetry.addLine("── PASTE INTO ArmModel ──")
                telemetry.addData("  kS", "%.4f", r.kS)
                telemetry.addData("  kV", "%.4f", r.kV)
                telemetry.addData("  kA", "%.4f", r.kA)
                telemetry.addData("  kCos", "%.4f", r.kCos)
                telemetry.addData("  kSin", "%.4f", r.kSin)
            }
            telemetry.addData(
                "range (deg)",
                "%.0f … %.0f",
                PARAMS.MIN_ANGLE_DEG,
                PARAMS.MAX_ANGLE_DEG,
            )
            telemetry.addData("V_batt (live)", "%.2f V", batteryVoltage())
        }
    }

    /** Live θ and configured ROM (degrees) for Dashboard / DS. */
    private fun addRangeTelemetry() {
        val thetaDeg = Math.toDegrees(readThetaRad())
        val softLo = PARAMS.MIN_ANGLE_DEG + PARAMS.RUN_STOP_MARGIN_DEG
        val softHi = PARAMS.MAX_ANGLE_DEG - PARAMS.RUN_STOP_MARGIN_DEG
        telemetry.addData("θ", "%.1f°  (ticks %d)", thetaDeg, motor.currentPosition)
        telemetry.addData("ROM hard", "%.0f° … %.0f°", PARAMS.MIN_ANGLE_DEG, PARAMS.MAX_ANGLE_DEG)
        telemetry.addData(
            "ROM soft",
            "%.0f° … %.0f°  (stop margin %.0f°)",
            softLo,
            softHi,
            PARAMS.RUN_STOP_MARGIN_DEG,
        )
    }

    private fun minAngleRad(): Double {
        return Math.toRadians(PARAMS.MIN_ANGLE_DEG)
    }

    private fun maxAngleRad(): Double {
        return Math.toRadians(PARAMS.MAX_ANGLE_DEG)
    }

    private fun stopMarginRad(): Double {
        return Math.toRadians(PARAMS.RUN_STOP_MARGIN_DEG)
    }

    private fun readThetaRad(): Double {
        return motor.currentPosition * ticksToRad +
            Math.toRadians(PARAMS.ENCODER_ZERO_OFFSET_DEG)
    }

    private fun cutPower() {
        if (::motor.isInitialized) {
            motor.power = 0.0
        }
    }
}
