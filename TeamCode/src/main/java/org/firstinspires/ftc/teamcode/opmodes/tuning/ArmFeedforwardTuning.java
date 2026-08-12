package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.opmodes.base.MarsLinearOpMode;

import java.util.ArrayList;

/**
 * Automated feedforward tuner for a single arm using ArmFeedforward (WPILib port).
 *
 * <p>Produces kS, kG, kV, kA in the units ArmFeedforward expects: V, V, V·s/rad, V·s²/rad.
 *
 * <p>The sin/cos dual-regressor model {@code V = kS·sign(ω) + c1·cos(θ) + c2·sin(θ) + kV·ω} absorbs
 * both encoder-zero offset and any CM angular offset (L-shaped arm, offset attachment), so the
 * encoder does not need to be zeroed at horizontal. After regression: {@code kG = sqrt(c1²+c2²)},
 * {@code φ = atan2(-c2, c1)} = angle where arm is horizontal in encoder ticks.
 *
 * <p>Procedure:
 *
 * <ol>
 *   <li>Position arm at one end of its range, press Start, then press A.
 *   <li>Quasistatic forward sweep: power ramps up slowly while recording samples.
 *   <li>Move arm back toward start, press A.
 *   <li>Quasistatic backward sweep: power ramps negative while recording samples.
 *   <li>OLS regression yields kS, kG (=√(c1²+c2²)), kV, φ, and R².
 *   <li>Step response (STEP_TRIALS trials): gravity-subtracted first-order fit → kA.
 *   <li>Results displayed until OpMode is stopped.
 * </ol>
 */
@Config
@TeleOp(name = "ArmFeedforwardTuning", group = "Tuning")
public class ArmFeedforwardTuning extends MarsLinearOpMode {

    // ── Config constants ──────────────────────────────────────────────────────
    public static class Params {
        /** Hardware map name of the arm motor. */
        public String MOTOR_NAME = "arm";

        /**
         * Encoder ticks for one full arm revolution (geared). = motor_encoder_ppr ×
         * external_gear_ratio. Incorrect value shifts kV/kA but does not affect kS or kG.
         */
        public double TICKS_PER_ARM_REV = 537.7;

        /** Stop forward sweep at this many encoder ticks past the zeroed start position. */
        public int SOFT_LIMIT_FWD = 2000;

        /** Stop backward sweep at this encoder tick offset (0 = back to start). */
        public int SOFT_LIMIT_BACK = 0;

        /** Power ramp rate during quasistatic sweeps, in power/second. */
        public double QUASISTATIC_RAMP = 0.03;

        /** Minimum arm angular velocity in rad/s; slower samples are ignored. */
        public double MIN_VELOCITY_RAD = 0.05;

        /** Fraction of battery voltage applied during step response trials. */
        public double STEP_FRACTION = 0.50;

        /** Number of step-response trials. */
        public int STEP_TRIALS = 3;

        /** Maximum duration of each step-response trial in seconds. */
        public double STEP_MAX_TIME_S = 3.0;

        /** Arm is considered "at rest" when |velocity| < this threshold in ticks/s. */
        public double COAST_THRESHOLD_TPS = 20.0;

        /** Maximum time to wait for arm to come to rest before starting a step trial. */
        public double COAST_TIMEOUT_S = 8.0;

        /** Step-fit lower exclusion: skip points where ω/ω_∞ < this fraction. */
        public double OMEGA_LOWER_FRACTION = 0.05;

        /** Step-fit upper exclusion: skip points where ω/ω_∞ > this fraction. */
        public double OMEGA_UPPER_FRACTION = 0.95;
    }

    public static Params PARAMS = new Params();

    // ── Online normal-equation accumulators ───────────────────────────────────
    // Model: V = kS·sign(ω) + c1·cos(θ) + c2·sin(θ) + kV·ω
    // Feature vector x = [sign(ω), cos(θ), sin(θ), ω], indices 0–3.
    private final double[][] XtX = new double[4][4];
    private final double[] XtY = new double[4];
    private double qsSumY = 0;
    private double qsSumY2 = 0;
    private int qsN = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

        // ── Hardware init ────────────────────────────────────────────────────
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, PARAMS.MOTOR_NAME);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        double ticksToRad = 2.0 * Math.PI / PARAMS.TICKS_PER_ARM_REV;

        // ── Wait for start ───────────────────────────────────────────────────
        telemetry.addData("Status", "Move arm to one end of range, then press Start.");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        // ── Phase 1: Quasistatic forward sweep ───────────────────────────────
        if (!waitForA(
                "Phase 1/5 – Move arm to one end of range, then press A to begin forward sweep."))
            return;

        // waitForA ends on a nextFrame (fresh bulk); position after operator move is current.
        int tickOffset = motor.getCurrentPosition();
        int samples1 =
                runQuasistaticSweep(motor, ticksToRad, tickOffset, true, "1/5 – Forward sweep");
        if (isStopRequested()) return;
        motor.setPower(0);

        // ── Phase 2: Quasistatic backward sweep ──────────────────────────────
        if (!waitForA(
                String.format(
                        "Phase 2/5 – Forward sweep done (%d samples). Move arm back, then press A"
                                + " for reverse sweep.",
                        samples1))) return;

        int samples2 =
                runQuasistaticSweep(motor, ticksToRad, tickOffset, false, "2/5 – Reverse sweep");
        if (isStopRequested()) return;
        motor.setPower(0);

        // ── Phase 3: OLS regression ──────────────────────────────────────────
        double kS = 0, c1 = 0, c2 = 0, kV = 0;
        double kG = 0, phiDeg = 0, rSquared = 0;
        double[] beta = null;

        if (qsN >= 5) {
            // Deep-copy so Gaussian elimination doesn't clobber the accumulators.
            double[][] A = new double[4][4];
            double[] b = new double[4];
            for (int i = 0; i < 4; i++) {
                b[i] = XtY[i];
                System.arraycopy(XtX[i], 0, A[i], 0, 4);
            }
            beta = solveGaussian(A, b);
        }

        if (beta != null) {
            kS = beta[0];
            c1 = beta[1];
            c2 = beta[2];
            kV = beta[3];
            kG = Math.sqrt(c1 * c1 + c2 * c2);
            phiDeg = Math.toDegrees(Math.atan2(-c2, c1));

            // R² using online accumulators (no individual samples stored).
            // SST = sumY² − sumY²/n
            // SSE = sumY2 − 2·βᵀ·XᵀY + βᵀ·XᵀX·β
            double sst = qsSumY2 - qsSumY * qsSumY / qsN;
            double sse = qsSumY2 - 2.0 * dot(beta, XtY) + quadForm(beta, XtX);
            rSquared = (sst == 0) ? 1.0 : 1.0 - sse / sst;
        }

        // ── Phase 4: Step response for kA ────────────────────────────────────
        var tauList = new ArrayList<Double>();
        var stepR2List = new ArrayList<Double>();

        if (beta != null && kV > 0) {
            for (int trial = 0; trial < PARAMS.STEP_TRIALS && !isStopRequested(); trial++) {
                // Coast to rest
                motor.setPower(0);
                double coastStart = getRuntime();
                while (nextFrame()) {
                    double velTps = Math.abs(motor.getVelocity());
                    telemetry.addData(
                            "Phase",
                            "4/5 – Step response: coasting to rest (trial %d/%d)",
                            trial + 1,
                            PARAMS.STEP_TRIALS);
                    telemetry.addData("Velocity (tps)", "%.1f", velTps);
                    if (velTps < PARAMS.COAST_THRESHOLD_TPS) break;
                    if (getRuntime() - coastStart > PARAMS.COAST_TIMEOUT_S) break;
                }
                if (isStopRequested()) break;

                // Prompt user to position arm near vertical (gravity ≈ 0 → best kA accuracy).
                if (!waitForA(
                        String.format(
                                "Trial %d/%d – Position arm near vertical (gravity ≈ 0), then press"
                                        + " A.",
                                trial + 1, PARAMS.STEP_TRIALS))) return;

                // Snapshot after waitForA's nextFrame (fresh bulk + battery epoch).
                double theta0 = (motor.getCurrentPosition() - tickOffset) * ticksToRad;
                double battV0 = batteryVoltage();

                // Gravity-subtracted net voltage and predicted terminal velocity.
                double vNet =
                        PARAMS.STEP_FRACTION * battV0
                                - c1 * Math.cos(theta0)
                                - c2 * Math.sin(theta0);
                double wInf = (vNet - kS) / kV;

                if (wInf <= 0) {
                    telemetry.addData(
                            "Phase",
                            "Trial %d/%d – wInf ≤ 0 (arm orientation problem). Skipping.",
                            trial + 1,
                            PARAMS.STEP_TRIALS);
                    continue;
                }

                // Apply step and record (t, ω).
                var sampleT = new ArrayList<Double>();
                var sampleW = new ArrayList<Double>();

                double stepStart = getRuntime();
                motor.setPower(PARAMS.STEP_FRACTION);

                while (nextFrame() && (getRuntime() - stepStart) < PARAMS.STEP_MAX_TIME_S) {
                    double t = getRuntime() - stepStart;
                    double omega = motor.getVelocity() * ticksToRad;
                    sampleT.add(t);
                    sampleW.add(omega);

                    telemetry.addData(
                            "Phase",
                            "4/5 – Step response: sampling (trial %d/%d)",
                            trial + 1,
                            PARAMS.STEP_TRIALS);
                    telemetry.addData("Time", "%.3f s", t);
                    telemetry.addData("ω (rad/s)", "%.3f", omega);
                    telemetry.addData("ω_∞ (rad/s)", "%.3f", wInf);
                }
                if (isStopRequested()) break;
                motor.setPower(0);

                // Linearized fit: ln(1 − ω/ω_∞) vs t → slope = −1/τ.
                var regT = new ArrayList<Double>();
                var regY = new ArrayList<Double>();
                for (int i = 0; i < sampleT.size(); i++) {
                    double ratio = sampleW.get(i) / wInf;
                    if (ratio < PARAMS.OMEGA_LOWER_FRACTION || ratio > PARAMS.OMEGA_UPPER_FRACTION)
                        continue;
                    regT.add(sampleT.get(i));
                    regY.add(Math.log(1.0 - ratio));
                }

                if (regT.size() >= 3) {
                    int m = regT.size();
                    double[] tArr = new double[m];
                    double[] yArr = new double[m];
                    for (int i = 0; i < m; i++) {
                        tArr[i] = regT.get(i);
                        yArr[i] = regY.get(i);
                    }
                    var fit = linearRegression(tArr, yArr, m);
                    if (fit.slope < 0) {
                        tauList.add(-1.0 / fit.slope);
                        stepR2List.add(fit.rSquared);
                    }
                }
            }
        }

        if (!isStopRequested()) {
            motor.setPower(0);
        }

        // Average τ across valid trials → kA.
        int validTrials = tauList.size();
        double avgTau = 0;
        double avgStepR2 = 0;
        for (double tau : tauList) avgTau += tau;
        for (double r2 : stepR2List) avgStepR2 += r2;
        if (validTrials > 0) {
            avgTau /= validTrials;
            avgStepR2 /= validTrials;
        }
        double kA = (beta != null && validTrials > 0) ? avgTau * kV : 0;

        // ── Phase 5: Results display until stopped ────────────────────────────
        while (nextFrame()) {
            telemetry.addLine("── RESULTS ──");
            if (beta != null) {
                telemetry.addData("kS", "%.4f V", kS);
                telemetry.addData("kG", "%.4f V", kG);
                telemetry.addData("kV", "%.4f V·s/rad", kV);
                if (validTrials > 0) telemetry.addData("kA", "%.4f V·s²/rad", kA);
                else telemetry.addData("kA", "FAILED – no valid trials");
                telemetry.addData("φ", "%.1f°  (encoder zero offset from horizontal)", phiDeg);
                telemetry.addData("R²", "%.4f  (quasistatic fit)", rSquared);
                telemetry.addData("samples", "%d fwd + %d rev = %d total", samples1, samples2, qsN);
            } else {
                telemetry.addData(
                        "Regression", "FAILED – insufficient samples (n=%d, need ≥ 5)", qsN);
            }

            if (validTrials > 0) {
                telemetry.addData("kA R²", "%.4f  (step response fit, avg)", avgStepR2);
                telemetry.addData("τ (avg)", "%.4f s", avgTau);
                telemetry.addData("kA valid trials", "%d / %d", validTrials, PARAMS.STEP_TRIALS);
                for (int i = 0; i < tauList.size(); i++) {
                    telemetry.addData(
                            String.format("  Trial %d", i + 1),
                            "τ=%.4f s  R²=%.4f",
                            tauList.get(i),
                            stepR2List.get(i));
                }
            }

            telemetry.addLine("");
            telemetry.addLine("── PASTE INTO ArmFeedforward CONSTRUCTOR ──");
            if (beta != null) {
                telemetry.addData("  kS =", "%.4f", kS);
                telemetry.addData("  kG =", "%.4f", kG);
                telemetry.addData("  kV =", "%.4f", kV);
                telemetry.addData("  kA =", "%.4f", kA);
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
    private int runQuasistaticSweep(
            DcMotorEx motor,
            double ticksToRad,
            int tickOffset,
            boolean forward,
            String phaseLabel) {
        int count = 0;
        double power = 0;

        while (nextFrame()) {
            double dt = loopDt();
            if (!(dt > 1e-6) || dt > 0.5) {
                // NaN before second advance, duplicate frame, or long gap after a wait.
                continue;
            }

            power += (forward ? 1.0 : -1.0) * PARAMS.QUASISTATIC_RAMP * dt;
            power = Math.max(-1.0, Math.min(1.0, power));
            motor.setPower(power);

            int ticks = motor.getCurrentPosition();
            double theta = (ticks - tickOffset) * ticksToRad;
            double omega = motor.getVelocity() * ticksToRad;
            double battV = batteryVoltage();
            double vApplied = power * battV;

            if (Math.abs(omega) > PARAMS.MIN_VELOCITY_RAD) {
                accumulateSample(vApplied, theta, omega);
                count++;
            }

            telemetry.addData("Phase", phaseLabel);
            telemetry.addData("Power", "%.4f", power);
            telemetry.addData("θ (rad)", "%.3f", theta);
            telemetry.addData("ω (rad/s)", "%.3f", omega);
            telemetry.addData("V applied (V)", "%.3f", vApplied);
            telemetry.addData("Samples", count);

            int relTicks = ticks - tickOffset;
            if (forward && relTicks >= PARAMS.SOFT_LIMIT_FWD) break;
            if (!forward && relTicks <= PARAMS.SOFT_LIMIT_BACK) break;
        }

        return count;
    }

    // ── Online normal-equation accumulation ──────────────────────────────────

    private void accumulateSample(double vApplied, double theta, double omega) {
        double[] x = {Math.signum(omega), Math.cos(theta), Math.sin(theta), omega};
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) XtX[i][j] += x[i] * x[j];
            XtY[i] += x[i] * vApplied;
        }
        qsSumY += vApplied;
        qsSumY2 += vApplied * vApplied;
        qsN++;
    }

    // ── Gamepad-A waiter ─────────────────────────────────────────────────────

    /**
     * Loops showing {@code message} until the A button is pressed (rising edge). Uses {@link
     * #nextFrame()} so bulk is fresh on the frame that returns — safe to read encoders immediately
     * after (operator may have moved the arm during the wait).
     */
    private boolean waitForA(String message) {
        while (nextFrame()) {
            telemetry.addData("Waiting", message);
            if (gamepad1.aWasPressed()) {
                return true;
            }
        }
        return false;
    }

    // ── Gaussian elimination with partial pivoting (4×4) ─────────────────────

    /**
     * Solves the 4×4 linear system A·x = b via Gaussian elimination with partial pivoting. A and b
     * are modified in-place; pass copies if they need to be preserved.
     *
     * @return solution vector, or null if A is (near-)singular
     */
    private static double[] solveGaussian(double[][] A, double[] b) {
        int n = 4;
        for (int col = 0; col < n; col++) {
            // Find row with largest pivot magnitude in column col.
            int maxRow = col;
            for (int row = col + 1; row < n; row++) {
                if (Math.abs(A[row][col]) > Math.abs(A[maxRow][col])) maxRow = row;
            }
            // Swap rows.
            double[] tmpRow = A[col];
            A[col] = A[maxRow];
            A[maxRow] = tmpRow;
            double tmpB = b[col];
            b[col] = b[maxRow];
            b[maxRow] = tmpB;

            if (Math.abs(A[col][col]) < 1e-12) return null; // singular

            // Eliminate below pivot.
            for (int row = col + 1; row < n; row++) {
                double factor = A[row][col] / A[col][col];
                for (int j = col; j < n; j++) A[row][j] -= factor * A[col][j];
                b[row] -= factor * b[col];
            }
        }
        // Back-substitution.
        double[] x = new double[n];
        for (int k = n - 1; k >= 0; k--) {
            x[k] = b[k];
            for (int j = k + 1; j < n; j++) x[k] -= A[k][j] * x[j];
            x[k] /= A[k][k];
        }
        return x;
    }

    // ── v^T · M · v ──────────────────────────────────────────────────────────

    private static double quadForm(double[] v, double[][] M) {
        double result = 0;
        for (int i = 0; i < v.length; i++)
            for (int j = 0; j < v.length; j++) result += v[i] * M[i][j] * v[j];
        return result;
    }

    // ── Dot product ──────────────────────────────────────────────────────────

    private static double dot(double[] a, double[] b) {
        double s = 0;
        for (int i = 0; i < a.length; i++) s += a[i] * b[i];
        return s;
    }

    // ── Simple OLS linear regression (for step-response fit) ─────────────────

    private static LinRegResult linearRegression(double[] x, double[] y, int n) {
        double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
        for (int i = 0; i < n; i++) {
            sumX += x[i];
            sumY += y[i];
            sumXY += x[i] * y[i];
            sumX2 += x[i] * x[i];
        }
        var r = new LinRegResult();
        double denom = n * sumX2 - sumX * sumX;
        if (Math.abs(denom) < 1e-12) {
            r.slope = 0;
            r.intercept = sumY / n;
            return r;
        }
        r.slope = (n * sumXY - sumX * sumY) / denom;
        r.intercept = (sumY - r.slope * sumX) / n;

        double meanY = sumY / n;
        double ssRes = 0, ssTot = 0;
        for (int i = 0; i < n; i++) {
            double predicted = r.intercept + r.slope * x[i];
            ssRes += (y[i] - predicted) * (y[i] - predicted);
            ssTot += (y[i] - meanY) * (y[i] - meanY);
        }
        r.rSquared = (ssTot == 0) ? 1.0 : 1.0 - ssRes / ssTot;
        return r;
    }

    private static class LinRegResult {
        double slope, intercept, rSquared;
    }
}
