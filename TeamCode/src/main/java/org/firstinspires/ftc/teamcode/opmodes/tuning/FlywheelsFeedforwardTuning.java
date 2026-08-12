package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.ArrayList;

/**
 * Automated feedforward tuner for the dual flywheel shooter.
 *
 * <p>When MOTORS_COUPLED is true, all motors are driven together and velocity is read from
 * motors[0], producing one set of gains. When false, each motor is tuned independently, producing
 * separate kS, kV, kA per motor.
 *
 * <p>Procedure (per motor or motor group): 1. Ramps power slowly until the motor(s) overcome
 * stiction (start spinning). 2. Steps from stiction power to full power in NUM_STEPS equal steps.
 * 3. At each step, waits SETTLE_TIME_S for the speed to stabilize, then takes NUM_SAMPLES readings
 * of velocity and applied voltage. 4. Fits a line voltage = kS + kV * velocity via least-squares
 * regression. 5. Applies a step input and records the velocity rise curve. Fits ln(1 - w/w_final)
 * vs t to extract the time constant tau, then computes kA = tau * kV. Repeats for
 * STEP_RESPONSE_TRIALS trials. 6. Displays kS, kV, kA, R², and all data points until the OpMode is
 * stopped.
 */
@Config
@TeleOp(name = "FlywheelsFeedforwardTuning", group = "Tuning")
public class FlywheelsFeedforwardTuning extends FlywheelsTuningBase {
    public static class Params {
        public int NUM_STEPS = 6;
        public double SETTLE_TIME_S = 5.0;
        public int NUM_SAMPLES = 25;
        public double STICTION_RAMP_RATE = 0.03; // power per second
        public double STICTION_THRESHOLD_TPS = 50.0;

        public double STEP_RESPONSE_POWER = 0.80;
        public double COAST_STOP_THRESHOLD_TPS = 10.0;
        public double COAST_TIMEOUT_S = 10.0;
        public int STEP_RESPONSE_TRIALS = 3;
        public double STEP_RESPONSE_MAX_TIME_S = 4.0;
        public double OMEGA_LOWER_FRACTION = 0.05;
        public double OMEGA_UPPER_FRACTION = 0.95;
    }

    public static Params PARAMS = new Params();

    private static class LinRegResult {
        double slope, intercept, rSquared;
    }

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

    private static class TuneResult {
        String label;
        double kS, kV, kA;
        double rSquared, avgStepR2, avgTau;
        int validTrials;
        double stictionPower, stictionVoltage;
        double[] avgVelocities, avgVoltages;
    }

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
    private TuneResult runTuningPass(
            String label,
            DcMotorEx[] powerMotors,
            DcMotorEx encoder,
            int passIndex,
            int totalPasses)
            throws InterruptedException {

        var result = new TuneResult();
        result.label = label;
        int steps = Math.max(PARAMS.NUM_STEPS, 2);

        // ── Phase 1: find stiction ───────────────────────────────────────────
        double power = 0;
        double lastTime = getRuntime();

        while (nextFrame()) {
            double now = getRuntime();
            double dt = now - lastTime;
            lastTime = now;
            if (dt < 1e-6) {
                continue;
            }

            power += PARAMS.STICTION_RAMP_RATE * dt;
            if (power > 1.0) power = 1.0;

            for (DcMotorEx m : powerMotors) m.setPower(power);

            double velocity = encoder.getVelocity();

            telemetry.addData("Pass", "%s (%d/%d)", label, passIndex + 1, totalPasses);
            telemetry.addData("Phase", "Finding stiction");
            telemetry.addData("Power", "%.4f", power);
            telemetry.addData("Velocity (tps)", "%.1f", velocity);

            if (Math.abs(velocity) > PARAMS.STICTION_THRESHOLD_TPS) {
                result.stictionPower = power;
                break;
            }

            if (power >= 1.0) {
                if (!isStopRequested()) {
                    for (DcMotorEx m : powerMotors) m.setPower(0);
                }
                telemetry.addData("ERROR", "Motor never started. Check connections.");
                while (nextFrame()) {
                    telemetry.addData("ERROR", "Motor never started. Check connections.");
                }
                return null;
            }
        }

        if (isStopRequested()) return null;

        result.stictionVoltage = result.stictionPower * batteryVoltage();

        // ── Phase 2: step through powers and collect data ────────────────────
        result.avgVelocities = new double[steps];
        result.avgVoltages = new double[steps];

        for (int step = 0; step < steps && !isStopRequested(); step++) {
            double stepPower =
                    result.stictionPower + (1.0 - result.stictionPower) * step / (steps - 1);

            for (DcMotorEx m : powerMotors) m.setPower(stepPower);

            // settle (deliberate wall-clock wait; live telemetry via nextFrame)
            double settleStart = getRuntime();
            while (nextFrame() && (getRuntime() - settleStart) < PARAMS.SETTLE_TIME_S) {
                telemetry.addData("Pass", "%s (%d/%d)", label, passIndex + 1, totalPasses);
                telemetry.addData("Phase", "Step %d/%d – Settling", step + 1, steps);
                telemetry.addData("Power", "%.4f", stepPower);
                telemetry.addData("Velocity (tps)", "%.1f", encoder.getVelocity());
                telemetry.addData(
                        "Time left", "%.1f s", PARAMS.SETTLE_TIME_S - (getRuntime() - settleStart));
            }

            if (isStopRequested()) return null;

            // sample (pace between samples for sysid; nextFrame refreshes bulk/battery)
            double totalVel = 0;
            double totalV = 0;
            int samples = Math.max(PARAMS.NUM_SAMPLES, 1);
            for (int s = 0; s < samples && !isStopRequested(); s++) {
                if (!nextFrame()) return null;
                double vel = encoder.getVelocity();
                double battV = batteryVoltage();
                totalVel += vel;
                totalV += stepPower * battV;

                telemetry.addData("Pass", "%s (%d/%d)", label, passIndex + 1, totalPasses);
                telemetry.addData(
                        "Phase", "Step %d/%d – Sampling %d/%d", step + 1, steps, s + 1, samples);
                telemetry.addData("Power", "%.4f", stepPower);
                telemetry.addData("Velocity (tps)", "%.1f", vel);
                telemetry.addData("Voltage (V)", "%.3f", stepPower * battV);

                sleep(20);
            }

            result.avgVelocities[step] = totalVel / samples;
            result.avgVoltages[step] = totalV / samples;
        }

        if (isStopRequested()) return null;

        for (DcMotorEx m : powerMotors) m.setPower(0);

        // ── Phase 3: least-squares fit  voltage = kS + kV · velocity ─────────
        LinRegResult fit = linearRegression(result.avgVelocities, result.avgVoltages, steps);
        result.kV = fit.slope;
        result.kS = fit.intercept;
        result.rSquared = fit.rSquared;

        // ── Phase 4: step response for kA ────────────────────────────────────
        double stepVoltage = PARAMS.STEP_RESPONSE_POWER * batteryVoltage();
        double wFinal = (stepVoltage - result.kS) / result.kV;

        var tauValues = new ArrayList<Double>();
        var stepRSquaredValues = new ArrayList<Double>();

        if (wFinal > 0) {
            for (int trial = 0;
                    trial < PARAMS.STEP_RESPONSE_TRIALS && !isStopRequested();
                    trial++) {
                // Coast to stop
                for (DcMotorEx m : powerMotors) m.setPower(0);
                double coastStart = getRuntime();
                while (nextFrame()) {
                    double vel = Math.abs(encoder.getVelocity());
                    telemetry.addData("Pass", "%s (%d/%d)", label, passIndex + 1, totalPasses);
                    telemetry.addData(
                            "Phase",
                            "Step Response – Coasting (trial %d/%d)",
                            trial + 1,
                            PARAMS.STEP_RESPONSE_TRIALS);
                    telemetry.addData("Velocity (tps)", "%.1f", vel);
                    if (vel < PARAMS.COAST_STOP_THRESHOLD_TPS) break;
                    if (getRuntime() - coastStart > PARAMS.COAST_TIMEOUT_S) break;
                    sleep(10);
                }
                if (isStopRequested()) break;
                sleep(200); // brief pause at rest

                // Apply step and sample
                var sampleTimes = new ArrayList<Double>();
                var sampleVels = new ArrayList<Double>();
                double stepStart = getRuntime();
                for (DcMotorEx m : powerMotors) m.setPower(PARAMS.STEP_RESPONSE_POWER);

                while (nextFrame()
                        && (getRuntime() - stepStart) < PARAMS.STEP_RESPONSE_MAX_TIME_S) {
                    double t = getRuntime() - stepStart;
                    double vel = encoder.getVelocity();
                    sampleTimes.add(t);
                    sampleVels.add(vel);

                    telemetry.addData("Pass", "%s (%d/%d)", label, passIndex + 1, totalPasses);
                    telemetry.addData(
                            "Phase",
                            "Step Response – Sampling (trial %d/%d)",
                            trial + 1,
                            PARAMS.STEP_RESPONSE_TRIALS);
                    telemetry.addData("Time", "%.3f s", t);
                    telemetry.addData("Velocity (tps)", "%.1f", vel);
                    telemetry.addData("w_final (tps)", "%.1f", wFinal);
                }

                if (isStopRequested()) break;
                for (DcMotorEx m : powerMotors) m.setPower(0);

                // Linearized regression: ln(1 - w/w_final) = -t/tau
                var regT = new ArrayList<Double>();
                var regY = new ArrayList<Double>();

                for (int i = 0; i < sampleTimes.size(); i++) {
                    double ratio = sampleVels.get(i) / wFinal;
                    if (ratio < PARAMS.OMEGA_LOWER_FRACTION || ratio > PARAMS.OMEGA_UPPER_FRACTION)
                        continue;
                    regT.add(sampleTimes.get(i));
                    regY.add(Math.log(1.0 - ratio));
                }

                if (regT.size() >= 3) {
                    int n = regT.size();
                    double[] tArr = new double[n];
                    double[] yArr = new double[n];
                    for (int i = 0; i < n; i++) {
                        tArr[i] = regT.get(i);
                        yArr[i] = regY.get(i);
                    }
                    LinRegResult stepFit = linearRegression(tArr, yArr, n);
                    if (stepFit.slope < 0) {
                        tauValues.add(-1.0 / stepFit.slope);
                        stepRSquaredValues.add(stepFit.rSquared);
                    }
                }
            }
        }

        // Compute kA from averaged tau
        result.validTrials = tauValues.size();
        if (result.validTrials > 0) {
            for (double t : tauValues) result.avgTau += t;
            result.avgTau /= result.validTrials;
            for (double r : stepRSquaredValues) result.avgStepR2 += r;
            result.avgStepR2 /= result.validTrials;
            result.kA = result.avgTau * result.kV;
        }

        return result;
    }

    @Override
    protected void runOpModeInternal() throws InterruptedException {
        telemetry.addData("Status", "Ready. Press Start to begin tuning.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        int numPasses = HARDWARE.MOTORS_COUPLED ? 1 : motors.length;
        var results = new TuneResult[numPasses];

        for (int pass = 0; pass < numPasses && !isStopRequested(); pass++) {
            String label;
            DcMotorEx[] powerMotors;
            DcMotorEx encoder;

            if (HARDWARE.MOTORS_COUPLED) {
                label = "All motors";
                powerMotors = motors;
                encoder = motors[0];
            } else {
                label = HARDWARE.MOTOR_NAMES[pass];
                powerMotors = new DcMotorEx[] {motors[pass]};
                encoder = motors[pass];
            }

            results[pass] = runTuningPass(label, powerMotors, encoder, pass, numPasses);
            if (results[pass] == null) return;
        }

        // ── Display results until stopped ────────────────────────────────────
        while (nextFrame()) {
            for (int p = 0; p < numPasses; p++) {
                TuneResult r = results[p];
                String pfx = numPasses > 1 ? r.label + " " : "";
                telemetry.addData("── " + r.label + " RESULTS ──", "");
                telemetry.addData(pfx + "kS", "%.4f V", r.kS);
                telemetry.addData(pfx + "kV", "%.6f V·s/tick", r.kV);
                if (r.validTrials > 0) {
                    telemetry.addData(pfx + "kA", "%.6f V·s²/tick", r.kA);
                    telemetry.addData(pfx + "tau", "%.4f s", r.avgTau);
                    telemetry.addData(pfx + "kA R²", "%.6f", r.avgStepR2);
                    telemetry.addData(
                            pfx + "kA valid trials",
                            "%d / %d",
                            r.validTrials,
                            PARAMS.STEP_RESPONSE_TRIALS);
                } else {
                    telemetry.addData(pfx + "kA", "FAILED – no valid trials");
                }
                telemetry.addData(pfx + "kS/kV R²", "%.6f", r.rSquared);
                telemetry.addData(pfx + "stiction power", "%.4f", r.stictionPower);
                telemetry.addData(pfx + "stiction voltage", "%.3f V", r.stictionVoltage);
                telemetry.addLine("");
                telemetry.addData("── " + r.label + " PASTE ──", "");
                telemetry.addData(pfx + "  kS =", "%.4f", r.kS);
                telemetry.addData(pfx + "  kV =", "%.6f", r.kV);
                telemetry.addData(pfx + "  kA =", "%.6f", r.kA);
                telemetry.addLine("");
                telemetry.addData("── " + r.label + " DATA POINTS ──", "");
                for (int i = 0; i < r.avgVelocities.length; i++) {
                    telemetry.addData(
                            String.format("%sStep %d", pfx, i + 1),
                            "%.1f tps @ %.3f V",
                            r.avgVelocities[i],
                            r.avgVoltages[i]);
                }
                telemetry.addLine("");
            }
        }
    }
}
