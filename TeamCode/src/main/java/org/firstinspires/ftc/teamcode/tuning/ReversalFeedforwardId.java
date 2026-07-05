package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

/**
 * Shared core for the reversal-based feedforward tuners ({@link AxialFeedforwardTuner} and the strafe
 * form in {@link LateralFeedforwardTuner}). It drives an open-loop square wave — alternating
 * {@code ±maxPower} — and fits {@code kS}, {@code kV}, and {@code kA} at once.
 *
 * <p>The reversals are the whole point: on a one-directional ramp, acceleration is nearly proportional
 * to velocity, so {@code kA·a} is collinear with {@code kV·v} and can't be separated (which is why the
 * stock tuner eyeballs kA). Reversals decorrelate the two.
 *
 * <p>To avoid differentiating noisy velocity, it fits the integral form
 * {@code ∫V dt = kS·∫sign(v)dt + kV·∫v dt + kA·(v − v0)} — the kA term is an exact velocity difference,
 * and integration smooths the data. One equation per sample gives a 3-parameter least-squares system
 * (solved by {@link TunerRegression#solve3}) whose coefficients are the constants.
 *
 * <p>The caller supplies the axis: {@code setPower} applies the scalar to the wheels (all wheels for
 * axial; the strafe pattern for lateral), and {@code wheelVel} returns the corresponding signed wheel
 * velocity in in/s from the localizer (drive-motor encoders are often unwired). {@code power} and
 * {@code wheelVel} must share a sign convention — both flip together when the square wave reverses.
 */
public final class ReversalFeedforwardId {
    private ReversalFeedforwardId() {}

    /** Fitted constants in tick units (kS in volts), matching {@code Params.kS/kV/kA}. */
    public static final class Result {
        public final boolean singular;
        public final double kS;
        public final double kV;
        public final double kA;
        public final int samples;

        Result(boolean singular, double kS, double kV, double kA, int samples) {
            this.singular = singular;
            this.kS = kS;
            this.kV = kV;
            this.kA = kA;
            this.samples = samples;
        }
    }

    public static Result identify(
            LinearOpMode opMode,
            Telemetry telemetry,
            DoubleConsumer setPower,
            DoubleSupplier wheelVel,
            VoltageSensor voltageSensor,
            double inPerTick,
            double maxPower,
            double halfCycle,
            int cycles,
            double signDeadband) {
        // integral-form normal equations for  cumV = kS*cumSign + kV*cumVel + kA*dvel
        double[][] ata = new double[3][3];
        double[] atb = new double[3];
        double cumV = 0, cumSign = 0, cumVel = 0;
        double v0 = Double.NaN, lastAppliedV = 0, lastSign = 0, lastVel = 0, lastTime = 0;
        int samples = 0;

        ElapsedTime runTime = new ElapsedTime();
        double totalTime = cycles * halfCycle;
        while (opMode.opModeIsActive() && runTime.seconds() < totalTime) {
            double t = runTime.seconds();
            int halfCycleIdx = (int) (t / halfCycle);
            double power = (halfCycleIdx % 2 == 0) ? maxPower : -maxPower;
            setPower.accept(power);

            double appliedV = power * voltageSensor.getVoltage();
            double vel = wheelVel.getAsDouble(); // signed in/s
            double signV = Math.abs(vel) < signDeadband ? 0.0 : Math.signum(vel);

            if (Double.isNaN(v0)) {
                v0 = vel;
            } else {
                // trapezoidal integration since the previous sample
                double dt = t - lastTime;
                cumV += 0.5 * (appliedV + lastAppliedV) * dt;
                cumSign += 0.5 * (signV + lastSign) * dt;
                cumVel += 0.5 * (vel + lastVel) * dt;

                double[] x = {cumSign, cumVel, vel - v0};
                for (int r = 0; r < 3; r++) {
                    for (int c = 0; c < 3; c++) {
                        ata[r][c] += x[r] * x[c];
                    }
                    atb[r] += x[r] * cumV;
                }
                samples++;
            }
            lastAppliedV = appliedV;
            lastSign = signV;
            lastVel = vel;
            lastTime = t;

            telemetry.addData("phase", "%d / %d", halfCycleIdx + 1, cycles);
            telemetry.addData("speed (in/s)", "%.1f", vel);
            telemetry.addData("samples", samples);
            telemetry.update();
        }
        setPower.accept(0);

        double[] fit = TunerRegression.solve3(ata, atb);
        if (fit == null) {
            return new Result(true, 0, 0, 0, samples);
        }
        // convert V-per-(in/s) and V-per-(in/s^2) to tick units, like Params.kV / kA
        return new Result(false, fit[0], fit[1] * inPerTick, fit[2] * inPerTick, samples);
    }
}
