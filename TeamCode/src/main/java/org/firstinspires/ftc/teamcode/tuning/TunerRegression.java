package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
 * Least-squares helpers for the automatic tuners, including a robust fit that discards the noisy
 * low-velocity samples on its own.
 *
 * <p>Feedforward ramp data has a characteristic shape: below a breakaway voltage the wheel barely
 * moves and the (velocity, voltage) samples are dominated by noise; above it the relationship is
 * cleanly linear. A plain fit over every sample is biased by that noise floor, which is why the stock
 * tools leave the final line-fit to a human reading a graph. {@link #robustFit} automates it: it sorts
 * by the independent variable and finds the lowest cutoff above which the fit is linear (its R² reaches
 * {@link #TARGET_R2}), keeping at least {@link #MIN_KEEP_FRACTION} of the samples so it can't "cheat"
 * by retaining a handful of points.
 */
@Config
public final class TunerRegression {
    /** A fit reaching this coefficient of determination is treated as "linear". */
    public static double TARGET_R2 = 0.95;
    /** Never discard so many low-velocity samples that fewer than this fraction remain. */
    public static double MIN_KEEP_FRACTION = 0.3;
    /** Below this many samples, skip cutoff selection and just fit everything. */
    public static int MIN_POINTS = 8;

    private TunerRegression() {}

    /** Result of a linear fit {@code y = intercept + slope·x}. */
    public static final class Result {
        public final double intercept;
        public final double slope;
        public final double r2;
        /** The x (velocity) cutoff below which samples were discarded. */
        public final double threshold;
        public final int used;
        public final int total;

        Result(double intercept, double slope, double r2, double threshold, int used, int total) {
            this.intercept = intercept;
            this.slope = slope;
            this.r2 = r2;
            this.threshold = threshold;
            this.used = used;
            this.total = total;
        }
    }

    /** Ordinary least-squares fit over every sample. */
    public static Result fitAll(List<double[]> samples) {
        return fitRange(sortedByX(samples), 0);
    }

    /**
     * Robust fit that automatically trims the noisy low-velocity samples: it scans increasing cutoffs
     * and returns the fit at the smallest cutoff whose R² reaches {@link #TARGET_R2} (subject to keeping
     * {@link #MIN_KEEP_FRACTION} of the data). If no cutoff reaches the target — unusually noisy data —
     * it returns the best-R² fit found so the caller can flag low confidence via {@link Result#r2}.
     */
    public static Result robustFit(List<double[]> samples) {
        int n = samples.size();
        if (n < MIN_POINTS) {
            return fitAll(samples);
        }

        List<double[]> sorted = sortedByX(samples);
        int minKeep = Math.max(MIN_POINTS, (int) Math.ceil(MIN_KEEP_FRACTION * n));
        int maxCutoff = n - minKeep;

        Result best = null;
        for (int i = 0; i <= maxCutoff; i++) {
            Result r = fitRange(sorted, i);
            if (r == null) {
                continue;
            }
            if (best == null || r.r2 > best.r2) {
                best = r;
            }
            if (r.r2 >= TARGET_R2) {
                // smallest cutoff that already looks linear — take it and stop
                return r;
            }
        }
        return best != null ? best : fitRange(sorted, 0);
    }

    /** Fits {@code y = intercept + slope·x} over {@code sorted[from .. end]}. */
    private static Result fitRange(List<double[]> sorted, int from) {
        int count = sorted.size() - from;
        if (count < 2) {
            return null;
        }
        double sx = 0, sy = 0, sxx = 0, sxy = 0, syy = 0;
        for (int i = from; i < sorted.size(); i++) {
            double x = sorted.get(i)[0], y = sorted.get(i)[1];
            sx += x;
            sy += y;
            sxx += x * x;
            sxy += x * y;
            syy += y * y;
        }
        double denom = count * sxx - sx * sx;
        if (Math.abs(denom) < 1e-9) {
            return null;
        }
        double slope = (count * sxy - sx * sy) / denom;
        double intercept = (sy - slope * sx) / count;
        double ssTot = syy - sy * sy / count;
        double ssRes = syy - intercept * sy - slope * sxy;
        double r2 = ssTot > 1e-9 ? 1.0 - ssRes / ssTot : 0.0;
        return new Result(intercept, slope, r2, sorted.get(from)[0], count, sorted.size());
    }

    private static List<double[]> sortedByX(List<double[]> samples) {
        List<double[]> sorted = new ArrayList<>(samples);
        sorted.sort(Comparator.comparingDouble(p -> p[0]));
        return sorted;
    }

    /**
     * Solves the 3x3 linear system {@code A x = b} by Gaussian elimination with partial pivoting.
     * Returns {@code x}, or {@code null} if the system is (near-)singular — e.g., the maneuver did not
     * excite all three feedforward terms independently.
     *
     * <p>Used for the joint integral-form feedforward fit
     * {@code ∫V dt = kS·∫sign(v)dt + kV·∫v dt + kA·Δv}, where the normal-equations matrix {@code A}
     * (= XᵀX) and vector {@code b} (= Xᵀy) are accumulated over every sample.
     */
    public static double[] solve3(double[][] A, double[] b) {
        double[][] m = {
                {A[0][0], A[0][1], A[0][2], b[0]},
                {A[1][0], A[1][1], A[1][2], b[1]},
                {A[2][0], A[2][1], A[2][2], b[2]},
        };
        for (int col = 0; col < 3; col++) {
            int pivot = col;
            for (int r = col + 1; r < 3; r++) {
                if (Math.abs(m[r][col]) > Math.abs(m[pivot][col])) {
                    pivot = r;
                }
            }
            if (Math.abs(m[pivot][col]) < 1e-12) {
                return null;
            }
            double[] tmp = m[col];
            m[col] = m[pivot];
            m[pivot] = tmp;

            for (int r = 0; r < 3; r++) {
                if (r == col) {
                    continue;
                }
                double f = m[r][col] / m[col][col];
                for (int c = col; c < 4; c++) {
                    m[r][c] -= f * m[col][c];
                }
            }
        }
        return new double[]{
                m[0][3] / m[0][0],
                m[1][3] / m[1][1],
                m[2][3] / m[2][2],
        };
    }
}
