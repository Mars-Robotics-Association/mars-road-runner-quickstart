package org.firstinspires.ftc.teamcode.tuning

import com.acmerobotics.dashboard.config.Config
import kotlin.math.abs
import kotlin.math.ceil

/**
 * Least-squares helpers for the automatic tuners, including a robust fit that discards the noisy
 * low-velocity samples on its own.
 *
 * <p>Feedforward ramp data has a characteristic shape: below a breakaway voltage the wheel barely
 * moves and the (velocity, voltage) samples are dominated by noise; above it the relationship is
 * cleanly linear. A plain fit over every sample is biased by that noise floor, which is why the stock
 * tools leave the final line-fit to a human reading a graph. [robustFit] automates it: it sorts
 * by the independent variable and finds the lowest cutoff above which the fit is linear (its R² reaches
 * [TARGET_R2]), keeping at least [MIN_KEEP_FRACTION] of the samples so it can't "cheat"
 * by retaining a handful of points.
 */
@Config
class TunerRegression private constructor() {
    /** Result of a linear fit `y = intercept + slope·x`. */
    class Result(
        @JvmField val intercept: Double,
        @JvmField val slope: Double,
        @JvmField val r2: Double,
        /** The x (velocity) cutoff below which samples were discarded. */
        @JvmField val threshold: Double,
        @JvmField val used: Int,
        @JvmField val total: Int,
    )

    companion object {
        /** A fit reaching this coefficient of determination is treated as "linear". */
        @JvmField
        var TARGET_R2 = 0.95

        /** Never discard so many low-velocity samples that fewer than this fraction remain. */
        @JvmField
        var MIN_KEEP_FRACTION = 0.3

        /** Below this many samples, skip cutoff selection and just fit everything. */
        @JvmField
        var MIN_POINTS = 8

        /** Ordinary least-squares fit over every sample. */
        @JvmStatic
        fun fitAll(samples: List<DoubleArray>): Result {
            return fitRange(sortedByX(samples), 0)!!
        }

        /**
         * Robust fit that automatically trims the noisy low-velocity samples: it scans increasing cutoffs
         * and returns the fit at the smallest cutoff whose R² reaches [TARGET_R2] (subject to keeping
         * [MIN_KEEP_FRACTION] of the data). If no cutoff reaches the target — unusually noisy data —
         * it returns the best-R² fit found so the caller can flag low confidence via [Result.r2].
         */
        @JvmStatic
        fun robustFit(samples: List<DoubleArray>): Result? {
            val n = samples.size
            if (n < MIN_POINTS) {
                return fitAll(samples)
            }

            val sorted = sortedByX(samples)
            val minKeep = maxOf(MIN_POINTS, ceil(MIN_KEEP_FRACTION * n).toInt())
            val maxCutoff = n - minKeep

            var best: Result? = null
            for (i in 0..maxCutoff) {
                val r = fitRange(sorted, i) ?: continue
                if (best == null || r.r2 > best.r2) {
                    best = r
                }
                if (r.r2 >= TARGET_R2) {
                    // smallest cutoff that already looks linear — take it and stop
                    return r
                }
            }
            return best ?: fitRange(sorted, 0)
        }

        /** Fits `y = intercept + slope·x` over `sorted[from .. end]`. */
        private fun fitRange(sorted: List<DoubleArray>, from: Int): Result? {
            val count = sorted.size - from
            if (count < 2) {
                return null
            }
            var sx = 0.0
            var sy = 0.0
            var sxx = 0.0
            var sxy = 0.0
            var syy = 0.0
            for (i in from until sorted.size) {
                val x = sorted[i][0]
                val y = sorted[i][1]
                sx += x
                sy += y
                sxx += x * x
                sxy += x * y
                syy += y * y
            }
            val denom = count * sxx - sx * sx
            if (abs(denom) < 1e-9) {
                return null
            }
            val slope = (count * sxy - sx * sy) / denom
            val intercept = (sy - slope * sx) / count
            val ssTot = syy - sy * sy / count
            val ssRes = syy - intercept * sy - slope * sxy
            val r2 = if (ssTot > 1e-9) 1.0 - ssRes / ssTot else 0.0
            return Result(intercept, slope, r2, sorted[from][0], count, sorted.size)
        }

        private fun sortedByX(samples: List<DoubleArray>): List<DoubleArray> {
            return samples.sortedBy { it[0] }
        }

        /**
         * Solves the 3x3 linear system `A x = b` by Gaussian elimination with partial pivoting.
         * Returns `x`, or `null` if the system is (near-)singular — e.g., the maneuver did not
         * excite all three feedforward terms independently.
         *
         * <p>Used for the joint integral-form feedforward fit
         * `∫V dt = kS·∫sign(v)dt + kV·∫v dt + kA·Δv`, where the normal-equations matrix `A`
         * (= XᵀX) and vector `b` (= Xᵀy) are accumulated over every sample.
         */
        @JvmStatic
        fun solve3(A: Array<DoubleArray>, b: DoubleArray): DoubleArray? {
            val m = arrayOf(
                doubleArrayOf(A[0][0], A[0][1], A[0][2], b[0]),
                doubleArrayOf(A[1][0], A[1][1], A[1][2], b[1]),
                doubleArrayOf(A[2][0], A[2][1], A[2][2], b[2]),
            )
            for (col in 0 until 3) {
                var pivot = col
                for (r in col + 1 until 3) {
                    if (abs(m[r][col]) > abs(m[pivot][col])) {
                        pivot = r
                    }
                }
                if (abs(m[pivot][col]) < 1e-12) {
                    return null
                }
                val tmp = m[col]
                m[col] = m[pivot]
                m[pivot] = tmp

                for (r in 0 until 3) {
                    if (r == col) {
                        continue
                    }
                    val f = m[r][col] / m[col][col]
                    for (c in col until 4) {
                        m[r][c] -= f * m[col][c]
                    }
                }
            }
            return doubleArrayOf(
                m[0][3] / m[0][0],
                m[1][3] / m[1][1],
                m[2][3] / m[2][2],
            )
        }
    }
}
