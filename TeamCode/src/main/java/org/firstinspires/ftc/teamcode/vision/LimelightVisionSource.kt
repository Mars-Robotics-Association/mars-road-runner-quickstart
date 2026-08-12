package org.firstinspires.ftc.teamcode.vision

import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.LLResultTypes
import com.qualcomm.hardware.limelightvision.LLStatus
import com.qualcomm.hardware.limelightvision.Limelight3A
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D
import org.firstinspires.ftc.robotcore.external.navigation.Position
import org.marsroboticsassociation.controllib.localization.vision.PlanarPnpSolver
import org.marsroboticsassociation.controllib.localization.vision.TagAmbiguityMath
import org.marsroboticsassociation.controllib.localization.vision.TagAmbiguitySolver
import org.marsroboticsassociation.controllib.localization.vision.VisionFrame
import org.marsroboticsassociation.controllib.localization.vision.VisionSource

/**
 * Live [VisionSource] over a [Limelight3A]: each [latest] reads the SDK's cached `LLResult`,
 * converts the MegaTag1/MegaTag2 botposes to WPILib [Pose2d], and re-solves per-tag PnP ambiguity
 * (with the camera's own calibration) into a [VisionFrame]. This is the vision *acquisition* half,
 * so the localizer policy consumes only `VisionFrame`s and is replayable off-robot.
 *
 * The actual PnP solve is delegated to an injected [PlanarPnpSolver] (the OpenCV boundary), so all
 * the pure ambiguity math lives in ControlLib. The acquisition tuning it reads (`tagSizeMeters`,
 * `ambiguityMinTagAreaPct`) lives in [HypothesisBankRoadRunnerLocalizer.Params] so the FtcDashboard
 * `@Config` keys stay together.
 */
class LimelightVisionSource(
    private val limelight: Limelight3A,
    private val telemetry: Telemetry?,
    private val pnp: PlanarPnpSolver,
) : VisionSource {

    // Lazily built per-tag ambiguity solver. Intrinsics come from the Limelight's own calibration,
    // fetched once (camMatVector/camDistCoeffs cached). Null until a valid calibration is obtained.
    // Rebuilt only when tagSizeMeters changes.
    private var ambiguitySolver: TagAmbiguitySolver? = null
    private var ambiguitySolverSig: String? = null
    private var camMatVector: DoubleArray? = null
    private var camDistCoeffs: DoubleArray? = null
    private var lastCalAttemptNanos: Long = Long.MIN_VALUE

    override fun updateRobotOrientation(headingDeg: Double) {
        limelight.updateRobotOrientation(headingDeg)
    }

    override fun getCalFx(): Double {
        val cam = camMatVector
        return if (cam != null) cam[0] else Double.NaN
    }

    // Remaining intrinsics from the OpenCV 3x3 camera matrix (row-major [fx,0,cx, 0,fy,cy, 0,0,1]).
    override fun getCalFy(): Double {
        val cam = camMatVector
        return if (cam != null && cam.size >= 6) cam[4] else Double.NaN
    }

    override fun getCalCx(): Double {
        val cam = camMatVector
        return if (cam != null && cam.size >= 6) cam[2] else Double.NaN
    }

    override fun getCalCy(): Double {
        val cam = camMatVector
        return if (cam != null && cam.size >= 6) cam[5] else Double.NaN
    }

    override fun getCalDistCoeffs(): DoubleArray? {
        return camDistCoeffs // the exact vector handed to the PnP solve (see ambiguitySolver())
    }

    /**
     * Reads the latest cached camera result and extracts every feature the policy and the telemetry
     * log consume. [VisionFrame.receiptNanos] is sampled here (at result receipt, not later) so a
     * back-date by latency is not inflated by downstream work.
     */
    override fun latest(): VisionFrame {
        val f = VisionFrame()
        val result = limelight.latestResult
        f.receiptNanos = System.nanoTime()
        if (result == null || !result.isValid) {
            f.valid = false
            return f
        }
        f.valid = true
        f.timestamp = result.timestamp
        f.tagCount = result.botposeTagCount
        f.avgDistM = result.botposeAvgDist
        f.latencySec = (result.captureLatency + result.targetingLatency) / 1000.0
        f.focusMetric = result.focusMetric
        f.stddevMt1 = result.stddevMt1

        // NOTE: getBotpose() is NEVER null — on malformed data the SDK returns a zero/identity
        // Pose3D at the field origin. So we cannot null-check; the caller treats the botpose as
        // real only when tagCount > 0 && avgDistM > 0. We still extract it here unconditionally.
        val mt1 = result.botpose
        val p = mt1.position.toUnit(DistanceUnit.INCH)
        f.mt1Pose = Pose2d(p.x, p.y, Rotation2d(mt1.orientation.getYaw(AngleUnit.RADIANS)))
        f.visionZIn = p.z
        f.visionRollDeg = mt1.orientation.getRoll(AngleUnit.DEGREES)
        f.visionPitchDeg = mt1.orientation.getPitch(AngleUnit.DEGREES)

        captureMt2Pose(result, f)
        captureAllTags(result, f)
        f.ambiguity = computePoseAmbiguity(result, f)
        return f
    }

    /**
     * Captures the raw, un-consumed MegaTag2 botpose (position→inches, heading in radians). Stays
     * null when unavailable (the MT2 field-origin zero-sentinel is skipped). Pure capture — no
     * gating, no fusion.
     */
    private fun captureMt2Pose(result: LLResult, f: VisionFrame) {
        val mt2 = result.botpose_MT2
        if (mt2 != null) {
            val p = mt2.position.toUnit(DistanceUnit.INCH)
            if (p.x != 0.0 || p.y != 0.0) { // skip the field-origin zero-sentinel
                f.mt2Pose =
                    Pose2d(p.x, p.y, Rotation2d(mt2.orientation.getYaw(AngleUnit.RADIANS)))
            }
        }
    }

    /**
     * Pose-level ambiguity for this frame (our own PnP), or NaN when it can't be scored —
     * intrinsics unset, corners not emitted by the pipeline, or the solve failed. NaN fails the
     * gate open. Also fills the winning tag's diagnostic fields on `f` (sol candidates, t6t*
     * poses, tx/ty, corner bbox, skew, tag id).
     *
     * Aggregation is the **minimum** per-tag ambiguity over tags large enough to be reliable
     * (`ambiguityMinTagAreaPct`), falling back to the overall min if none qualify.
     */
    private fun computePoseAmbiguity(result: LLResult, f: VisionFrame): Double {
        val est = ambiguitySolver() ?: return Double.NaN
        val fiducials = result.fiducialResults ?: return Double.NaN
        var bestQualified = Double.NaN
        var bestAny = Double.NaN
        var winQual: TagAmbiguitySolver.PnpSolutions? = null
        var winAny: TagAmbiguitySolver.PnpSolutions? = null
        var winQualFr: LLResultTypes.FiducialResult? = null
        var winAnyFr: LLResultTypes.FiducialResult? = null
        var winQualId = -1
        var winAnyId = -1
        for (fr in fiducials) {
            val s = est.solve(fr.targetCorners) ?: continue
            val a = s.ratio
            if (bestAny.isNaN() || a < bestAny) {
                bestAny = a
                winAny = s
                winAnyFr = fr
                winAnyId = fr.fiducialId
            }
            if (
                fr.targetArea >= HypothesisBankRoadRunnerLocalizer.PARAMS.ambiguityMinTagAreaPct &&
                    (bestQualified.isNaN() || a < bestQualified)
            ) {
                bestQualified = a
                winQual = s
                winQualFr = fr
                winQualId = fr.fiducialId
            }
        }
        val win = if (winQual != null) winQual else winAny
        val winFr = if (winQual != null) winQualFr else winAnyFr
        if (win != null && winFr != null) {
            f.reprojErrPx = win.reprojErrBest
            f.solBestRvec = win.rvecBest
            f.solBestTvec = win.tvecBest
            f.solAltRvec = win.rvecAlt
            f.solAltTvec = win.tvecAlt
            f.solTagId = if (winQual != null) winQualId else winAnyId
            f.skew = winFr.skew
            f.t6tCs = pose6(winFr.targetPoseCameraSpace)
            f.t6tRs = pose6(winFr.targetPoseRobotSpace)
            f.t6rFs = pose6(winFr.robotPoseFieldSpace)
            f.tagTxDeg = winFr.targetXDegrees
            f.tagTyDeg = winFr.targetYDegrees
            val bbox = cornerBounds(winFr.targetCorners)
            f.tagMinXPx = bbox[0]
            f.tagMaxXPx = bbox[1]
            f.tagMinYPx = bbox[2]
            f.tagMaxYPx = bbox[3]
        }
        return if (bestQualified.isNaN()) bestAny else bestQualified
    }

    /**
     * The ambiguity solver, built from the Limelight's own camera calibration (fetched once and
     * cached). Returns null until a valid calibration is obtained. Rebuilds only when
     * `tagSizeMeters` changes.
     */
    private fun ambiguitySolver(): TagAmbiguitySolver? {
        val tagSizeMeters = HypothesisBankRoadRunnerLocalizer.PARAMS.tagSizeMeters
        if (tagSizeMeters <= 0) {
            return null
        }
        if (camMatVector == null) {
            fetchCalibration()
        }
        val cam = camMatVector ?: return null // camera not yet calibrated/ready — try again later
        val sig = tagSizeMeters.toString()
        if (sig != ambiguitySolverSig) {
            ambiguitySolver = TagAmbiguitySolver(pnp, cam, camDistCoeffs, tagSizeMeters)
            ambiguitySolverSig = sig
        }
        return ambiguitySolver
    }

    /**
     * Lazy fallback for loading the camera calibration (intrinsics + distortion): tries once, then
     * is throttled so a not-yet-ready camera doesn't trigger an HTTP request every loop. **Prefer
     * [prefetchCalibration] at init** — once [Limelight3A.start] is polling, the SDK's calibration
     * GET reliably times out.
     */
    private fun fetchCalibration() {
        if (camMatVector != null) {
            return
        }
        val now = System.nanoTime()
        if (now - lastCalAttemptNanos < 2_000_000_000L) {
            return
        }
        lastCalAttemptNanos = now
        if (!tryLoadCalibration()) {
            log("ambiguity cal", "unavailable — gate inert, retrying")
        }
    }

    /**
     * Load the camera calibration up-front, retrying until the camera is ready or [timeoutMs]
     * elapses. **Call this once after constructing the source but before [Limelight3A.start]**,
     * while the camera's HTTP server is idle. No-op (returns true) once a matrix is cached.
     *
     * @return true if a usable calibration is loaded by the deadline.
     */
    override fun prefetchCalibration(timeoutMs: Long): Boolean {
        if (camMatVector != null) {
            return true
        }
        val deadline = System.nanoTime() + maxOf(0L, timeoutMs) * 1_000_000L
        var attempts = 0
        while (true) {
            attempts++
            if (tryLoadCalibration()) { // a successful GET also proves the camera is up
                return true
            }
            if (System.nanoTime() >= deadline) {
                break
            }
            try {
                Thread.sleep(20)
            } catch (e: InterruptedException) {
                Thread.currentThread().interrupt()
                break
            }
        }
        val status: LLStatus? = limelight.status
        val reachability =
            if (status != null && (status.hwType != 0 || status.name.isNotEmpty())) {
                String.format(
                    "camera reachable (name='%s' hw=%d) but calibration endpoints empty" +
                        " — recalibrate the Limelight",
                    status.name,
                    status.hwType,
                )
            } else {
                "camera not reachable — not yet booted, or wrong address/wiring"
            }
        log(
            "ambiguity cal",
            String.format(
                "PREFETCH FAILED after %d attempts / %dms — AMBIGUITY GATE INERT (%s)",
                attempts,
                timeoutMs,
                reachability,
            ),
        )
        return false
    }

    /**
     * One untimed attempt to fetch + cache a usable calibration. Returns true and logs the source
     * on success; returns false (leaving [camMatVector] null) when every source is
     * missing/unreachable.
     */
    private fun tryLoadCalibration(): Boolean {
        val cal = firstUsableCal() ?: return false
        camMatVector = cal.camMatVector
        camDistCoeffs = cal.distortionCoefficients
        log(
            "ambiguity cal",
            String.format(
                "%s reproj=%.3f res=%.0fx%.0f",
                cal.displayName,
                cal.reprojectionError,
                cal.resX,
                cal.resY,
            ),
        )
        return true
    }

    /**
     * The first usable camera calibration, preferring a user ChArUco recalibration (latest → file →
     * eeprom) and falling back to the factory default. Returns null only when every source is
     * missing/garbled.
     */
    private fun firstUsableCal(): LLResultTypes.CalibrationResult? {
        var cal = usableOrNull(limelight.calLatest)
        if (cal == null) cal = usableOrNull(limelight.calFile)
        if (cal == null) cal = usableOrNull(limelight.calEEPROM)
        if (cal == null) cal = usableOrNull(limelight.calDefault)
        return cal
    }

    private fun log(key: String, value: String) {
        telemetry?.addData(key, value)
    }

    companion object {
        /** Flatten a Pose3D to `[x,y,z(m), yaw,pitch,roll(deg)]`; null-safe. */
        private fun pose6(p: Pose3D?): DoubleArray? {
            if (p == null) {
                return null
            }
            val t: Position = p.position.toUnit(DistanceUnit.METER)
            val o = p.orientation
            return doubleArrayOf(
                t.x,
                t.y,
                t.z,
                o.getYaw(AngleUnit.DEGREES),
                o.getPitch(AngleUnit.DEGREES),
                o.getRoll(AngleUnit.DEGREES),
            )
        }

        /**
         * Captures EVERY detected fiducial's id + image corners into `f.allTagIds`/`f.allTagCorners`,
         * in the SDK's fiducial order. Independent of calibration (raw detector output), so it runs
         * even when the PnP ambiguity solve can't.
         */
        private fun captureAllTags(result: LLResult, f: VisionFrame) {
            val fids = result.fiducialResults
            if (fids == null || fids.isEmpty()) {
                return
            }
            val n = fids.size
            val ids = IntArray(n)
            val corners =
                Array(n) { i ->
                    ids[i] = fids[i].fiducialId
                    flattenCorners(fids[i].targetCorners)
                }
            f.allTagIds = ids
            f.allTagCorners = corners
        }

        /**
         * A tag's image corners flattened to `[x0,y0,x1,y1,x2,y2,x3,y3]` (px), or all-NaN if absent
         * / not exactly the 4 corners of one tag.
         */
        private fun flattenCorners(corners: List<List<Double>>?): DoubleArray {
            val out =
                doubleArrayOf(
                    Double.NaN,
                    Double.NaN,
                    Double.NaN,
                    Double.NaN,
                    Double.NaN,
                    Double.NaN,
                    Double.NaN,
                    Double.NaN,
                )
            if (corners == null || corners.size != 4) {
                return out
            }
            for (i in 0 until 4) {
                val c = corners[i]
                if (c != null && c.size >= 2) {
                    out[2 * i] = c[0]
                    out[2 * i + 1] = c[1]
                }
            }
            return out
        }

        /**
         * Pixel bounding box `[minX, maxX, minY, maxY]` of a tag's image corners, or all-NaN if
         * absent.
         */
        private fun cornerBounds(corners: List<List<Double>>?): DoubleArray {
            if (corners == null || corners.isEmpty()) {
                return doubleArrayOf(Double.NaN, Double.NaN, Double.NaN, Double.NaN)
            }
            var minX = Double.POSITIVE_INFINITY
            var maxX = Double.NEGATIVE_INFINITY
            var minY = Double.POSITIVE_INFINITY
            var maxY = Double.NEGATIVE_INFINITY
            for (c in corners) {
                if (c == null || c.size < 2) {
                    continue
                }
                val x = c[0]
                val y = c[1]
                minX = minOf(minX, x)
                maxX = maxOf(maxX, x)
                minY = minOf(minY, y)
                maxY = maxOf(maxY, y)
            }
            if (!minX.isFinite()) {
                return doubleArrayOf(Double.NaN, Double.NaN, Double.NaN, Double.NaN)
            }
            return doubleArrayOf(minX, maxX, minY, maxY)
        }

        private fun usableOrNull(cal: LLResultTypes.CalibrationResult?): LLResultTypes.CalibrationResult? {
            return if (
                cal != null &&
                    cal.isValid &&
                    TagAmbiguityMath.isUsableCameraMatrix(cal.camMatVector)
            ) {
                cal
            } else {
                null
            }
        }
    }
}
