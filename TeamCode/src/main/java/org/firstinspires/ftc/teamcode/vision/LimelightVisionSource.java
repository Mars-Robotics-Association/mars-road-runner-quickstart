package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.marsroboticsassociation.controllib.localization.vision.PlanarPnpSolver;
import org.marsroboticsassociation.controllib.localization.vision.TagAmbiguityMath;
import org.marsroboticsassociation.controllib.localization.vision.TagAmbiguitySolver;
import org.marsroboticsassociation.controllib.localization.vision.VisionFrame;
import org.marsroboticsassociation.controllib.localization.vision.VisionSource;

/**
 * Live {@link VisionSource} over a {@link Limelight3A}: each {@link #latest()} reads the SDK's
 * cached {@code LLResult}, converts the MegaTag1/MegaTag2 botposes to WPILib {@link Pose2d}, and
 * re-solves per-tag PnP ambiguity (with the camera's own calibration) into a {@link VisionFrame}.
 * This is the vision <i>acquisition</i> half, so the localizer policy consumes only {@code
 * VisionFrame}s and is replayable off-robot.
 *
 * <p>The actual PnP solve is delegated to an injected {@link PlanarPnpSolver} (the OpenCV
 * boundary), so all the pure ambiguity math lives in ControlLib. The acquisition tuning it reads
 * ({@code tagSizeMeters}, {@code ambiguityMinTagAreaPct}) lives in {@link
 * HypothesisBankRoadRunnerLocalizer.Params} so the FtcDashboard {@code @Config} keys stay together.
 */
public final class LimelightVisionSource implements VisionSource {

    private final Limelight3A limelight;
    private final Telemetry telemetry;
    private final PlanarPnpSolver pnp;

    // Lazily built per-tag ambiguity solver. Intrinsics come from the Limelight's own calibration,
    // fetched once (camMatVector/camDistCoeffs cached). Null until a valid calibration is obtained.
    // Rebuilt only when tagSizeMeters changes.
    private TagAmbiguitySolver ambiguitySolver = null;
    private String ambiguitySolverSig = null;
    private double[] camMatVector = null;
    private double[] camDistCoeffs = null;
    private long lastCalAttemptNanos = Long.MIN_VALUE;

    public LimelightVisionSource(Limelight3A limelight, Telemetry telemetry, PlanarPnpSolver pnp) {
        this.limelight = limelight;
        this.telemetry = telemetry;
        this.pnp = pnp;
    }

    @Override
    public void updateRobotOrientation(double headingDeg) {
        limelight.updateRobotOrientation(headingDeg);
    }

    @Override
    public double getCalFx() {
        return camMatVector != null ? camMatVector[0] : Double.NaN;
    }

    // Remaining intrinsics from the OpenCV 3x3 camera matrix (row-major [fx,0,cx, 0,fy,cy, 0,0,1]).
    @Override
    public double getCalFy() {
        return camMatVector != null && camMatVector.length >= 6 ? camMatVector[4] : Double.NaN;
    }

    @Override
    public double getCalCx() {
        return camMatVector != null && camMatVector.length >= 6 ? camMatVector[2] : Double.NaN;
    }

    @Override
    public double getCalCy() {
        return camMatVector != null && camMatVector.length >= 6 ? camMatVector[5] : Double.NaN;
    }

    @Override
    public double[] getCalDistCoeffs() {
        return camDistCoeffs; // the exact vector handed to the PnP solve (see ambiguitySolver())
    }

    /**
     * Reads the latest cached camera result and extracts every feature the policy and the telemetry
     * log consume. {@link VisionFrame#receiptNanos} is sampled here (at result receipt, not later)
     * so a back-date by latency is not inflated by downstream work.
     */
    @Override
    public VisionFrame latest() {
        VisionFrame f = new VisionFrame();
        LLResult result = limelight.getLatestResult();
        f.receiptNanos = System.nanoTime();
        f.valid = result != null && result.isValid();
        if (!f.valid) {
            return f;
        }
        f.timestamp = result.getTimestamp();
        f.tagCount = result.getBotposeTagCount();
        f.avgDistM = result.getBotposeAvgDist();
        f.latencySec = (result.getCaptureLatency() + result.getTargetingLatency()) / 1000.0;
        f.focusMetric = result.getFocusMetric();
        f.stddevMt1 = result.getStddevMt1();

        // NOTE: getBotpose() is NEVER null — on malformed data the SDK returns a zero/identity
        // Pose3D at the field origin. So we cannot null-check; the caller treats the botpose as
        // real
        // only when tagCount > 0 && avgDistM > 0. We still extract it here unconditionally.
        Pose3D mt1 = result.getBotpose();
        Position p = mt1.getPosition().toUnit(DistanceUnit.INCH);
        f.mt1Pose =
                new Pose2d(
                        p.x, p.y, new Rotation2d(mt1.getOrientation().getYaw(AngleUnit.RADIANS)));
        f.visionZIn = p.z;
        f.visionRollDeg = mt1.getOrientation().getRoll(AngleUnit.DEGREES);
        f.visionPitchDeg = mt1.getOrientation().getPitch(AngleUnit.DEGREES);

        captureMt2Pose(result, f);
        captureAllTags(result, f);
        f.ambiguity = computePoseAmbiguity(result, f);
        return f;
    }

    /**
     * Captures the raw, un-consumed MegaTag2 botpose (position→inches, heading in radians). Stays
     * null when unavailable (the MT2 field-origin zero-sentinel is skipped). Pure capture — no
     * gating, no fusion.
     */
    private void captureMt2Pose(LLResult result, VisionFrame f) {
        Pose3D mt2 = result.getBotpose_MT2();
        if (mt2 != null) {
            Position p = mt2.getPosition().toUnit(DistanceUnit.INCH);
            if (p.x != 0.0 || p.y != 0.0) { // skip the field-origin zero-sentinel
                f.mt2Pose =
                        new Pose2d(
                                p.x,
                                p.y,
                                new Rotation2d(mt2.getOrientation().getYaw(AngleUnit.RADIANS)));
            }
        }
    }

    /**
     * Pose-level ambiguity for this frame (our own PnP), or NaN when it can't be scored —
     * intrinsics unset, corners not emitted by the pipeline, or the solve failed. NaN fails the
     * gate open. Also fills the winning tag's diagnostic fields on {@code f} (sol candidates, t6t*
     * poses, tx/ty, corner bbox, skew, tag id).
     *
     * <p>Aggregation is the <b>minimum</b> per-tag ambiguity over tags large enough to be reliable
     * ({@code ambiguityMinTagAreaPct}), falling back to the overall min if none qualify.
     */
    private double computePoseAmbiguity(LLResult result, VisionFrame f) {
        TagAmbiguitySolver est = ambiguitySolver();
        if (est == null) {
            return Double.NaN;
        }
        java.util.List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null) {
            return Double.NaN;
        }
        double bestQualified = Double.NaN;
        double bestAny = Double.NaN;
        TagAmbiguitySolver.PnpSolutions winQual = null, winAny = null;
        LLResultTypes.FiducialResult winQualFr = null, winAnyFr = null;
        int winQualId = -1, winAnyId = -1;
        for (LLResultTypes.FiducialResult fr : fiducials) {
            TagAmbiguitySolver.PnpSolutions s = est.solve(fr.getTargetCorners());
            if (s == null) {
                continue;
            }
            double a = s.ratio;
            if (Double.isNaN(bestAny) || a < bestAny) {
                bestAny = a;
                winAny = s;
                winAnyFr = fr;
                winAnyId = fr.getFiducialId();
            }
            if (fr.getTargetArea()
                            >= HypothesisBankRoadRunnerLocalizer.PARAMS.ambiguityMinTagAreaPct
                    && (Double.isNaN(bestQualified) || a < bestQualified)) {
                bestQualified = a;
                winQual = s;
                winQualFr = fr;
                winQualId = fr.getFiducialId();
            }
        }
        TagAmbiguitySolver.PnpSolutions win = winQual != null ? winQual : winAny;
        LLResultTypes.FiducialResult winFr = winQual != null ? winQualFr : winAnyFr;
        if (win != null) {
            f.reprojErrPx = win.reprojErrBest;
            f.solBestRvec = win.rvecBest;
            f.solBestTvec = win.tvecBest;
            f.solAltRvec = win.rvecAlt;
            f.solAltTvec = win.tvecAlt;
            f.solTagId = winQual != null ? winQualId : winAnyId;
            f.skew = winFr.getSkew();
            f.t6tCs = pose6(winFr.getTargetPoseCameraSpace());
            f.t6tRs = pose6(winFr.getTargetPoseRobotSpace());
            f.t6rFs = pose6(winFr.getRobotPoseFieldSpace());
            f.tagTxDeg = winFr.getTargetXDegrees();
            f.tagTyDeg = winFr.getTargetYDegrees();
            double[] bbox = cornerBounds(winFr.getTargetCorners());
            f.tagMinXPx = bbox[0];
            f.tagMaxXPx = bbox[1];
            f.tagMinYPx = bbox[2];
            f.tagMaxYPx = bbox[3];
        }
        return Double.isNaN(bestQualified) ? bestAny : bestQualified;
    }

    /** Flatten a Pose3D to {@code [x,y,z(m), yaw,pitch,roll(deg)]}; null-safe. */
    private static double[] pose6(Pose3D p) {
        if (p == null) {
            return null;
        }
        Position t = p.getPosition().toUnit(DistanceUnit.METER);
        org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles o =
                p.getOrientation();
        return new double[] {
            t.x,
            t.y,
            t.z,
            o.getYaw(AngleUnit.DEGREES),
            o.getPitch(AngleUnit.DEGREES),
            o.getRoll(AngleUnit.DEGREES)
        };
    }

    /**
     * Captures EVERY detected fiducial's id + image corners into {@code f.allTagIds}/{@code
     * f.allTagCorners}, in the SDK's fiducial order. Independent of calibration (raw detector
     * output), so it runs even when the PnP ambiguity solve can't.
     */
    private static void captureAllTags(LLResult result, VisionFrame f) {
        java.util.List<LLResultTypes.FiducialResult> fids = result.getFiducialResults();
        if (fids == null || fids.isEmpty()) {
            return;
        }
        int n = fids.size();
        int[] ids = new int[n];
        double[][] corners = new double[n][];
        for (int i = 0; i < n; i++) {
            ids[i] = fids.get(i).getFiducialId();
            corners[i] = flattenCorners(fids.get(i).getTargetCorners());
        }
        f.allTagIds = ids;
        f.allTagCorners = corners;
    }

    /**
     * A tag's image corners flattened to {@code [x0,y0,x1,y1,x2,y2,x3,y3]} (px), or all-NaN if
     * absent / not exactly the 4 corners of one tag.
     */
    private static double[] flattenCorners(java.util.List<java.util.List<Double>> corners) {
        double[] out = {
            Double.NaN, Double.NaN, Double.NaN, Double.NaN,
            Double.NaN, Double.NaN, Double.NaN, Double.NaN
        };
        if (corners == null || corners.size() != 4) {
            return out;
        }
        for (int i = 0; i < 4; i++) {
            java.util.List<Double> c = corners.get(i);
            if (c != null && c.size() >= 2) {
                out[2 * i] = c.get(0);
                out[2 * i + 1] = c.get(1);
            }
        }
        return out;
    }

    /**
     * Pixel bounding box {@code [minX, maxX, minY, maxY]} of a tag's image corners, or all-NaN if
     * absent.
     */
    private static double[] cornerBounds(java.util.List<java.util.List<Double>> corners) {
        if (corners == null || corners.isEmpty()) {
            return new double[] {Double.NaN, Double.NaN, Double.NaN, Double.NaN};
        }
        double minX = Double.POSITIVE_INFINITY, maxX = Double.NEGATIVE_INFINITY;
        double minY = Double.POSITIVE_INFINITY, maxY = Double.NEGATIVE_INFINITY;
        for (java.util.List<Double> c : corners) {
            if (c == null || c.size() < 2) {
                continue;
            }
            double x = c.get(0), y = c.get(1);
            minX = Math.min(minX, x);
            maxX = Math.max(maxX, x);
            minY = Math.min(minY, y);
            maxY = Math.max(maxY, y);
        }
        if (!Double.isFinite(minX)) {
            return new double[] {Double.NaN, Double.NaN, Double.NaN, Double.NaN};
        }
        return new double[] {minX, maxX, minY, maxY};
    }

    /**
     * The ambiguity solver, built from the Limelight's own camera calibration (fetched once and
     * cached). Returns null until a valid calibration is obtained. Rebuilds only when {@code
     * tagSizeMeters} changes.
     */
    private TagAmbiguitySolver ambiguitySolver() {
        double tagSizeMeters = HypothesisBankRoadRunnerLocalizer.PARAMS.tagSizeMeters;
        if (tagSizeMeters <= 0) {
            return null;
        }
        if (camMatVector == null) {
            fetchCalibration();
            if (camMatVector == null) {
                return null; // camera not yet calibrated/ready — try again on a later frame
            }
        }
        String sig = Double.toString(tagSizeMeters);
        if (!sig.equals(ambiguitySolverSig)) {
            ambiguitySolver =
                    new TagAmbiguitySolver(pnp, camMatVector, camDistCoeffs, tagSizeMeters);
            ambiguitySolverSig = sig;
        }
        return ambiguitySolver;
    }

    /**
     * Lazy fallback for loading the camera calibration (intrinsics + distortion): tries once, then
     * is throttled so a not-yet-ready camera doesn't trigger an HTTP request every loop. <b>Prefer
     * {@link #prefetchCalibration} at init</b> — once {@link Limelight3A#start()} is polling, the
     * SDK's calibration GET reliably times out.
     */
    private void fetchCalibration() {
        if (camMatVector != null) {
            return;
        }
        long now = System.nanoTime();
        if (now - lastCalAttemptNanos < 2_000_000_000L) {
            return;
        }
        lastCalAttemptNanos = now;
        if (!tryLoadCalibration()) {
            log("ambiguity cal", "unavailable — gate inert, retrying");
        }
    }

    /**
     * Load the camera calibration up-front, retrying until the camera is ready or {@code timeoutMs}
     * elapses. <b>Call this once after constructing the source but before {@link
     * Limelight3A#start()}</b>, while the camera's HTTP server is idle. No-op (returns true) once a
     * matrix is cached.
     *
     * @return true if a usable calibration is loaded by the deadline.
     */
    @Override
    public boolean prefetchCalibration(long timeoutMs) {
        if (camMatVector != null) {
            return true;
        }
        long deadline = System.nanoTime() + Math.max(0L, timeoutMs) * 1_000_000L;
        int attempts = 0;
        while (true) {
            attempts++;
            if (tryLoadCalibration()) { // a successful GET also proves the camera is up
                return true;
            }
            if (System.nanoTime() >= deadline) {
                break;
            }
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }
        LLStatus status = limelight.getStatus();
        String reachability =
                (status != null && (status.getHwType() != 0 || !status.getName().isEmpty()))
                        ? String.format(
                                "camera reachable (name='%s' hw=%d) but calibration endpoints empty"
                                        + " — recalibrate the Limelight",
                                status.getName(), status.getHwType())
                        : "camera not reachable — not yet booted, or wrong address/wiring";
        log(
                "ambiguity cal",
                String.format(
                        "PREFETCH FAILED after %d attempts / %dms — AMBIGUITY GATE INERT (%s)",
                        attempts, timeoutMs, reachability));
        return false;
    }

    /**
     * One untimed attempt to fetch + cache a usable calibration. Returns true and logs the source
     * on success; returns false (leaving {@link #camMatVector} null) when every source is
     * missing/unreachable.
     */
    private boolean tryLoadCalibration() {
        LLResultTypes.CalibrationResult cal = firstUsableCal();
        if (cal == null) {
            return false;
        }
        camMatVector = cal.getCamMatVector();
        camDistCoeffs = cal.getDistortionCoefficients();
        log(
                "ambiguity cal",
                String.format(
                        "%s reproj=%.3f res=%.0fx%.0f",
                        cal.getDisplayName(),
                        cal.getReprojectionError(),
                        cal.getResX(),
                        cal.getResY()));
        return true;
    }

    /**
     * The first usable camera calibration, preferring a user ChArUco recalibration (latest → file →
     * eeprom) and falling back to the factory default. Returns null only when every source is
     * missing/garbled.
     */
    private LLResultTypes.CalibrationResult firstUsableCal() {
        LLResultTypes.CalibrationResult cal = usableOrNull(limelight.getCalLatest());
        if (cal == null) cal = usableOrNull(limelight.getCalFile());
        if (cal == null) cal = usableOrNull(limelight.getCalEEPROM());
        if (cal == null) cal = usableOrNull(limelight.getCalDefault());
        return cal;
    }

    private static LLResultTypes.CalibrationResult usableOrNull(
            LLResultTypes.CalibrationResult cal) {
        return (cal != null
                        && cal.isValid()
                        && TagAmbiguityMath.isUsableCameraMatrix(cal.getCamMatVector()))
                ? cal
                : null;
    }

    private void log(String key, String value) {
        if (telemetry != null) {
            telemetry.addData(key, value);
        }
    }
}
