package org.firstinspires.ftc.teamcode.vision;

import org.marsroboticsassociation.controllib.localization.vision.PlanarPnpSolver;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;

import java.util.ArrayList;
import java.util.List;

/**
 * OpenCV-backed implementation of the library's {@link PlanarPnpSolver} seam — the single native
 * call the multi-hypothesis localizer needs. Wraps OpenCV's {@code solvePnPGeneric(...,
 * SOLVEPNP_IPPE_SQUARE, ...)}, marshalling the pure {@code double[]} arrays the seam passes to/from
 * OpenCV {@link Mat}s. All of the interpretation (best/second selection, ambiguity ratio) lives in
 * the pure {@code TagAmbiguitySolver} in ControlLib; this class is only the OpenCV boundary.
 *
 * <p>Requires the OpenCV native library, which the FTC SDK loads on the robot. Allocates a handful
 * of temporary {@code Mat}s per call and releases them all.
 */
public final class OpenCvPlanarPnpSolver implements PlanarPnpSolver {

    @Override
    public List<PnpSolution> solveIppeSquare(
            double[] objectPoints,
            double[] imagePoints,
            double[] cameraMatrix,
            double[] distCoeffs) {
        Mat camMat = new Mat(3, 3, CvType.CV_64F);
        camMat.put(0, 0, cameraMatrix); // 9 row-major elements fill the 3x3
        MatOfDouble dist =
                (distCoeffs != null && distCoeffs.length > 0)
                        ? new MatOfDouble(distCoeffs)
                        : new MatOfDouble(0, 0, 0, 0, 0);

        Point3[] op = new Point3[4];
        for (int i = 0; i < 4; i++) {
            op[i] =
                    new Point3(
                            objectPoints[3 * i], objectPoints[3 * i + 1], objectPoints[3 * i + 2]);
        }
        MatOfPoint3f objPts = new MatOfPoint3f(op);

        Point[] ip = new Point[4];
        for (int i = 0; i < 4; i++) {
            ip[i] = new Point(imagePoints[2 * i], imagePoints[2 * i + 1]);
        }
        MatOfPoint2f imgPts = new MatOfPoint2f(ip);

        List<Mat> rvecs = new ArrayList<>();
        List<Mat> tvecs = new ArrayList<>();
        Mat reproj = new Mat();
        Mat rvec = new Mat();
        Mat tvec = new Mat();
        List<PnpSolution> out = new ArrayList<>();
        try {
            int n =
                    Calib3d.solvePnPGeneric(
                            objPts,
                            imgPts,
                            camMat,
                            dist,
                            rvecs,
                            tvecs,
                            false,
                            Calib3d.SOLVEPNP_IPPE_SQUARE,
                            rvec,
                            tvec,
                            reproj);
            if (n < 1) {
                return out;
            }
            int total = (int) reproj.total();
            for (int i = 0; i < rvecs.size(); i++) {
                double err = Double.NaN;
                if (i < total) {
                    double[] v = reproj.get(i, 0);
                    err = (v != null && v.length > 0) ? v[0] : Double.NaN;
                }
                out.add(new PnpSolution(vec3(rvecs.get(i)), vec3(tvecs.get(i)), err));
            }
            return out;
        } finally {
            camMat.release();
            dist.release();
            objPts.release();
            imgPts.release();
            reproj.release();
            rvec.release();
            tvec.release();
            for (Mat m : rvecs) {
                m.release();
            }
            for (Mat m : tvecs) {
                m.release();
            }
        }
    }

    /** Read a 3x1 CV_64F vector Mat into a length-3 array. */
    private static double[] vec3(Mat m) {
        double[] o = new double[3];
        m.get(0, 0, o);
        return o;
    }
}
