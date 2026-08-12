package org.firstinspires.ftc.teamcode.vision

import org.marsroboticsassociation.controllib.localization.vision.PlanarPnpSolver
import org.opencv.calib3d.Calib3d
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.MatOfDouble
import org.opencv.core.MatOfPoint2f
import org.opencv.core.MatOfPoint3f
import org.opencv.core.Point
import org.opencv.core.Point3

/**
 * OpenCV-backed implementation of the library's [PlanarPnpSolver] seam — the single native call the
 * multi-hypothesis localizer needs. Wraps OpenCV's `solvePnPGeneric(..., SOLVEPNP_IPPE_SQUARE,
 * ...)`, marshalling the pure `double[]` arrays the seam passes to/from OpenCV [Mat]s. All of the
 * interpretation (best/second selection, ambiguity ratio) lives in the pure `TagAmbiguitySolver` in
 * ControlLib; this class is only the OpenCV boundary.
 *
 * Requires the OpenCV native library, which the FTC SDK loads on the robot. Allocates a handful of
 * temporary `Mat`s per call and releases them all.
 */
class OpenCvPlanarPnpSolver : PlanarPnpSolver {

    override fun solveIppeSquare(
        objectPoints: DoubleArray,
        imagePoints: DoubleArray,
        cameraMatrix: DoubleArray,
        distCoeffs: DoubleArray?,
    ): List<PlanarPnpSolver.PnpSolution> {
        val camMat = Mat(3, 3, CvType.CV_64F)
        camMat.put(0, 0, *cameraMatrix) // 9 row-major elements fill the 3x3
        val dist =
            if (distCoeffs != null && distCoeffs.isNotEmpty()) {
                MatOfDouble(*distCoeffs)
            } else {
                MatOfDouble(0.0, 0.0, 0.0, 0.0, 0.0)
            }

        val op =
            Array(4) { i ->
                Point3(
                    objectPoints[3 * i],
                    objectPoints[3 * i + 1],
                    objectPoints[3 * i + 2],
                )
            }
        val objPts = MatOfPoint3f(*op)

        val ip =
            Array(4) { i ->
                Point(imagePoints[2 * i], imagePoints[2 * i + 1])
            }
        val imgPts = MatOfPoint2f(*ip)

        val rvecs = ArrayList<Mat>()
        val tvecs = ArrayList<Mat>()
        val reproj = Mat()
        val rvec = Mat()
        val tvec = Mat()
        val out = ArrayList<PlanarPnpSolver.PnpSolution>()
        try {
            val n =
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
                    reproj,
                )
            if (n < 1) {
                return out
            }
            val total = reproj.total().toInt()
            for (i in rvecs.indices) {
                var err = Double.NaN
                if (i < total) {
                    val v = reproj.get(i, 0)
                    err = if (v != null && v.isNotEmpty()) v[0] else Double.NaN
                }
                out.add(PlanarPnpSolver.PnpSolution(vec3(rvecs[i]), vec3(tvecs[i]), err))
            }
            return out
        } finally {
            camMat.release()
            dist.release()
            objPts.release()
            imgPts.release()
            reproj.release()
            rvec.release()
            tvec.release()
            for (m in rvecs) {
                m.release()
            }
            for (m in tvecs) {
                m.release()
            }
        }
    }

    companion object {
        /** Read a 3x1 CV_64F vector Mat into a length-3 array. */
        private fun vec3(m: Mat): DoubleArray {
            val o = DoubleArray(3)
            m.get(0, 0, o)
            return o
        }
    }
}
