package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.roadrunner.Pose2d

class Drawing private constructor() {
    companion object {
        @JvmStatic
        fun drawRobot(c: Canvas, t: Pose2d) {
            val ROBOT_RADIUS = 9.0

            c.setStrokeWidth(1)
            c.strokeCircle(t.position.x, t.position.y, ROBOT_RADIUS)

            val halfv = t.heading.vec().times(0.5 * ROBOT_RADIUS)
            val p1 = t.position.plus(halfv)
            val p2 = p1.plus(halfv)
            c.strokeLine(p1.x, p1.y, p2.x, p2.y)
        }

        @JvmStatic
        fun drawPoseHistory(c: Canvas, color: String, poseHistory: Collection<Pose2d>) {
            val xPoints = DoubleArray(poseHistory.size)
            val yPoints = DoubleArray(poseHistory.size)

            var i = 0
            for (t in poseHistory) {
                xPoints[i] = t.position.x
                yPoints[i] = t.position.y

                i++
            }

            c.setStrokeWidth(1)
            c.setStroke(color)
            c.strokePolyline(xPoints, yPoints)
        }
    }
}
