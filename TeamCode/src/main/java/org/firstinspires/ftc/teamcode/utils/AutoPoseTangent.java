package org.firstinspires.ftc.teamcode.utils;

public class AutoPoseTangent extends AutoPose {
    public double td;

    public AutoPoseTangent(double x, double y, double hd, double td) {
        super(x, y, hd);
        this.td = td;
    }

    public double getTangentRadians() {
        return Math.toRadians(td);
    }
}
