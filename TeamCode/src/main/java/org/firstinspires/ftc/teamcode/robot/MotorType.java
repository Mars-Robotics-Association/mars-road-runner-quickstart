package org.firstinspires.ftc.teamcode.robot;

import java.util.function.Function;

public enum MotorType {
    GOBILDA_6000(1.0, 28.0, 0.5, 0.0157, 0.0212, 9.2,
            t -> 0.5426 * Math.pow(t, 6)
                    - 11.87 * Math.pow(t, 5)
                    + 98.464 * Math.pow(t, 4)
                    - 362.34 * Math.pow(t, 3)
                    + 391.96 * Math.pow(t, 2)
                    + 984.49 * t, 5.96),
    GOBILDA_1150(1.0 + (46.0 / 11.0), 28.0, 0.5, 0.0157, 0.0212, 9.2, null, 6),
    GOBILDA_223((1.0 + (46.0 / 11.0)) * (1.0 + (46.0 / 11.0)), 28.0, 0.5, 0.0157, 0.0212, 9.2, null, 6);

    private final double gearRatio;
    private final double pulsesPerRevolution;
    private final double R;
    private final double Kt;
    private final double Ke;
    private final double Imax;
    private final Function<Double, Double> vcurve;
    private final double timeToMaxV;

    /**
     * @param gearRatio           motor turns to output shaft turns
     * @param pulsesPerRevolution encoder counts per motor turn
     * @param R                   resistance (ohms)
     * @param Kt                  stall torque vs stall current N*m / A
     * @param Ke                  back EMF V_generated / ω
     * @param Imax                stall current in amps
     */
    MotorType(double gearRatio, double pulsesPerRevolution, double R, double Kt, double Ke, double Imax, Function<Double, Double> vcurve, double timeToMaxV) {
        this.gearRatio = gearRatio;
        this.pulsesPerRevolution = pulsesPerRevolution;
        this.R = R;
        this.Kt = Kt;
        this.Ke = Ke;
        this.Imax = Imax;
        this.vcurve = vcurve;
        this.timeToMaxV = timeToMaxV;
    }

    public double getGearRatio() {
        return gearRatio;
    }

    public double getPulsesPerRevolution() {
        return pulsesPerRevolution;
    }

    /**
     * @return resistance ohms
     */
    public double getResistance() {
        return R;
    }

    /**
     * @return stall torque vs. stall current N*m / A
     */
    public double getKt() {
        return Kt;
    }

    /**
     * @return back EMF V_generated / ω
     */
    public double getKe() {
        return Ke;
    }

    /**
     * @return stall current in amps
     */
    public double getImax() {
        return Imax;
    }
}
