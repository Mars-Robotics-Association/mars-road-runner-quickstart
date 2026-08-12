package org.firstinspires.ftc.teamcode.robot

import java.util.function.Function
import kotlin.math.pow

enum class MotorType(
    private val gearRatio: Double,
    private val pulsesPerRevolution: Double,
    private val R: Double,
    private val Kt: Double,
    private val Ke: Double,
    private val Imax: Double,
    private val vcurve: Function<Double, Double>?,
    private val timeToMaxV: Double,
) {
    GOBILDA_6000(
        1.0, 28.0, 0.5, 0.0157, 0.0212, 9.2,
        Function { t ->
            0.5426 * t.pow(6) -
                11.87 * t.pow(5) +
                98.464 * t.pow(4) -
                362.34 * t.pow(3) +
                391.96 * t.pow(2) +
                984.49 * t
        },
        5.96,
    ),
    GOBILDA_1150(1.0 + (46.0 / 11.0), 28.0, 0.5, 0.0157, 0.0212, 9.2, null, 6.0),
    GOBILDA_223((1.0 + (46.0 / 11.0)) * (1.0 + (46.0 / 11.0)), 28.0, 0.5, 0.0157, 0.0212, 9.2, null, 6.0);

    /**
     * @param gearRatio           motor turns to output shaft turns
     * @param pulsesPerRevolution encoder counts per motor turn
     * @param R                   resistance (ohms)
     * @param Kt                  stall torque vs stall current N*m / A
     * @param Ke                  back EMF V_generated / ω
     * @param Imax                stall current in amps
     */
    fun getGearRatio(): Double = gearRatio

    fun getPulsesPerRevolution(): Double = pulsesPerRevolution

    /**
     * @return resistance ohms
     */
    fun getResistance(): Double = R

    /**
     * @return stall torque vs. stall current N*m / A
     */
    fun getKt(): Double = Kt

    /**
     * @return back EMF V_generated / ω
     */
    fun getKe(): Double = Ke

    /**
     * @return stall current in amps
     */
    fun getImax(): Double = Imax
}
