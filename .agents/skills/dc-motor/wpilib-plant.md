# WPILib Plant Identification

## Storing constants in SI units

When a subsystem uses WPILib plant identification, store kV and kA directly in WPILib SI units
(V·s/rad and V·s²/rad) rather than V/TPS. Document the derivation from characterization data in
the field initializer so the source is clear:

```java
public double kV = measuredVolts * ticksPerRev / (maxTps      * 2 * Math.PI);  // V·s/rad
public double kA = measuredVolts * ticksPerRev / (maxAccelTps * 2 * Math.PI);  // V·s²/rad
```

## Deriving J and b from kV, kA, and DCMotor

Given characterized kV and kA (SI units) and a `DCMotor` object, J and b can be derived
analytically — no additional measurements needed:

```java
// Total system moment of inertia (kg·m²) — includes motor rotor + load
double J = FtcMotors.calcJ(motor, kA);   // = kA * kT / R

// Viscous damping (N·m·s/rad)
double b = FtcMotors.calcB(motor, kV);   // = (kT/R) * (kV − kE), where kE = 1/KvRadPerSecPerVolt
```

`J` reflects the total inertia of the characterized mechanism — exactly what
`createDCMotorSystem` needs. `b` near zero is normal for flywheels with good bearings; negative
indicates noise or model mismatch.

Use `FtcMotors` in `MarsCommonFtc/ControlLib` (`org.marsroboticsassociation.controllib.util`)
for both `DCMotor` constants (RS-555 / goBILDA Yellow Jacket) and these helpers.

## LinearSystemLoop voltage

Pass the hub voltage read at startup as `maxVoltage` rather than a hardcoded nominal:

```java
double startupVoltage = hub.getInputVoltage(VoltageUnit.VOLTS);
loop = new LinearSystemLoop<>(..., startupVoltage, dtSeconds);
```

In the per-loop update, clamp the voltage command to `±hubVoltage` (the live reading) before
dividing to get motor power. This ensures the clamp tightens if the battery has sagged since
startup.
