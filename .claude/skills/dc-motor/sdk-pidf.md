# SDK PIDF Coefficient Scaling

The SDK PIDF coefficients are scaled by 32767 before being stored as integers in the motor
controller. To convert from real units:

- **F (kV):** `F = kV * 32767`. Example: motor measured at max 2496 ticks/sec → `F = 32767 / 2496 ≈ 13`.
- **P, I, D** work the same way. Example: a kP of 0.004 PWM per tick of error → `P = 0.004 * 32767 ≈ 131`.
