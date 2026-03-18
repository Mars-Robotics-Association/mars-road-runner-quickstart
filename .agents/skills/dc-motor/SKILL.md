---
name: dc-motor
description: FTC DcMotor conventions: RunMode, PIDF coefficient scaling, voltage sampling strategy, multi-motor subsystem design, and hub identification. Use when writing or reviewing any motor control code.
---

## RunMode

- `RUN_WITHOUT_ENCODER` is the preferred mode for custom motor control. Despite the name, it still
  reads encoder values — it just doesn't use them for internal control. Prefer this over the SDK's
  built-in PIDF controller; custom feedback/feedforward is almost always better.
- `RUN_USING_ENCODER` enables the SDK's built-in PIDF velocity controller. Only use this if you
  explicitly want the SDK to handle velocity control.

## Voltage (and current) sampling

Prefer approaches higher on this list:

1. **Sample on setpoint change** _(most preferred)_ — read battery voltage once each time a new
   setpoint is commanded. Good enough for feedforward and most feedback controllers, and avoids
   per-loop I²C overhead.
2. **Sample once on construction** — read voltage at startup and hold it fixed for the life of the
   subsystem. Acceptable when voltage variation is negligible for the application.
3. **Fixed nominal voltage** _(least preferred)_ — hardcoding a constant (e.g. 12 V) ignores
   real battery sag and should be a last resort.

Voltage reads are not part of the bulk read and add I²C overhead — that's why per-loop reads
are not the default.

**Exception — state-space / LQR controllers:** per-loop voltage sampling is acceptable when the
model requires it. Per-loop **current** sampling is also allowed and is often preferred over
voltage if the state-space model uses current as a state or input. In either case, the subsystem's
`update` method must provide an **overload that accepts the measured value as a parameter** so
callers who already sampled it elsewhere in the OpMode can pass it in rather than triggering a
redundant read.

## Characterization constant units

Store motor constants in voltage-based units. Duty-cycle-based units (power fraction) are
disfavored because they bake in an assumed battery voltage and become inaccurate as the battery
drains. Preferred units per constant:

| Constant | Units |
|---|---|
| kV | V / (tick/s) |
| kA | V / (tick/s²) |
| kS | V |
| kP | V / (tick/s of error) |
| kI | V / (tick of integrated error) |
| kD | V / (tick/s²of error) |

**WPILib exception:** when using WPILib plant identification, store constants in WPILib SI units
and read `wpilib-plant.md` in this skill directory for units, J/b derivation, and
`LinearSystemLoop` voltage patterns.

## Encoder velocity measurement

The hub's `getVelocity()` is computed from a firmware-side ring buffer, not from raw encoder
reads. See `encoder-velocity.md` in this skill directory for how it works, why
finite-differencing position doesn't improve on it, and validated noise characteristics.

## SDK PIDF coefficient scaling

Only relevant if using `RUN_USING_ENCODER` (disfavored — see RunMode above). If needed, read
`sdk-pidf.md` in this skill directory.

## Multi-motor subsystems

When a subsystem drives multiple motors in tandem, represent each motor as a record pairing its
hardware-map name with its direction — e.g. `MotorConfig(String name, DcMotorSimple.Direction direction)` —
rather than keeping a name array and a direction array separately in `Params`. This keeps the two
values inseparable and eliminates the need for a guard clause verifying array lengths match.
`motors[0]` is conventionally the encoder source; all motors receive the same power command.

## Motor instantiation

Use `QuantizedPowerMotor` (the project wrapper) rather than getting a raw `DcMotorEx` from the
hardware map. It suppresses negligible power changes to reduce I²C chatter.

## Identifying which hub a motor is on

Use `HubHelper` to determine whether a motor is on the Control Hub or Expansion Hub. Do not
manually iterate `LynxModule` instances or hardcode hub assumptions.
