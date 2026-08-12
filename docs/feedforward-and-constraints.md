# Feedforward and Constraint Extensions

The `mars` branch builds Road Runner core/actions from source (`libs/road-runner/`
submodule), which adds several feedforward and path-constraint features on top of
the stock quickstart. All of them are **opt-in and default to no-ops**, so a robot
that has only completed the standard tuning behaves exactly as before.

This document covers, for both `MecanumDrive` and `TankDrive`:

1. [Yaw-coupling feedforward](#1-yaw-coupling-feedforward) — cancels straight-line curl
2. [Anisotropic mecanum feedforward](#2-anisotropic-mecanum-feedforward) — axial vs. strafe constants
3. [Wheel voltage constraint](#3-wheel-voltage-constraint) — back-EMF/traction-aware limits
4. [Centripetal acceleration constraint](#4-centripetal-acceleration-constraint) — cornering limit

Tune these as part of the [automated tuning flow](tuning.md) — after localization and the
drive feedforward (`kS`/`kV`/`kA` via `AxialFeedforwardTuner`), and before the feedback
gains (`PathFeedbackGainTuner` runs with the production feedforward path, so these
constants are in the loop). They refine an
already-working robot. Background on why the plant is identified (not “tuned”) and how
Road Runner uses it for feedforward, planning, and gains:
[Plant Model and Identification](tuning-theory.md).

---

## 1. Yaw-coupling feedforward

**Problem it solves.** A robot commanded to drive perfectly straight open-loop usually
curls to one side. Motor-constant tolerance, unequal wheel wear, and friction that
varies with the weight distribution add up to a small yaw disturbance proportional to
translation speed. The heading controller fights it with feedback, but a feedforward
term removes it before it becomes error.

**Model.** The correction is a yaw-mode voltage computed from the *chassis* velocity
and distributed to the wheels as a pure yaw moment (it cancels in translation):

```
V_yaw = kSAxial·sign(v_axial) + kVAxial·v_axial
      + kSLateral·sign(v_lateral) + kVLateral·v_lateral
```

- `MecanumDrive` uses all four constants (axial from forward motion, lateral from strafing).
- `TankDrive` uses only the axial pair — it has no lateral motion.

Constants are in **volts** (`kS*`) and **volts per inch/s** (`kV*`), matching the drive
feedforward once `kV`/`kA` are divided by `inPerTick`.

### Parameters

| MecanumDrive.Params | TankDrive.Params | Meaning |
|---------------------|------------------|---------|
| `yawCouplingKsAxial`   | `yawCouplingKsAxial` | constant yaw bias per direction of forward motion |
| `yawCouplingKvAxial`   | `yawCouplingKvAxial` | yaw bias growing with forward speed |
| `yawCouplingKsLateral` | —                    | constant yaw bias per direction of strafing |
| `yawCouplingKvLateral` | —                    | yaw bias growing with strafe speed |

All default to `0` (disabled).

### Tuning procedure — automatic (`YawCouplingTuner`)

Run the **`YawCouplingTuner`** OpMode (registered by `TuningOpModes`, group `quickstart`).
Unlike the stock ramp loggers, it does the whole job on the robot: it drives an open-loop
ramp, measures the parasitic yaw rate against chassis velocity, fits the constants, writes
them into live `PARAMS`, and prints values to paste into source for persistence.

1. Finish localization and drive-feedforward (`kS`/`kV`/`kA`) tuning first.
2. Clear a straight lane ~5–6 ft long (both a forward lane and a sideways lane on mecanum).
3. Run `YawCouplingTuner`. On mecanum it ramps forward, stops, then ramps sideways; on tank
   it ramps forward only. When it finishes it writes `yawCouplingKsAxial` / `KvAxial`
   (and `KsLateral` / `KvLateral` on mecanum) into live `PARAMS` and shows the same values
   on Driver Station and Dashboard telemetry.
4. Re-run a straight-line check (no paste needed in-session). The curl should be largely
   gone before heading feedback engages. Paste into source before restart or redeploy.

> The sign is derived from the yaw plant gain `−kV · trackWidth`, but conventions vary — if
> a re-test shows the curl got *worse*, negate that pair. `MAX_POWER`, `RAMP_TIME`, and
> `MIN_SPEED` are adjustable on Dashboard; a longer/faster ramp with a clear lane gives a
> cleaner fit.

### What it's doing under the hood

Each term is fit from the chassis-level response, so only a body-frame localizer (dead
wheels or drive encoders + IMU) is needed — no per-wheel identification. It regresses the
yaw rate `ω = a + b·v` over the ramp and converts to the nulling yaw-mode voltage via the
small-signal yaw plant gain `kV · trackWidth`:

- `yawCouplingKsAxial = −kV · trackWidth · a`
- `yawCouplingKvAxial = −kV · trackWidth · b`

(and likewise for the lateral pair from the strafe ramp). Each term vanishes with its
source velocity, so a pure strafe incurs no axial correction and vice versa.

---

## 2. Anisotropic mecanum feedforward

*(MecanumDrive only.)*

**Problem it solves.** A mecanum drive's rollers scrub sideways when strafing, so the
effective `kS`/`kV`/`kA` for lateral motion are higher than for forward/rotational
motion. A single feedforward calibrated on forward motion under-drives strafing (and
vice versa).

**Model.** Each wheel's velocity is split into the component from axial+angular motion
and the component from lateral motion. The `axial` constants drive the former, the
`lateral` constants the latter. With equal constants it reduces exactly to the stock
single-feedforward behavior.

### Parameters

| Param | Meaning |
|-------|---------|
| `useAnisotropicFeedforward` | `false` (default) = stock single feedforward; `true` = use the lateral constants below |
| `lateralKS`, `lateralKV`, `lateralKA` | strafe-calibrated feedforward, in tick units like `kS`/`kV`/`kA` |

### Tuning procedure — automatic (`LateralFeedforwardTuner`)

Run the **`LateralFeedforwardTuner`** OpMode (mecanum only; registered by `TuningOpModes`).
It is the strafe counterpart of `AxialFeedforwardTuner` (see the section below): same ramp +
reverse procedure applied as a pure strafe for `lateralKS`/`lateralKV`/`lateralKA`. Wheel
velocity is the localizer's lateral chassis velocity times the drive's `lateralMultiplier`.

1. Run `AxialFeedforwardTuner` to get `kS`/`kV`/`kA` (the **axial** constants). A full fit
   writes them into live `PARAMS` for the rest of the RC session.
2. Run `LateralFeedforwardTuner`. It **ramps LEFT** (robot +y) first; a wall on the robot's
   left is OK. Leave room on the right for the reverse phase.
3. On success it writes `lateralKS` / `lateralKV` / `lateralKA` and sets
   `useAnisotropicFeedforward = true` on live `PARAMS`. Paste into source before
   restart/redeploy.

Expect **lateral kS** to be clearly larger than axial (often ~1.5–2×) — roller scrub is
mostly extra static/coulomb friction. **lateral kV / kA** are often only modestly higher
(or close to axial): once sliding, viscous loss and effective inertia need not double.
If lateral kS ≈ axial kS, re-check the strafe fit; if lateral kV is *much* larger than
axial with a poor ramp R², suspect localization scale or a stuck/slow ramp. Rotation
uses the axial constants (each wheel rolls in its drive direction during a turn).

## Bonus: automatic axial kS / kV / **kA** (`AxialFeedforwardTuner`)

Not a new road-runner feature, but the same "make it automatic" idea applied to the main
feedforward — specifically **kA**, which the stock `ManualFeedforwardTuner` leaves to
eyeballing a target-vs-actual velocity graph.

**Why kA can't be read off a ramp.** On a slow ramp, acceleration is nearly proportional to
velocity, so the `kA·a` term is collinear with `kV·v` and no regression can separate them. kA
becomes identifiable only when the maneuver contains **reversals** that decorrelate `a` from
`v`. `AxialFeedforwardTuner` therefore runs **two phases**:

1. **Slow forward ramp** (same idea as `ForwardRampLogger`) → `kS`, `kV` via
   `V = kS + kV·v`, with a robust low-velocity cutoff. High-speed-only multi-level cruise
   pins `kV` well but leaves the intercept poorly leveraged — a few percent slope error can
   move `kS` by ~0.3 V (e.g. ramp `kS ≈ 1.07` vs cruise-only `kS ≈ 0.7`).
2. **Reverse square wave** → residual `kA` via
   `∫(V − kS·sign(v) − kV·v) dt = kA·Δv` (no noisy `dv/dt`).

Defaults: ramp `0.1` power/s to `0.9`, then `KA_POWER = 0.6` for 4 reverse half-cycles.
The ramp ends early if velocity collapses under power (e.g. hits a wall). Phase 2 starts
reverse and, when the localizer measured enough ramp travel (start → wall), each half-cycle
runs across that corridor (minus `END_MARGIN_IN`) instead of a fixed 1 s — so the kA phase
uses the free space the robot just drove. Short ramps fall back to timed `HALF_CYCLE` holds.
Localization must be sign-correct (forward → +x). Dashboard knobs: `STALL_*`, `MOVING_SPEED`,
`MIN_RAMP_TICKS_PER_SEC`, `END_MARGIN_IN`, `MIN_TRAVEL_IN`.

Velocity comes from the localizer (drive-motor encoders are often unwired). Sanity-check the
reported kS/kV against your ramp values and use the reported kA in place of the hand-matched
one. `LateralFeedforwardTuner` is the same maneuver/fit applied to strafe, yielding
`lateralKS`/`lateralKV`/`lateralKA`. The shared core lives in `ReversalFeedforwardId`.

---

## 3. Wheel voltage constraint

**Problem it solves.** The stock profile uses a fixed max wheel velocity and fixed
acceleration caps. In reality the acceleration a motor can produce falls off as it speeds
up (back-EMF steals voltage), so a single accel cap is either too conservative at low
speed or unachievable at high speed. `WheelVoltageConstraint` derives both the velocity
and acceleration limits from the motors' voltage budget: it keeps every wheel's
feedforward voltage `kS + kV·u + kA·u̇` within `±maxVoltage`.

A single instance is used as **both** the velocity and acceleration constraint. Its
velocity limit subsumes the fixed wheel-velocity cap. It also folds in the
[anisotropic feedforward](#2-anisotropic-mecanum-feedforward) and
[yaw-coupling](#1-yaw-coupling-feedforward) constants, so the planner budgets the same
voltage the follower will actually apply.

### Parameters

| Param | Meaning |
|-------|---------|
| `useWheelVoltageConstraint` | `false` (default) = fixed `maxWheelVel` / profile-accel caps; `true` = voltage-budget limits |
| `maxVoltageForPlanning` | voltage budget for planning; use `~11` for a 12 V battery to leave headroom for sag and feedback |
| `cruiseFraction` | fraction of the budget spent cruising (default `0.95`); the remainder is reserved so some acceleration is always available |

### Requirements

- Calibrated, **positive** `kV` and `kA` (the constraint throws otherwise).
- On mecanum with `useAnisotropicFeedforward`, the lateral constants are used for the
  strafe component; otherwise the axial constants are used isotropically.
- Any yaw-coupling `kV*` must be smaller in magnitude than the corresponding drive `kV`.

Because it requires tuned constants, this constraint is opt-in. When disabled the drive
keeps using `maxWheelVel` / `minProfileAccel` / `maxProfileAccel` exactly as before.

---

## 4. Centripetal acceleration constraint

**Problem it solves.** Nothing in the stock profile stops the robot from taking a tight
curve fast enough to slide sideways. `CentripetalAccelVelConstraint` enforces
`v²·κ ≤ maxCentripetalAccel` (with `κ` the path curvature), a proxy for the lateral
traction limit.

### Parameters

| Param | Meaning |
|-------|---------|
| `maxCentripetalAccel` | cornering acceleration cap, in in/s². `0` (default) = disabled |

When set `> 0`, it is added to the default velocity constraint (composed via
`MinVelConstraint`) alongside the wheel-velocity/voltage and angular-velocity limits.
Straight segments (`κ ≈ 0`) are unaffected. Start with a value a bit below the lateral
acceleration at which your wheels visibly slip, then tighten if the robot pushes wide
through turns.

---

## Where the wiring lives

- **Params:** the `Params` inner class of `TeamCode/.../MecanumDrive.java` and
  `TankDrive.java`.
- **Constraints:** `makeWheelVoltageConstraint()` / `makeDefaultVelConstraint()` build
  `defaultVelConstraint` and `defaultAccelConstraint` from the params.
- **Feedforward + yaw coupling:** applied in `FollowTrajectoryAction.run()`. Pure
  `TurnAction`s are intentionally left untouched — a stationary rotation has no
  translational yaw coupling, and the anisotropic model reduces to the axial constants.

If you tune these values on FTC Dashboard, note that the constraint objects are built once
(at drive construction) from `PARAMS`, so changes to the constraint-related params
(`useWheelVoltageConstraint`, `maxVoltageForPlanning`, `cruiseFraction`,
`maxCentripetalAccel`) take effect on the next OpMode init. The feedforward and
yaw-coupling params are read every loop and update live.
