# Automated Tuning Flow

The `mars` branch replaces most of the stock Road Runner tuning procedure with on-robot
sysid OpModes that drive a maneuver, fit the constants, and print values to paste into
`Params`. This is the recommended flow; the [stock manual procedure](https://rr.brott.dev/docs/v1-0/tuning/)
remains available and is a useful fallback when an automatic fit looks suspect.

All tuners are registered under the `quickstart` OpMode group. Steps marked **manual**
can't be automated (they measure physical geometry or require human observation).

| # | Step | OpMode | Sets | Mode |
|---|------|--------|------|------|
| 1 | Motor directions | `MecanumMotorDirectionDebugger` / `DeadWheelDirectionDebugger` | motor/encoder directions | manual |
| 2 | Linear scale | `ForwardPushTest` (+ `LateralPushTest` for mecanum dead wheels) | `inPerTick`, `lateralInPerTick` | manual push |
| 3 | Angular scale / wheel geometry | `AngularRampLogger` (or the OTOS/Pinpoint tuners) | `trackWidthTicks`, dead-wheel positions | semi-auto |
| 4 | Drive feedforward | **`AxialFeedforwardTuner`** | `kS`, `kV`, `kA` | **automatic** |
| 5 | Track width correction | **`TrackWidthTuner`** | `trackWidthTicks` | **automatic** |
| 6 | Strafe feedforward (mecanum) | **`LateralFeedforwardTuner`** | `lateralKS/KV/KA`, enable `useAnisotropicFeedforward` | **automatic** |
| 7 | Straight-line curl | **`YawCouplingTuner`** | `yawCoupling*` | **automatic** |
| 8 | Feedback gains | **`FeedbackGainTuner`** | `axialGain`, `lateralGain`, `headingGain` + vel gains (tank: `turnGain`, `turnVelGain`) | **automatic** |
| 9 | Verification | `ManualFeedbackTuner`, `SplineTest` | — | manual check |

Notes on the flow:

- **Step 4 replaces `ForwardRampLogger` + `ManualFeedforwardTuner`.** The reversal-based
  fit identifies all three constants at once — including `kA`, which the stock procedure
  leaves to eyeballing a velocity graph. The stock OpModes are still registered if you
  want to cross-check the fit.
- **Step 5** spins the robot through the tuned feedforward and corrects `trackWidthTicks`
  from the commanded-vs-actual yaw rate (measured by the hub IMU, so it works with any
  localizer). It runs *after* step 4 because the spin is driven through `kS`/`kV`. On
  drive-encoder setups it verifies the `AngularRampLogger` value; on Pinpoint/OTOS it is
  the *only* way to measure the value — see
  [below](#trackwidthticks-with-a-pinpoint-or-otos).
- **Steps 6–7** are MARS feedforward extensions; see the
  [feedforward and constraints guide](feedforward-and-constraints.md) for the models
  behind them and for the opt-in path constraints (wheel voltage, centripetal).
- **Step 8 replaces guess-and-check gain tuning.** `ManualFeedbackTuner` is still the
  right way to *verify* the result, but you should no longer need it to find the numbers.
- The automatic tuners print values on Driver Station / Dashboard telemetry when they
  finish. Paste them into the `Params` inner class of `MecanumDrive` / `TankDrive` so
  they persist.
- **Using a goBILDA Pinpoint or SparkFun OTOS?** Step 3 changes character: those devices
  own localization entirely, and `trackWidthTicks` matters much less. See
  [below](#trackwidthticks-with-a-pinpoint-or-otos).

---

## trackWidthTicks with a Pinpoint or OTOS

With drive-encoder (or dead-wheel) localization, `trackWidthTicks` is critical: it is
how heading is *estimated*, so an error corrupts the pose and everything downstream.
With a Pinpoint or OTOS it plays no part in localization at all:

- **Pinpoint** computes heading onboard from its own pods. What it needs is the pod
  resolution (`inPerTick`) and the pod offsets — `PinpointLocalizer.PARAMS.parYTicks` /
  `perpXTicks`, which `AngularRampLogger` measures.
- **OTOS** is self-contained; its scalars and mounting offset come from the four
  `OTOS*Tuner` OpModes.

What `trackWidthTicks` still does on these setups is feed the *drive model*
(`inPerTick · trackWidthTicks` = effective track width in inches): how much wheel speed
the follower commands per rad/s of turning, the planner's wheel-velocity/voltage limits
on turning, and the yaw-coupling conversion in `YawCouplingTuner`. Get it wrong and
turns are fed forward too weakly or strongly — feedback then covers the difference, at
the cost of transient tracking during heading changes.

In practice that makes it a **get-within-~10%-and-move-on** parameter:

1. Start from a tape measure: track width, center to center of the drive wheels, divided
   by `inPerTick` (OTOS reports inches, so with `inPerTick = 1` it's just inches). On
   mecanum the effective value is usually somewhat *larger* than the geometric one
   because the rollers scrub during rotation.
2. Note that `AngularRampLogger` cannot measure `trackWidthTicks` on these setups — the
   regression needs drive encoders in the tuning view, and the Pinpoint/OTOS
   configurations don't include them. Don't hunt for the missing output; run
   **`TrackWidthTuner`** (step 5) after the feedforward is tuned. It spins the robot
   through the feedforward with a ramped commanded yaw rate, regresses the actual yaw
   rate (from the hub IMU) against it, and prints the corrected `trackWidthTicks` — a
   wrong value shows up as a slope off 1.0, and the correction is just dividing by it.
   Re-run after pasting; the slope should come back ≈ 1.00.
3. Residual error is absorbed automatically: `FeedbackGainTuner` tunes the heading loop
   against the plant as it actually responds, so a few percent of track-width error just
   shifts the heading gains it lands on.

---

## Automatic feedback gains (`FeedbackGainTuner`)

**Problem it solves.** The stock procedure for `axialGain`, `lateralGain`, `headingGain`
and the `*VelGain`s is "guess a number, run `ManualFeedbackTuner`, look at the plots,
repeat." `FeedbackGainTuner` finds the gains on the robot in a couple of minutes.

### How it works

Two ideas combined:

1. **The feedforward already contains a plant model.** With voltage-compensated
   feedforward, each axis of the closed loop behaves like a second-order system governed
   by the motor time constant `τ = kA / kV`. Picking a control bandwidth `ωn` and damping
   `ζ` then *determines* the gains:

   ```
   posGain = ωn² · τ
   velGain = 2·ζ·ωn·τ − 1   (clamped at 0; the plant contributes damping of its own)
   ```

   So instead of searching a 6-dimensional gain space, the tuner searches one knob per
   axis — bandwidth — and the pos/vel gains always stay consistently matched.

2. **Automated bump testing.** For each axis it offsets the target pose by a step
   (8 in translation / 40° heading by default), lets the controller drive there, and
   measures overshoot, error-sign reversals, and settling time. Starting from a
   conservative bandwidth, it raises `ωn` by ~1.4× while the response stays clean and
   stops at the last clean setting once the response starts to ring — the classic
   "turn it up until it oscillates, then back off," automated. Every test steps out and
   then back, so the robot ends each iteration where it started.

Axes are tuned in order **heading → axial → lateral** (heading first so it can hold the
robot straight during the translation steps). On tank drive only the turn controller is
tuned; path following uses Ramsete, whose defaults (`ramseteZeta`, `ramseteBBar`) rarely
need adjustment.

The step tests drive the wheels through the same `setDriveCommand` path the trajectory
follower uses, so anisotropic feedforward, yaw coupling, and battery-voltage compensation
are all in the loop — the gains are tuned against exactly the plant they'll control.

### Procedure

1. Finish localization and feedforward tuning first (steps 1–7 above). `kA` must be
   tuned and positive — `τ = kA/kV` is the model the synthesis relies on. The tuner
   refuses to run otherwise.
2. Clear about 2 ft around the robot in every direction. It rotates in place, then steps
   forward/back, then sideways.
3. Run `FeedbackGainTuner`. Watch the iteration telemetry; when it finishes it prints
   all six gains (two for tank) and writes them into `PARAMS`, so they are live for the
   rest of the session.
4. Paste the values into `Params`, then verify with `ManualFeedbackTuner` and
   `SplineTest`.

### Knobs (Dashboard)

| Knob | Default | Meaning |
|------|---------|---------|
| `STEP_INCHES` / `STEP_DEGREES` | 8 / 40 | step sizes for the bump tests |
| `ZETA` | 1.0 | target damping ratio (lower = snappier but bouncier) |
| `MAX_OVERSHOOT_FRAC` | 0.12 | overshoot above this fraction of the step rejects a response |
| `MAX_POS_GAIN` | 20 | ceiling on the synthesized position gains |
| `SETTLE_TOL_IN` / `SETTLE_TOL_DEG` | 0.6 / 2.5 | settle band that defines "arrived" |
| `MAX_ITERS` | 6 | step-pair iterations per axis |

### If it fails

- **"robot barely moved on the first step"** — the feedforward or localization is off;
  re-check steps 2–4 and motor directions.
- **"never settled"** — the robot stalls just outside the settle band, usually an
  underestimated `kS`. Re-run `AxialFeedforwardTuner`, or widen `SETTLE_TOL_IN`.
- **"oscillatory even at the lowest bandwidth"** — usually noisy or laggy localization;
  check the localizer before blaming the gains.
- A failed axis keeps whatever gains `Params` already had rather than writing a bad
  guess; its failure reason is shown in the final telemetry.

### Tank turn-gain sign fix

The stock quickstart's tank `TurnAction` applies `turnGain · (actual − target)` — a sign
convention under which *positive* gains produce positive feedback. This branch flips it
to `turnGain · (target − actual)`, matching `HolonomicController`, so the positive gains
produced by `FeedbackGainTuner` behave correctly. If you previously made tank turns work
with negative gains, negate them.
