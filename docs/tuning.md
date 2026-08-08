# Automated Tuning Flow

The `mars` branch replaces most of the stock Road Runner tuning procedure with on-robot
sysid OpModes that drive a maneuver, fit the constants, write them into the live `PARAMS`
statics (so later steps in the same robot-controller process can chain without pasting), and
print the same values for pasting into source so they survive restart or redeploy. This is
the recommended flow; the [stock manual procedure](https://rr.brott.dev/docs/v1-0/tuning/)
remains available and is a useful fallback when an automatic fit looks suspect.

For the physics, plant model, and *why* the steps are ordered this way (identification vs.
knob-turning), see **[Plant Model and Identification](tuning-theory.md)**.

Most drive tuners are registered under the `quickstart` OpMode group;
`Pinpoint Offset Tuner` is under `Tuning`. Steps marked **manual** can't be
automated (they measure physical geometry or require human observation).

| # | Step | OpMode | Sets | Mode |
|---|------|--------|------|------|
| 1 | Motor directions | `MecanumMotorDirectionDebugger` / `DeadWheelDirectionDebugger` | motor/encoder directions | manual |
| 2 | Linear scale | `ForwardPushTest` (+ `LateralPushTest` for mecanum dead wheels) | `inPerTick`, `lateralInPerTick` | manual push |
| 3 | Angular scale / wheel geometry | `AngularRampLogger` (dead wheels); **`Pinpoint Offset Tuner`** (Pinpoint pods); OTOS tuners | `trackWidthTicks`, dead-wheel / Pinpoint offsets | semi-auto |
| 4 | Drive feedforward | **`AxialFeedforwardTuner`** | `kS`, `kV`, `kA` | **automatic** |
| 5 | Track width correction | **`TrackWidthTuner`** | `trackWidthTicks` | **automatic** |
| 6 | Strafe feedforward (mecanum) | **`LateralFeedforwardTuner`** | `lateralKS/KV/KA`, enable `useAnisotropicFeedforward` | **automatic** |
| 7 | Straight-line curl | **`YawCouplingTuner`** | `yawCoupling*` | **automatic** |
| 8 | Feedback gains | **`FeedbackGainTuner`** | `axialGain`, `lateralGain`, `headingGain` + vel gains (tank: `turnGain`, `turnVelGain`) | **automatic** |
| 9 | Verification | `ManualFeedbackTuner`, `SplineTest` | — | manual check |

Notes on the flow:

- **Step 4 replaces `ForwardRampLogger` + `ManualFeedforwardTuner`.** A slow open-loop ramp
  fits `kS`/`kV` (same model as `ForwardRampLogger`), then a reverse square wave fits residual
  `kA`. The ramp stops early if the robot is no longer moving under power (wall contact); the
  reverse phase then uses the measured start→wall distance as its corridor (not a fixed 1 s
  per half-cycle). Stock OpModes remain for cross-check; `kS`/`kV` should match the ramp
  closely. Localization must already be sign-correct (forward → +x).
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
- The automatic tuners (steps 4–8) write successful fits into the live `PARAMS` statics
  when they finish, so you can run the next step without pasting first — statics normally
  last for the whole RC process across OpMode stop/start. They also print the values on
  Driver Station / Dashboard telemetry: paste into the `Params` inner class of
  `MecanumDrive` / `TankDrive` (or localizer params) before you restart the app or
  redeploy, or the session values are lost.
- **One-stop copy page:** after chaining the automatic steps (or any time), open the
  Driver Station **Utility** menu and run **`Show Drive Params`**. It reads only the live
  statics (no hardware), dumps geometry, feedforward, yaw coupling, gains, and localizer
  offsets in paste-friendly formats (`kV`/`kA` as scientific notation), and mirrors the
  same lines to FTC Dashboard. Use it when you are ready to commit session values into
  source.
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
  `perpXTicks`. Measure those with **`Pinpoint Offset Tuner`** (step 3; see
  [below](#pinpoint-pod-offsets-pinpoint-offset-tuner)); do not expect
  `AngularRampLogger` to produce them on a Pinpoint setup.
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
   rate (from the hub IMU) against it, writes the corrected `trackWidthTicks` into live
   `PARAMS`, and prints it — a wrong value shows up as a slope off 1.0, and the
   correction is just dividing by it. Re-run (no paste needed in-session); the slope
   should come back ≈ 1.00.
3. Residual error is absorbed automatically: `FeedbackGainTuner` tunes the heading loop
   against the plant as it actually responds, so a few percent of track-width error just
   shifts the heading gains it lands on.

---

## Pinpoint pod offsets (`Pinpoint Offset Tuner`)

**Problem it solves.** The Pinpoint needs the mounting position of each odometry pod
relative to the robot center: the parallel pod's $y$ offset (`parYTicks`) and the
perpendicular pod's $x$ offset (`perpXTicks`). Wrong offsets make the reported pose
orbit in circles on the Dashboard field view whenever the robot spins in place, even
though the chassis is not translating.

Tape-measure first guesses are fine to start, but small errors still show up as that
circular drift. `Pinpoint Offset Tuner` (OpMode group `Tuning`) turns the spin into a
measurement: it watches how far the pose drifts after each half-revolution and prints
corrected offsets in inches.

### How it works

While the robot spins about a fixed point, an offset error looks like a constant
position bias that rotates with heading. After a half-revolution ($\pi$ rad) the bias
has flipped sign, so the field displacement between start and sample is twice the
offset error (with a sign that depends on which axis is wrong). The tuner:

1. Ignores translation commands and only accepts yaw from the right stick, so the
   chassis stays put while heading accumulates.
2. Unwraps heading and samples the reported field pose each time total rotation
   crosses an odd multiple of $\pi$ (every $180^\circ$).
3. Averages those displacements over several half-revolutions to reject noise.
4. Converts average field displacement $(d_x, d_y)$ into offset corrections:

$$
e_x = \frac{d_x}{2},\qquad e_y = \frac{d_y}{2}
$$

$$
x' = x - e_x,\qquad y' = y - e_y
$$

where $x$ / $y$ are the currently configured perpendicular / parallel offsets in
inches, $e_x$ / $e_y$ are the estimated errors, and $x'$ / $y'$ are the corrected
values printed on telemetry.

### Procedure

1. Finish motor/encoder directions and linear scale first (steps 1–2). `inPerTick`
   must be set — the tuner reports inches and you convert to ticks with it.
2. Put a starting guess in `PinpointLocalizer.PARAMS` if you have one (tape measure is
   fine; zeros also work and just take more correction on the first pass).
3. Clear space to spin in place. Run **Pinpoint Offset Tuner**.
4. Spin consistently in **one** direction with the right stick for several full
   revolutions. Left stick is ignored on purpose.
5. Read the corrected **perpX** and **parY** inches from telemetry (`>>>` lines).
6. Paste into `PinpointLocalizer.Params` as ticks:

   ```
   parYTicks  = parY_inches  / inPerTick
   perpXTicks = perpX_inches / inPerTick
   ```

7. Re-run. The pose trail on Dashboard should stay near the green origin marker
   instead of tracing a circle. Iterate once more if residual drift remains.

### If it fails

- **Pose still orbits after pasting** — confirm you converted inches → ticks with the
  same `inPerTick` the drive uses, and that encoder directions are correct before
  blaming the offsets.
- **No samples appear** — keep spinning the same way until total rotation exceeds
  $180^\circ$; samples only fire on odd half-revolutions.
- **Displacements huge / unstable** — the robot is translating while turning (floor
  grip, or you nudged with the left stick). Re-center and spin cleaner; average more
  revolutions.

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

2. **Automated profiled bump testing.** For each axis it moves a smooth cosine reference
   to an offset pose (8 in translation / 40° heading by default over ~1.2 s), lets the
   holonomic controller track it through the production feedforward path, and measures
   overshoot, error-sign reversals, velocity chatter, and settling time. Starting from a
   conservative bandwidth, it raises `ωn` by ~1.4× while the response stays clean and
   stops at the last clean setting once the response starts to ring — the classic
   "turn it up until it oscillates, then back off," automated. Every test steps out and
   then back, so the robot ends each iteration where it started.

   Pure static setpoint steps are avoided on purpose: holding a fixed target through
   `kS · sign(v)` lives in the low-speed relay region and produces a high-frequency
   "chihuahua" shake that is not representative of trajectory tracking (where the
   reference velocity keeps the sign consistent until the profile ends).

   During a measurement bump only the axis under test has feedback gains (other axes at
   zero) so the score isolates that axis. Exception: lateral bumps keep a soft heading
   fixture so open-loop strafe curl does not dominate the lateral score. Between axes, a
   hold-level recenter returns the robot to field home.

Axes are tuned in order **heading → axial → lateral**. On tank drive only the turn
controller is tuned; path following uses Ramsete, whose defaults (`ramseteZeta`,
`ramseteBBar`) rarely need adjustment.

The step tests drive the wheels through the same `setDriveCommand` path the trajectory
follower uses, so anisotropic feedforward, yaw coupling, and battery-voltage compensation
are all in the loop — the gains are tuned against exactly the plant they'll control.

### Procedure

1. Finish localization and feedforward tuning first (steps 1–7 above). `kA` must be
   tuned and positive — `τ = kA/kV` is the model the synthesis relies on. The tuner
   refuses to run otherwise. Prior automatic steps write into live `PARAMS`, so you can
   continue in the same RC session without pasting first.
2. Clear about 2 ft around the robot in every direction. It rotates in place, then steps
   forward/back, then sideways.
3. Run `FeedbackGainTuner`. Watch the iteration telemetry; when it finishes it prints
   all six gains (two for tank) and writes them into `PARAMS`, so they are live for the
   rest of the session. Each run also writes two CSVs on the hub under `/sdcard/FIRST/`:
   `fbgain_samples_<stamp>.csv` (every control-loop sample) and
   `fbgain_summary_<stamp>.csv` (one row per out/back leg, plus a `meta` row with plant
   params). Pull them with `telemetry/pull.sh`.
4. Paste the values into `Params` before restart/redeploy, then verify with
   `ManualFeedbackTuner` and `SplineTest`.

### Knobs (Dashboard)

| Knob | Default | Meaning |
|------|---------|---------|
| `STEP_INCHES` / `STEP_DEGREES` | 8 / 40 | step sizes for the bump tests |
| `MOVE_SEC` | 1.2 | seconds for the smooth reference to travel the step |
| `ZETA` | 1.0 | target damping ratio (lower = snappier but bouncier) |
| `START_BANDWIDTH` | 0.35 | initial `ωn` as a fraction of the plant pole `1/τ` |
| `MAX_START_OMEGA` | 3.0 | absolute cap on starting `ωn` (rad/s) |
| `MAX_OVERSHOOT_FRAC` | 0.28 | overshoot above this: keep the gain but stop growing ω |
| `REJECT_OVERSHOOT_FRAC` | 0.45 | overshoot above this hard-fails the setting |
| `HOLD_POS_GAIN` / `HOLD_VEL_GAIN` | 2.5 / 0.3 | translation gains for between-phase recenter only |
| `HOLD_HEADING_POS` / `HOLD_HEADING_VEL` | 0.7 / 0.45 | soft heading for recenter, and as a lateral measurement fixture |
| home recenter | on | between-phase return to field home after heading and after axial |
| `FFT_MIN_HOLD_SEC` | 0.85 | hold-phase length so the oscillation FFT has enough samples |
| `FFT_FMIN_HZ` | 2.5 | ignore peaks below this (and below ~1.5 cycles/window) — settle tails are ~1 Hz |
| `FFT_PEAK_RATIO_EDGE` | 8 | error peak/median → stop growing ω **if** hold RMS ≥ `FFT_ERR_RMS_EDGE_MUL` × settle band |
| `FFT_PEAK_RATIO_REJECT` | 12 | with matching cmd peak **and** hold RMS ≥ `FFT_ERR_RMS_REJECT_MUL` × settle band → hard-reject |
| `FFT_ERR_RMS_EDGE_MUL` | 1.25 | min hold-error RMS / settle-tol for FFT soft edge (quiet holds ignore spectrum) |
| `FFT_ERR_RMS_REJECT_MUL` | 2.0 | min hold-error RMS / settle-tol for FFT hard ring |
| `MAX_HARD_REJECT_STREAK` | 2 | give up an axis after this many hard rejects with no clean best |
| `MAX_VEL_FLIPS` | 12 | axis-velocity sign flips above this reject (catches kS chatter) |
| `MAX_POS_GAIN` | 20 | ceiling on translation position gains |
| `MAX_HEADING_POS_GAIN` | 10 | ceiling on heading / tank-turn position gain |
| `SETTLE_TOL_IN` / `SETTLE_TOL_DEG` | 0.6 / 2.5 | settle band that defines "arrived" |
| `MAX_ITERS` | 6 | step-pair iterations per axis |

### If it fails

- **"robot barely moved on the first step"** — the feedforward or localization is off;
  re-check steps 2–4 and motor directions.
- **"never settled"** — the robot stalls just outside the settle band, usually an
  underestimated `kS`. Re-run `AxialFeedforwardTuner`, or widen `SETTLE_TOL_IN`.
- **"oscillatory even at the lowest bandwidth"** — usually noisy or laggy localization;
  check the localizer before blaming the gains.
- **"velocity chatter even at the lowest bandwidth"** — high-frequency motor shake with
  little position motion. Almost always an **overestimated `kS`** (or stiction much higher
  than the ramp-fit Coulomb term): at low speed `kS · sign(v)` becomes bang-bang. Re-run
  `AxialFeedforwardTuner` / `LateralFeedforwardTuner` and prefer the ramp fit; if the
  shake only appears while holding still, you can also lower `kS` slightly (~10–20%) and
  re-try. Very small `τ = kA/kV` (underestimated `kA`) also inflates the first gains —
  the pre-start telemetry prints `tau` and the first `posGain` so you can spot this.
- Lateral tracks curl into an asterisk — open-loop strafe yaw; the soft heading fixture
  should limit this. If curl is still huge, finish `YawCouplingTuner` first or raise
  `HOLD_HEADING_POS` slightly for the lateral phase only.
- **"stalled mid-step"** — robot against a wall or immovable load; clear space and re-run
  (the tuner aborts instead of thrashing for six iterations).
- A failed axis keeps whatever gains `Params` already had rather than writing a bad
  guess; its failure reason is shown in the final telemetry.

### Tank turn-gain sign fix

The stock quickstart's tank `TurnAction` applies `turnGain · (actual − target)` — a sign
convention under which *positive* gains produce positive feedback. This branch flips it
to `turnGain · (target − actual)`, matching `HolonomicController`, so the positive gains
produced by `FeedbackGainTuner` behave correctly. If you previously made tank turns work
with negative gains, negate them.
