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
| 8 | Feedback gains | **`PathFeedbackGainTuner`** (mecanum) | `axialGain`, `lateralGain`, `headingGain` (vel gains stay 0 by default) | **automatic** |
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
3. Residual error is absorbed automatically: `PathFeedbackGainTuner` ladders the heading
   gain on real path tracking, so a few percent of track-width error just shifts the
   heading gain it lands on.

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

## Automatic feedback gains (`PathFeedbackGainTuner`)

**Problem it solves.** The stock procedure for `axialGain`, `lateralGain`, `headingGain`
is "guess a number, run `ManualFeedbackTuner`, look at the plots, repeat."
`PathFeedbackGainTuner` does the same out-and-back line you use by eye, ladders each
position gain, and backs off when the path starts to stop-then-lunge or thrash — without
short bump tests that happily climb to unusable gains of 10–20.

Mecanum only (holonomic path gains). Tank: use `ManualFeedbackTuner` for turn gains;
path following uses Ramsete defaults.

### How it works

For each axis it runs long out-and-back legs (`DISTANCE`, default 64 in) at a discrete
gain ladder (`START_GAIN` … `MAX_GAIN` by `GAIN_STEP`), scores command chatter in the
low-speed window and **stop-short-then-lunge** hesitation near path end, then keeps the
last fully quiet clean gain (preferring zero-hesitation when the top clean still had mild
hes).

Axes order:

1. **Axial** — field-X `lineToX` at heading 0; soft heading hold.
2. **Heading** — same path with axial locked at its best.
3. **Lateral** (optional) — return home, turn 90°, strafe along the same field-X corridor
   at constant heading.

Velocity gains stay at `VEL_GAIN` (default 0). Production feedforward via
`setDriveCommand` is in the loop (anisotropic FF, yaw coupling, voltage compensation).

### Procedure

1. Finish localization and feedforward tuning first (steps 1–7). Positive $k_V$/$k_A$
   required. Prior automatic steps write into live `PARAMS`, so you can continue in the
   same RC session without pasting first.
2. Clear a straight corridor of about `DISTANCE + 12` inches and room to spin at the
   start for the lateral reorient.
3. Run `PathFeedbackGainTuner`. It writes gains into live `PARAMS` and returns to the
   start pose. CSVs under `/sdcard/FIRST/`: `path_fbgain_samples_<stamp>.csv` and
   `path_fbgain_summary_<stamp>.csv`. Pull with `telemetry/pull.sh`.
4. Paste into `Params` before restart/redeploy; verify with `ManualFeedbackTuner` and
   `SplineTest`.

### Knobs (Dashboard)

| Knob | Default | Meaning |
|------|---------|---------|
| `DISTANCE` | 64 | out-and-back length (in) |
| `CYCLES` | 2 | out-and-backs per gain step |
| `START_GAIN` / `HEADING_START_GAIN` | 2 / 3 | first gain on the ladder (heading starts higher) |
| `MAX_GAIN` / `GAIN_STEP` | 12 / 1 | ladder ceiling and step |
| `HOLD_HEADING_GAIN` | 4 | heading while axial ladder runs |
| `TUNE_LATERAL` | true | run lateral phase after heading |
| `VEL_GAIN` | 0 | all three velocity gains |
| `MAX_*_CMD_FLIPS` | 16–18 | hard reject on low-speed command sign flips |
| `HESITATION_V_FAST` / `HESITATION_REMAINING_MIN` | 8 / 1.5 | stop-short re-lunge detection |
| `MAX_HESITATIONS_EDGE` / `MAX_HESITATIONS_REJECT` | 2 / 3 | soft edge vs hard reject |

### If it fails

- **Chatter / hesitation at the lowest gain** — feedforward or localization is off;
  re-check steps 2–7 before raising `START_GAIN`.
- **Stop-then-lunge before reverse** — gain too high for this chassis; the ladder should
  back off to the last zero-hesitation clean step. If it still feels high by eye, drop
  one more manually and paste that.
- **Heading limp or stranded low** — old runs could soft-edge at 2 with a bad axial
  companion; heading now starts at 3 with hold 4 and a clean axial first.
- **Lateral only** — set `TUNE_LATERAL=false` if you only want axial/heading.
- Tank drive: OpMode refuses; use `ManualFeedbackTuner`.

### Tank turn-gain sign fix

The stock quickstart's tank `TurnAction` applies `turnGain · (actual − target)` — a sign
convention under which *positive* gains produce positive feedback. This branch flips it
to `turnGain · (target − actual)`, matching `HolonomicController`. If you previously made
tank turns work with negative gains, negate them.
