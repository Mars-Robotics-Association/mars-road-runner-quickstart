# Encoder Velocity Measurement on the REV Hub

## How the hub computes velocity (best current understanding)

It's tempting to assume `getCurrentPosition()` and `getVelocity()` each hand you an exact,
instantaneous number. Neither is perfect, but they're imperfect in very different ways — and
knowing how drives every velocity-control decision below.

`getCurrentPosition()` is a **live counter**: it returns the encoder's hardware edge count at the
moment the hub reads it, with no firmware averaging and no fixed update cadence. Its imperfections
are (1) quantization to whole encoder ticks, and (2) timing — the count crosses the hub link and
your control loop before you see it, so the instant you *read* it isn't exactly the instant it was
*sampled*. That gap is small on the Control Hub (USB-direct) but larger and jumpier on an Expansion
Hub, where the read crosses the RS485 hub-to-hub link, and I2C sensor traffic in a real loop adds
further jitter. Even so, the *value* carries none of velocity's averaging lag.

`getVelocity()` is **not** instantaneous. Per [Game Manual 0](https://gm0.org/en/latest/docs/software/adv-control-system/sdk-motors.html),
the firmware keeps a ring buffer of encoder positions, adding one every **10 ms**, and reports
`(newest − oldest) / span` across the buffer — an average over a **~50 ms window**, refreshed at
~100 Hz and held constant between refreshes. That has two consequences:

- **Lag.** A 50 ms boxcar average lags the true signal by about half its width, so `getVelocity()`
  trails the live position by **~25 ms**, plus a small sample-and-hold offset up to the 10 ms
  refresh that a PLL (locked to the velocity-update edges) could pin down — about **28 ms** total.
- **Quantization.** A 50 ms window can only resolve one extra tick at a time, so velocity arrives
  in steps of **20 TPS** (1 tick / 50 ms), matching observed telemetry.

(REV does not publish firmware source, so the buffer mechanics are an educated guess calibrated to
match observed behavior; the live-position / windowed-velocity split itself is confirmed by
burst-reading both back-to-back with `System.nanoTime()` stamps.)

The sim model (`EncoderSim`, in `MarsCommonFtc/ControlLib`) implements this as a 6-entry ring buffer with no added noise.
All velocity "noise" comes from integer tick rounding propagating through the ring buffer —
matching the real hardware, where the encoder counters are digital edge counters with no analog
noise source.

## What this means for control loops

- **`getVelocity()` updates at ~100 Hz** and is held between refreshes, so at a 13 ms control loop
  some iterations read the same velocity twice. Position is live and fresh on every read, so a
  faster loop gets genuinely newer position.
- **Bulk reads** provide position and velocity from the same hub response. They batch I/O to cut
  overhead; they do not change the firmware's velocity refresh rate.
- Running the loop faster than ~100 Hz won't make `getVelocity()` any fresher (it is capped at the
  firmware refresh), but it does give fresher position — useful if you difference position yourself
  for lower-lag velocity (next section).

## Computing velocity yourself from position

Because position is live, differencing it — `(pos_now - pos_prev) / dt` — gives velocity with
**lower lag** than `getVelocity()`'s ~25 ms. The price is noise: differentiating amplifies two
imperfections the firmware's long window otherwise hides.

- **Quantization over a short window.** Differentiating a signal quantized to 1 tick over a window
  `W` produces velocity noise `σ_v = √2·σ_q / W`. The firmware's 50 ms window gives ~8–20 TPS; a
  single 13 ms loop step gives `1 / 0.013 ≈ 77 TPS`. The shorter the window, the noisier — this is
  the fundamental noise-vs-lag tradeoff.
- **Timestamp jitter.** `System.nanoTime()` records when your code *read* the value, not when the
  hub *hardware-sampled* it. That jitter `δt` becomes a velocity error of `v·δt/dt` that grows with
  speed. It is **far worse on an Expansion Hub**, whose encoder data crosses the RS485 link to the
  Control Hub (added latency and jitter), than on the **Control Hub** (USB-direct). Put any encoder
  you intend to difference for velocity on the **Control Hub**.
- **Doing better than `getVelocity()` on both noise and lag** takes a model-based estimator (Kalman
  / alpha-beta) driven by the **commanded motor input**, not position differencing alone — a plain
  linear filter on position only slides along the noise-vs-lag curve.

### Wiring implication

On FTC robots you rarely difference *drive*-motor encoders for velocity (odometry uses position, and
drive velocity is forgiving), so the better wiring is to put the **drive motors on the Expansion
Hub** and reserve the **Control Hub** for mechanisms whose encoder timing matters — flywheels, arms,
lifts: anything you velocity-control or differentiate position on.

## Noise characteristics (from telemetry analysis)

Telemetry data from motor characterization runs confirms:

| Metric | Value |
|---|---|
| Quantization step | 20 TPS (all readings are multiples of 20) |
| High-frequency noise (measured - LPF) | ~11 TPS std dev |
| Effective measurement window | ~50 ms (consistent with 6-entry / 5-interval ring buffer) |
| Mean loop time (observed) | ~13 ms |

The unit test sim (`EncoderSim` inside `FlywheelMotorSim`, both in `MarsCommonFtc/ControlLib`)
produces pure quantization noise with no added Gaussian noise. Sim-sweep analysis showed that
with 20 TPS quantization steps, the velocity LPF needs a cutoff ≤ 6.5 Hz to keep filtered
velocity within 10 TPS of truth, and the acceleration LPF needs ≤ 4 Hz to keep below 50 TPS².
These findings were applied to production configs in `MarsCommonFtc/ControlLib`
(`VelocityMotorPF.measurementLpfCutoffHz = 6.5`, `FlywheelStateSpace.accelLpfCutoffHz = 4.0`).

## Ports 0 and 3 vs ports 1 and 2

Ports 0 and 3 are connected to dedicated hardware quadrature decoders and are reliable at high
speeds. Ports 1 and 2 use software decoding and can lose steps with high-CPR encoders (>4000
counts/rev). Use ports 0 and 3 for any encoder where accuracy matters (flywheels, odometry).
