# Encoder Velocity Measurement on the REV Hub

## How the hub computes velocity (best current understanding)

According to [Game Manual 0](https://gm0.org/en/latest/docs/software/adv-control-system/sdk-motors.html),
the REV hub firmware maintains a ring buffer of encoder positions, with a new sample added every
**10 ms**. `getVelocity()` is derived from this buffer, giving an effective measurement window
of ~50 ms. Both position and velocity update at ~100 Hz as the window slides forward.

**Model (assumed but unverified):** The hardware encoder counter runs continuously.
The firmware snapshots this counter every 10 ms into a ring buffer. `getCurrentPosition()`
returns the most recent snapshot; `getVelocity()` returns
`(newest_position - oldest_position) / span`. With 6 entries (5 intervals × 10 ms = 50 ms
span), this yields 20 TPS quantization (1 tick / 50 ms), matching observed telemetry. GM0 says
"5-value ring buffer" — the 6th value is likely the live counter acting as the newest entry.
REV does not publish firmware source, so this is an educated guess calibrated to match real
behavior.

The sim model (`EncoderSim`) implements this as a 6-entry ring buffer with no added noise.
All velocity "noise" comes from integer tick rounding propagating through the ring buffer —
matching the real hardware, where the encoder counters are digital edge counters with no analog
noise source.

## What this means for control loops

- **Both position and velocity update at ~100 Hz** (every 10 ms firmware sample). At a 13 ms
  control loop, most iterations get a fresh value, but some will read the same value twice
  depending on timing alignment.
- **Bulk reads** provide position and velocity from the same hub response. They do not change the
  underlying firmware sample rate — they reduce I/O overhead by batching reads, not by sampling
  faster.
- Running the control loop faster than 100 Hz yields diminishing returns for encoder-based
  feedback, since the underlying data doesn't change faster than the firmware cycle.

## Why finite-differencing position doesn't help much

A natural idea is to compute velocity yourself as `(pos_now - pos_prev) / dt` to get fresher
readings. In practice this adds more noise than it removes:

- The timestamps recorded by the control loop (via `System.nanoTime()`) reflect when the position
  was *read*, not when the firmware *sampled* it. Jitter in the control loop rate causes
  misalignment with the hub's locked 10 ms sample cycle, introducing noise that has nothing to do
  with the actual motor.
- At a 13 ms loop, a single tick of encoder error spans `1 / 0.013 = ~77 TPS` of noise, compared
  to `1 / 0.050 = 20 TPS` from the hub's 50 ms window.
- There is no way to recover the actual firmware sample timestamp from the SDK, so the
  misalignment cannot be corrected.

## Noise characteristics (from telemetry analysis)

Telemetry data from PayloadTest runs confirms:

| Metric | Value |
|---|---|
| Quantization step | 20 TPS (all readings are multiples of 20) |
| High-frequency noise (measured - LPF) | ~11 TPS std dev |
| Effective measurement window | ~50 ms (consistent with 6-entry / 5-interval ring buffer) |
| Mean loop time (observed) | ~13 ms |

The unit test sim (`EncoderSim` inside `FlywheelMotorSim`) produces pure quantization noise
with no added Gaussian noise. Sim-sweep analysis showed that with 20 TPS quantization steps,
the velocity LPF needs a cutoff ≤ 6.5 Hz to keep filtered velocity within 10 TPS of truth, and
the acceleration LPF needs ≤ 4 Hz to keep below 50 TPS². These findings were applied to
production configs (`VelocityMotorPF.measurementLpfCutoffHz = 6.5`,
`FlywheelStateSpace.accelLpfCutoffHz = 4.0`).

## Ports 0 and 3 vs ports 1 and 2

Ports 0 and 3 are connected to dedicated hardware quadrature decoders and are reliable at high
speeds. Ports 1 and 2 use software decoding and can lose steps with high-CPR encoders (>4000
counts/rev). Use ports 0 and 3 for any encoder where accuracy matters (flywheels, odometry).
