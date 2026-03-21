---
name: ftc-robot-code
description: Apply FTC SDK conventions, gotchas, and Road Runner pitfalls when writing or reviewing robot Java code. Use when writing any FTC OpMode, subsystem, or Road Runner code.
---

## Language target

See `sourceCompatibility`/`targetCompatibility` in `build.common.gradle` for the Java version.
Modern language features (`var`, records, switch expressions, text blocks, sealed classes) are
available as long as the version is 17+.

## Coding rules

**No hardware commands after `opModeIsActive()` returns false.**
In a `LinearOpMode`, never send any hardware commands (motor power, servo position, sensor reads,
etc.) after `opModeIsActive()` returns `false`. The SDK takes sole responsibility for shutting down
hardware at that point; sending commands after this will crash the Control Hub.

**No `sleep()` in control or sampling loops.**
Do not use `sleep()` to intentionally slow down a control loop or sensor polling loop — prefer
higher sample rates. `sleep()` is fine when an OpMode is just waiting for stop.
Exception: sysid / feedforward tuning opmodes (e.g. `FlywheelsFeedforwardTuning`,
`ArmFeedforwardTuning`) may use short sleeps to pace sample collection or wait for mechanical
settling between trials — that is deliberate test sequencing, not loop-rate throttling.

**Guard against near-zero dt in control loops.**
Whether dt is passed in or computed internally (e.g. `now - prevTime`), return early when
`dt < 1e-6`. A near-zero dt likely means a duplicate call in the same frame and can cause
divide-by-zero in derivative estimates or double-update observers and filters.

## @Config / FTC Dashboard parameters

`@Config` exposes public static parameters to FTC Dashboard for real-time tuning. Put parameters
in a public static inner class, then add a public static field holding an instance of it.

## FTC Dashboard telemetry

**Tuning OpModes (graphs only):** wrap with `MultipleTelemetry` so output goes to both the driver
station and Dashboard. The project uses `DashboardTelemetryPacketAccess` as the access point — do
not call `FtcDashboard.getInstance().getTelemetry()` directly:

```java
DashboardTelemetryPacketAccess packetAccess = new DashboardTelemetryPacketAccess();
telemetry = new MultipleTelemetry(telemetry, packetAccess.dashboardTelemetry);
```

See `FlywheelsTuningBase.initHardware()` for the established pattern.

**Field overlay drawing:** use `DashboardTelemetryPacketAccess` to get the underlying
`TelemetryPacket` and draw on its `fieldOverlay()` directly. Do not create a separate
`TelemetryPacket` and send it manually — that writes extra telemetry lines and duplicates data.

## Encoder velocity

`getVelocity()` is computed in the hub firmware, not derived from `getCurrentPosition()`.
See `encoder-velocity.md` in the `dc-motor` skill for details on timing, noise, and why
finite-differencing position is not an improvement.

## Bulk caching

Use `BulkReads` (in `robot/BulkReads.java`) — not `LynxModule.BulkCachingMode.AUTO`. `BulkReads`
sets all hubs to `MANUAL` mode and exposes `readAll()`, which clears the cache so the *next*
hardware read triggers a fresh bulk fetch. Call it once at the top of every loop:

```java
var bulkReads = new BulkReads(hardwareMap);   // init
// in loop:
bulkReads.readAll();   // fresh bulk read; all subsequent reads this loop use cached data
```

`AUTO` mode (used in some older tuning opmodes) clears the cache automatically but gives no
control over when the fresh read fires. Prefer `BulkReads` for any new opmode.

## Road Runner gotchas

**`Vector2d.angleCast()` requires a unit vector.**
`Vector2d.angleCast()` constructs `Rotation2d(x, y)` directly without normalizing. Calling it on
an unnormalized vector (e.g. a raw position difference) silently scales all subsequent
`Rotation2d.times()` results by the vector's magnitude. Always normalize first:

```java
double norm = Math.hypot(diff.x, diff.y);
Rotation2d dir = new Rotation2d(diff.x / norm, diff.y / norm);
```

**`Rotation2d.inverse()` is the conjugate, not the multiplicative inverse.**
For unit rotations these are identical and it works correctly. On a non-unit rotation they differ —
another reason to always normalize before constructing a `Rotation2d` from a vector.
