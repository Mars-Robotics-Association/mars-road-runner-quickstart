# Road Runner Quickstart — MARS Branch

This repo starts from the standard [Road Runner Quickstart](https://rr.brott.dev/docs/v1-0/tuning/) for FTC SDK 11 and adds
MARS-specific libraries on the `mars` branch. The `master` branch is kept in
sync with upstream.

## What `mars` adds on top of `master`

### MarsCommonFtc submodule

Shared modules live in the [MarsCommonFtc](https://github.com/Mars-Robotics-Association/MarsCommonFtc) repository, consumed as a git submodule at `MarsCommonFtc/`. This allows multiple MARS FTC teams to share the same code with version pinning via tags.

| Module | Purpose |
|--------|---------|
| `ControlLib` | MARS control library — motion profiles, filters, localization, simulation, and hardware helpers |
| `ControlLab` | Desktop Java app for offline tuning and signal visualization (uses XChart + ControlLib) |
| `WpiMath` | Port of WPILib's math library (geometry, kinematics, estimators, trajectories, controllers, filters, state-space) |

See the [MarsCommonFtc setup guide](MarsCommonFtc/docs/SETUP.md) for detailed instructions on adding it to other robot projects.

### Road Runner submodule (`libs/road-runner/`)

The Road Runner core and actions libraries are built from source via Git submodule. The FTC-specific utilities continue to be pulled from Maven for compatibility.

Building core from source adds several opt-in feedforward and path-constraint features
that are wired into `MecanumDrive`/`TankDrive` — yaw-coupling feedforward (straight-line
curl compensation), anisotropic mecanum feedforward, a back-EMF/traction-aware wheel
voltage constraint, and a centripetal acceleration limit. All default to no-ops; see the
[feedforward and constraints tuning guide](doc/feedforward-and-constraints.md).

### `ControlLib` contents (`org.marsroboticsassociation.controllib`)

**Motion profiles**
- `SCurvePosition` / `SCurveVelocity` — jerk-limited S-curve motion profiles
- `PositionTrajectoryManager` / `VelocityTrajectoryManager` — stateful trajectory runners

**Filters**
- `BiquadLowPassVarDt` — biquad low-pass filter with variable dt
- `IIR1LowPassVarDt` — first-order IIR low-pass with variable dt
- `KalmanFilter` / `KalmanFilterOperations` — scalar Kalman filter

**Localization**
- `FieldPoseEstimator` — fuses odometry with sensor updates for field-relative pose
- `localization/pinpoint/` — odometry driver for the goBILDA Pinpoint odometry computer

**Motor controllers**
- `FlywheelSimple` / `FlywheelStateSpace` — flywheel velocity controllers
- `MotorBase` / `VelocityMotorBase` — base classes for motor control
- `VelocityMotorPF` / `VelocityMotorSdkPidf` — velocity motor implementations

**Hardware helpers**
- `FtcMotors` — motor utility wrappers
- `GoBildaRgbLight` — driver for the goBILDA RGB indicator light

**Simulation**
- `EncoderSim` — software encoder model for unit testing
- `FlywheelMotorSim` — flywheel motor physics simulation

**Utilities**
- `InterpLUT` / `LUT` / `LinInterpTable` — lookup tables with interpolation
- `RunningIntervalStats` — online mean/variance statistics
- `SetOnChange` — value wrapper that fires a callback only on change
- `ProjectileMotion` — projectile launch angle / exit velocity calculator

### `ControlLab` desktop app

A standalone Java application (`./gradlew :ControlLab:run`) for offline
analysis. Reads CSV signal files, applies filters from `ControlLib`, and
renders plots via XChart.

## Cloning

This repo uses nested submodules. Clone with:

```bash
git clone --recurse-submodules <repo-url>
# or, after a plain clone:
git submodule update --init --recursive
```

## Build notes

- Road Runner core and actions libraries are built from source via composite
  build with the `libs/road-runner/` submodule. Dependency substitution in
  `settings.gradle` automatically redirects Maven artifacts to local projects.
- `WpiMath` and `ControlLib` are plain Java modules; they run on the desktop
  JVM and on Android equally. `ControlLib` uses a shadow JAR to relocate EJML
  and avoid classpath conflicts on the robot.
- Shared modules are included via `projectDir` redirects in `settings.gradle`
  pointing into `MarsCommonFtc/`. Project names (`:ControlLib`, `:WpiMath`, etc.)
  are unchanged, so all existing dependency declarations work as-is.
