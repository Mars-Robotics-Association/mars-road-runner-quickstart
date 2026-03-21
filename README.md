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
| `RuckigNative` | Android NDK module that wraps the [Ruckig](https://ruckig.com/) C++ trajectory planner via JNI |

See the [MarsCommonFtc setup guide](MarsCommonFtc/docs/SETUP.md) for detailed instructions on adding it to other robot projects.

### External git submodules (`external/`)

| Submodule | What it is |
|-----------|-----------|
| `external/road-runner` | Road Runner core library (source) |
| `external/road-runner-ftc` | Road Runner FTC adapter (source) |
| `external/ftc-dashboard` | FTC Dashboard (source) |
| `external/road-runner-wrapper` | Composite build wrapper so all three above are built from source instead of pulled from Maven |

### `ControlLib` contents (`org.marsroboticsassociation.controllib`)

**Motion profiles**
- `SCurvePosition` / `SCurveVelocity` — jerk-limited S-curve motion profiles
- `PositionTrajectoryManager` / `VelocityTrajectoryManager` — stateful trajectory runners
- `ruckig/RuckigController` — Java wrapper around the Ruckig JNI layer for time-optimal trajectories

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
renders plots via XChart. Requires CMake on PATH to also build the Ruckig JNI
library for desktop use.

### `RuckigNative` NDK module

Wraps the Ruckig C++ library as an Android shared library (`libruckig_jni.so`).
Exposes a JNI interface consumed by `RuckigController` in `ControlLib`.
The `ControlLab` desktop Gradle task also builds a desktop version of the same
JNI wrapper via CMake so the same `RuckigController` class works off-robot.

## Cloning

This repo uses nested submodules. Clone with:

```bash
git clone --recurse-submodules <repo-url>
# or, after a plain clone:
git submodule update --init --recursive
```

## Build notes

- Road Runner, FTC Dashboard, and the Road Runner FTC adapter are all built
  from source via the composite build in `external/road-runner-wrapper/`.
  There is no need to modify `TeamCode/build.gradle` to point at Maven
  artifacts — dependency substitution is handled automatically in
  `settings.gradle`.
- `RuckigNative` is built by the Android NDK as part of the normal Android
  build. The `ControlLab` desktop run task builds a separate desktop JNI
  library via CMake (requires CMake on PATH; the task is silently skipped if
  CMake is absent).
- `WpiMath` and `ControlLib` are plain Java modules; they run on the desktop
  JVM and on Android equally. `ControlLib` uses a shadow JAR to relocate EJML
  and avoid classpath conflicts on the robot.
- Shared modules are included via `projectDir` redirects in `settings.gradle`
  pointing into `MarsCommonFtc/`. Project names (`:ControlLib`, `:WpiMath`, etc.)
  are unchanged, so all existing dependency declarations work as-is.
