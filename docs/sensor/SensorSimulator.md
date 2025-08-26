# SensorSimulator Module – API Reference

## Overview

The SensorSimulator module implements a compact, deterministic IMU sensor model used by the simulator to provide noisy, drifting measurements of a `RigidBodySimulator` instance.

Design goals: fidelity to simple MEMS-like behaviour (zero-mean Gaussian measurement noise and low-frequency bias/drift), deterministic behaviour appropriate for repeatable tests, and light CPU/memory cost.

## Class: SensorSimulator

### Responsibilities

- Provide per-axis angular velocity measurements that combine the rigid-body's true angular velocity with an accumulated drift term and per-axis Gaussian noise.
- Provide per-axis Euler-angle measurements (orientation) computed from the `RigidBodySimulator` and perturbed by a separate accumulated drift plus Gaussian noise.
- Maintain independent drift accumulators for angular velocity (`driftOmega`) and orientation (`driftAngles`) and advance them each update step using a configured `driftRate`.
- Expose configuration (noise stddev and drift rates), numeric self-checks, and a deterministic reset path for test repeatability.

### Public API

#### Constructors / special members

- ```cpp
  SensorSimulator(RigidBodySimulator* source)`
  ```

  ***Constructor***. Construct a `SensorSimulator` bound to `source`. An assert in the constructor requires `source != NULL`.

- ```cpp
  SensorSimulator(const SensorSimulator& other)`
  ```

  ***Copy constructor***: copies drift states, configuration and the internal `GaussianNoiseGenerator` state.

- ```cpp
  SensorSimulator& operator=(const SensorSimulator& other)`
  ```

  ***Copy-assignment***: copies the same members as the copy constructor and returns `*this`.

- ```cpp
  ~SensorSimulator()`
  ```

  ***Default destructor***; class contains no dynamic allocation.

#### Configuration methods

- ```cpp
  void setNoiseStdDev(const Vector3f& stddev)`
  ```

  Set per-axis standard deviation used by the `GaussianNoiseGenerator` when producing additive zero-mean noise for both angular velocity and orientation reads.

- ```cpp
  void setDriftRate(const Vector3f& rate)`
  ```

  Set the per-axis `driftRate` applied during `update(dt)` to both `driftOmega` and `driftAngles` (see "Drift semantics" below for units and implications).

#### Core methods

- ```cpp
  void update(float dt)`
  ```

  Advance the internal drift accumulators. The implementation asserts `dt > 0.0f`, checks numeric validity of internal vectors, and then performs:

  - `driftOmega = driftOmega + driftRate * dt;`
  - `driftAngles = driftAngles + driftRate * dt;`
  - After the update the vectors are re-checked for numeric validity (asserts).

- ```cpp
  Vector3f readAngularVelocity() const`
  ```

  Returns: `realOmega + driftOmega + noise`, where `realOmega` is obtained from `source->getOmega()` and `noise` is sampled per-axis with stddev from `noiseStdDev`.

  The function asserts the `source` pointer is non-null and validates numerics of the produced vector before returning.

- ```cpp
  Vector3f readOrientation() const`
  ```

  Returns: `realAngles + driftAngles + noise`, where `realAngles` is constructed using the `RigidBodySimulator` accessors in the following order:
  - X component = `source->getPitch()`
  - Y component = `source->getYaw()`
  - Z component = `source->getRoll()`

  Important: the returned `Vector3f` packs orientation as (pitch, yaw, roll). This ordering is implementation-defined in the current code and callers must consume the vector accordingly.

- ```cpp
  bool checkNumerics() const`
  ```

  Returns true when all internal Vector3f members (`driftOmega`, `driftAngles`, `driftRate`, `noiseStdDev`) and the `GaussianNoiseGenerator` are numerically valid and when `source` is non-null and `source->checkNumerics()` returns true.

- ```cpp
  void reset()`
  ```

  Zeros the two drift accumulators and `driftRate`, and calls `noiseGenerator.reset()` to clear any generator cached state. Asserts validate the cleared vectors.

#### Internal State (members)

- `Vector3f driftOmega` — accumulated angular-velocity bias (units discussed below).
- `Vector3f driftAngles` — accumulated Euler-angle bias.
- `Vector3f driftRate` — rate applied to both `driftOmega` and `driftAngles` at each update.
- `Vector3f noiseStdDev` — per-axis Gaussian noise standard deviation used for both omega and angle reads.
- `RigidBodySimulator* source` — pointer to the rigid-body providing the true motion values. The constructor asserts this pointer is non-null; several methods also assert it before use.
- `mutable GaussianNoiseGenerator noiseGenerator` — generates zero-mean Gaussian samples; mutable so `read*()` methods remain const.

&nbsp;

### Sensor modeling and physical assumptions

This section documents the exact numerical model implemented and key assumptions that callers must be aware of.

1) Additive measurement model (per-axis, independent):

    - Angular velocity: $omega_{meas} = omega_{true} + driftOmega + N(0, sigma^2)$
    - Orientation: $theta_{meas} = theta_{true} + driftAngles + N(0, sigma^2)$

   Noise is generated per-axis by `GaussianNoiseGenerator::generate(mean, stddev)` and is assumed zero-mean and statistically independent between axes.

2) Drift semantics and units (explicit):

   - The implementation advances both `driftOmega` and `driftAngles` using the same `driftRate` vector:
       - `driftOmega += driftRate * dt;`

         (if `driftOmega` is interpreted in $rad/s$, `driftRate` must be provided in $rad/s^2$)

       - `driftAngles += driftRate * dt;`

          (if `driftAngles` is interpreted in $rad$, `driftRate` must be provided in $rad/s$)

   - Because the same `driftRate` vector is applied to two state variables with different physical dimensions in typical interpretations, the `driftRate` must be choose consistently with the intended meaning. In practice this design is a compact way to represent a configurable bias random-walk; typical usage is to set `driftRate` values small enough that angle drift and omega bias remain physically plausible over the simulated time window.

3) Orientation component ordering (implementation detail):

   - `readOrientation()` constructs the returned `Vector3f` as (pitch, yaw, roll) following the exact calls in `SensorSimulator.cpp`:
       - X = `source->getPitch()`
       - Y = `source->getYaw()`
       - Z = `source->getRoll()`

   - This ordering is non-standard compared with some codebases (which often use roll,pitch,yaw). The implementation is explicit and must be respected by consumers and tests.

4) Numerical safety and assertions:

   - The implementation uses `assert()` to ensure `source` is non-null at construction and at read calls, and to check `dt > 0.0f` for `update()`.
   - Each `Vector3f` member provides `assertVectorCheck()` / `checkNumerics()` calls (see `includes/physics/Vector3f.hpp`) which the simulator calls before and after critical state changes.

5) Random number generation and determinism:

   - The project supplies a `GaussianNoiseGenerator` (implementation note: Box–Muller transform is used in this codebase's generator) that exposes `reset()` and a copyable state. Because the simulator copies and resets the generator, simulations can be made deterministic by controlling seed/state at the generator level. See `includes/sensor/GaussianNoise.hpp` and `src/sensor/GaussianNoise.cpp` for generator details.

6) Limitations of the implemented model:

   - No cross-axis correlation of noise is modeled.
   - No temperature, vibration, scale factor, saturation, or quantisation effects are modeled.
   - The drift representation is a simple integrator; it is not a full stochastic process model (e.g., no explicit Allan-variance or power-law noise modelling).

References and code pointers:

- Implementation: `includes/sensor/SensorSimulator.hpp` and `src/sensor/SensorSimulator.cpp`.
- Noise generator: `includes/sensor/GaussianNoise.hpp` / corresponding implementation (Box–Muller is used in the project noise generator).
- Typical sensor model references: standard MEMS IMU modelling uses additive white Gaussian noise plus slow bias/drift (see sensor modelling sections in navigation/filtering textbooks). The code implements the simple additive + random-walk drift variant.

&nbsp;

## Usage example

```cpp
#include <cassert>
#include <iomanip>
#include <iostream>

#include "includes/physics/RigidBodySimulator.hpp"
#include "includes/sensor/SensorSimulator.hpp"

int main() {

    Vector3f.          inertia(10.0f, 8.0f, 12.0f);
    RigidBodySimulator spacecraft(inertia);
    SensorSimulator    imu(&spacecraft);

    // Per-axis noise stddev used for both gyro and orientation reads
    Vector3f noiseStd(0.001f, 0.001f,
                      0.001f); // rad/s for omega, rad for angles (user-chosen)
    // A small driftRate;
    Vector3f driftRate(1e-6f, 1e-6f, 1e-6f);

    // Ensure deterministic start (for testing): clear drifts and RNG cached state, then
    // configure.
    imu.reset();
    imu.setNoiseStdDev(noiseStd);
    imu.setDriftRate(driftRate);

    const float dt.         = 0.01f; // seconds
    const int   steps       = 10000;
    const int   reportEvery = 1000;

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Starting simulation (" << steps << " steps, dt=" << dt
              << "s)\n";
    std::cout << "Noise stddev (per-axis): [" << noiseStd.getX() << ", "
              << noiseStd.getY() << ", " << noiseStd.getZ() << "]\n";
    std::cout << "Drift rate (per-axis):   [" << driftRate.getX() << ", "
              << driftRate.getY() << ", " << driftRate.getZ() << "]\n\n";

    Vector3f trueOmega = spacecraft.getOmega();
    std::cout << "Initial true angular velocity (rad/s): [" << trueOmega.getX()
              << ", " << trueOmega.getY() << ", " << trueOmega.getZ() << "]\n";

    Vector3f trueAngles(spacecraft.getPitch(), spacecraft.getYaw(),
                        spacecraft.getRoll());
    std::cout << "Initial true orientation (pitch,yaw,roll) (rad): ["
              << trueAngles.getX() << ", " << trueAngles.getY() << ", "
              << trueAngles.getZ() << "]\n\n";

    // Column formatting for periodic reports
    const int WSTEP = 6;
    const int W     = 12; // numeric column width
    std::cout << std::right;
    // Header row: step, true omega (x,y,z), measured omega (x,y,z), measured
    // angles (pitch,yaw,roll)
    std::cout << std::setw(WSTEP) << "step" << "  " << std::setw(W) << "t.ω_x"
              << std::setw(W) << "t.ω_y" << std::setw(W) << "t.ω_z"
              << std::setw(W) << "m.ω_x" << std::setw(W) << "m.ω_y"
              << std::setw(W) << "m.ω_z" << std::setw(W) << "m.pitch"
              << std::setw(W) << "m.yaw" << std::setw(W) << "m.roll"
              << "\n";
    int totalWidth = WSTEP + 1 + W * 9;
    for (int i = 0; i < totalWidth; ++i)
        std::cout << '-';
    std::cout << '\n';

    for (int step = 0; step < steps; ++step) {

        // simple fixed torque input for demo
        Vector3f torque(0.1f, -0.05f, 0.02f);
        spacecraft.update(dt, torque);

        imu.update(dt);

        Vector3f measuredOmega  = imu.readAngularVelocity();
        Vector3f measuredAngles = imu.readOrientation();

        // Periodic user feedback
        if ((step % reportEvery) == 0) {
            Vector3f trueOmegaNow = spacecraft.getOmega();
            std::cout << std::setw(WSTEP) << step << ' ' << std::setw(W)
                      << trueOmegaNow.getX() << std::setw(W)
                      << trueOmegaNow.getY() << std::setw(W)
                      << trueOmegaNow.getZ() << std::setw(W)
                      << measuredOmega.getX() << std::setw(W)
                      << measuredOmega.getY() << std::setw(W)
                      << measuredOmega.getZ() << std::setw(W)
                      << measuredAngles.getX() << std::setw(W)
                      << measuredAngles.getY() << std::setw(W)
                      << measuredAngles.getZ() << "\n";
        }

        // Numeric safety check
        if (!imu.checkNumerics()) {
            std::cerr << "Numeric check failed at step " << step
                      << ". Aborting.\n";
            break;
        }

        // small example of a user-driven event: reset sensor halfway through
        if (step == (steps / 2)) {
            std::cout << "\n  Mid-simulation: resetting sensor drift and RNG state "
                         "for repeatability.\n" << std::endl;
            imu.reset();
            // Reapply configured driftRate and noise after reset (reset zeroes
            // driftRate).
            imu.setNoiseStdDev(noiseStd);
            imu.setDriftRate(driftRate);
        }
    }

    std::cout << "\nSimulation complete.\n";
    return 0;
}

```

### Expected behaviour and examples

Below is an example of the console output produced by the `main.cpp` usage example above. The table columns match the program's header and periodic report: step, true angular velocity (x,y,z), measured angular velocity (x,y,z), and measured orientation (pitch,yaw,roll). Values are illustrative — exact numbers depend on initial conditions, torque input, `noiseStdDev`, `driftRate` and RNG seed/state.

Header and divider (printed once at start)

```text
 step | t.ω_x   | t.ω_y     | t.ω_z     | m.ω_x    | m.ω_y     | m.ω_z     | m.pitch   | m.yaw     | m.roll
```

Sample periodic rows (every `reportEvery` steps):

```text
   0 | 0.000100 | -0.000063 |  0.000017 | 0.001702 | -0.000322 |  0.000191 | -0.001500 | -0.000302 |  0.000120
1000 | 0.101303 | -0.061336 |  0.013223 | 0.101932 | -0.061514 |  0.012088 | -0.311029 |  0.074805 |  0.504589
2000 | 0.205555 | -0.119333 |  0.005962 | 0.205454 | -0.118668 |  0.005424 | -1.212306 |  0.195257 |  2.039102
                                                ...
  Mid-simulation: resetting sensor drift and RNG state for repeatability.
                                                ...
8000 | 0.255117 | -0.585477 |  0.091330 | 0.254634 | -0.585758 |  0.092200 | 0.174502 | 2.583578 | -0.584282
9000 | 0.213140 | -0.685439 | -0.199484 | 0.214368 | -0.684448 | -0.197101 | 0.271821 | 2.049884 |  2.406160

  Simulation complete.
```

Notes:

- `t.ω_*` are the true angular-velocity components read from the `RigidBodySimulator` at the reporting instant.
- `m.ω_*` are the measured angular velocities returned by `SensorSimulator::readAngularVelocity()` and include additive Gaussian noise and the current `driftOmega` bias.
- `m.pitch, m.yaw, m.roll` are the components of the `Vector3f` returned by `readOrientation()` and follow the implementation ordering (pitch, yaw, roll). They include additive Gaussian noise and the `driftAngles` bias.
- Exact numbers will vary; the sample rows above are intended to show formatting and relative magnitudes only.

### Compliance & Safety

- Numeric checks: the implementation uses asserts and `Vector3f::checkNumerics()` to detect NaN/Inf or invalid values early in tests.
- Dynamic allocations: none in `SensorSimulator`; the class contains only stack-allocated objects and a pointer to the external `RigidBodySimulator`.
- Thread-safety: not thread-safe — external synchronization is required for concurrent access to `SensorSimulator` or its `source`.

---

### Limitations & future work

- The shared `driftRate` applied to both `driftOmega` and `driftAngles` is a compact and simple design choice but can be confusing with respect to physical units; a future change could separate the per-axis rates for the two accumulators to avoid unit ambiguity.
- Add support for correlated multi-axis noise, Allan-variance based drift models, saturation/clipping, and temperature-dependent drift.

---
