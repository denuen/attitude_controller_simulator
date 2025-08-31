# PID Module API Reference

## Overview

The `PID` module implements a single-axis Proportional-Integral-Derivative (PID) controller with advanced features including anti-windup protection and derivative filtering. It is designed for closed-loop control systems such as attitude control for rigid bodies. The implementation provides deterministic, real-time safe operation with comprehensive numerical validation. The controller maintains internal state for integral accumulation and derivative calculation, and supports configurable derivative smoothing.

## Class: PID

### Responsibilities

- Computes a 1D control signal given setpoint, measurement, and time step using the classical PID algorithm
- Implements anti-windup protection with hardcoded saturation limits
- Provides derivative filtering using a first-order low-pass filter to reduce noise sensitivity
- Validates all floating-point operations for NaN/infinity conditions
- Maintains internal state for integral accumulation, previous error, and filtered derivative

### Public API

#### Constructors & Rule of Three

- ```cpp
  PID()
  ```

  Default constructor. Initializes all gains to zero, integral and previous error to zero, filtered derivative to 0.0f, derivative filtering coefficient to 1.0 (no filtering), and anti-windup time constant (`antiWindupTau`) to `0.1f`.

- ```cpp
  PID(float kp, float ki, float kd)
  ```

  Parameterized constructor with gain validation. Asserts that all gain values are finite (not NaN or infinite). Integral and internal state are initialized to zero and `derivativeAlpha` defaults to `1.0f`.

- ```cpp
  PID(const PID& pid)
  ```

  Copy constructor. Copies gains and internal state variables (including integral, previous error, anti-windup tau and derivative alpha).
  Note: the runtime filtered derivative state is intentionally not copied; `filteredDerivative` is initialized to `0.0f` in the copy constructor to avoid transferring transient filter state between instances.

- ```cpp
  PID& operator=(const PID& pid)
  ```

  Assignment operator with self-assignment protection. Copies gains and internal persistent state (integral, previous error, anti-windup tau, derivative alpha). The filtered derivative is is not copied, instead, it is set to 0.0f,

- ```cpp
  ~PID()
  ```

  Destructor (no dynamic memory allocation).

&nbsp;

#### Operators

- ```cpp
  bool operator==(const PID& pid) const
  ```

  Equality comparison with epsilon tolerance (`1e-6f`) for floating-point values. Compares gains (`kp`, `ki`, `kd`), integral accumulator, previous error, and derivative smoothing coefficient (`derivativeAlpha`).

- ```cpp
  inline bool operator!=(const PID& pid) const
  ```

  Inequality operator implemented as negation of equality operator.

&nbsp;

#### Accessors (Getters)

- ```cpp
  inline float getKp() const
  inline float getKi() const
  inline float getKd() const
  ```

  Return the current proportional, integral, and derivative gain coefficients respectively.

- ```cpp
  inline float getIntegral() const
  ```

  Returns the current integral accumulator value.

- ```cpp
  inline float getPrevErr() const
  ```

  Returns the previous error value used for derivative term calculation.

- ```cpp
  inline float getDerivativeSmoothing() const
  ```

  Returns the alpha coefficient for derivative filtering (`derivativeAlpha`) in the range [0.0, 1.0].

- ```cpp
  inline float getAntiWindupTau() const
  ```

  Returns the anti-windup time constant (`antiWindupTau`) used by the implicit backward-Euler anti-windup integrator.

&nbsp;

#### Mutators (Setters)

- ```cpp
  void setKp(float kp)
  void setKi(float ki)
  void setKd(float kd)
  ```

  Set individual gain coefficients. Each method validates that the input value is finite via assertions.

- ```cpp
  void setGains(float kp, float ki, float kd)
  ```

  Sets all three gain coefficients simultaneously with validation.

- ```cpp
  void setDerivativeSmoothing(float alpha)
  ```

  Sets the alpha factor for derivative filtering. The implementation asserts the value is finite; `checkNumerics()` additionally enforces `0.0f <= alpha <= 1.0f`.

- ```cpp
  void setAntiWindupTau(float tau)
  ```

  Sets the anti-windup time constant used by the implicit integrator. The implementation asserts `tau` is finite; `checkNumerics()` enforces `tau > 0.0f`.

&nbsp;

#### Core Methods

- ```cpp
  float compute(const float setpoint, const float measure, const float dt)
  ```

  Implements a discrete-time PID controller with an implicit backward-Euler anti-windup integrator and optional exponential smoothing on the derivative term.

  **Continuous form (for reference):**

  u(k) = Kp·e(k) + Ki·\int e(t) dt + Kd·(d e/dt)_{filtered}

  **Implementation Details (code behavior):**
  - Asserts `dt > 0.0f` and that computed error is finite.
  - Error: `error = setpoint - measure`.
  - Derivative (backward finite difference): `rawDerivative = (error - previousError) / dt`.
  - Derivative filtering (exponential moving average / EMA):
    `filteredDerivative = derivativeAlpha * filteredDerivative + (1 - derivativeAlpha) * rawDerivative`.
    - `derivativeAlpha = 1.0f` means no filtering (output follows previous filteredDerivative only if alpha==1; note the implementation uses this convention so alpha values closer to 0 provide more weight to the new raw derivative).
  - Proportional and derivative terms are computed independently from the integral update.
  - Integral update uses an implicit/backward-Euler style anti-windup integrator when `ki > 0`:
    - It computes a candidate integral assuming no saturation, computes the unsaturated output candidate, clamps the output to the hard limits and, if saturation occurs, solves for the integral using the implicit formula with an anti-windup time constant `tau`.
    - The implementation also enforces an `effectiveTau = max(antiWindupTau, dt * 10.0f)` to avoid very small tau values causing numerical issues.
  - Integral accumulator is clamped to `DEFAULT_INTEGRAL_MIN` / `DEFAULT_INTEGRAL_MAX` after the implicit update.
  - Final output is clamped to `DEFAULT_OUTPUT_MIN` / `DEFAULT_OUTPUT_MAX`.
  - The implementation asserts the final output is finite before returning it and updates `previousError` and `integral` state.

  **Hardcoded numerical limits:**
  - Integral clamping: `DEFAULT_INTEGRAL_MIN = -100.0f`, `DEFAULT_INTEGRAL_MAX = +100.0f`.
  - Output clamping: `DEFAULT_OUTPUT_MIN = -1000.0f`, `DEFAULT_OUTPUT_MAX = +1000.0f`.

  **Parameters:**
  - `setpoint`: Desired reference value
  - `measure`: Current measured value
  - `dt`: Time step (must be positive)

  **Returns:** Control output with applied saturation limits

- ```cpp
  void reset()
  ```

  Resets integral accumulator, previous error, and filtered derivative to zero. Does not affect gains, derivative smoothing or anti-windup tau.

- ```cpp
  bool checkNumerics() const
  ```

  Validates numerical integrity of internal state and configuration. Returns `false` if any value is NaN or infinite, or if configuration values are out-of-range.

  **Checks performed:**
  - All gains and state variables are finite
  - `antiWindupTau > 0.0f`
  - `0.0f <= derivativeAlpha <= 1.0f`

&nbsp;

### Internal State

**Private Members:**

- PID gain coefficients: `float kp, ki, kd`
- Integral accumulator with anti-windup: `float integral`
- Previous error for derivative calculation: `float previousError`
- Low-pass filtered derivative term: `float filteredDerivative`
- Filter coefficient (0.0 to 1.0): `float derivativeAlpha`

**Hardcoded Constants:**

```cpp
static const float DEFAULT_INTEGRAL_MIN = -100.0f;
static const float DEFAULT_INTEGRAL_MAX = +100.0f;
static const float DEFAULT_OUTPUT_MIN = -1000.0f;
static const float DEFAULT_OUTPUT_MAX = +1000.0f;
```

&nbsp;

### Usage Example

**Note:** This module can be used independently for single-axis control, as shown in the example below. For multi-axis (e.g., pitch, yaw, roll) closed-loop control, use the `PIDController` module, which internally manages three `PID` instances in a SISO fashion (not MIMO).

```cpp
// Configure controller for pitch control
PID pitchController(2.0f, 0.05f, 0.8f);

// Apply moderate derivative filtering to reduce noise sensitivity
pitchController.setDerivativeSmoothing(0.2f);

float dt = 0.01f;   // Control loop timestep (10 ms)
float setpoint = 0.1f;  // Desired pitch angle (rad)
float measurement = 0.0f;  // Initial measured pitch
float controlOutput = 0.0f;

// Simulated closed-loop control loop (100 iterations = 1 second)
for (int i = 0; i < 100; ++i) {
    // Simulate measurement update (e.g., from sensor fusion)
    measurement += 0.001f;

    // Compute PID control signal
    controlOutput = pitchController.compute(setpoint, measurement, dt);

    // Optional: validate numerical integrity in debug builds
    assert(pitchController.checkNumerics());

    // Example anomaly recovery: reset state if saturation persists
    if (controlOutput >= 1000.0f || controlOutput <= -1000.0f) {
        pitchController.reset();
    }
}
```

### Advanced Usage - Tuning Guidelines

**Proportional Gain (Kp):**

- Improve system responsiveness and reduces rise time
- Excessive values may induce overshoot and sustained oscillations

**Integral Gain (Ki):**

- Eliminates steady-state error by accumulating error over time
- Subject to anti-windup constraints with clamping at ±100.0f to prevent integral windup
- Excessive integral gain can cause slow response and instability due to windup

**Derivative Gain (Kd):**

- Provides predictive damping to reduce overshoot and improve stability
- Amplifies high-frequency noise if unfiltered
- Recommended to apply derivative smoothing with `setDerivativeSmoothing()` using values typically between 0.1 and 0.3 for noisy measurements

### Compliance & Safety

- All floating-point operations are validated using `assert()` to detect NaN and infinite values, ensuring numerical integrity
- Implementation avoids dynamic memory allocation, enabling deterministic and real-time safe execution
- Designed for consistent execution time suitable for embedded and real-time control systems
- Compatible with C++11 standards, maintaining backward compatibility where applicable
- Thread-safe for single-instance usage; no shared mutable state is present across instances

---
