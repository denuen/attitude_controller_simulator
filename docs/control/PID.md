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

  Default constructor. Initializes all gains to zero, integral and previous error to zero, filtered derivative to zero, and derivative filtering coefficient to 1.0 (no filtering).

- ```cpp
  PID(float kp, float ki, float kd)
  ```

  Parameterized constructor with gain validation. Asserts that all gain values are finite (not NaN or infinite).

- ```cpp
  PID(const PID& pid)
  ```

  Copy constructor. Copies all gain values and internal state variables including integral, previous error, derivative alpha factor.
  **Note**: filteredDerivative state is not copied; instead, it is set to 0.0f.

- ```cpp
  PID& operator=(const PID& pid)
  ```

  Assignment operator with self-assignment protection. Copies all gain and internal state variables.
  **Note**: filteredDerivative state is not copied; instead, it is set to 0.0f.

- ```cpp
  ~PID()
  ```

  Destructor (no dynamic memory allocation).

&nbsp;

#### Operators

- ```cpp
  bool operator==(const PID& pid) const
  ```

  Equality comparison with epsilon tolerance (1e-6f) for floating-point values. Compares gains (`kp`, `ki`, `kd`), integral accumulator, previous error, and derivative filtering coefficient (`derivativeAlpha`).

- ```cpp
  inline bool operator!=(const PID& pid) const
  ```

  Inequality operator implemented as negation of equality operator.

&nbsp;

#### Accessors (Getters)

- ```cpp
  inline float getKp(void) const
  inline float getKi(void) const
  inline float getKd(void) const
  ```

  Return the current proportional, integral, and derivative gain coefficients respectively.

- ```cpp
  inline float getIntegral(void) const
  ```

  Returns the current integral accumulator value.

- ```cpp
  inline float getPrevErr(void) const
  ```

  Returns the previous error value used for derivative term calculation.

- ```cpp
  inline float getFilteredDerivative(void) const
  ```

  Returns the current filtered derivative value.

- ```cpp
  inline float getDerivativeSmoothing(void) const
  ```

  Returns the alpha coefficient for derivative filtering (0.0 to 1.0 range).

&nbsp;

#### Mutators (Setters)

- ```cpp
  void setKp(const float kp)
  void setKi(const float ki)
  void setKd(const float kd)
  ```

  Set individual gain coefficients. Each method validates that the input value is finite.

- ```cpp
  void setGains(const float kp, const float ki, const float kd)
  ```

  Sets all three gain coefficients simultaneously with validation.

- ```cpp
  void setDerivativeSmoothing(float alpha)
  ```

  Sets the alpha factor for derivative filtering. Must be in range [0.0, 1.0] where:
  - `alpha = 1.0`: No filtering (raw derivative)
  - `alpha = 0.0`: Maximum filtering (heavily smoothed derivative)

&nbsp;

#### Core Methods

- ```cpp
  float compute(const float setpoint, const float measure, const float dt)
  ```

  $$
  u(k) = K_p \cdot e(k) + K_i \cdot \int e(t)\,dt + K_d \cdot \frac{d e}{dt}_{\text{filtered}}
  $$

  **Note**: the above formula expresses the PID control law in continuous form (continuous time domain, CT). In the software implementation, this relationship is discretized (discrete time domain, TD) for numerical execution, using finite approximations for the integral and the derivative, as well as a low-pass filter on the derivative term to reduce sensitivity to noise.

  **Implementation Details:**
  - Integral term uses rectangular approximation: `integral += error * dt`
  - Derivative uses backward finite difference: `(error - previousError) / dt`
  - Derivative filtering: `filteredDerivative = alpha * rawDerivative + (1 - alpha) * filteredDerivative`
  - Integral clamping: `[-100.0f, +100.0f]` (hardcoded)
  - Output clamping: `[-1000.0f, +1000.0f]` (hardcoded)

  **Parameters:**
  - `setpoint`: Desired reference value
  - `measure`: Current measured value
  - `dt`: Time step (must be positive)

  **Returns:** Control output with applied saturation limits

- ```cpp
  void reset(void)
  ```

  Resets integral accumulator, previous error, and filtered derivative to zero. Does not affect gains or filtering coefficient.

- ```cpp
  bool checkNumerics() const
  ```

  Validates numerical integrity of all internal state variables. Returns false if any value is NaN or infinite.

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
