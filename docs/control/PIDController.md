# PIDController Module – API Reference

## Overview

The PIDController module implements a three-axis attitude control system using independent PID controllers for pitch, yaw, and roll axes. This module provides coordinated control of spacecraft or aircraft orientation by managing three separate control loops simultaneously.

**Design Rationale:**
Designed for decoupled three-axis attitude control assuming small angle approximations and weak cross-coupling. Each axis uses an independent PID controller to enable individual tuning for different inertial characteristics. Optimized for real-time embedded systems with consistent interface patterns and comprehensive numerical validation.

## Class: PIDController

### Responsibilities

- Coordinate three independent PID controllers for attitude control
- Compute three-axis torque commands from attitude errors
- Provide unified interface for gain setting and controller management
- Maintain synchronization of derivative filtering across all axes
- Support simultaneous reset and validation of all control loops
- Enable individual axis tuning while maintaining system coherence

### Public API

#### Constructors & Rule of Three/Five

- ```cpp
  PIDController()
  ```

  ***Default constructor***. Constructs a PIDController with zero gains on all three axes.

- ```cpp
  explicit PIDController(const Vector3f& kp, const Vector3f& ki, const Vector3f& kd)
  ```

  ***Parameterized constructor***. Constructs a PIDController with specified gain vectors for each axis.

- ```cpp
  PIDController(const PIDController& pidController)
  ```

  ***Copy constructor*** preserving all three PID controller states.

- ```cpp
  PIDController& operator=(const PIDController& pidController)
  ```

  ***Assignment operator*** with self-assignment protection.

- ```cpp
  ~PIDController()
  ```

  ***Default destructor*** (no dynamic memory allocation).

&nbsp;

#### Core Methods

- ```cpp
  Vector3f compute(const Vector3f& setpoint, const Vector3f& measure, float dt)
  ```

  **Axis Mapping**: X → Pitch, Y → Yaw, Z → Roll
  **Control Law**: Independent PID computation per axis
  **Parameters**: Attitude setpoint (rad), current measurement (rad), time step (s)
  **Returns**: Three-axis torque vector (N⋅m)

- ```cpp
  void setGains(const Vector3f& kp, const Vector3f& ki, const Vector3f& kd)
  ```

  Sets gain values for all three axes simultaneously using vector components.

- ```cpp
  void setSmoothing(const float alpha)
  ```

  Sets derivative filtering coefficient uniformly across all three axes.

- ```cpp
  bool checkNumerics(void) const
  ```

  **Parameters**: None
  **Returns**: true if all three PID controllers are numerically valid

- ```cpp
  void reset(void)
  ```

  Resets integral and derivative states of all three PID controllers.

&nbsp;

### Internal State

**Private Members:**

- `PID pidPitch` – PID controller for pitch axis (X-component)
- `PID pidYaw` – PID controller for yaw axis (Y-component)
- `PID pidRoll` – PID controller for roll axis (Z-component)

&nbsp;

### Axis Configuration

**Coordinate System Convention:**

- **X-axis (Pitch)**: Rotation about body X-axis (nose up/down)
- **Y-axis (Yaw)**: Rotation about body Y-axis (nose left/right)
- **Z-axis (Roll)**: Rotation about body Z-axis (bank left/right)

**Vector Component Mapping:**

```cpp
Vector3f gains(kp_pitch, kp_yaw, kp_roll);
Vector3f attitude(pitch, yaw, roll);       // [rad]
Vector3f torques(τ_pitch, τ_yaw, τ_roll);  // [N⋅m]
```

&nbsp;

### Control Architecture

**Independent Axis Control:**

$$ \tau_x = PID_{pitch}(r_x - \theta_x, \Delta t) $$

$$ \tau_y = PID_{yaw}(r_y - \theta_y, \Delta t) $$

$$ \tau_z = PID_{roll}(r_z - \theta_z, \Delta t) $$

**Vector form:**

$$
\vec{\tau} =
[\tau_x \\ \tau_y \\ \tau_z]^T = [PID_x \\ \\ PID_y \\ \\ PID_z]^T (\vec{r} - \vec{\theta})
$$

&nbsp;

### Usage Example

```cpp
#include "includes/control/PIDController.hpp"
#include "physics/Vector3f.hpp"

// Configure three-axis attitude controller
Vector3f kp(2.0f, 1.5f, 2.5f);   // Pitch, Yaw, Roll proportional gains
Vector3f ki(0.1f, 0.05f, 0.15f); // Integral gains
Vector3f kd(0.2f, 0.1f, 0.25f);  // Derivative gains

PIDController attitudeController(kp, ki, kd);
attitudeController.setSmoothing(0.8f);  // Light derivative filtering

float dt = 0.01f;  // 10ms control loop

// Example current attitude and setpoint (from sensors or test vectors)
Vector3f currentAttitude(0.01f, -0.02f, 0.03f); // radians
Vector3f setpoint(0.0f, 0.0f, 0.0f);            // desired level orientation

// Control loop
for (int step = 0; step < 1000; ++step) {
    // Compute control torques
    Vector3f torqueCommand = attitudeController.compute(setpoint, currentAttitude, dt);

    // Apply torques to the system (placeholder: integrate externally in RigidBodySimulator or ActuatorDriver)
    // e.g., rbs.applyTorque(torqueCommand);

    // Optional: monitor numeric validity
    assert(attitudeController.checkNumerics());

    // Update currentAttitude for next iteration (e.g., read from simulator)
    // currentAttitude = rbs.getOrientation();
}
```

### Expected Output (conceptual)

```cpp
// Sample conceptual output per iteration
Attitude Error: pitch=0.01 rad, yaw=-0.02 rad, roll=0.03 rad
Control Output: τ_pitch=0.22 N⋅m, τ_yaw=-0.03 N⋅m, τ_roll=0.08 N⋅m

Attitude Error: pitch=0.009 rad, yaw=-0.018 rad, roll=0.027 rad
Control Output: τ_pitch=0.20 N⋅m, τ_yaw=-0.027 N⋅m, τ_roll=0.072 N⋅m

// ...
// Each step produces updated torque commands according to PIDController compute results
```

***Note***: Conceptual output per iteration can be monitored via torqueCommand and currentAttitude updates.

### Advanced Usage – Multi-Axis Tuning Guidelines

- **Decoupled tuning**: Start with individual axis tuning in single-axis tests
- **Cross-coupling consideration**: Monitor for axis interaction in coupled maneuvers
- **Gain scaling**: Scale gains based on axis-specific inertia and control authority
- **Symmetric tuning**: Roll and pitch often use similar gains for symmetric vehicles
- **Yaw axis special**: Typically requires lower gains due to larger inertia and reduced control authority

### Physical Interpretation

- **Pitch control**: Actuated via elevator or canard surfaces, ensuring longitudinal stability.
- **Yaw control**: Achieved through rudder deflection or differential thrust, providing directional stability.
- **Roll control**: Managed by ailerons, spoilers, or reaction wheels, enabling lateral stability.
- **Control coupling**: Effectiveness relies on small-angle conditions where cross-axis coupling remains negligible.
- **Torque distribution**: Each axis receives torque proportional to its corresponding attitude error, assuming a direct mapping between error and corrective action.

### Physical Basis & Control Assumptions

#### Control Paradigm

Implements **decoupled three-axis attitude control**, treating roll, pitch, and yaw as independent SISO systems. Relies on **small-angle approximations** and assumes weak inertial cross-coupling.

#### Underlying Physics

- **Euler angle dynamics**: 3-2-1 (yaw-pitch-roll) sequence.
- **Decoupled dynamics**: Each axis approximated as a second-order rotational system.
- **Body-frame application**: Torques are computed and applied in the principal axes of the body frame.
- **Linear superposition**: Total rotational response approximated as sum of independent axis responses.
- **Future extension**: Migration to quaternion-based representation planned for large-angle maneuvers.

#### Stability and Validity Assumptions

- **Small deviations**: Control validity ensured only for near-trim conditions.
- **Symmetric inertia**: Negligible products of inertia.
- **Negligible gyroscopic effects**: Higher-order cross-axis coupling ignored.
- **Constant mass properties**: Inertia tensor fixed during operation.
- **Adequate actuator authority**: Control torques assumed achievable within actuator limits.

### Compliance & Safety

- **Numerical robustness**: Each PID sub-controller validates against NaN/Inf states.
- **Deterministic execution**: Bounded, O(1) operations guarantee reproducible outputs across cycles.
- **Memory safety**: No dynamic allocation; all components stack-allocated.
- **Thread-safety**: Not inherently provided. External synchronization required for concurrent use.
- **Consistent reset**: All axes reset simultaneously to prevent asymmetric transients.
- **Standards alignment**: Conforms to C++98 and MISRA-C++:2008 constraints.

### Performance Characteristics

- **Computational complexity**: O(1) per control cycle (3× single PID complexity)
- **Memory footprint**: 3× PID controller memory (approximately 84 bytes)
- **Execution time**: Deterministic and bounded for real-time control loops
- **Control bandwidth**: Limited by individual PID controller performance

### Limitations & Future Extensions

- **Current limitation**: Assumes decoupled dynamics with small angle approximations
- **Current limitation**: No adaptive gain scheduling for varying flight conditions
- **Current limitation**: Fixed axis assignment may not suit all vehicle configurations
- **Planned extension**: Coupled control law for strong cross-coupling scenarios
- **Planned extension**: Gain scheduling based on flight envelope parameters
- **Planned extension**: Quaternion-based attitude representation for large angle maneuvers

**Note**: `compute()` does not enforce actuator saturation limits; higher-level modules (`ActuatorDriver`) must handle physical constraints.
