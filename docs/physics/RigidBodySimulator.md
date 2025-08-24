# RigidBodySimulator Module API Reference

## Overview

The `RigidBodySimulator` module implements a rigid body rotational dynamics simulator based on Euler's rotational equations of motion, with full gyroscopic coupling effects. This module serves as the core physics engine for simulating three-axis rotational motion of aerospace vehicles, spacecraft, satellites, and other rigid bodies subjected to applied external torques. The implementation provides deterministic, real-time capable simulation suitable for control system design, hardware-in-the-loop testing, and flight dynamics analysis.

**Note:** If modules introduce random noise (e.g., Gaussian sensor noise), simulation results are deterministic and repeatable only if the random number generator seed is fixed. Otherwise, results may vary between runs.

## Physical Model Implementation

The simulator employs Euler's rotational equations of motion in body-fixed coordinates, which govern the angular dynamics of a rigid body:

$$
\begin{aligned}
I_x \dot{\omega}_x - (I_y - I_z)\, \omega_y \omega_z &= M_x \\
I_y \dot{\omega}_y - (I_z - I_x)\, \omega_z \omega_x &= M_y \\
I_z \dot{\omega}_z - (I_x - I_y)\, \omega_x \omega_y &= M_z
\end{aligned}
$$

Where:

- $I_x, I_y, I_z$ represent the principal moments of inertia about the body-fixed axes
- $ω_x, ω_y, ω_z$ denote the angular velocity components in the body frame
- $M_x, M_y, M_z$ represent the applied external torque components

The implementation assumes a diagonal inertia tensor, representing principal axis alignment, which simplifies computational requirements while maintaining physical accuracy for most aerospace applications.

## Gyroscopic Coupling Effects

The simulator captures gyroscopic coupling phenomena through the cross-coupling terms in Euler's equations. These nonlinear coupling effects are essential for simulation of:

- **Gyroscopic precession** during attitude maneuvers
- **Dynamic coupling** between roll, pitch, and yaw motions
- **Inertial coupling** effects in asymmetric vehicles
- **Momentum transfer** between rotational axes

The gyroscopic terms $\mathbf{(I_j - I_k)ω_jω_k}$ create the characteristic nonlinear dynamics observed in rotating rigid bodies, enabling prediction of rotational behaviors such as tumbling motion and attitude instabilities.

## Numerical Integration Methodology

The module implements a first-order explicit Euler scheme with fixed time-step advancement:

$\omega(t + \Delta t) = \omega(t) + \dot{\omega}(t)\Delta t$

$\theta(t + \Delta t) = \theta(t) + \omega(t) \Delta t$

While computationally efficient and suitable for real-time applications, the first-order integration scheme requires careful time-step selection to maintain numerical stability and accuracy; it is not recommended for long-duration propagation or highly dynamic maneuvers without further refinement (e.g., higher-order integrators).
The implementation is optimized for deterministic execution timing, essential for real-time control applications.

## Attitude Representation and Normalization

Angular positions are represented using classical Euler angles $(\phi, \theta, \psi)$ corresponding to roll, pitch, and yaw rotations. The simulator incorporates automatic angle normalization to prevent angular accumulation and maintain bounded representations:

- **Roll, Pitch, Yaw angles**: Normalized to $[-\pi, \pi]$ range
- **Continuous normalization**: Applied at each integration step

This normalization ensures consistent attitude representation throughout extended simulation runs and prevents numerical drift in angle values.

## Model Assumptions

The current implementation incorporates the following simplifying assumptions:

- **Rigid Body**: The body is perfectly rigid; no structural flexibility or elastic deformation is modeled;
- **Principal Axis Alignment**: Constant inertia tensor aligned with body principal axes (diagonal inertia tensor only);
- **Constant Inertia Properties**: No mass property variations during flight simulation (no fuel slosh, no deployables, etc.);
- **External Torques Only**: No internal momentum exchange mechanisms (e.g., reaction wheels)
- **Euler Integration**: First-order accuracy (in larger projects this may occur in potential stability limitations)

## Computational Characteristics

The simulator is designed for deterministic real-time execution with:

- **Fixed computational complexity**: $O(1)$ per integration step
- **Minimal memory footprint**: Static allocation of state variables
- **Predictable timing**: Deterministic execution with no heap allocation, except for RAII cases (e.g. std::queue)
- **MISRA:2008 C++ compliance**: Safety-critical coding standards adherence

## Application Domains

The `RigidBodySimulator` is particularly suited for:

- **Flight control system development** and validation
- **Spacecraft attitude control** system design
- **Hardware-in-the-loop simulation** environments
- **Real-time pilot training simulators**
- **Autonomous vehicle** guidance system testing
- **Robotics applications** requiring rotational dynamics

The module's emphasis on computational efficiency and deterministic behavior makes it well-suited for embedded real-time systems where precise timing guarantees are essential for closed-loop control stability.

## Class: RigidBodySimulator

### Responsibilities

- Simulates rigid body rotational dynamics using physically accurate equations of motion
- Integrates Euler's rotational equations: $\tau = I\cdotα + ω \times (I\cdotω)$ where gyroscopic terms are included
- Maintains attitude representation using Euler angles (pitch, yaw, roll) with automatic wrapping
- Provides numerical integration of angular velocities and attitude angles using explicit Euler method
- Validates physical parameters and numerical stability throughout simulation execution
- Supports dynamic reconfiguration of inertia properties and initial conditions

### Public API

#### Constructors & Rule of Three

- ```cpp
  RigidBodySimulator()
  ```

  ***Default constructor***. Initializes rigid body with unit inertia $(1,1,1) kg⋅m^2$ and zero initial state for all angles and angular velocities. Automatically computes inverse inertia for computational efficiency.

- ```cpp
  explicit RigidBodySimulator(const Vector3f& inertia)
  ```

  ***Parameterized constructor*** with specified principal moments of inertia.

  **Parameters:**
  - `inertia`: Vector containing principal moments $(I_{xx}, I_{yy}, I_{zz})$ in $kg\cdot m^2$

  **Validation:** Assert; error handling via `SimulationManager` using `ErrorCodes` (see `ErrorCode.hpp`)

  **Initialization:** Sets all angles and angular velocities to zero, computes inverse inertia

- ```cpp
  RigidBodySimulator(const RigidBodySimulator& rbSim)
  ```

  ***Copy constructor***. Creates independent copy of all state variables and physical parameters, including recomputation of inverse inertia.

- ```cpp
  RigidBodySimulator& operator=(const RigidBodySimulator& rbSim)
  ```

  ***Assignment operator*** with self-assignment protection. Copies complete state including angles, angular velocities, inertia properties, and timestep information.

- ```cpp
  ~RigidBodySimulator()
  ```

  ***Destructor***. No dynamic memory management required.

&nbsp;

#### Configuration Methods

- ```cpp
  void setPitch(const float pitch)
  void setYaw(const float yaw)
  void setRoll(const float roll)
  ```

  Individual attitude angle setters with numerical validation.

  **Parameters:**
  - Angle values in radians (any finite value accepted)

  **Validation:** Assert; error handling via `SimulationManager` using `ErrorCodes` (see `ErrorCode.hpp`)

  **Range:** No range restrictions applied; normalization occurs during simulation update

- ```cpp
  void setOmega(const Vector3f& omega)
  ```

  Sets the angular velocity vector in body-fixed coordinates.

  **Parameters:**
  - `omega`: Angular velocity vector $(\omega_x, \omega_y, \omega_z)$ in $rad/s$

  **Coordinate Frame:** Components represent angular velocities about body-fixed $X, Y, Z$ axes respectively

- ```cpp
  void setInertia(const Vector3f& inertia)
  ```

  Sets principal moments of inertia and recomputes inverse for efficiency.

  **Parameters:**
  - `inertia`: Principal moments vector $(I_{xx}, I_{yy}, I_{zz})$ in $kg \cdot m^2$

  **Physical Constraints:** All components must be positive (validated by assertion)

  **Automatic Update:** Triggers recomputation of inverse inertia matrix

&nbsp;

#### State Accessors

- ```cpp
  inline float getPitch(void) const
  inline float getYaw(void) const
  inline float getRoll(void) const
  ```

  Current attitude angle accessors returning values in radians.

  **Returns:** Current Euler angles normalized to $[-\pi, \pi]$ range

  **Physical Meaning:**
  - Pitch: Rotation about body X-axis (nose up/down)
  - Yaw: Rotation about body Y-axis (nose left/right)
  - Roll: Rotation about body Z-axis (wing bank)

- ```cpp
  inline float getDt(void) const
  ```

  Returns the last timestep used in dynamics integration.

  **Returns:** Time interval in seconds from most recent `update()` call

  **Usage:** Debugging and integration step monitoring

- ```cpp
  inline const Vector3f& getOmega(void) const
  ```

  Returns current angular velocity vector in body-fixed coordinates.

  **Returns:** Reference to internal angular velocity state $(\omega_x, \omega_y, \omega_z)$ in $rad/s$

- ```cpp
  inline const Vector3f& getInertia(void) const
  inline const Vector3f& getInverseInertia(void) const
  ```

  Access to inertia properties for external calculations.

  **Returns:** Principal moments of inertia and their inverses respectively

  **Usage:** External torque calculations, stability analysis, control system design

&nbsp;

#### Core Methods

- ```cpp
  void update(float dt, const Vector3f& torque)
  ```

  Performs one integration step of rigid body rotational dynamics using Euler's equations.

  **Parameters:**

  - `dt`: Integration timestep in seconds (must be positive)
  - `torque`: Applied torque vector $(\tau_x, \tau_y, \tau_z)$ in $N \cdot m$

  **Algorithm Implementation:**

  1. **Angular Momentum Calculation:** $L = I \cdot \omega$ (component-wise multiplication)
  2. **Gyroscopic Term:** $\omega \times L = \omega \times (I \cdot \omega)$
  3. **Net Torque:** $\tau_{net}$ = $\tau_{applied} - \omega \times (I \cdot \omega)$
  4. **Angular Acceleration:** $\alpha = I^{-1}\cdot \tau_{net}$
  5. **Velocity Integration:** $\omega(t+dt) = \omega(t) + \alpha \cdot dt$
  6. **Angle Integration:** $\theta(t+dt) = \theta(t) + \omega \cdot dt$
  7. **Angle Normalization:** Wrap angles to $[-\pi, \pi]$ range

  **Physical Accuracy:** Includes gyroscopic coupling essential for realistic rotational dynamics
  **Numerical Method:** First-order explicit Euler integration

- ```cpp
  bool checkNumerics(void) const
  ```

  Comprehensive numerical validation of all internal state variables.

  **Validation Criteria:**

  - All angles finite (not NaN or infinite)
  - Angular velocity components valid
  - Inertia values positive and finite
  - Timestep positive and finite
  - Inverse inertia consistency

  **Returns:** True if all state is numerically valid, false otherwise

&nbsp;

### Internal Architecture

**Private State Variables:**

- `float pitch, yaw, roll`: Euler angle representation in radians
- `float dt`: Last integration timestep for debugging/monitoring
- `Vector3f omega`: Body-fixed angular velocity vector
- `Vector3f inertia`: Principal moments of inertia $(I_{xx}, I_{yy}, I_{zz})$
- `Vector3f inverseInertia`: Precomputed inverse for computational efficiency

**Private Methods:**

- ```cpp
  void compute_inverse_inertia()
  ```

  Computes component-wise inverse: $(I_{xx}, I_{yy}, I_{zz})^{-1}$ with validation

- ```cpp
  void normalize_angles()
  ```

  Wraps Euler angles to $[-\pi, \pi]$ range using modular arithmetic to prevent numerical overflow

&nbsp;

### Usage Example

```cpp
#include "RigidBodySimulator.hpp"
#include <iostream>
#include <cmath>

int main() {
    // Define inertia for a small satellite (CubeSat class)
    Vector3f satelliteInertia(0.12f, 0.15f, 0.20f);  // kg⋅m²
    RigidBodySimulator satellite(satelliteInertia);

    // Initial conditions: at rest, zero attitude
    //(it is initialized by default to these values)

    // satellite.setPitch(0.0f);
    // satellite.setYaw(0.0f);
    // satellite.setRoll(0.0f);
    // satellite.setOmega(Vector3f(0.0f, 0.0f, 0.0f));

    // Simulation parameters
    float dt = 0.05f;   // 20 Hz update rate
    int steps = 200;    // simulate 10 seconds
    Vector3f torqueImpulse(0.0f, 0.0f, 0.02f); // 0.02 N⋅m applied about Z

    for (int i = 0; i < steps; ++i) {
        if (i == 0) {
            // Apply torque only at the very first step
            satellite.update(dt, torqueImpulse);
        } else {
            // Free dynamics afterwards
            satellite.update(dt, Vector3f(0.0f, 0.0f, 0.0f));
        }

        // Log roll angle and angular rate about Z
        float roll = satellite.getRoll();
        Vector3f omega = satellite.getOmega();

        std::cout << "t = " << i * dt << " s"
                  << " | Roll = " << roll
                  << " rad | ωz = " << omega.getZ()
                  << " rad/s" << std::endl;
    }

    return 0;
}
```

---
