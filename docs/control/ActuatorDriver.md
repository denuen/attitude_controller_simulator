# ActuatorDriver Module – API Reference

## Overview

The ActuatorDriver module simulates realistic actuator behavior by applying torque commands to a rigid body after a configurable time delay. This module represents the dynamics of actuator response times, command buffering, and saturation effects essential for realistic control system simulation.

**Design Rationale:**
Designed to model real-world actuator limitations including command transmission delays, execution latency, and saturation monitoring. Implements a time-stamped command queue to accurately simulate actuator response delays. Optimized for deterministic simulation with precise timing control and comprehensive command validation.

## Class: ActuatorDriver

### Responsibilities

- Queue torque commands with precise timing information
- Apply commands to rigid body after specified delay periods
- Monitor command buffer status and saturation conditions
- Provide deterministic actuator delay simulation
- Validate command magnitude against saturation limits
- Support actuator reset and reconfiguration during simulation

---

### Public API

#### Constructors & Rule of Three/Five

- ```cpp
  ActuatorDriver()
  ```

  ***Default constructor***. Constructs an ActuatorDriver with zero delay and no associated rigid body.

- ```cpp
  explicit ActuatorDriver(RigidBodySimulator* rbs, float delay)
  ```

  ***Parameterized constructor***. Constructs an ActuatorDriver with specified rigid body and delay in seconds.

- ```cpp
  ActuatorDriver(const ActuatorDriver& actuatorDriver)
  ```

  ***Copy constructor*** preserving command buffer and timing state.

- ```cpp
  ActuatorDriver& operator=(const ActuatorDriver& actuatorDriver)
  ```

  ***Assignment operator*** with comprehensive state transfer.

- ```cpp
  ~ActuatorDriver()
  ```

  ***Default destructor*** (no dynamic memory allocation).

&nbsp;

#### Configuration Methods

- ```cpp
  void setDelay(float delay)
  ```

  Sets command execution delay in seconds with non-negative validation.

- ```cpp
  float getDelay() const
  ```

  Returns current command delay setting.

- ```cpp
  size_t getBufferedCommandCount() const
  ```

  Returns number of commands currently in the execution queue.

&nbsp;

#### Core Methods

- ```cpp
  void sendCommand(const Vector3f& torque)
  ```

  **Parameters**: Torque vector to be applied (N⋅m)

  **Function**: Queues command with current timestamp for delayed execution

  **Validation**: Checks torque vector for numerical validity

- ```cpp
  void update(float dt)
  ```

  **Parameters**: Time step increment (s)

  **Function**: Advances simulation time and executes expired commands

  **Algorithm**: Processes commands whose (issue_time + delay) ≤ current_time

- ```cpp
  bool hasCommandsExceedingLimit(float saturationLimit) const
  ```

  **Parameters**: Saturation limit magnitude (N⋅m)

  **Returns**: true if any buffered command exceeds the specified limit

- ```cpp
  bool checkNumerics() const
  ```

  **Parameters**: None

  **Returns**: true if all internal timing and command states are numerically valid

- ```cpp
  void reset()
  ```

  Clears all buffered commands and resets internal timing to zero.

&nbsp;

### Internal State

**Private Members:**

- `RigidBodySimulator* rbs` – Pointer to target rigid body simulator
- `std::queue<TimedCommand> commandBuffer` – Queue of pending torque commands with timestamps
- `float delay` – Command execution delay in seconds
- `float currentTime` – Current simulation time for delay calculation

**Private Structures:**

```cpp
struct TimedCommand {
    Vector3f torque;        // Torque vector to apply
    float timeIssued;       // Timestamp when command was issued
};
```

&nbsp;

### Timing and Execution Algorithm

**Command Queuing:**

$ t_{issue} = t_{current} $

$ t_{execute} = t_{issue} + \text{delay} $

**Command Execution Logic:**

1. **Time advancement**: $ t_{current} \leftarrow t_{current} + \Delta t $
2. **Command processing**: For each queued command where $ t_{execute} \leq t_{current} $
3. **Effective time calculation**: $ t_{effective} = \min(\Delta t, t_{current} - t_{execute}) $
4. **Torque application**: Apply torque for $ t_{effective} $ duration
5. **Command removal**: Remove processed command from queue

**Fractional Time Step Handling:**
Commands executing mid-timestep receive proportional application time to maintain energy conservation and timing accuracy.

&nbsp;

### Usage Example

```cpp
#include "includes/control/ActuatorDriver.hpp"
#include "includes/physics/RigidBodySimulator.hpp"
#include <iostream>
#include <iomanip>
#include <cassert>

int main() {
    // Initialize spacecraft and actuator system
    Vector3f           inertia(50.0f, 40.0f, 60.0f);
    RigidBodySimulator spacecraft(inertia);

    // Realistic actuator delay (50ms typical for electric actuators)
    float actuatorDelay = 0.05f;
    ActuatorDriver actuator(&spacecraft, actuatorDelay);

    float dt = 0.001f; // 1ms simulation timestep

    // Control system loop
    for (int step = 0; step < 10000; ++step) {
        Vector3f controlTorque(1.0f, -0.5f, 0.2f);
        actuator.sendCommand(controlTorque);
        actuator.update(dt);

        size_t bufferedCommands = actuator.getBufferedCommandCount();
        bool   saturated = actuator.hasCommandsExceedingLimit(10.0f);

        // Feedback: orientation state and angular velocity
        if (step % 1000 == 0) {
            std::cout << std::left << std::setw(10)
                      << ("Step: " + std::to_string(step)) << " | "
                      << std::fixed << std::setprecision(6)
                      << "Orientation: ["
                      << spacecraft.getPitch() << ", "
                      << spacecraft.getYaw() << ", "
                      << spacecraft.getRoll() << "]"
                      << " | Angular vel: ["
                      << spacecraft.getOmega().getX() << ", "
                      << spacecraft.getOmega().getY() << ", "
                      << spacecraft.getOmega().getZ() << "]"
                      << " | Buffer: " << bufferedCommands
                      << (saturated ? " | SATURATED" : "")
                      << std::endl;
        }

        // backlog warnings
        if (bufferedCommands > 50) {
            std::cout << "[Warning] Command buffer backlog: "
                      << bufferedCommands << std::endl;
        }
        if (saturated) {
            std::cout << "[Warning] Command exceeds saturation limit!"
                      << std::endl;
        }

        assert(actuator.checkNumerics()); // verifies numerical state consistency

        // Arbitrary reset policy for demonstration purposes
        if (bufferedCommands > 100) {
            std::cout
                << "[Reset] Excessive command backlog detected. Buffer reset."
                << std::endl;
            actuator.reset();
        }
    }
}
```

### Expected Output (conceptual):

```cpp
Step: 0    | Orientation: [0.000000, 0.000000, 0.000000] | Angular vel: [0.000000, 0.000000, 0.000000] | Buffer: 1
Step: 1000 | Orientation: [-0.003848, 0.001025, 0.006158] | Angular vel: [0.015686, -0.009799, 0.002607] | Buffer: 49

...

Step: 9000 | Orientation: [-0.284981, 0.069387, 0.463030] | Angular vel: [0.137209, -0.083258, 0.018297] | Buffer: 49
```

**Buffer Status:**

- *Typical*: 0–5 commands (no delay or very fast actuators)
- *49–50 commands*: expected for 50 ms delay with 1 ms timestep (as in this example)
- *50*: may indicate timing mismatch, excessive delay, or simulation bottleneck

**Saturation Monitoring:**

If any command exceeds the configured limit (e.g., 10 N·m), the output line will include the label `SATURATED`.

**Final Considerations:**

- Buffer size reflects actuator delay and simulation timestep:
  $ \text{buffer size} \approx \frac{\text{delay}}{dt} $ under steady‑state conditions.
- Orientation and angular velocity evolve smoothly under constant torque, consistent with rigid body dynamics absent external disturbances.
- No saturation is observed in this example since all commands remain within the 10 N·m limit.
- If the buffer grows above expected values, review actuator delay, timestep, or command rate for mismatches.
- Output columns are fixed‑width for easy parsing and log analysis.
- All values are printed with 6 decimal digits for clarity and reproducibility.

### Advanced Usage – Actuator Modeling Guidelines

- **Delay selection**: Use realistic values based on actuator type (electric: 10-100ms, hydraulic: 1-20ms)
- **Command rate**: Match command rate to actual control system update frequency
- **Saturation monitoring**: Set limits based on physical actuator capabilities
- **Buffer monitoring**: Large buffers indicate timing mismatch or excessive delay
- **Reset strategy**: Periodic reset can prevent command accumulation in fault scenarios

### Physical Interpretation

- **Transport delay**: Represents signal transmission and processing delays
- **Actuator dynamics**: Models finite response time of physical actuators
- **Command buffering**: Simulates internal actuator command processing
- **Saturation detection**: Monitors for actuator limit violations
- **Time precision**: Maintains accurate timing for control system validation

### Physical Basis & Control Assumptions

#### Control Paradigm / Operating Principle

The module implements **pure time delay modeling** with ideal actuator response characteristics. It assumes actuators behave as perfect torque sources with only transport delay effects, without bandwidth limitations, slew rate constraints, or dynamic response characteristics.

#### Underlying Physics

- **Linear Actuator Model**: Assumes perfect torque generation capability up to saturation limits
- **Transport Delay Only**: Models signal transmission and processing delays without actuator dynamics
- **Ideal Command Execution**: Once delay expires, commanded torque applied instantaneously and perfectly
- **Energy Conservation**: Fractional timestep execution preserves energy balance during partial command application
- **First-In-First-Out (FIFO)**: Command queue maintains temporal ordering of control inputs

#### Stability and Validity Assumptions

- **Fixed Time Delay**: Assumes constant, deterministic delay independent of command magnitude or system state
- **No Actuator Dynamics**: Valid when actuator bandwidth >> control bandwidth (typically 10× rule)
- **Linear Saturation**: Simple magnitude-based saturation detection without hysteresis or deadband
- **Perfect Torque Generation**: Assumes actuators can generate commanded torques exactly
- **No Rate Limitations**: Does not model slew rate or bandwidth constraints of physical actuators

#### Parameter Interpretation

- **Delay (seconds)**: Total latency from command issue to torque application
  - Signal transmission: Cable/wireless propagation delays
  - Processing time: Digital signal processing and control algorithm execution
  - Actuator settling: Time for actuator to reach commanded output
- **Command Buffer**: Represents internal actuator command processing pipeline
- **Saturation Limit (N⋅m)**: Maximum torque capability based on actuator specifications
- **Time Precision**: Limited by simulation timestep and floating-point precision

#### Limitations

- **No Actuator Bandwidth**: Missing frequency-dependent response characteristics requiring transfer function modeling
- **No Slew Rate Limits**: Cannot model maximum rate of change constraints on torque output
- **No Power Limitations**: Assumes unlimited power availability for torque generation
- **No Failure Modes**: Does not model actuator faults, degradation, or partial failures
- **Fixed Delay**: Real actuators may have load-dependent or temperature-dependent delays

***Note:*** For fully realistic and non-deterministic actuator simulation, integration with additional modules such as `GaussianNoise` and `SensorSimulator` is required. These modules introduce stochastic effects and sensor inaccuracies, enabling the ActuatorDriver to model real-world uncertainties and measurement noise in closed-loop control scenarios.

### Actuator Types and Typical Delays

| Actuator Type  | Typical Delay Range |  Application                      |
|----------------|---------------------|-----------------------------------|
| Electric Motor | 10-100 ms           | Control surfaces, reaction wheels |
| Hydraulic      | 1-20 ms             | Primary flight controls           |
| Pneumatic      | 50-200 ms           | Secondary systems                 |
| Thruster       | 1-10 ms             | Spacecraft attitude control       |

### Compliance & Safety

- **Numerical validation**: Comprehensive validation of timing states and command vectors
- **Dynamic allocations**: Uses STL queue with controlled growth
- **Determinism**: Guaranteed deterministic behavior with precise timing
- **Thread-safety**: Not thread-safe - requires external synchronization
- **Command validation**: All torque commands validated before queuing

### Performance Characteristics

- **Computational complexity**: O(n) where n is number of buffered commands
- **Memory footprint**: Scales with command buffer size (typically <100 commands)
- **Timing precision**: Limited by floating-point precision and timestep size
- **Queue efficiency**: STL queue provides efficient FIFO command processing

### Limitations & Future Extensions

- **Current limitation**: Single uniform delay for all command types
- **Current limitation**: No actuator rate limiting or slew rate constraints
- **Current limitation**: No command blending or interpolation
- **Planned extension**: Per-axis delay configuration for asymmetric actuators
- **Planned extension**: Rate limiting and acceleration constraints
- **Planned extension**: Command interpolation for smooth actuator response
