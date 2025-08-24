# ActuatorDriver Module API Reference

## Overview

The `ActuatorDriver` module simulates realistic actuator systems with time delays and command buffering. It models the temporal behavior of physical actuators such as servo motors, hydraulic systems, or control surface actuators that require finite time to respond to control commands. The implementation provides a FIFO command queue with timestamp-based execution, enabling accurate simulation of actuator dynamics in flight control systems.

## Class: ActuatorDriver

### Responsibilities

- Buffers torque commands in a time-stamped queue with FIFO ordering
- Simulates fixed actuator delays by deferring command execution
- Maintains internal simulation clock for precise timing calculations
- Applies delayed commands to associated rigid body simulators
- Provides command queue monitoring and saturation limit checking
- Validates numerical integrity of all buffered commands and timing data

### Public API

#### Constructors & Rule of Three

```cpp
ActuatorDriver()
```
Default constructor. Initializes actuator with zero delay, null rigid body pointer, and empty command buffer. The internal clock starts at zero.

```cpp
explicit ActuatorDriver(RigidBodySimulator* rbs, float delay)
```
Parameterized constructor with target rigid body and delay specification.

**Parameters:**
- `rbs`: Pointer to target RigidBodySimulator (must be non-null)
- `delay`: Command execution delay in seconds (must be non-negative)

Validates that the rigid body pointer is non-null and delay is non-negative using assertions.

```cpp
ActuatorDriver(const ActuatorDriver& actuatorDriver)
```
Copy constructor. Creates deep copy of command buffer, timing state, and configuration. The rigid body pointer is copied as-is (shallow copy of pointer).

```cpp
ActuatorDriver& operator=(const ActuatorDriver& actuatorDriver)
```
Assignment operator with self-assignment protection. Copies all internal state including buffered commands and timing information.

```cpp
~ActuatorDriver()
```
Destructor. Command buffer is automatically cleaned up by std::queue destructor.

#### Configuration Methods

```cpp
void setDelay(float delay)
```
Sets the command execution delay in seconds. The new delay applies to all future commands; already buffered commands retain their original execution times.

**Parameters:**
- `delay`: New delay value in seconds (must be non-negative)

Validates that delay is non-negative using assertions.

```cpp
inline float getDelay() const
```
Returns the current command delay in seconds.

#### Queue Management & Monitoring

```cpp
inline size_t getBufferedCommandCount() const
```
Returns the number of commands currently queued for future execution. Useful for monitoring actuator loading and detecting command backlogs.

```cpp
bool hasCommandsExceedingLimit(float saturationLimit) const
```
Checks if any buffered command exceeds the specified magnitude limit without modifying the queue.

**Parameters:**
- `saturationLimit`: Maximum allowed torque magnitude (must be positive and finite)

**Returns:** True if any queued command has `torque.magnitude() > saturationLimit`

**Implementation:** Creates temporary queue copy to avoid destructive iteration.

#### Core Methods

```cpp
void sendCommand(const Vector3f& torque)
```
Queues a torque command for delayed execution. The command is timestamped with the current internal time and will be executed after the configured delay period.

**Parameters:**
- `torque`: Three-dimensional torque vector to be applied

**Algorithm:**
1. Validates torque vector numerical integrity
2. Creates TimedCommand with current timestamp
3. Enqueues command in FIFO buffer

```cpp
void update(float dt)
```
Advances internal clock and processes any commands whose delay has expired. This is the core temporal processing method that must be called regularly.

**Parameters:**
- `dt`: Time step in seconds (must be non-negative)

**Algorithm:**
1. Advances internal clock: `currentTime += dt`
2. Processes queue front-to-back until unexpired command found
3. For each expired command:
   - Calculates exact execution timing within timestep
   - Calls `rbs->update(effectiveTime, command.torque)`
   - Removes command from queue

**Timing Logic:**
- Commands execute when: `commandTime + delay ≤ currentTime + epsilon`
- Epsilon tolerance (1e-6f) handles floating-point precision issues
- Effective time calculation ensures proper temporal integration

```cpp
void reset()
```
Clears all buffered commands and resets internal clock to zero. Does not modify delay setting or rigid body association.

```cpp
bool checkNumerics() const
```
Validates numerical integrity of delay, current time, and all buffered commands. Returns false if any value is NaN, infinite, or negative (for time values).

### Internal Architecture

#### TimedCommand Structure
```cpp
struct TimedCommand {
    Vector3f torque;      // Torque vector to be applied
    float timeIssued;     // Timestamp when command was queued
};
```

**Private Members:**
- `RigidBodySimulator* rbs`: Target simulator for command execution
- `std::queue commandBuffer`: FIFO command queue
- `float delay`: Execution delay in seconds
- `float currentTime`: Internal simulation clock

### Usage Example

```cpp
// Initialize actuator with 50ms delay
RigidBodySimulator aircraft(Vector3f(1.0f, 2.0f, 3.0f));
ActuatorDriver actuator(&aircraft, 0.05f);

// Control loop with realistic actuator delays
float dt = 0.01f;  // 100 Hz control frequency
Vector3f controlTorque(0.5f, 0.0f, -0.2f);

for (int i = 0; i  10) {
        std::cout << "Warning: Command backlog detected" << std::endl;
    }

    // Check for saturation issues
    if (actuator.hasCommandsExceedingLimit(10.0f)) {
        std::cout << "Warning: Commands exceed saturation limit" << std::endl;
    }
}

// Reset for new simulation run
actuator.reset();
assert(actuator.getBufferedCommandCount() == 0);
```

### Advanced Usage - Variable Delay Simulation

```cpp
// Simulate actuator with load-dependent delays
ActuatorDriver adaptiveActuator(&aircraft, 0.0f);

for (float commandMagnitude = 1.0f; commandMagnitude <= 10.0f; commandMagnitude += 1.0f) {
    // Larger commands take longer to execute
    float adaptiveDelay = 0.01f + (commandMagnitude * 0.005f);
    adaptiveActuator.setDelay(adaptiveDelay);

    Vector3f command(commandMagnitude, 0.0f, 0.0f);
    adaptiveActuator.sendCommand(command);
    actuator.update(0.01f);
}
```

### Physical Modeling & Assumptions

**Delay Model:**
- Fixed, uniform delay independent of command magnitude or direction
- No rate limiting or slew rate constraints
- Instantaneous command execution after delay expiration

**Command Buffering:**
- Unlimited buffer capacity (memory permitting)
- FIFO execution order strictly maintained
- Perfect command fidelity (no degradation or noise)

**Temporal Accuracy:**
- Sub-timestep execution timing for commands expiring mid-step
- Floating-point epsilon tolerance for timing comparisons
- Monotonic time progression assumption

### Performance Characteristics

**Computational Complexity:**
- `sendCommand()`: O(1) - constant time queue insertion
- `update()`: O(n) where n is number of expired commands
- `hasCommandsExceedingLimit()`: O(n) where n is total queued commands

**Memory Usage:**
- Fixed overhead: ~32 bytes per instance
- Variable: ~20 bytes per queued command
- No dynamic allocation beyond std::queue growth

### Limitations

**Physical Realism:**
- No actuator dynamics (bandwidth, resonance, nonlinearity)
- No rate limiting or velocity constraints
- No power consumption or thermal effects
- No failure modes or degraded operation

**Timing Model:**
- Fixed delay assumption (real actuators have variable response times)
- No jitter or stochastic delays
- Perfect clock synchronization assumed

**Command Handling:**
- No command prioritization or preemption
- No command interpolation or smoothing
- No feedback on actuator state or health

### Compliance & Safety

- Thread-safe for single-instance use (no shared mutable state)
- Exception-safe operations with RAII cleanup
- Comprehensive input validation with assertions
- No raw pointer ownership (non-owning reference to rigid body)
- C++11 compatible with standard library containers

### Integration Considerations

**Timestep Sensitivity:**
- Smaller timesteps provide higher timing accuracy
- Large timesteps may cause temporal aliasing effects
- Recommended: `dt ≤ delay/10` for accurate simulation

**Queue Management:**
- Monitor buffer depth in real-time systems
- Consider implementing buffer size limits for memory-constrained environments
- Reset periodically in long-running simulations to prevent memory growth

***

_This documentation reflects the actual implementation in `ActuatorDriver.hpp` and `ActuatorDriver.cpp`. All method signatures and behavior descriptions are validated against the source code._
