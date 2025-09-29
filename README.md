# Attitude Controller Simulator

A C++98 framework for simulating closed-loop 3D attitude control of rigid bodies. The project provides a deterministic execution environment for control algorithm development and validation under realistic physical and sensor constraints. The simulator follows MISRA C++:2008 coding standards and features a modular architecture that separates physics, control, sensor, and actuator models for independent testing and integration.

**Note:**
Modules such as `GaussianNoise` and `SensorSimulator` introduce stochastic elements (Gaussian noise, sensor drift), making simulation results non-deterministic unless the random number generator seed is fixed. Deterministic modules such as `ActuatorDriver`, `RigidBodySimulator`, and control algorithms always produce repeatable results given identical inputs.

## Overview

The simulator implements a closed-loop control system with the following components:

- **Rigid Body Physics**: Euler angle rotational dynamics implementation in `RigidBodySimulator`
- **Control System**: Multi-axis PID controllers with individual axis tuning (`PID`, `PIDController`)
- **Sensor Simulation**: IMU measurements with configurable drift and Gaussian noise (`SensorSimulator`, `GaussianNoise`)
- **Actuator Dynamics**: Command buffering with configurable delays and torque limits (`ActuatorDriver`)
- **Input Parsing**: XML (txt for testing) configuration file loading and validation (`InputParser`)
- **Simulation Management**: Orchestrates all modules, enforces safety limits, and exports CSV logs (`SimulationManager`)
- **Visualization System**: Python-based data analysis and 3D visualization package (**IN PROGRESS**)

## Project Structure

```text
attitude_controller_simulator/
│
├── main.cpp                                       # Main application entry point
├── Makefile                                       # Build system for C++ components
├── requirements.txt                               # Python dependencies
│
├── includes/                                      # C++ header files
│   ├── control/
│   │   ├── ActuatorDriver.hpp                     # Actuator delay and torque limiting
│   │   ├── PID.hpp                                # Single-axis PID controller
│   │   └── PIDController.hpp                      # Multi-axis PID coordination
│   ├── io/
│   │   └── InputParser.hpp                        # XML configuration parsing
│   ├── manager/
│   │   ├── ErrorCodes.hpp                         # Error code definitions
│   │   └── SimulationManager.hpp                  # Main simulation orchestration
│   ├── physics/
│   │   ├── RigidBodySimulator.hpp                 # Euler angle dynamics
│   │   └── Vector3f.hpp                           # 3D vector mathematics
│   └── sensor/
│       ├── GaussianNoise.hpp                      # Random noise generation
│       └── SensorSimulator.hpp                    # IMU simulation with drift
│
├── src/                                           # C++ implementation files
│   ├── control/
│   │   ├── ActuatorDriver.cpp
│   │   ├── PID.cpp
│   │   └── PIDController.cpp
│   ├── io/
│   │   └── InputParser.cpp
│   ├── manager/
│   │   ├── ErrorCodes.cpp
│   │   └── SimulationManager.cpp
│   ├── physics/
│   │   ├── RigidBodySimulator.cpp
│   │   └── Vector3f.cpp
│   └── sensor/
│       ├── GaussianNoise.cpp
│       └── SensorSimulator.cpp
│
├── visualization/                                 # Python visualization package
│   ├── __init__.py                                # Package initialization
│   ├── __main__.py                                # Module entry point
│   ├── AttitudeAnalysisApplication.py             # Main application interface
│   ├── AttitudeVisualizer.py                      # Core visualization engine
│   ├── SimulationDataLoader.py                    # Data loading and validation
│   ├── AttitudeMathUtils.py                       # Mathematical utilities
│   ├── RigidBodyGeometry.py                       # 3D geometry representation
│   └── VisualizationConfig.py                     # Configuration and enums
│
├── simulation_input/                              # Configuration files
│   ├── config_best_case.xml                       # Optimal conditions
│   ├── config_normal_case.xml                     # Standard conditions
│   └── config_worst_case.xml                      # Challenging conditions
│
├── simulation_output/                             # Generated data files
│
├── tests/                                         # Test suites
│   ├── control/
│   │   └── test_pid_controller.cpp
│   ├── gtest/
│   │   └── test_gtest.cpp                         # Google Test framework tests
│   ├── io/
│   │   ├── test_input_parser.cpp
│   │   └── templates/                             # Test configuration files
│   ├── manager/
│   │   └── test_simulation_manager.cpp
│   └── test_pid_rbs_vec3f.cpp                     # Core module integration tests
│
└── docs/                                          # API documentation (deprecated)
    ├── control/
    ├── physics/
    ├── sensor/
    ├── io/
    └── manager/
```

## Build and Usage

### C++ Simulator

```bash
# Build main simulator application
make

# Build all test executables
make all-tests

# Build specific test executables
make test_pid_rbs_vec3f          # Core module tests
make test_pid_controller         # PID controller tests
make test_input_parser           # Configuration parser tests
make test_simulation_manager     # Simulation manager tests
make test_gtest                  # Google Test suite (requires C++17)

# Clean builds
make clean                       # Remove object files
make fclean                      # Remove object files and executables
make re                          # Clean and rebuild

# Release build with optimizations and runtime error handling (no assert)
make RELEASE=1

# Run simulation
./attitude_simulator config_file.xml [duration] [timestep]
./attitude_simulator config_best_case.xml 10.0 0.01
```

### Python Visualization (**IN PROGRESS**)

```bash
# Install Python dependencies
pip install -r requirements.txt

# Run visualization package
python -m visualization
```

## Core Features

### C++ Simulation Engine

- **Euler Angle Dynamics**: 3D rotational physics with gyroscopic effects
- **PID Control**: Configurable proportional-integral-derivative controllers per axis
- **Sensor Models**: IMU simulation with drift accumulation and Gaussian noise
- **Actuator Delays**: Command buffering with configurable time delays
- **XML Configuration**: Flexible parameter definition for different test cases
- **CSV Export**: Simulation data logging for analysis and visualization

### Python Visualization Package (**IN PROGRESS**)

- **Interactive Menu**: Command-line interface for visualization selection
- **Static Plots**: Multi-orientation rigid body visualization
- **3D Animation**: Real-time attitude evolution with telemetry overlay
- **Trajectory Analysis**: 3D attitude path visualization in Euler angle space
- **Data Validation**: Robust CSV parsing with error handling

## System Architecture

The simulator follows a modular data-flow architecture:

```text
XML Config → SimulationManager → [RigidBodySimulator ← ActuatorDriver ← PIDController ← SensorSimulator]
                ↓
          CSV Export → Python Visualization
```

Each C++ module has independent test coverage through custom test suites and Google Test framework integration, following an iterative/incremental V-model development process.

## Configuration Files

The simulator includes three predefined test scenarios:

- **`config_best_case.xml`**: Optimal conditions with zero sensor drift and low noise
- **`config_normal_case.xml`**: Standard operating conditions with typical sensor characteristics
- **`config_worst_case.xml`**: Challenging conditions with high noise, sensor drift, and actuator limitations

Each configuration defines:

- PID controller gains (Kp, Ki, Kd) for each axis
- Physical properties (moment of inertia)
- Sensor characteristics (drift rate, noise standard deviation)
- Actuator properties (delay, torque limits)
- Initial conditions (attitude, angular velocity)
- Setpoint sequence for attitude commands

## Testing

The project includes comprehensive test coverage:

- **Custom Test Suites**: Individual module testing for core components
- **Google Test Integration**: Unit and integration tests using modern C++ testing framework
- **Configuration Validation**: Test templates for valid and invalid XML configurations
- **Python Package Tests**: Data validation and visualization component testing (**IN PROGRESS**)

## Standards Compliance

- **C++98**: Maintains compatibility with legacy embedded systems
- **MISRA C++:2008**: Follows safety-critical coding standards
- **Single-threaded**: Deterministic execution with fixed time-step integration
- **Assertion-based Validation**: All inputs and intermediate values are validated

## License

MIT License - See [LICENSE](LICENSE) for details.
