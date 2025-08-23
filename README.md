# Attitude Controller Simulator

A modular and extensible C++98 framework for simulating closed-loop 3D attitude control of rigid bodies, this project is designed to provide a deterministic execution environment for the development, numerical validation, and benchmarking of control algorithms under realistic physical and sensor constraints. Emphasizing MISRA C++:2008 coding standards for reliability and safety, the simulator features a strict modular architecture that facilitates independent testing and integration of physics, control, sensor, and actuator models. Its design supports repeatable, traceable simulation runs, enabling rigorous verification of control strategies and system behaviors. The framework is particularly suited for educational purposes and rapid prototyping, offering a robust platform for exploring flight dynamics, validating PID tuning, and investigating closed-loop control in a controlled, single-threaded context.

## Overview

The simulator implements a modular closed-loop system:

- **Rigid Body Physics**: Euler rotational dynamics with gyroscopic effects.
- **Control System**: 3-axis PID controllers with anti-windup and derivative filtering.
- **Sensor Simulation**: IMU-style measurements with deterministic drift and Gaussian noise.
- **Actuator Dynamics**: FIFO command buffering with configurable delays.
- **Input Parsing**: TXT and XML configuration loading.
- **Simulation Management**: Orchestrates modules, enforces safety limits, and logs simulation time.

## Project Structure (in development)

The directory layout below summarizes the modular organization of the simulator. This structure is actively evolving as new features and modules are added. Expect additional folders and files as the project expands to support input parsing, simulation management, and advanced logging.

```md
attitude_controller_simulator
|
├── docs
│   ├── control                                    # API references for control modules
│   ├── io                                         # API references for input/output modules
│   ├── manager                                    # API references for manager modules
│   ├── physics                                    # API references for physics simulation modules (e.g., rigid body, vectors)
│   └── sensor                                     # API references for sensor modeling modules
|
├── includes
|   |
│   ├── control                                     # Control system headers (PID, controllers, actuators)
│   │   ├── ActuatorDriver.hpp                          # Actuator command buffering and delay simulation
│   │   ├── PID.hpp                                     # Single-axis PID controller logic
│   │   └── PIDController.hpp                           # Multi-axis PID controller
|   |
│   ├── io                                          # Input/output module
│   │   └── InputParser.hpp                             # Simulation configuration parser
|   |
│   ├── manager                                     # Simulation management and error handling
│   │   ├── ErrorCodes.hpp                              # Error codes and status enums
│   │   └── SimulationManager.hpp                       # Simulation loop and coordination logic
|   |
│   ├── physics                                     # Physics engine
│   │   ├── RigidBodySimulator.hpp                      # Rigid body dynamics simulation
│   │   └── Vector3f.hpp                                # 3D vector math utilities
|   |
│   └── sensor                                      # Sensor simulation headers
│       ├── GaussianNoise.hpp                           # Gaussian noise generation (sensor noise)
│       └── SensorSimulator.hpp                         # IMU and sensor modeling
|
├── src
│   ├── control
│   │   ├── ActuatorDriver.cpp
│   │   ├── PID.cpp
│   │   └── PIDController.cpp
│   ├── io
│   │   └── InputParser.cpp
│   ├── manager
│   │   ├── ErrorCodes.cpp
│   │   └── SimulationManager.cpp
│   ├── physics
│   │   ├── RigidBodySimulator.cpp
│   │   └── Vector3f.cpp
│   └── sensor
│       ├── GaussianNoise.cpp
│       └── SensorSimulator.cpp
|
├── tests
|    ├── control
|    │   └── test_pid_controller.cpp                 # Custom tester for PID only
|    ├── gtest
|    │   └── test_gtest.cpp                          # Comprehensive integration and unit tests using Google Test framework
|    ├── io
|    │   ├── templates                               # XML and txt templates for good/bad configurations (test utility)
|    │   └── test_input_parser.cpp                   # Custom tester for InputParser only
|    └── test_pid_rbs_vec3f.cpp                      # Custom tester for PID, PIDController, RigidBodySimulator, Vector3f
|
|
├── LICENSE
├── Makefile
└── README.md
```

## Build and Test

```bash
# Build all test executables (default)
make all

# Build and run core module tests
make test_pid_rbs_vec3f
make test_pid_controller

# Build and run InputParser tests
make test_input_parser

# Build and run Google Test suite (requires C++17)
make test_gtest

# Clean object files
make clean

# Clean object files and executables
make fclean

# Clean and rebuild all targets
make re

# Release build (optimizations, disables asserts)
make release TARGET=<target_name>

# Show all available targets and usage
make help
```

## Use Cases

- **Flight Dynamics Education**: Learn and validate control theory with realistic physics.
- **Algorithm Development**: PID tuning and closed-loop control strategy testing.
- **Robustness Analysis**: Monte Carlo simulations with sensor noise and drift.
- **Simulation Management**: Test and log full system behavior in deterministic loops.

## Architecture

The system follows a clean data-flow architecture:

***Configuration → Sensor Reading → Control Computation → Actuation → Physics Update***

Each module exposes a strict interface and is independently testable. The project includes both custom test suites for each module and unit testing suite based on Google Test, ensuring correctness and robustness across the entire system.

## Documentation

- [API Reference](docs)
- [Test Suites](tests)

## Compliance

- **C++98**: No modern C++ features used.
- **MISRA C++:2008**: Safety-critical coding standards.
- **Deterministic Execution**: Single-threaded, fixed time-step. A multi-threaded approach could be introduced in a future version of the project, possibly using C++11 or later.
- **Assertions**: Validate all inputs and variables before, during, and after operations.

## License

MIT License - See [LICENSE](LICENSE) for details.
