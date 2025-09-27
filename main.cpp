#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

#include "includes/manager/ErrorCodes.hpp"
#include "includes/manager/SimulationManager.hpp"
#include "includes/physics/Vector3f.hpp"

void printUsage(const char* programName);
void handleSimulationError(ErrorCode error);
void printSimulationBanner(float duration, float timestep);
void printSimulationSummary(const SimulationManager& simManager, float actualDuration);
bool parseArguments(int argc, char* argv[], std::string& configFile, float& duration, float& timestep);
bool fileExists(const std::string& filename) {
	std::ifstream file(filename.c_str());
	return (file.good());
}

int main(int argc, char* argv[]) {
	std::string	configFile;
	float		duration, timestep;

	if (argc == 1) {
		std::cerr << "Error: no configuration files provided." << std::endl;
		printUsage(argv[0]);
		return (-1);
	}
	if (argc == 2) {
		if (std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help") {
			printUsage(argv[0]);
			return (0);
		}
	}

	// Parse command line arguments
	if (!parseArguments(argc, argv, configFile, duration, timestep)) {
		printUsage(argv[0]);
		return (1);
	}

	// Check if config file exists
	if (!fileExists(configFile)) {
		std::cerr << "Error: Configuration file '" << configFile << "' not found" << std::endl;
		std::cerr << "Make sure the file exists and is readable" << std::endl;
		return (1);
	}

	printSimulationBanner(duration, timestep);

	std::cout << "Creating SimulationManager..." << std::endl;
	SimulationManager	simManager(timestep);

	// Load configuration from XML file and initialize modules
	std::cout << "Loading configuration from '" << configFile << "'..." << std::endl;
	ErrorCode	configError = simManager.loadConfiguration(configFile);
	if (configError != ERR_SUCCESS) {
		handleSimulationError(configError);
		return (static_cast<int>(configError));
	}

	std::cout << "Modules initialized successfully!" << std::endl;
	std::cout << "Starting simulation..." << std::endl;

	// Run the simulation
	ErrorCode	runError = simManager.run(duration);

	// Check for errors during simulation
	if (runError != ERR_SUCCESS) {
		handleSimulationError(runError);

		// For some errors, we might still want to show partial results
		if (runError == ERR_RT_CONTROL_SATURATION) {
			std::cout << "\nSimulation completed with warnings." << std::endl;
			printSimulationSummary(simManager, simManager.getCurrentTime());
			return (0); // Warning, not fatal error
		}

		return (static_cast<int>(runError));
	}

	// Print successful completion summary
	printSimulationSummary(simManager, duration);

	// Export simulation log
	std::cout << "\nExporting simulation log to "
				 "'simulation_output/simulation_output.csv'..." << std::endl;
	ErrorCode exportError = simManager.exportLog("simulation_output/simulation_output.csv");
	if (exportError != ERR_SUCCESS) {
		std::cerr << "Warning: Failed to export simulation log" << std::endl;
	} else {
		std::cout << "Simulation log exported successfully!" << std::endl;
	}

	std::cout << "\nSimulation completed successfully!" << std::endl;
	return (0);
}


void	printUsage(const char* programName) {
	std::cout << "\n=== Attitude Controller Simulator ===" << std::endl;
	std::cout << "Usage: " << programName
			<< " [config_file] [duration] [timestep]" << std::endl;
	std::cout << "\nParameters:" << std::endl;
	std::cout << "  config_file - XML configuration file (required)"
			<< std::endl;
	std::cout
		<< "                If a filename without path is provided, the file"
		<< " will be searched for in the simulation_input/ directory"
		<< std::endl;
	std::cout
		<< "  duration    - Simulation duration in seconds (default: 10.0)"
		<< std::endl;
	std::cout
		<< "  timestep    - Integration timestep in seconds (default: 0.01)"
		<< std::endl;
}

void	printSimulationBanner(float duration, float timestep) {
	std::cout << "\n" << std::string(60, '=') << std::endl;
	std::cout << "    ATTITUDE CONTROLLER SIMULATOR" << std::endl;
	std::cout << std::string(60, '=') << std::endl;
	std::cout << std::fixed << std::setprecision(3);
	std::cout << "Simulation Duration: " << duration << " seconds" << std::endl;
	std::cout << "Integration Timestep: " << timestep << " seconds"
			<< std::endl;
	std::cout << "Expected Steps: " << static_cast<int>(duration / timestep)
			<< std::endl;
	std::cout << std::string(60, '-') << std::endl;
	std::cout << "Initializing simulation modules..." << std::endl;
}

void printSimulationSummary(const SimulationManager& simManager, float actualDuration) {
	std::cout << std::string(60, '-') << std::endl;
	std::cout << "SIMULATION COMPLETE" << std::endl;
	std::cout << std::fixed << std::setprecision(3);
	std::cout << "Actual Duration: " << actualDuration << " seconds"
			<< std::endl;
	std::cout << "Steps Completed: " << simManager.getStepCount() << std::endl;

	// Get final state for summary
	Vector3f	finalAttitude = simManager.getCurrentAttitude();
	Vector3f	finalOmega = simManager.getCurrentAngularVelocity();

	std::cout << std::setprecision(2);
	std::cout << "Final Attitude (deg): ["
			<< finalAttitude.getX() * 180.0f / 3.14159f << ", "
			<< finalAttitude.getY() * 180.0f / 3.14159f << ", "
			<< finalAttitude.getZ() * 180.0f / 3.14159f << "]" << std::endl;
	std::cout << "Final Angular Velocity (deg/s): ["
			<< finalOmega.getX() * 180.0f / 3.14159f << ", "
			<< finalOmega.getY() * 180.0f / 3.14159f << ", "
			<< finalOmega.getZ() * 180.0f / 3.14159f << "]" << std::endl;

	std::cout << std::string(60, '=') << std::endl;
}

void	handleSimulationError(ErrorCode error) {
	std::cerr << "\n" << std::string(60, '!') << std::endl;
	std::cerr << "SIMULATION ERROR OCCURRED" << std::endl;
	std::cerr << std::string(60, '!') << std::endl;

	switch (error) {
	case ERR_SUCCESS:
		// Should not happen
		break;
	case ERR_CNF_OUT_OF_RANGE:
		std::cerr << "Error: Invalid simulation parameters detected"
				<< std::endl;
		std::cerr << "Check timestep, duration, and physical parameters"
				<< std::endl;
		break;
	case ERR_RT_NAN_DETECTED:
		std::cerr << "Error: Numerical instability detected" << std::endl;
		std::cerr << "Try reducing the timestep or checking initial conditions" << std::endl;
		break;
	case ERR_RT_CONTROL_SATURATION:
		std::cerr << "Warning: Actuator saturation limits exceeded" << std::endl;
		std::cerr << "Control demands may be too aggressive" << std::endl;
		break;
	case ERR_RT_SENSOR_FAILURE:
		std::cerr << "Error: Sensor system failure detected" << std::endl;
		std::cerr << "Check sensor noise and bias parameters" << std::endl;
		break;
	case ERR_RT_CONTROL_DIVERGENCE:
		std::cerr << "Error: Control system divergence detected" << std::endl;
		std::cerr << "Check PID gains and system stability" << std::endl;
		break;
	default:
		std::cerr << "Error: Unknown error code " << static_cast<int>(error) << std::endl;
		break;
	}

	std::cerr << std::string(60, '!') << std::endl;
}

bool parseArguments(int argc, char* argv[], std::string& configFile, float& duration, float& timestep) {
	configFile = "";	// must be set by user
	duration = 10.0f; // 10 second default simulation
	timestep = 0.01f; // 10ms default timestep

	if (argc > 4) {
		std::cerr << "Error: Too many arguments provided" << std::endl;
		return (false);
	}

	int argIndex = 1;

	if (argc >= 2) {
		// Check if first argument is a config file (ends with .xml) or a number
		std::string firstArg = argv[1];
		if (firstArg.length() > 4
			&& firstArg.substr(firstArg.length() - 4) == ".xml") {
			// If user provides just filename, prepend input directory
			if (firstArg.find('/') == std::string::npos) {
				configFile = "simulation_input/" + firstArg;
			} else {
				configFile = firstArg;
			}
			argIndex = 2;
		} else {
			// First argument is not a config file, treat as duration
			argIndex = 1;
		}
	}

	// Parse duration
	if (argc > argIndex) {
		char* endptr;
		duration = std::strtof(argv[argIndex], &endptr);
		if (*endptr != '\0' || duration <= 0.0f) {
			std::cerr << "Error: Invalid duration '" << argv[argIndex] << "'" << std::endl;
			std::cerr << "Duration must be between 0 and 1000 seconds" << std::endl;
			return (false);
		}
		argIndex++;
	}

	// Parse timestep
	if (argc > argIndex) {
		char* endptr;
		timestep = std::strtof(argv[argIndex], &endptr);
		if (*endptr != '\0' || timestep <= 0.0f || timestep > 1.0f) {
			std::cerr << "Error: Invalid timestep '" << argv[argIndex] << "'" << std::endl;
			std::cerr << "Timestep must be between 0 and 1.0 seconds" << std::endl;
			return (false);
		}

		// Check for reasonable timestep
		if (timestep > 0.1f) {
			std::cout << "Warning: Large timestep may cause numerical instability" << std::endl;
		}
	}

	// Validate timestep vs duration
	if (timestep > duration / 10.0f) {
		std::cerr << "Warning: Timestep is large compared to duration" << std::endl;
		std::cerr << "Consider using a smaller timestep for better accuracy" << std::endl;
	}

	// Require explicit config file
	if (configFile.empty()) {
		std::cerr << "Error: No configuration file specified." << std::endl;
		return (false);
	}

	return (true);
}
