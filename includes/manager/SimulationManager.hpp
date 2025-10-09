#ifndef SIMULATIONMANAGER_HPP
#define SIMULATIONMANAGER_HPP

#include "../control/ActuatorDriver.hpp"
#include "../control/PIDController.hpp"
#include "../io/InputParser.hpp"
#include "../physics/RigidBodySimulator.hpp"
#include "../sensor/SensorSimulator.hpp"
#include "ErrorCodes.hpp"
#include <string>
#include <vector>

const int TIMELOG_INSTANCES = 10000;

enum SimulationState {
	STATE_UNINITIALIZED,
	STATE_INITIALIZED,
	STATE_RUNNING,
	STATE_PAUSED,
	STATE_ERROR,
	STATE_COMPLETED
};

// Structure to hold historical simulation data for logging
struct SimulationLogEntry {
	float	time;
	float	pitch;
	float	yaw;
	float	roll;
	float	omega_x;
	float	omega_y;
	float	omega_z;

	SimulationLogEntry() :
	time(0.0f), pitch(0.0f), yaw(0.0f), roll(0.0f),
	omega_x(0.0f), omega_y(0.0f), omega_z(0.0f) {}
	SimulationLogEntry(float t, float p, float y_, float r, float wx, float wy, float wz) :
	time(t), pitch(p), yaw(y_), roll(r), omega_x(wx), omega_y(wy), omega_z(wz) {}
};

class SimulationManager {

	private:
		InputParser						config_;
		PIDController					controller_;
		RigidBodySimulator				dynamics_;
		SensorSimulator					sensors_;
		ActuatorDriver					actuator_;

		float							simTime_; // Accumulated simulation time
		float							timeStep_; // Fixed dt
		float							maxSimTime_; // Simulation deadline
		int								stepCount_; // Number of simulation steps completed
		std::vector<float>				timeLog_; // Timestamp for CSV (legacy)
		std::vector<SimulationLogEntry>	logData_; // Historical data for CSV export
		SimulationState					currentState_; // Current machine state
		ErrorCode						lastError_; // Last error encountered

		// Timing validation
		float							maxJitter_; // Maximum allowed timing jitter
		float							deadlineMiss_; // Deadline miss counter

		// Safety limits
		float							maxAttitudeRate_; // Maximum safe angular rate
		float							controlSatLimit_; // Control saturation threshold

		// It initialize the private member variables according to the InputParser execution

		ErrorCode	stepOnce();
		ErrorCode	validateState();
		ErrorCode	logState();
		ErrorCode	finalize();

		// Safety utility
		bool		isStateValid() const;
		bool		isTimingValid() const;
		bool		areControlLimitsRespected() const;

		/*
		**	Implements recovery strategies:
		*
		*	- ERR_INIT_SENSOR_FAILURE:
		*		switch to last-known-good values with degraded performance warning
		*	- ERR_RT_CONTROL_DIVERGENCE:
		*		apply emergency attitude recovery sequence
		*	- ERR_T_DEADLINE_MISS:
		*		log event and continue with timing adjustment
		*	- Critical errors:
		**		transition to STATE_ERROR and halt simulation safely
		*/
		void		handleError(ErrorCode error);

		// Copy operations are not semantically meaningful for simulation manager
		SimulationManager(const SimulationManager& simManager);
		SimulationManager&	operator=(const SimulationManager& simManager);

	public:
		SimulationManager();

		SimulationManager(float timestep);

		SimulationManager(const std::string& configFile, float timestep);

		// State and error reporting
		inline SimulationState	getState() const { return (currentState_); }
		inline ErrorCode		getLastError() const { return (lastError_); }
		const char*				getErrorString() const;

		// Diagnostic information
		inline float			getCurrentTime() const { return (simTime_); }
		inline float			getDeadlineMissCount() const { return (deadlineMiss_); }
		inline int				getStepCount() const { return (stepCount_); }

		// State access methods for status reporting
		Vector3f				getCurrentAttitude() const;
		Vector3f				getCurrentAngularVelocity() const;

		// Load configuration with comprehensive error checking
		ErrorCode				loadConfiguration(const std::string& filename);

		// Initialize modules using loaded configuration
		ErrorCode				initializeModulesFromConfig();

		// Configure safety limits and timing constraints
		ErrorCode				setSafetyLimits(float maxRate, float controlLimit, float jitterTolerance);

		// Run simulation with error monitoring
		ErrorCode				run(float endTime);

		// Pause/resume functionality for debugging
		ErrorCode				pause();
		ErrorCode				resume();

		// Export logged data with integrity validation
		ErrorCode				exportLog(const std::string& filename) const;

		~SimulationManager();
};

#endif
