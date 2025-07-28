#ifndef SIMULATIONMANAGER_HPP
#define SIMULATIONMANAGER_HPP

#include "../control/ActuatorDriver.hpp"
#include "../control/PIDController.hpp"
#include "../io/InputParser.hpp"
#include "../physics/RigidBodySimulator.hpp"
#include "../sensor/SensorSimulator.hpp"
#include "ErrorCodes.hpp"
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

class SimulationManager {

	private:
		InputParser			config;
		PIDController		controller;
		RigidBodySimulator	dynamics;
		SensorSimulator		sensors;
		ActuatorDriver		actuator;

		float				simTime;		// accumulated simulation time
		float				timeStep;		// fixed dt
		float				maxSimTime;		// simulation deadline
		std::vector<float>	timeLog;		// timestamp for CSV
		SimulationState		currentState;	// current machine state
		ErrorCode			lastError;		// last error encountered

		// Timing validation
		float				maxJitter;		// maximum allowed timing jitter
		float				deadlineMiss;	// deadline miss counter

		// Safety limits
		float				maxAttitudeRate;	// maximum safe angular rate
		float				controlSatLimit;	// control saturation threshold

		// It initialize the private member variables according to the InputParser execution
		ErrorCode			initializeModules();


		ErrorCode			stepOnce();
		ErrorCode			validateState();
		ErrorCode			logState();
		ErrorCode			finalize();

		// Safety utility
		bool				isStateValid() const;
		bool				isTimingValid() const;
		bool				areControlLimitsRespected() const;

		/*
		**	Implements recovery strategies:
		**	ERR_INIT_SENSOR_FAILURE		-> switch to last-known-good values with degraded performance warning
		**	ERR_RT_CONTROL_DIVERGENCE	-> apply emergency attitude recovery sequence
		**	ERR_T_DEADLINE_MISS			-> log event and continue with timing adjustment
		**
		**	Critical errors				-> transition to STATE_ERROR and halt simulation safely
		*/
		void				handleError(ErrorCode error);

		// Copy operations are not semantically meaningful for simulation manager
		SimulationManager(const SimulationManager& simManager);
		SimulationManager& operator=(const SimulationManager& simManager);

	public:
		SimulationManager();

		// State and error reporting
		inline SimulationState	getState() const { return (currentState); }
		inline ErrorCode		getLastError() const { return (lastError); }
		const char*				getErrorString() const;

		// Diagnostic information
		inline float			getCurrentTime() const { return (simTime); }
		inline float			getDeadlineMissCount() const { return (deadlineMiss); }

		// Load configuration with comprehensive error checking
		ErrorCode				loadConfiguration(const std::string& filename);

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
