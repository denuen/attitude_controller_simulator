#include "../../includes/manager/SimulationManager.hpp"
#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>

SimulationManager::SimulationManager() :
config_(), controller_(), dynamics_(Vector3f(5.0f, 4.0f, 6.0f)),
sensors_(&dynamics_), actuator_(&dynamics_, 0.012f), simTime_(0.0f),
timeStep_(0.01f), maxSimTime_(0.0f), stepCount_(0),
currentState_(STATE_UNINITIALIZED), lastError_(ERR_SUCCESS),
maxJitter_(0.001f), deadlineMiss_(0.0f), maxAttitudeRate_(10.0f),
controlSatLimit_(100.0f) {

	timeLog_.reserve(TIMELOG_INSTANCES);
}

SimulationManager::SimulationManager(float timeStep_) :
config_(), controller_(), dynamics_(Vector3f(5.0f, 4.0f, 6.0f)),
sensors_(&dynamics_), actuator_(&dynamics_, 0.012f), simTime_(0.0f),
timeStep_(timeStep_), maxSimTime_(0.0f), stepCount_(0),
currentState_(STATE_UNINITIALIZED), lastError_(ERR_SUCCESS),
maxJitter_(0.001f), deadlineMiss_(0.0f), maxAttitudeRate_(10.0f),
controlSatLimit_(100.0f) {

	assert(timeStep_ > 0.0f && timeStep_ <= 1.0f);
	timeLog_.reserve(TIMELOG_INSTANCES);
}

SimulationManager::SimulationManager(const std::string& config_File, float timeStep_) :
config_(), controller_(), dynamics_(Vector3f(1.0f, 1.0f, 1.0f)),
sensors_(&dynamics_), actuator_(&dynamics_, 0.01f), simTime_(0.0f),
timeStep_(timeStep_), maxSimTime_(0.0f), stepCount_(0),
currentState_(STATE_UNINITIALIZED), lastError_(ERR_SUCCESS),
maxJitter_(0.001f), deadlineMiss_(0.0f), maxAttitudeRate_(10.0f),
controlSatLimit_(100.0f) {

	assert(timeStep_ > 0.0f && timeStep_ <= 1.0f);
	timeLog_.reserve(TIMELOG_INSTANCES);

	// Load configuration from XML file
	lastError_ = config_.loadConfigFromXMLSafe(config_File);
	if (lastError_ != ERR_SUCCESS) {
		currentState_ = STATE_ERROR;
	} else {
		// Initialize dynamics_ with loaded inertia
		dynamics_ = RigidBodySimulator(config_.getInertia());
		// Initialize actuator_ with loaded delay
		actuator_ = ActuatorDriver(&dynamics_, config_.getActuatorDelay());
	}
}



ErrorCode	SimulationManager::initializeModulesFromConfig() {
	if (currentState_ == STATE_INITIALIZED) {
		lastError_ = ERR_SYS_INVALID_STATE;
		return (lastError_);
	}

	if (!dynamics_.checkNumerics()) {
		lastError_ = ERR_INIT_DYNAMICS_FAILED;
		return (lastError_);
	}

	// Initialize sensors with configuration parameters
	sensors_.setNoiseStdDev(config_.getNoiseStdDev());
	sensors_.setDriftRate(config_.getDriftRate());

	if (!sensors_.checkNumerics()) {
		lastError_ = ERR_INIT_SENSORS_FAILED;
		return (lastError_);
	}

	// Initialize controller with configuration gains
	Vector3f	kp = config_.getKp();
	Vector3f	ki = config_.getKi();
	Vector3f	kd = config_.getKd();

	Vector3f	pitchGains(kp.getX(), ki.getX(), kd.getX()); // P,I,D for pitch (X)
	Vector3f	yawGains(kp.getY(), ki.getY(), kd.getY()); // P,I,D for yaw (Y)
	Vector3f	rollGains(kp.getZ(), ki.getZ(), kd.getZ()); // P,I,D for roll (Z)

	controller_.setAllGains(pitchGains, yawGains, rollGains);
	controller_.setSmoothing(config_.getControllerSmoothing());
	controller_.setAntiWindup(config_.getControllerAntiWindup());

	if (!controller_.checkNumerics()) {
		lastError_ = ERR_INIT_CONTROLLER_FAILED;
		return (lastError_);
	}

	// Initialize actuator with configuration parameters
	Vector3f	maxTorquePerAxis = config_.getMaxTorquePerAxis();
	actuator_.setMaxTorquePerAxis(maxTorquePerAxis);
	actuator_.setMaxTorqueMagnitude(config_.getMaxTorqueMagnitude());

	if (!actuator_.checkNumerics()) {
		lastError_ = ERR_INIT_ACTUATOR_FAILED;
		return (lastError_);
	}

	// Initialize simulation parameters
	simTime_ = 0.0f;
	stepCount_ = 0;
	timeLog_.clear();
	timeLog_.reserve(TIMELOG_INSTANCES);
	logData_.clear();
	logData_.reserve(TIMELOG_INSTANCES);

	// Set initial conditions from configuration
	Vector3f	initialAttitude = config_.getInitialAttitude();
	dynamics_.setPitch(initialAttitude.getX());
	dynamics_.setYaw(initialAttitude.getY());
	dynamics_.setRoll(initialAttitude.getZ());

	// Set initial angular velocities from configuration
	dynamics_.setOmega(config_.getInitialAngularVelocity());

	currentState_ = STATE_INITIALIZED;
	lastError_	 = ERR_SUCCESS;
	return (lastError_);
}

ErrorCode	SimulationManager::stepOnce() {
	if (currentState_ != STATE_RUNNING) {
		lastError_ = ERR_SYS_INVALID_STATE;
		return (lastError_);
	}

	// Validate state before processing
	ErrorCode	validationResult = validateState();
	if (validationResult != ERR_SUCCESS) {
		lastError_ = validationResult;
		handleError(validationResult);
		return (lastError_);
	}

	// 1. Update sensor readings based on current rigid body state
	sensors_.update(timeStep_);
	Vector3f	measured = sensors_.getMeasuredOrientation();

	// 2. Compute PID control commands - setpoint is zero (level attitude)
	Vector3f	setpoint(0.0f, 0.0f, 0.0f);
	Vector3f	cmd = controller_.compute(setpoint, measured, timeStep_);

	// PIDcontroller_ outputs: X=pitch_torque, Y=yaw_torque, Z=roll_torque
	// RigidBodySimulator expects: X=roll_torque, Y=pitch_torque, Z=yaw_torque
	Vector3f	remappedCmd(cmd.getZ(), cmd.getX(), cmd.getY()); // Z→X, X→Y, Y→Z

	// 4. Send commands to actuator_ (with delay simulation)
	actuator_.sendCommand(remappedCmd);

	// 5. Update actuator_ (applies delayed commands to rigid body)
	actuator_.update(timeStep_);

	// 6. Update rigid body dynamics_ with applied torques
	// The actuator_ applies torques to the dynamics_, now integrate the motion
	Vector3f	appliedTorque =
		actuator_.getLastAppliedTorque(); // Get actual applied torque
	dynamics_.update(timeStep_, appliedTorque);

	// 7. Log current state
	ErrorCode	logResult = logState();
	if (logResult != ERR_SUCCESS) {
		lastError_ = logResult;
		return (lastError_);
	}

	// 8. Advance simulation time
	simTime_ += timeStep_;
	stepCount_++;

	// 9. Check if simulation should end
	if (maxSimTime_ > 0.0f && simTime_ >= maxSimTime_) {
		currentState_ = STATE_COMPLETED;
	}

	lastError_ = ERR_SUCCESS;
	return (lastError_);
}

ErrorCode	SimulationManager::validateState() {
	// Check all modules for numerical validity
	if (!controller_.checkNumerics()) {
		return (ERR_RT_NAN_DETECTED);
	}
	if (!dynamics_.checkNumerics()) {
		return (ERR_RT_NAN_DETECTED);
	}
	if (!sensors_.checkNumerics()) {
		return (ERR_RT_SENSOR_FAILURE);
	}
	if (!actuator_.checkNumerics()) {
		return (ERR_RT_NAN_DETECTED);
	}

	// Check simulation parameters
	if (std::isnan(simTime_) || std::isinf(simTime_) || simTime_ < 0.0f
		|| std::isnan(timeStep_) || std::isinf(timeStep_) || timeStep_ <= 0.0f
		|| std::isnan(maxSimTime_) || std::isinf(maxSimTime_) || maxSimTime_ < 0.0f
		|| std::isnan(maxJitter_) || std::isinf(maxJitter_) || maxJitter_ < 0.0f
		|| std::isnan(deadlineMiss_) || std::isinf(deadlineMiss_)
		|| deadlineMiss_ < 0.0f || std::isnan(maxAttitudeRate_)
		|| std::isinf(maxAttitudeRate_) || maxAttitudeRate_ <= 0.0f
		|| std::isnan(controlSatLimit_) || std::isinf(controlSatLimit_)
		|| controlSatLimit_ <= 0.0f) {
		return (ERR_RT_NAN_DETECTED);
	}

	// Check physical limits
	Vector3f	omega = dynamics_.getOmega();
	float		omegaMag =
		std::sqrt(omega.getX() * omega.getX() + omega.getY() * omega.getY()
				+ omega.getZ() * omega.getZ());
	if (omegaMag > maxAttitudeRate_) {
		return (ERR_RT_ATTITUDE_LIMIT_EXCEEDED);
	}

	// Check control saturation
	if (actuator_.isBufferSaturated(controlSatLimit_)) {
		return (ERR_RT_CONTROL_SATURATION);
	}

	return (ERR_SUCCESS);
}

ErrorCode	SimulationManager::logState() {
	// Log current state to timeLog_ for legacy compatibility
	if (timeLog_.size() < TIMELOG_INSTANCES) {
		timeLog_.push_back(simTime_);
	}

	// Log complete state data for accurate CSV export
	SimulationLogEntry	entry(simTime_, dynamics_.getPitch(), dynamics_.getYaw(),
							 dynamics_.getRoll(), dynamics_.getOmega().getX(),
							 dynamics_.getOmega().getY(),
							 dynamics_.getOmega().getZ());

	logData_.push_back(entry);

	return (ERR_SUCCESS);
}

ErrorCode	SimulationManager::finalize() {
	// Cleanup and final validation
	if (currentState_ == STATE_COMPLETED || currentState_ == STATE_ERROR) {
		// Reset all modules to clean state
		controller_.reset();
		actuator_.reset();
		sensors_.reset();

		currentState_ = STATE_UNINITIALIZED;
		lastError_	 = ERR_SUCCESS;
		return (ERR_SUCCESS);
	}

	lastError_ = ERR_SYS_INVALID_STATE;
	return (lastError_);
}

bool	SimulationManager::isStateValid() const {
	return (currentState_ != STATE_ERROR && controller_.checkNumerics()
			&& dynamics_.checkNumerics() && sensors_.checkNumerics()
			&& actuator_.checkNumerics());
}

bool	SimulationManager::isTimingValid() const {
	return (timeStep_ > 0.0f && timeStep_ < 1.0f && simTime_ >= 0.0f
			&& (maxSimTime_ <= 0.0f || simTime_ <= maxSimTime_ + timeStep_));
}

bool	SimulationManager::areControlLimitsRespected() const {
	Vector3f	omega = dynamics_.getOmega();
	float		omegaMag = std::sqrt(omega.getX() * omega.getX()
						+ omega.getY() * omega.getY() + omega.getZ() * omega.getZ());
	return (omegaMag <= maxAttitudeRate_ && !actuator_.isBufferSaturated(controlSatLimit_));
}

void	SimulationManager::handleError(ErrorCode error) {
	lastError_ = error;

	switch (error) {
	case ERR_INIT_SENSORS_FAILED:
		std::cerr << "WARNING: Sensor initialization failed, using defaults\n";
		break;

	case ERR_RT_CONTROL_DIVERGENCE:
	case ERR_RT_ATTITUDE_LIMIT_EXCEEDED:
		std::cerr << "ERROR: Control divergence detected, applying emergency "
					 "recovery\n";
		controller_.reset(); // Reset integral windup
		break;

	case ERR_T_DEADLINE_MISS:
		// Log and continue
		deadlineMiss_ += 1.0f;
		std::cerr << "WARNING: Deadline miss detected (count: " << deadlineMiss_ << ")\n";
		break;

	case ERR_RT_NAN_DETECTED:
	case ERR_SYS_MEMORY_ALLOCATION:
	case ERR_SYS_FILE_IO:
		// Critical errors - halt simulation
		std::cerr << "CRITICAL ERROR: " << getErrorString() << " - halting simulation\n";
		currentState_ = STATE_ERROR;
		break;

	default:
		std::cerr << "ERROR: " << getErrorString() << "\n";
		currentState_ = STATE_ERROR;
		break;
	}
}

const char*	SimulationManager::getErrorString() const {
	return (errorCodeToString(lastError_));
}

ErrorCode	SimulationManager::loadConfiguration(const std::string& filename) {
	lastError_ = config_.loadConfigFileSafe(filename);
	if (lastError_ != ERR_SUCCESS) {
		return lastError_;
	}

	// Reinitialize dynamics_ and actuator_ with loaded parameters
	dynamics_ = RigidBodySimulator(config_.getInertia());
	actuator_ = ActuatorDriver(&dynamics_, config_.getActuatorDelay());

	return (initializeModulesFromConfig());
}

ErrorCode	SimulationManager::setSafetyLimits(float maxRate, float controlLimit, float jitterTolerance) {
	if (std::isnan(maxRate) || std::isinf(maxRate) || maxRate <= 0.0f
		|| std::isnan(controlLimit) || std::isinf(controlLimit)
		|| controlLimit <= 0.0f || std::isnan(jitterTolerance)
		|| std::isinf(jitterTolerance) || jitterTolerance < 0.0f) {
		lastError_ = ERR_CNF_OUT_OF_RANGE;
		return (lastError_);
	}

	maxAttitudeRate_ = maxRate;
	controlSatLimit_ = controlLimit;
	maxJitter_ = jitterTolerance;

	lastError_ = ERR_SUCCESS;
	return (lastError_);
}

ErrorCode	SimulationManager::run(float endTime) {
	if (currentState_ != STATE_INITIALIZED) {
		lastError_ = ERR_SYS_INVALID_STATE;
		return (lastError_);
	}

	if (std::isnan(endTime) || std::isinf(endTime) || endTime <= 0.0f) {
		lastError_ = ERR_CNF_OUT_OF_RANGE;
		return (lastError_);
	}

	maxSimTime_ = endTime;
	simTime_ = 0.0f;
	currentState_ = STATE_RUNNING;

	// Main simulation loop
	while (currentState_ == STATE_RUNNING) {
		ErrorCode stepResult = stepOnce();
		if (stepResult != ERR_SUCCESS) {
			break;
		}
	}

	return (lastError_);
}

ErrorCode	SimulationManager::pause() {
	if (currentState_ == STATE_RUNNING) {
		currentState_ = STATE_PAUSED;
		lastError_	 = ERR_SUCCESS;
	} else {
		lastError_ = ERR_SYS_INVALID_STATE;
	}
	return (lastError_);
}

ErrorCode	SimulationManager::resume() {
	if (currentState_ == STATE_PAUSED) {
		currentState_ = STATE_RUNNING;
		lastError_ = ERR_SUCCESS;
	} else {
		lastError_ = ERR_SYS_INVALID_STATE;
	}
	return (lastError_);
}

ErrorCode	SimulationManager::exportLog(const std::string& filename) const {
	if (logData_.empty()) {
		return (ERR_SYS_FILE_IO);
	}

	std::ofstream file(filename.c_str());
	if (!file.is_open()) {
		return (ERR_SYS_FILE_IO);
	}

	// Write CSV header
	file << "time,pitch,yaw,roll,omega_x,omega_y,omega_z\n";

	// Write logged historical data
	for (size_t i = 0; i < logData_.size(); ++i) {
		const SimulationLogEntry& entry = logData_[i];
		file << entry.time << "," << entry.pitch << "," << entry.yaw << ","
			<< entry.roll << "," << entry.omega_x << "," << entry.omega_y
			<< "," << entry.omega_z << "\n";
	}

	file.close();
	return (ERR_SUCCESS);
}

Vector3f	SimulationManager::getCurrentAttitude() const {
	return (Vector3f(dynamics_.getRoll(), dynamics_.getPitch(), dynamics_.getYaw()));
}

Vector3f	SimulationManager::getCurrentAngularVelocity() const {
	return (dynamics_.getOmega());
}

SimulationManager::~SimulationManager() {
	finalize();
}
