#include "../../includes/control/ActuatorDriver.hpp"
#include <cassert>
#include <iostream>
#include <cmath>

float	ActuatorDriver::clamp(float value, float max) {
	if (max > 0.0f) {
		if (value > max) {
			return (max);
		} else if (value < -max) {
			return (-max);
		}
	}
	return (value);
}

ActuatorDriver::ActuatorDriver() :
rbs_(NULL), commandBuffer_(), delay_(0.0f),
currentTime_(0.0f), maxTorqueMagnitude_(0.0f),
maxTorquePerAxis_(), lastAppliedCommand_() {

}

ActuatorDriver::ActuatorDriver(RigidBodySimulator* rbs, float delay) :rbs_(rbs), commandBuffer_(), delay_(delay),
currentTime_(0.0f), maxTorqueMagnitude_(0.0f),
maxTorquePerAxis_(), lastAppliedCommand_() {
	assert(rbs_ != NULL && "Error: AD requires a non-null rbs");
	assert(!std::isnan(delay_) && !std::isinf(delay_)
		&& delay_ >= 0.0f && "Error: delay must be a finite non-negative number");
}

ActuatorDriver::ActuatorDriver(const ActuatorDriver& a) :
rbs_(a.rbs_), commandBuffer_(a.commandBuffer_), delay_(a.delay_),
currentTime_(a.currentTime_), maxTorqueMagnitude_(a.maxTorqueMagnitude_),
maxTorquePerAxis_(a.maxTorquePerAxis_), lastAppliedCommand_(a.lastAppliedCommand_) {

}

ActuatorDriver&	ActuatorDriver::operator=(const ActuatorDriver& a) {
	if (this != &a) {
		rbs_ = a.rbs_;
		commandBuffer_ = a.commandBuffer_;
		delay_ = a.delay_;
		currentTime_ = a.currentTime_;
		maxTorqueMagnitude_ = a.maxTorqueMagnitude_;
		maxTorquePerAxis_ = a.maxTorquePerAxis_;
		lastAppliedCommand_ = a.lastAppliedCommand_;
	}
	return (*this);
}

void	ActuatorDriver::setDelay(float delay) {
	assert(!std::isnan(delay) && !std::isinf(delay)
		&& delay >= 0.0f && "Error: delay must be a finite non-negative number");
	delay_ = delay;
}

void	ActuatorDriver::setMaxTorquePerAxis(Vector3f& maxTorques) {
	maxTorques.assertCheck();
	assert(maxTorques.getX() >= 0.0f
		&& maxTorques.getY() >= 0.0f
		&& maxTorques.getZ() >= 0.0f
		&& "Error: the maximum torque magnitudes must be finite and non-negative");
	maxTorquePerAxis_ = maxTorques;
}

void	ActuatorDriver::setMaxTorqueMagnitude(float maxMagnitude) {
	assert(!std::isnan(maxMagnitude) && !std::isinf(maxMagnitude)
		&& maxMagnitude >= 0.0f && "Error: torque magnitude cap must be finite and non-negative");
	maxTorqueMagnitude_ = maxMagnitude;
}

void	ActuatorDriver::sendCommand(const Vector3f& torque) {
	Vector3f	safeTorque(torque);
	safeTorque.setX(clamp(torque.getX(), maxTorquePerAxis_.getX()));
	safeTorque.setY(clamp(torque.getY(), maxTorquePerAxis_.getY()));
	safeTorque.setZ(clamp(torque.getZ(), maxTorquePerAxis_.getZ()));
	
	// Queue the (possibly saturated) torque enforcing global magnitude cap if set
	if (maxTorqueMagnitude_ > 0.0f) {
		float	mag = safeTorque.magnitude();
		if (mag > maxTorqueMagnitude_) {
			float	scaleFactor = maxTorqueMagnitude_ / mag;
			safeTorque *= scaleFactor;
		}	
	}

	commandBuffer_.push(TimedCommand(safeTorque, currentTime_));
}

void	ActuatorDriver::update(float dt) {
	assert(!std::isinf(dt) && !std::isnan(dt)
		&& dt >= 0.0f && "Error: dt must be finite and non-negative");
	
	const float	epsilon = 1e-6f;
	
	currentTime_ += dt;
	while (!commandBuffer_.empty()) {
		const TimedCommand&	cmd = commandBuffer_.front();

		if (cmd.timeIssued_ + delay_ <= currentTime_ + epsilon) {
			float	effectiveTime = 0.0f;
			float	absoluteExecTime = cmd.timeIssued_ + delay_;
			float	timeFromExecutionToEnd = currentTime_ - absoluteExecTime;
			
			if (timeFromExecutionToEnd > 0.0f) {
				// The cmd has been executed in a previous timestep or at the 
				// start of the current one
				effectiveTime = std::min(timeFromExecutionToEnd, dt);
			} else {
				// The cmd is being executed through the current timestep
				effectiveTime = dt;
			}
			
			// Ensure effectiveTime > 0
			effectiveTime = std::max(effectiveTime, epsilon);
			
			lastAppliedCommand_ = TimedCommand(cmd.torque_, absoluteExecTime);
			rbs_->update(effectiveTime, cmd.torque_);
			commandBuffer_.pop();
		} else {
			break ;
		}
	}
}

bool	ActuatorDriver::checkNumerics() const {
	if (std::isnan(delay_) || std::isinf(delay_) || delay_ < 0.0f) {
		return (0);
	}

	if (std::isnan(currentTime_) || std::isinf(currentTime_)
		|| currentTime_ < 0.0f) {
		return (0);
	}

	std::queue<TimedCommand> tmp = commandBuffer_;
	while (!tmp.empty()) {
		const TimedCommand& cmd = tmp.front();

		if (std::isnan(cmd.timeIssued_) || std::isinf(cmd.timeIssued_)
			|| cmd.timeIssued_ < 0.0f) {
			return (0);
		}

		if (!cmd.torque_.checkNumerics()) {
			return (0);
		}

		tmp.pop();
	}

	return (1);
}

bool	ActuatorDriver::isBufferSaturated(float saturationLimit)const  {
	assert(!std::isnan(saturationLimit) && !std::isinf(saturationLimit)
		&& saturationLimit > 0.0f && "Error: saturation limit must be a finite non-negative value");

	std::queue<TimedCommand>	tmp = commandBuffer_;
	while (!tmp.empty()) {
		const TimedCommand&	cmd = tmp.front();
		if (cmd.torque_.magnitude() > saturationLimit) {
			return (true);
		}	
		tmp.pop();
	}
	return (false);
}

void	ActuatorDriver::reset() {
	while (commandBuffer_.size()) {
		commandBuffer_.pop();
	}
	currentTime_ = 0.0f;
}

ActuatorDriver::~ActuatorDriver() {

}

