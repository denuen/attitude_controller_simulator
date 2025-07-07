#include "../../includes/control/ActuatorDriver.hpp"
#include <cassert>

ActuatorDriver::ActuatorDriver(void) :
rbs(NULL), delay(0.0f), currentTime(0.0f) {

}

ActuatorDriver::ActuatorDriver(RigidBodySimulator* rbs, float delay) :
rbs(rbs), delay(delay), currentTime(0.0f) {

	assert(rbs != NULL && "Error: ActuatorDriver constructor: ActuatorDriver requires a non-null RigidBodySimulator");
	assert(delay >= 0.0f && "Error: ActuratorDriver constructor: delay must be non-negative");

}

ActuatorDriver::ActuatorDriver(const ActuatorDriver& actuatorDriver) :
rbs(actuatorDriver.rbs), commandBuffer(actuatorDriver.commandBuffer),
delay(actuatorDriver.delay), currentTime(actuatorDriver.currentTime) {

}

ActuatorDriver&	ActuatorDriver::operator=(const ActuatorDriver& actuatorDriver) {

	if (this != &actuatorDriver) {
		rbs = actuatorDriver.rbs;
		commandBuffer = actuatorDriver.commandBuffer;
		delay = actuatorDriver.delay;
		currentTime = actuatorDriver.currentTime;
	}
	return (*this);
}

void	ActuatorDriver::setDelay(float delay) {

	this->delay = delay;
	assert(delay >= 0.0f && "Error: ActuatorDriver assignment operator: delay must be non-negative");

}

void	ActuatorDriver::sendCommand(const Vector3f& torque) {

	const_cast<Vector3f&>(torque).assertVectorCheck();
	commandBuffer.push(TimedCommand(torque, currentTime));

}

void	ActuatorDriver::update(float dt) {

	assert(dt >= 0.0f && "Error: Actuator driver update method: dt must be non-negative");

	currentTime += dt;

	while (!commandBuffer.empty()) {
		const TimedCommand&	cmd = commandBuffer.front();

		if (cmd.timeIssued + delay <= currentTime) {
			if (rbs != NULL) {
				rbs->update(dt, cmd.torque);
			} else {
				assert(false && "Error: ActuatorDriver update method: RigidBodySimulator pointer is NULL during command execution.");
			}
			commandBuffer.pop();
		} else {
			break;
		}
	}
	
}

void	ActuatorDriver::reset(void) {

	while (commandBuffer.size())
		commandBuffer.pop();
	currentTime = 0.0f;

}

ActuatorDriver::~ActuatorDriver(void) {

}
