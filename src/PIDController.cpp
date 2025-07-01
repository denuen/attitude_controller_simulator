#include "PIDController.hpp"
#include <cassert>

PIDController::PIDController() :
pidPitch(), pidYaw(), pidRoll() {

}

PIDController::PIDController(const Vector3f& kp, const Vector3f& ki, const Vector3f& kd) :
pidPitch(kp.getX(), ki.getX(), kd.getX()),
pidYaw(kp.getY(), ki.getY(), kd.getY()),
pidRoll(kp.getZ(), ki.getZ(), kd.getZ()) {

}

PIDController::PIDController(const PIDController& pidController) :
pidPitch(pidController.pidPitch),
pidYaw(pidController.pidYaw),
pidRoll(pidController.pidRoll) {

}

PIDController&	PIDController::operator=(const PIDController& pidController) {

	if (this != &pidController) {
		pidPitch = pidController.pidPitch;
		pidYaw = pidController.pidYaw;
		pidRoll = pidController.pidRoll;
	}
	return (*this);
}

void	PIDController::setGains(const Vector3f& kp, const Vector3f& ki, const Vector3f& kd) {

	pidPitch.setGains(kp.getX(), ki.getX(), kd.getX());
	pidYaw.setGains(kp.getY(), ki.getY(), kd.getY());
	pidRoll.setGains(kp.getZ(), ki.getZ(), kd.getZ());

}

void	PIDController::setSmoothing(const float alpha) {

	pidPitch.setDerivativeSmoothing(alpha);
	pidYaw.setDerivativeSmoothing(alpha);
	pidRoll.setDerivativeSmoothing(alpha);

}

Vector3f	PIDController::compute(const Vector3f& setpoint, const Vector3f& measure, float dt) {
	Vector3f	torque;

	assert(dt > 0.0f && "Error: PIDController::compute: dt must be positive");

	torque.setX(pidPitch.compute(setpoint.getX(), measure.getX(), dt));
	torque.setY(pidYaw.compute(setpoint.getY(), measure.getY(), dt));
	torque.setZ(pidRoll.compute(setpoint.getZ(), measure.getZ(), dt));

	return (torque);
}

void	PIDController::reset(void) {
	pidPitch.reset();
	pidYaw.reset();
	pidRoll.reset();
}

PIDController::~PIDController() {

}
