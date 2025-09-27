#include "../../includes/control/PIDController.hpp"
#include <cassert>
#include <iostream>
#include <cmath>

PIDController::PIDController() :
pitch_(), yaw_(), roll_() {

}

PIDController::PIDController(const Vector3f& pitchG, const Vector3f& yawG, const Vector3f& rollG) :
pitch_(pitchG.getX(), pitchG.getY(), pitchG.getZ()),
yaw_(yawG.getX(), yawG.getY(), yawG.getZ()),
roll_(rollG.getX(), rollG.getY(), rollG.getZ()) {

}

PIDController::PIDController(const PID& pitch, const PID& yaw, const PID& roll) :
pitch_(pitch), yaw_(yaw), roll_(roll) {

}

PIDController::PIDController(const PIDController& p) :
pitch_(p.pitch_), yaw_(p.yaw_), roll_(p.roll_) {

}

PIDController&	PIDController::operator=(const PIDController& p) {
	if (this != &p) {
		pitch_ = p.pitch_;
		yaw_ = p.yaw_;
		roll_ = p.roll_;
	}
	return (*this);
}

void	PIDController::setPitchGains(float kp, float ki, float kd) {
	assert(!std::isnan(kp) && !std::isinf(kp) 
		&& !std::isnan(ki) && !std::isinf(ki) 
		&& !std::isnan(kd) && !std::isinf(kd)
		&& "Error: gain values must be finite.");

	pitch_.setKp(kp);
	pitch_.setKi(ki);
	pitch_.setKd(kd);
}

void	PIDController::setYawGains(float kp, float ki, float kd) {
	assert(!std::isnan(kp) && !std::isinf(kp) 
		&& !std::isnan(ki) && !std::isinf(ki) 
		&& !std::isnan(kd) && !std::isinf(kd)
		&& "Error: gain values must be finite.");

yaw_.setKp(kp);
	yaw_.setKi(ki);
	yaw_.setKd(kd);
}

void	PIDController::setRollGains(float kp, float ki, float kd) {
	assert(!std::isnan(kp) && !std::isinf(kp) 
		&& !std::isnan(ki) && !std::isinf(ki) 
		&& !std::isnan(kd) && !std::isinf(kd)
		&& "Error: gain values must be finite.");

	roll_.setKp(kp);
	roll_.setKi(ki);
	roll_.setKd(kd);
}

void	PIDController::setAllGains(Vector3f& pitchGains, Vector3f& yawGains, Vector3f& rollGains) {
	pitch_.setGains(pitchGains.getX(), pitchGains.getY(), pitchGains.getZ());
	yaw_.setGains(yawGains.getX(), yawGains.getY(), yawGains.getZ());
	roll_.setGains(rollGains.getX(), rollGains.getY(), rollGains.getZ());
}

void	PIDController::setSmoothing(float alpha) {
	pitch_.setDerivativeSmoothing(alpha);
	yaw_.setDerivativeSmoothing(alpha);
	roll_.setDerivativeSmoothing(alpha);
}

void	PIDController::setAntiWindup(float tau) {
	pitch_.setAntiWindupTau(tau);
	yaw_.setAntiWindupTau(tau);
	roll_.setAntiWindupTau(tau);
}
	
Vector3f	PIDController::compute(Vector3f& setpoint, Vector3f& measure, float dt) {
	assert(dt > 0.0f && "Error: dt must be positive");

	Vector3f	torque;

	torque.setX(pitch_.compute(setpoint.getX(), measure.getX(), dt));
	torque.setY(yaw_.compute(setpoint.getY(), measure.getY(), dt));
	torque.setZ(roll_.compute(setpoint.getZ(), measure.getZ(), dt));

	return (torque);
}

bool	PIDController::checkNumerics() const {
	return (pitch_.checkNumerics() && yaw_.checkNumerics() && roll_.checkNumerics());
}

void	PIDController::reset() {
	pitch_.reset();
	yaw_.reset();
	roll_.reset();
}

PIDController::~PIDController() {

}

