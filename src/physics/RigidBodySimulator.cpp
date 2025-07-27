#include "../../includes/physics/RigidBodySimulator.hpp"
#include <math.h>
#include <cassert>
#include <iostream>

RigidBodySimulator::RigidBodySimulator():
pitch(0.0f), yaw(0.0f), roll(0.0f), dt(0.0f), omega(0.0f, 0.0f, 0.0f), inertia(1.0f, 1.0f, 1.0f) {

	compute_inverse_inertia();

}

RigidBodySimulator::RigidBodySimulator(const Vector3f& inertia) :
pitch(0.0f), yaw(0.0f), roll(0.0f), dt(0.0f), omega(0.0f, 0.0f, 0.0f), inertia(inertia) {

	compute_inverse_inertia();

}

RigidBodySimulator::RigidBodySimulator(const RigidBodySimulator& rbSim) {

	pitch = rbSim.pitch;
	yaw = rbSim.yaw;
	roll = rbSim.roll;
	dt = rbSim.dt;
	omega = rbSim.omega;
	inertia = rbSim.inertia;
	compute_inverse_inertia();

}

RigidBodySimulator&	RigidBodySimulator::operator=(const RigidBodySimulator& rbSim) {

	if (this != &rbSim) {
		pitch = rbSim.pitch;
		yaw = rbSim.yaw;
		roll = rbSim.roll;
		dt = rbSim.dt;
		omega = rbSim.omega;
		inertia = rbSim.inertia;
		compute_inverse_inertia();
	}
	return (*this);
}

void	RigidBodySimulator::compute_inverse_inertia() {

	assert(inertia.getX() > 0.0f && inertia.getY() > 0.0f
		&& inertia.getZ() > 0.0f && "Error: Inertia components must be positive");

	inverseInertia.setX(1.0f / inertia.getX());
	inverseInertia.setY(1.0f / inertia.getY());
	inverseInertia.setZ(1.0f / inertia.getZ());

}

void	RigidBodySimulator::setPitch(const float pitch) {

	assert(!std::isinf(pitch) && !std::isnan(pitch) && "Error: Pitch value must be finite");
	this->pitch = pitch;

}

void	RigidBodySimulator::setYaw(const float yaw) {

	assert(!std::isinf(yaw) && !std::isnan(yaw) && "Error: Yaw value must be finite");
	this->yaw = yaw;

}

void	RigidBodySimulator::setRoll(const float roll) {

	assert(!std::isinf(roll) && !std::isnan(roll) && "Error: Roll value must be finite");
	this->roll = roll;

}

void	RigidBodySimulator::setOmega(const Vector3f& omega) {

	this->omega = omega;

}

void	RigidBodySimulator::setInertia(const Vector3f& inertia) {

	assert(inertia.getX() > 0.0f && inertia.getY() > 0.0f
			&& inertia.getZ() > 0.0f && "Error: Inertia components must be positive");

	this->inertia = inertia;

	compute_inverse_inertia();

}

bool	RigidBodySimulator::checkNumerics(void) const {

	if (std::isnan(pitch) || std::isinf(pitch)
		|| std::isnan(yaw) || std::isinf(yaw)
		|| std::isnan(roll) || std::isinf(roll)
		|| std::isnan(dt) || std::isinf(dt) || dt < 0.0f
		|| !omega.checkNumerics() || !inertia.checkNumerics()
		|| !inverseInertia.checkNumerics()
		|| inertia.getX() <= 0.0f || inertia.getY() <= 0.0f || inertia.getZ() <= 0.0f) {
			return (0);
	}
	return (1);
}

void	RigidBodySimulator::update(float dt, const Vector3f& torque) {

	assert(dt > 0.0f && "Error: Time step must be positive");

	assert(!std::isinf(torque.getX()) && !std::isinf(torque.getY()) && !std::isinf(torque.getZ())
		&& !std::isnan(torque.getX()) && !std::isnan(torque.getY()) && !std::isnan(torque.getZ())
		&& "Error: Torque components must be finite");

	// I·ω
	Vector3f	angularMomentum = componentMultiply(inertia, omega);

	// ω × (I·ω)
	Vector3f	gyroscopicTerm = cross(omega, angularMomentum);

	// net torque: τ - ω × (I·ω)
	Vector3f	netTorque = torque - gyroscopicTerm;

	// angular acceleration: α = I^(-1) * netTorque
	Vector3f	angularAcceleration = componentMultiply(inverseInertia, netTorque);

	// euler integration for angular velocity: ω = ω_0 + α * dt
	omega = omega + angularAcceleration * dt;

	assert(!std::isinf(omega.getX()) && !std::isinf(omega.getY()) && !std::isinf(omega.getZ())
		&& !std::isnan(omega.getX()) && !std::isnan(omega.getY()) && !std::isnan(omega.getZ())
		&& "Error: Angular velocity components must be finite after update");

	pitch += omega.getX() * dt;
	yaw += omega.getY() * dt;
	roll += omega.getZ() * dt;

	assert(!std::isinf(pitch) && !std::isinf(yaw) && !std::isinf(roll)
		&& !std::isnan(pitch) && !std::isnan(yaw) && !std::isnan(roll)
		&& "Error: Angle components must be finite after update");

	normalize_angles();

	this->dt = dt;

}

void RigidBodySimulator::normalize_angles() {

	const float	PI = 3.14159265359f;
	const float	TWO_PI = 2.0f * PI;

	// [-π, π] range
	while (pitch > PI) pitch -= TWO_PI;
	while (pitch < -PI) pitch += TWO_PI;

	while (yaw > PI) yaw -= TWO_PI;
	while (yaw < -PI) yaw += TWO_PI;

	while (roll > PI) roll -= TWO_PI;
	while (roll < -PI) roll += TWO_PI;

}

RigidBodySimulator::~RigidBodySimulator() {

}
