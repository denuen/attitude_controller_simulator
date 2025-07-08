#include "../../includes/physics/RigidBodySimulator.hpp"
#include <math.h>
#include <cassert>

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

void RigidBodySimulator::compute_inverse_inertia() {

	assert(inertia.getX() > 0.0f && inertia.getY() > 0.0f
		&& inertia.getZ() > 0.0f && "Error: Inertia components must be positive");

	inverseInertia.setX(1.0f / inertia.getX());
	inverseInertia.setY(1.0f / inertia.getY());
	inverseInertia.setZ(1.0f / inertia.getZ());

}

void RigidBodySimulator::setPitch(const float pitch) {

	assert(!isinf(pitch) && !isnan(pitch) && "Error: Pitch value must be finite");
	this->pitch = pitch;

}

void RigidBodySimulator::setYaw(const float yaw) {

	assert(!isinf(yaw) && !isnan(yaw) && "Error: Yaw value must be finite");
	this->yaw = yaw;

}

void RigidBodySimulator::setRoll(const float roll) {

	assert(!isinf(roll) && !isnan(roll) && "Error: Roll value must be finite");
	this->roll = roll;

}

void RigidBodySimulator::setOmega(const Vector3f& omega) {

	this->omega = omega;

}

void RigidBodySimulator::setInertia(const Vector3f& inertia) {

	assert(inertia.getX() > 0.0f && inertia.getY() > 0.0f
			&& inertia.getZ() > 0.0f && "Error: Inertia components must be positive");

	this->inertia = inertia;

	compute_inverse_inertia();

}

void RigidBodySimulator::update(float dt, const Vector3f& torque) {

	assert(dt > 0.0f && "Error: Time step must be positive");

	assert(!isinf(torque.getX()) && !isinf(torque.getY()) && !isinf(torque.getZ())
		&& !isnan(torque.getX()) && !isnan(torque.getY()) && !isnan(torque.getZ())
		&& "Error: Torque components must be finite");

	// I·ω
	Vector3f inertiaOmega = componentMultiply(inertia, omega);

	// ω × (I·ω)
	Vector3f gyroscopicTerm = cross(omega, inertiaOmega);

	// net torque: τ - ω × (I·ω)
	Vector3f netTorque = torque - gyroscopicTerm;

	// angular acceleration: α = I^(-1) * netTorque
	Vector3f angularAcceleration = componentMultiply(inverseInertia, netTorque);

	// euler integration for angular velocity: ω = ω + α * dt
	omega = omega + angularAcceleration * dt;

	assert(!isinf(omega.getX()) && !isinf(omega.getY()) && !isinf(omega.getZ())
		&& !isnan(omega.getX()) && !isnan(omega.getY()) && !isnan(omega.getZ())
		&& "Error: Angular velocity components must be finite after update");

	pitch += omega.getX() * dt;
	yaw += omega.getY() * dt;
	roll += omega.getZ() * dt;

	assert(!isinf(pitch) && !isinf(yaw) && !isinf(roll)
		&& !isnan(pitch) && !isnan(yaw) && !isnan(roll)
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
