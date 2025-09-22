#include "../../includes/physics/RigidBodySimulator.hpp"
#include <cassert>
#include <cmath>

RigidBodySimulator::RigidBodySimulator() :
pitch_(0.0f), yaw_(0.0f), roll_(0.0f), dt_(0.0f),
omega_(0.0f, 0.0f, 0.0f), inertia_(1.0f, 1.0f, 1.0f) {
	computeInverseInertia();
}

RigidBodySimulator::RigidBodySimulator(const Vector3f& inertia) :
pitch_(0.0f), yaw_(0.0f), roll_(0.0f), dt_(0.0f),
omega_(0.0f, 0.0f, 0.0f), inertia_(inertia) {
	computeInverseInertia();
}

RigidBodySimulator::RigidBodySimulator(const RigidBodySimulator& rbs) :
pitch_(rbs.pitch_), yaw_(rbs.yaw_), roll_(rbs.roll_), dt_(rbs.dt_),
omega_(rbs.omega_), inertia_(rbs.inertia_),
inverseInertia_(rbs.inverseInertia_) {
}

RigidBodySimulator& RigidBodySimulator::operator=(const RigidBodySimulator& rbs) {
	if (this != &rbs) {
		pitch_ = rbs.pitch_;
		yaw_ = rbs.yaw_;
		roll_ = rbs.roll_;
		dt_ = rbs.dt_;
		omega_ = rbs.omega_;
		inertia_ = rbs.inertia_;
		inverseInertia_ = rbs.inverseInertia_;
	}
	return (*this);
}

void RigidBodySimulator::setPitch(float pitch) {
	assert(!std::isinf(pitch) && !std::isnan(pitch)
		&& "Error: Pitch value must be finite");
	pitch_ = pitch;
}

void RigidBodySimulator::setYaw(float yaw) {
	assert(!std::isinf(yaw) && !std::isnan(yaw)
		&& "Error: Yaw value must be finite");
	yaw_ = yaw;
}

void RigidBodySimulator::setRoll(float roll) {
	assert(!std::isinf(roll) && !std::isnan(roll)
		&& "Error: Roll value must be finite");
	roll_ = roll;
}

void RigidBodySimulator::setInertia(const Vector3f& inertia) {
	assert(inertia.getX() > 0.0f && inertia.getY() > 0.0f
		&& inertia.getZ() > 0.0f
		&& "Error: Inertia components must be positive");

	inertia_ = inertia;
	computeInverseInertia();
}

void RigidBodySimulator::update(float dt, const Vector3f& torque) {
	assert(!std::isnan(dt) && !std::isinf(dt) && dt > 0.0f
		&& "Error: time step 'dt' must be a finite positive number");
	assert(torque.checkNumerics() && "Error: Torque components must be finite");

	updateAngularVelocity(dt, torque);
	updateEulerAngles(dt);
	normalizeAngles();

	dt_ = dt;
}

void RigidBodySimulator::updateAngularVelocity(float dt, const Vector3f& torque) {
	// Compute angular momentum: I·ω
	const Vector3f angularMomentum = componentMultiply(inertia_, omega_);

	// Compute gyroscopic term: ω × (I·ω)
	const Vector3f gyroscopicTerm = cross(omega_, angularMomentum);

	// Net torque: τ - ω × (I·ω)
	const Vector3f netTorque = torque - gyroscopicTerm;

	// Angular acceleration: α = I^(-1) * netTorque
	const Vector3f angularAcceleration = componentMultiply(inverseInertia_, netTorque);

	// Euler integration: ω = ω_0 + α * dt
	omega_ = omega_ + angularAcceleration * dt;

	assert(omega_.checkNumerics()
		&& "Error: angular velocity components must be finite after update");
}

void RigidBodySimulator::updateEulerAngles(float dt) {
	const float epsilon	 = 1e-6f;

	// Body angular rates (p, q, r)
	const float p = omega_.getX();
	const float q = omega_.getY();
	const float r = omega_.getZ();

	// Current Euler angles
	const float phi	  = roll_;
	const float theta = pitch_;

	// Trigonometric values
	const float sinPhi	 = sinf(phi);
	const float cosPhi	 = cosf(phi);
	float		cosTheta = cosf(theta);

	// The tangent is computed after a check on the cosine value to avoid
	// division by (near)zero when theta is near pi/2
	if (std::fabs(cosTheta) < epsilon) {
		if (cosTheta >= 0)
			cosTheta = epsilon;
		else
			cosTheta = -epsilon;
	}

	const float tanTheta = sinf(theta) / cosTheta;

	// Compute Euler angle derivatives using 3-2-1 transformation
	const float phiDot = p + (q * sinPhi + r * cosPhi) * tanTheta;
	const float thetaDot = q * cosPhi - r * sinPhi;
	const float psiDot = (q * sinPhi + r * cosPhi) / cosTheta;

	// Integrate Euler angles
	roll_ += phiDot * dt;
	pitch_ += thetaDot * dt;
	yaw_ += psiDot * dt;

	assert(!std::isinf(pitch_) && !std::isinf(yaw_) && !std::isinf(roll_)
		&& !std::isnan(pitch_) && !std::isnan(yaw_) && !std::isnan(roll_)
		&& "Error: angle components must be finite after update");
}

void	RigidBodySimulator::computeInverseInertia() {
	assert(inertia_.getX() > 0.0f && inertia_.getY() > 0.0f
		&& inertia_.getZ() > 0.0f
		&& "Error: Inertia components must be positive");

	inverseInertia_.setX(1.0f / inertia_.getX());
	inverseInertia_.setY(1.0f / inertia_.getY());
	inverseInertia_.setZ(1.0f / inertia_.getZ());
}

void	RigidBodySimulator::normalizeAngles() {
	const float PI = 3.14159265359f;
	const float TWO_PI = 2.0f * PI;

	// [-π, π] range
	while (pitch_ > PI)
		pitch_ -= TWO_PI;
	while (pitch_ < -PI)
		pitch_ += TWO_PI;

	while (yaw_ > PI)
		yaw_ -= TWO_PI;
	while (yaw_ < -PI)
		yaw_ += TWO_PI;

	while (roll_ > PI)
		roll_ -= TWO_PI;
	while (roll_ < -PI)
		roll_ += TWO_PI;
}

bool RigidBodySimulator::checkNumerics() const {
	if (std::isnan(pitch_) || std::isinf(pitch_) || std::isnan(yaw_)
		|| std::isinf(yaw_) || std::isnan(roll_) || std::isinf(roll_)
		|| std::isnan(dt_) || std::isinf(dt_) || dt_ < 0.0f
		|| !omega_.checkNumerics() || !inertia_.checkNumerics()
		|| !inverseInertia_.checkNumerics() || inertia_.getX() <= 0.0f
		|| inertia_.getY() <= 0.0f || inertia_.getZ() <= 0.0f) {
		return (0);
	}
	return (1);
}

RigidBodySimulator::~RigidBodySimulator() {

}
