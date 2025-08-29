#include "../../includes/control/PID.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>

const float	PID::DEFAULT_INTEGRAL_MAX = 100.0f;
const float	PID::DEFAULT_INTEGRAL_MIN = -100.0f;
const float	PID::DEFAULT_OUTPUT_MAX = 1000.0f;
const float	PID::DEFAULT_OUTPUT_MIN = -1000.0f;

PID::PID() :
kp_(0.0f), ki_(0.0f), kd_(0.0f), integral_(0.0f), antiWindupTau_(0.1f),
previousError_(0.0f), filteredDerivative_(0.0f), derivativeAlpha_(1.0f) {

}

PID::PID(float kp, float ki, float kd) :
kp_(kp), ki_(ki), kd_(kd), integral_(0.0f), antiWindupTau_(0.1f),
previousError_(0.0f), filteredDerivative_(0.0f), derivativeAlpha_(1.0f) {
	assert(!std::isinf(kp_) && !std::isnan(kp_) && !std::isinf(ki_)
		&& !std::isnan(ki_) && !std::isinf(kd_) && !std::isnan(kd_)
		&& "Error: gain values must be finite");
}

PID::PID(const PID& pid) : filteredDerivative_(0.0f) {
	kp_ = pid.kp_;
	ki_ = pid.ki_;
	kd_ = pid.kd_;
	integral_ = pid.integral_;
	antiWindupTau_ = pid.antiWindupTau_;
	previousError_ = pid.previousError_;
	derivativeAlpha_ = pid.derivativeAlpha_;
}

PID&	PID::operator=(const PID& pid) {
	if (this != &pid) {
		kp_ = pid.kp_;
		ki_ = pid.ki_;
		kd_ = pid.kd_;
		integral_ = pid.integral_;
		antiWindupTau_ = pid.antiWindupTau_;
		previousError_ = pid.previousError_;
		derivativeAlpha_ = pid.derivativeAlpha_;
	}
	return (*this);
}

bool	PID::operator==(const PID& pid) const {
	static const float	epsilon = 1e-6f;

	return (std::fabs(kp_ - pid.kp_) < epsilon
			&& std::fabs(ki_ - pid.ki_) < epsilon
			&& std::fabs(kd_ - pid.kd_) < epsilon
			&& std::fabs(integral_ - pid.integral_) < epsilon
			&& std::fabs(previousError_ - pid.previousError_) < epsilon
			&& std::fabs(derivativeAlpha_ - pid.derivativeAlpha_) < epsilon);
}

void	PID::setKp(float kp) {
	assert(!std::isinf(kp) && !std::isnan(kp)
		&& "Error: kp value must be finite");
	kp_ = kp;
}

void PID::setKi(float ki) {
	assert(!std::isinf(ki) && !std::isnan(ki)
		&& "Error: ki value must be finite");
	ki_ = ki;
}

void PID::setKd(float kd) {
	assert(!std::isinf(kd) && !std::isnan(kd)
		&& "Error: kd value must be finite");
	kd_ = kd;
}

void PID::setGains(float kp, float ki, float kd) {
	assert(!std::isinf(kp) && !std::isnan(kp) && !std::isinf(ki)
		&& !std::isnan(ki) && !std::isinf(kd) && !std::isnan(kd)
		&& "Error: gain values must be finite");

	kp_ = kp;
	ki_ = ki;
	kd_ = kd;
}

void PID::setDerivativeSmoothing(float alpha) {
	assert(!std::isinf(alpha) && !std::isnan(alpha)
		&& "Error: alpha value must be finite");
	derivativeAlpha_ = alpha;
}

void PID::setAntiWindupTau(float tau) {
	assert(!std::isinf(tau) && !std::isnan(tau)
		&& "Error: tau value must be finite");
	antiWindupTau_ = tau;
}

float PID::compute(const float setpoint, const float measure, const float dt) {
	float	ek;
	float	rawDerivative;
	float	res;

	assert(dt > 0.0f && "Error: dt must be positive");

	// Discrete PID implementation (Backward Euler)
	// Error at the time t (e(t) = r(t) - y(t))
	ek = setpoint - measure;
	assert(!std::isinf(ek) && !std::isnan(ek)
		&& "Error: the error value at the k-iteration must be finite");

	/* Riemann sum in discrete time */
	// Derivative (finite difference) with optional exponential smoothing
	rawDerivative = (ek - previousError_) / dt;

	// Exponential moving average (EMA) filter to reduce the derivative noise
	// d_filt = α * d_prev + (1 - α) * d_new
	filteredDerivative_ = derivativeAlpha_ * filteredDerivative_
						+ (1.0f - derivativeAlpha_) * rawDerivative;

	// Compute unsaturated (ideal) output using current integral state
	// u[k] = Kp * e[k] + Ki * ∑(e[i] * dt) + Kd * (e[k] - e[k - 1]) / dt
	float unsatOutput = (kp_ * ek) + (ki_ * integral_) + (kd_ * filteredDerivative_);

	// Compute saturated output
	float satOutput = std::min(std::max(unsatOutput, PID::DEFAULT_OUTPUT_MIN),
							PID::DEFAULT_OUTPUT_MAX);

	// Back-calculation anti-windup:
	// Continuous form: I_dot = e + (1/Tt) * (satOutput - unsat) / ki  (when ki != 0)
	// Discretization: I[k+1] = I[k] + dt * (e + (1/Tt) * (satOutput - unsatOutput)/Ki)
	if (std::fabs(ki_) > 0.0f) {
		float correction = (satOutput - unsatOutput) / ki_;
		integral_ += dt * (ek + (1.0f / antiWindupTau_) * correction);
	} else {
		integral_ += dt * ek;
	}

	// Clamp integral to sensible limits to avoid numerical explosion
	integral_ = std::min(std::max(integral_, PID::DEFAULT_INTEGRAL_MIN),
						 PID::DEFAULT_INTEGRAL_MAX);

	// Recompute final controller output using the (possibly) updated integral
	res = (kp_ * ek) + ki_ * integral_ + (kd_ * filteredDerivative_);

	// Final saturation
	res = std::min(std::max(res, PID::DEFAULT_OUTPUT_MIN),
				PID::DEFAULT_OUTPUT_MAX);

	previousError_ = ek;
	assert(!std::isinf(res) && !std::isnan(res)
		&& "Error: invalid PID output");
		
	return (res);
}

void PID::reset() {
	integral_ = 0.0f;
	previousError_ = 0.0f;
	filteredDerivative_ = 0.0f;
}

bool PID::checkNumerics() const {
	if (std::isnan(kp_) || std::isinf(kp_)
		|| std::isnan(ki_) || std::isinf(ki_)
		|| std::isnan(kd_) || std::isinf(kd_)
		|| std::isnan(integral_) || std::isinf(integral_)
		|| std::isnan(previousError_) || std::isinf(previousError_)
		|| std::isnan(filteredDerivative_) || std::isinf(filteredDerivative_)
		|| std::isnan(antiWindupTau_) || std::isinf(antiWindupTau_)
		|| std::isnan(derivativeAlpha_) || std::isinf(derivativeAlpha_)
		|| antiWindupTau_ <= 0.0f
		|| derivativeAlpha_ < 0.0f || derivativeAlpha_ > 1.0f) {
			return (false);
		}
	return (true);
}

PID::~PID() {
}
