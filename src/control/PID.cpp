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
	assert(dt > 0.0f && "Error: dt must be positive");

	// Discrete PID implementation (Implicit Backward Euler)
	// Error at the time t (e(t) = r(t) - y(t))
	const float error = setpoint - measure;
	assert(!std::isinf(error) && !std::isnan(error) && "Error: invalid error");

	// Derivative (finite difference) with optional exponential smoothing
	// Exponential moving average (EMA) filter to reduce the derivative noise
	// filteredDerivative = alpha * filteredDerivative_prev + (1 - alpha) * rawDerivative
	const float rawDerivative = (error - previousError_) / dt;
	filteredDerivative_ = derivativeAlpha_ * filteredDerivative_
						+ (1.0f - derivativeAlpha_) * rawDerivative;

	// Proportional and derivative terms, integral independent
	const float proportionalTerm = kp_ * error;
	const float derivativeTerm = kd_ * filteredDerivative_;

	float newIntegral = integral_;

	if (ki_ > 0.0f) {
		// Implicit solution
		// I[k+1] = ( I[k] + dt*e[k] + (dt/tau)*((uSat - (Kp*e + Kd*d))/Ki) ) / (1 + dt/tau)

		// First estimate without integral (pdTerm = Kp*e + Kd*d)
		const float pdTerm = proportionalTerm + derivativeTerm;

		// Ensure tau is reasonable to prevent numerical issues
		const float effectiveTau = std::max(antiWindupTau_, dt * 10.0f);

		// Without saturation, the output would be pdTerm + Ki*Inew
		// To solve with the implicit approach the saturation expressed in function of
		// Inew is needed.
		// Try both the cases: saturated and not saturated

		// Non-saturated case
		const float integralCandidate = (integral_ + dt * error) / (1.0f + (dt / effectiveTau));
		const float unSatOutputCandidate = pdTerm + ki_ * integralCandidate;

		float satOutput = std::min(std::max(unSatOutputCandidate, PID::DEFAULT_OUTPUT_MIN),
							PID::DEFAULT_OUTPUT_MAX);

		if (unSatOutputCandidate == satOutput) {
			// No saturation
			newIntegral = integralCandidate;
		} else {
			// Saturation case: implicit backward + clamping
			// I[k+1] = ( I[k] + dt*e[k] + (dt/tau)*((uSat - pdTerm)/Ki) ) / (1 + dt/tau)
			newIntegral = (integral_ + dt * error
				+ (dt / effectiveTau) * ((satOutput - pdTerm) / ki_))
				/ (1.0f + (dt / effectiveTau));
		}
	}

	// Clamp
	newIntegral = std::min(std::max(newIntegral, PID::DEFAULT_INTEGRAL_MIN),
					PID::DEFAULT_INTEGRAL_MAX);

	integral_ = newIntegral;

	// Final output
	float output = proportionalTerm + ki_ * integral_ + derivativeTerm;

	// Final saturation
	output = std::min(std::max(output, PID::DEFAULT_OUTPUT_MIN),
					PID::DEFAULT_OUTPUT_MAX);

	previousError_ = error;
	assert(!std::isinf(output) && !std::isnan(output) && "Error: invalid PID output");

	return (output);
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
