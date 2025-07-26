#include "../../includes/control/PID.hpp"
#include <iostream>
#include <cassert>
#include <math.h>

const float	PID::DEFAULT_INTEGRAL_MAX = 100.0f;
const float PID::DEFAULT_INTEGRAL_MIN = -100.0f;
const float	PID::DEFAULT_OUTPUT_MAX = 1000.0f;
const float PID::DEFAULT_OUTPUT_MIN = -1000.0f;

PID::PID() :
kp(0.0f), ki(0.0f), kd(0.0f), integral(0.0f), previousError(0.0f), filteredDerivative(0.0f), derivativeAlpha(1.0f) {

}

PID::PID(const float kp, const float ki, const float kd) :
kp(0.0f), ki(0.0f), kd(0.0f), integral(0.0f), previousError(0.0f), filteredDerivative(0.0f), derivativeAlpha(1.0f) {

	assert(!isinf(kp) && !isnan(kp)
		&& !isinf(ki) && !isnan(ki)
		&& !isinf(kd) && !isnan(kd)
		&& "Error: PID constructor with arguments: gain values must be finite");

	this->kp = kp;
	this->ki = ki;
	this->kd = kd;

}

PID::PID(const PID& pid) {

	kp = pid.kp;
	ki = pid.ki;
	kd = pid.kd;
	integral = pid.integral;
	previousError = pid.previousError;

}

PID&	PID::operator=(const PID& pid) {

	if (this != &pid) {
		kp = pid.kp;
		ki = pid.ki;
		kd = pid.kd;
		integral = pid.integral;
		previousError = pid.previousError;
	}

	return (*this);
}

bool	PID::operator==(const PID& pid) const {

	static const float epsilon = 1e-6f;

	return (std::fabs(kp - pid.getKp()) < epsilon
			&& std::fabs(ki - pid.getKi()) < epsilon
			&& std::fabs(kd - pid.getKd()) < epsilon
			&& std::fabs(integral - pid.getIntegral()) < epsilon
			&& std::fabs(previousError - pid.getPrevErr()) < epsilon
			&& std::fabs(derivativeAlpha - pid.getDerivativeSmoothing()) < epsilon);
}

void	PID::setKp(const float kp) {

	assert(!isinf(kp) && !isnan(kp) && "Error: PID setKp: value must be finite");
	this->kp = kp;

}

void	PID::setKi(const float ki) {

	assert(!isinf(ki) && !isnan(ki) && "Error: PID setKi: value must be finite");
	this->ki = ki;

}

void	PID::setKd(const float kd) {

	assert(!isinf(kd) && !isnan(kd) && "Error: PID setKd: value must be finite");
	this->kd = kd;

}

void	PID::setGains(const float kp, const float ki, const float kd) {

		assert(!isinf(kp) && !isnan(kp)
		&& !isinf(ki) && !isnan(ki)
		&& !isinf(kd) && !isnan(kd)
		&& "Error: PID constructor with arguments: gain values must be finite");

		this->kp = kp;
		this->ki = ki;
		this->kd = kd;

}

void	PID::setDerivativeSmoothing(float alpha) {

	assert(alpha >= 0.0f && alpha <= 1.0f && "Error: alpha must be in [0,1]");

	derivativeAlpha = alpha;

}

bool	PID::checkNumerics(void) const{

	if (isnan(kp) || isinf(kp)
		|| isnan(ki) || isinf(ki)
		|| isnan(kd) || isinf(kd)
		|| isnan(integral) || isinf(integral)
		|| isnan(previousError) || isinf(previousError)
		|| isnan(filteredDerivative) || isinf(filteredDerivative)
		|| isnan(derivativeAlpha) || isinf(derivativeAlpha)) {
			return (0);
	}
	return (1);
}

float	PID::compute(const float setpoint, const float measure, const float dt) {

	float	ek;
	float	rawDerivative;
	float	result;

	assert(dt > 0.0f && "Error: dt must be positive");

	// Error at the time t
	ek = setpoint - measure;
	assert(!isinf(ek) && !isnan(ek) && "Error: the error value at the k-iteration must be finite");

	// Riemann sum in discrete time
	integral += ek * dt;
	if (integral > DEFAULT_INTEGRAL_MAX)
		integral = DEFAULT_INTEGRAL_MAX;
	else if (integral < DEFAULT_INTEGRAL_MIN)
		integral = DEFAULT_INTEGRAL_MIN;

	// Finite filtered derivate in discrete time
	rawDerivative = (ek - previousError) / dt;
	filteredDerivative = derivativeAlpha * rawDerivative + (1.0f - derivativeAlpha) * filteredDerivative;

	// u[k] = Kp * e[k] + Ki * ∑(e[i] * dt) + Kd * (e[k] - e[k - 1]) / dt
	result = kp * ek + ki * integral + kd * filteredDerivative;
	if (result > DEFAULT_OUTPUT_MAX)
		result = DEFAULT_OUTPUT_MAX;
	else if (result < DEFAULT_OUTPUT_MIN)
		result = DEFAULT_OUTPUT_MIN;

	previousError = ek;

	assert(!isinf(result) && !isnan(result) && "Error: PID output invalid");
	return (result);

}

void	PID::reset(void) {

	integral = 0.0f;
	previousError = 0.0f;

}


PID::~PID() {

}
