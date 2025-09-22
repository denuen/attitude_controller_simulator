#include "../../includes/sensor/SensorSimulator.hpp"
#include <cassert>
#include <stdio.h>
#include <cmath>

SensorSimulator::SensorSimulator(RigidBodySimulator* source) :
driftOmega_(0.0f), driftAngles_(0.0f), driftRate_(0.0f),
noiseStdDev_(0.0f), source_(source), noiseGenerator_() {
	assert(source != NULL
		&& "Error: source must be a non-null pointer");
}

SensorSimulator::SensorSimulator(const SensorSimulator& s) :
driftOmega_(s.driftOmega_), driftAngles_(s.driftAngles_),
driftRate_(s.driftRate_), noiseStdDev_(s.noiseStdDev_),
source_(s.source_), noiseGenerator_(s.noiseGenerator_){

}

SensorSimulator&	SensorSimulator::operator=(const SensorSimulator& s) {
	if (this != &s) {
		driftOmega_ = s.driftOmega_;
		driftAngles_ = s.driftAngles_;
		driftRate_ = s.driftRate_;
		noiseStdDev_ = s.noiseStdDev_;
		source_ = s.source_;
		noiseGenerator_ = s.noiseGenerator_;
	}
	return (*this);
}

void	SensorSimulator::update(float dt) {
	assert(!std::isnan(dt) && !std::isinf(dt)
		&& dt > 0.0f && "Error: dt must be finite and non-negative");

	driftOmega_.assertCheck();
	driftAngles_.assertCheck();
	driftRate_.assertCheck();

	driftOmega_	= driftOmega_ + driftRate_ * dt;
	driftAngles_ = driftAngles_ + driftRate_ * dt;

	driftOmega_.assertCheck();
	driftAngles_.assertCheck();
	driftRate_.assertCheck();
}

Vector3f	SensorSimulator::getMeasuredOmega() const {
	assert(source_ != NULL
		&& "Error: source must be a non-null pointer");

	Vector3f	noise;
	Vector3f	res;

	noise.setX(noiseGenerator_.generate(0.0f, noiseStdDev_.getX()));
	noise.setY(noiseGenerator_.generate(0.0f, noiseStdDev_.getY()));
	noise.setZ(noiseGenerator_.generate(0.0f, noiseStdDev_.getZ()));

	noise.assertCheck();

	res = source_->getOmega() + driftOmega_ + noise;
	res.assertCheck();

	return (res);
}

Vector3f	SensorSimulator::getMeasuredOrientation() const {
	assert(source_ != NULL
		&& "Error: source must be a non-null pointer");

	Vector3f	noise;
	Vector3f	angles(source_->getPitch(), source_->getYaw(), source_->getRoll());
	Vector3f	res;

	noise.setX(noiseGenerator_.generate(0.0f, noiseStdDev_.getX()));
	noise.setY(noiseGenerator_.generate(0.0f, noiseStdDev_.getY()));
	noise.setZ(noiseGenerator_.generate(0.0f, noiseStdDev_.getZ()));

	noise.assertCheck();

	res = angles + driftAngles_ + noise;
	res.assertCheck();

	return (res);
}

bool	SensorSimulator::checkNumerics() const {
	if (!driftOmega_.checkNumerics() || !driftAngles_.checkNumerics()
		|| !driftRate_.checkNumerics() || !noiseStdDev_.checkNumerics() || !source_
		|| !source_->checkNumerics() || !noiseGenerator_.checkNumerics()) {
		return (0);
	}
	return (1);
}

void	SensorSimulator::reset() {
	driftOmega_.setVariables(0.0f, 0.0f, 0.0f);
	driftAngles_.setVariables(0.0f, 0.0f, 0.0f);
	driftRate_.setVariables(0.0f, 0.0f, 0.0f);
	noiseGenerator_.reset();
}

SensorSimulator::~SensorSimulator() {

}
