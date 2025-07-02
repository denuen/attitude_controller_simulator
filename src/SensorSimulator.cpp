#include "SensorSimulator.hpp"
#include <iostream>
#include <cassert>



SensorSimulator::SensorSimulator(RigidBodySimulator* source) :
driftOmega(0.0f, 0.0f, 0.0f), driftAngles(0.0f, 0.0f, 0.0f),
driftRate(0.0f, 0.0f, 0.0f), noiseStdDev(0.0f, 0.0f, 0.0f), source(source) {

	assert(source != nullptr && "Error: SensorSimulator constructor: source must be a non-null pointer");

}

SensorSimulator::SensorSimulator(const SensorSimulator& sensorSimulator) :
driftOmega(sensorSimulator.driftOmega), driftAngles(sensorSimulator.driftAngles),
driftRate(sensorSimulator.driftRate), noiseStdDev(sensorSimulator.noiseStdDev),
source(sensorSimulator.source) {

}

SensorSimulator&	SensorSimulator::operator=(const SensorSimulator& sensorSimulator) {

	if (this != &sensorSimulator) {
		driftOmega = sensorSimulator.driftOmega;
		driftAngles = sensorSimulator.driftAngles;
		noiseStdDev = sensorSimulator.noiseStdDev;
		source = sensorSimulator.source;
	}

	return (*this);
}

void	SensorSimulator::update(float dt) {

	assert(dt > 0.0f && "Error: SensorSimulator::update: dt must be positive");
	driftOmega.assertVectorCheck();
	driftAngles.assertVectorCheck();
	driftRate.assertVectorCheck();

	driftOmega = driftOmega + driftRate * dt;
	driftAngles = driftAngles + driftRate * dt;

	driftOmega.assertVectorCheck();
	driftAngles.assertVectorCheck();
	driftRate.assertVectorCheck();
}

Vector3f	SensorSimulator::readAngularVelocity() const {
	Vector3f	noise;
	Vector3f	realOmega;

	realOmega = source->getOmega();

	/*Gaussian noise should take (mean, stddev) as parameters and should
	/*generate a random gaussian number with mean=0 and stddev */
	noise.setX(/*Gaussian noise*/);
	noise.setY(/*Gaussian noise*/);
	noise.setZ(/*Gaussian noise*/);

	return (realOmega + driftOmega + noise);
}
