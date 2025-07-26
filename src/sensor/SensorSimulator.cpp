#include "../../includes/sensor/SensorSimulator.hpp"
#include <iostream>
#include <cassert>
#include <cstddef>


SensorSimulator::SensorSimulator(RigidBodySimulator* source) :
driftOmega(0.0f, 0.0f, 0.0f), driftAngles(0.0f, 0.0f, 0.0f),
driftRate(0.0f, 0.0f, 0.0f), noiseStdDev(0.0f, 0.0f, 0.0f), source(source), noiseGenerator() {

	assert(source != NULL && "Error: SensorSimulator constructor: source must be a non-null pointer");

}

SensorSimulator::SensorSimulator(const SensorSimulator& sensorSimulator) :
driftOmega(sensorSimulator.driftOmega), driftAngles(sensorSimulator.driftAngles),
driftRate(sensorSimulator.driftRate), noiseStdDev(sensorSimulator.noiseStdDev),
source(sensorSimulator.source), noiseGenerator(sensorSimulator.noiseGenerator) {

}

SensorSimulator&	SensorSimulator::operator=(const SensorSimulator& sensorSimulator) {

	if (this != &sensorSimulator) {
		driftOmega = sensorSimulator.driftOmega;
		driftAngles = sensorSimulator.driftAngles;
		driftRate = sensorSimulator.driftRate;
		noiseStdDev = sensorSimulator.noiseStdDev;
		source = sensorSimulator.source;
		noiseGenerator = sensorSimulator.noiseGenerator;
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

Vector3f	SensorSimulator::readAngularVelocity(void) const {

	assert(source != NULL && "Error: SensorSimulator::readAngularVelocity: source pointer is null");

	Vector3f	noise;
	Vector3f	result;
	Vector3f	realOmega;

	realOmega = source->getOmega();
	realOmega.assertVectorCheck();

	// Generate Gaussian noise for each axis with mean=0 and stddev from noiseStdDev
	noise.setX(noiseGenerator.generate(0.0f, noiseStdDev.getX()));
	noise.setY(noiseGenerator.generate(0.0f, noiseStdDev.getY()));
	noise.setZ(noiseGenerator.generate(0.0f, noiseStdDev.getZ()));

	noise.assertVectorCheck();

	result = realOmega + driftOmega + noise;
	result.assertVectorCheck();

	return (result);
}

Vector3f	SensorSimulator::readOrientation() const {

	assert(source != NULL && "Error: SensorSimulator::readOrientation: source pointer is null");

	Vector3f	noise;
	Vector3f	result;
	Vector3f	realAngles(
		source->getPitch(),
		source->getYaw(),
		source->getRoll()
	);

	realAngles.assertVectorCheck();

	// Generate Gaussian noise for each axis with mean=0 and stddev from noiseStdDev
	noise.setX(noiseGenerator.generate(0.0f, noiseStdDev.getX()));
	noise.setY(noiseGenerator.generate(0.0f, noiseStdDev.getY()));
	noise.setZ(noiseGenerator.generate(0.0f, noiseStdDev.getZ()));

	noise.assertVectorCheck();

	result = realAngles + driftAngles + noise;
	result.assertVectorCheck();

	return (result);
}

bool	SensorSimulator::checkNumerics(void) const{

	if (!driftOmega.checkNumerics() || !driftAngles.checkNumerics()
		|| !driftRate.checkNumerics() || !noiseStdDev.checkNumerics()
		|| !source || !source->checkNumerics() || !noiseGenerator.checkNumerics()) {
			return (0);
	}
	return (1);
}

void	SensorSimulator::reset() {

	driftOmega.setVariables(0.0f, 0.0f, 0.0f);
	driftAngles.setVariables(0.0f, 0.0f, 0.0f);
	driftRate.setVariables(0.0f, 0.0f, 0.0f);

	noiseGenerator.reset();

	driftOmega.assertVectorCheck();
	driftAngles.assertVectorCheck();
	driftRate.assertVectorCheck();

}

SensorSimulator::~SensorSimulator() {

}
