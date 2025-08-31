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
	driftOmega.assertCheck();
	driftAngles.assertCheck();
	driftRate.assertCheck();

	driftOmega = driftOmega + driftRate * dt;
	driftAngles = driftAngles + driftRate * dt;

	driftOmega.assertCheck();
	driftAngles.assertCheck();
	driftRate.assertCheck();

}

Vector3f	SensorSimulator::readAngularVelocity(void) const {

	assert(source != NULL && "Error: SensorSimulator::readAngularVelocity: source pointer is null");

	Vector3f	noise;
	Vector3f	result;
	Vector3f	realOmega;

	realOmega = source->getOmega();
	realOmega.assertCheck();

	// Generate Gaussian noise for each axis with mean=0 and stddev from noiseStdDev
	noise.setX(noiseGenerator.generate(0.0f, noiseStdDev.getX()));
	noise.setY(noiseGenerator.generate(0.0f, noiseStdDev.getY()));
	noise.setZ(noiseGenerator.generate(0.0f, noiseStdDev.getZ()));

	noise.assertCheck();

	result = realOmega + driftOmega + noise;
	result.assertCheck();

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

	realAngles.assertCheck();

	// Generate Gaussian noise for each axis with mean=0 and stddev from noiseStdDev
	noise.setX(noiseGenerator.generate(0.0f, noiseStdDev.getX()));
	noise.setY(noiseGenerator.generate(0.0f, noiseStdDev.getY()));
	noise.setZ(noiseGenerator.generate(0.0f, noiseStdDev.getZ()));

	noise.assertCheck();

	result = realAngles + driftAngles + noise;
	result.assertCheck();

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

	driftOmega.assertCheck();
	driftAngles.assertCheck();
	driftRate.assertCheck();

}

SensorSimulator::~SensorSimulator() {

}
