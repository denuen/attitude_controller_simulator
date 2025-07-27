#include "../../includes/sensor/GaussianNoise.hpp"

#include <cstdlib>
#include <math.h>
#include <ctime>
#include <cassert>
#include <cstddef>
#include <iostream>

GaussianNoiseGenerator::GaussianNoiseGenerator(void) :
hasSpare(false), spare(0.0f) {

}

GaussianNoiseGenerator::GaussianNoiseGenerator(const GaussianNoiseGenerator& gng) :
hasSpare(gng.hasSpare), spare(gng.spare) {

}

GaussianNoiseGenerator&	GaussianNoiseGenerator::operator=(const GaussianNoiseGenerator& gng) {

	if (this != &gng) {
		hasSpare = gng.hasSpare;
		spare = gng.spare;
	}
	return (*this);
}

/**
 *	Generates a random number from a Gaussian (normal) distribution using the Box-Muller transform.
 *
 *	This method implements the Box-Muller transform algorithm to generate normally distributed
 *	random numbers. It generates two independent standard normal variates per call and stores
 *	one for the next call to improve efficiency.
 *
 *	the param 'mean' is the mean (μ) of the desired Gaussian distribution
 *	the param 'stddev' is the standard deviation (σ) of the desired Gaussian distribution.
 *
 *	It returns random float value drawn from the Gaussian distribution N(mean, stddev²)
 *
 *	The implementation caches one generated value to be used in the next call,
 *	making it more efficient than generating one value at a time.
 *
 *	Algorithm details:
 *	- u, v: Two independent uniform random variables in the range [-1, 1]
 *	- s: The sum of squares (u² + v²), used to check if the point (u,v) lies within the unit circle
 *	- The algorithm rejects points outside the unit circle to ensure proper distribution
 **/
float	GaussianNoiseGenerator::generate(float mean, float stddev) {

	float	u;
	float	v;
	float	s;
	float	result;

	assert(stddev >= 0.0f && "Error: Standard deviation must be non-negative");
	assert(!std::isnan(mean) && !std::isinf(mean) && "Error: Mean must be finite");
	assert(!std::isnan(stddev) && !std::isinf(stddev) && "Error: Standard deviation must be finite");

	if (stddev == 0.0f) {
		return (mean);
	}

	if (hasSpare) {
		hasSpare = false;
		result = mean + stddev * spare;

		assert(!std::isnan(result) && !std::isinf(result) && "Error: Generated value must be finite");
		return (result);
	}

	do {
		u = 2.0f * (static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX)) - 1.0f;
		v = 2.0f * (static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX)) - 1.0f;
		s = u * u + v * v;
	} while (s >= 1.0f || s == 0.0f);

	float	factor = sqrt(-2.0f * log(s) / s);
	spare = v * factor;
	hasSpare = true;

	result = mean + stddev * u * factor;

	assert(!std::isnan(result) && !std::isinf(result) && "Error: Generated value must be finite");

	return (result);
}

void	GaussianNoiseGenerator::initSeed() {

	std::srand(static_cast<unsigned int>(std::time(NULL)));

}

bool	GaussianNoiseGenerator::checkNumerics(void) const {

	if (std::isnan(spare) || std::isinf(spare)) {
		return (0);
	}
	return (1);
}

GaussianNoiseGenerator::~GaussianNoiseGenerator(void) {

}
