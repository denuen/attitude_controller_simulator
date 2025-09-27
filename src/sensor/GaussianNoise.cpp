#include "../../includes/sensor/GaussianNoise.hpp"
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>

GaussianNoiseGenerator::GaussianNoiseGenerator() : spare_(0.0f), hasSpare_(false) {
}

GaussianNoiseGenerator::GaussianNoiseGenerator(const GaussianNoiseGenerator& g) :
spare_(g.spare_), hasSpare_(g.hasSpare_) {
}

GaussianNoiseGenerator&	GaussianNoiseGenerator::operator=(const GaussianNoiseGenerator& g) {
	if (this != &g) {
		spare_	  = g.spare_;
		hasSpare_ = g.hasSpare_;
	}
	return (*this);
}

float	GaussianNoiseGenerator::generate(float mean, float stddev) {
	assert(stddev >= 0.0f && "Error: Standard deviation must be non-negative");
	assert(!std::isnan(mean) && !std::isinf(mean)
		&& "Error: Mean must be finite");
	assert(!std::isnan(stddev) && !std::isinf(stddev)
		&& "Error: Standard deviation must be finite");

	const float	epsilon = 1e-6f;
	float		u, v, s;
	float		res;

	if (hasSpare_) {
		hasSpare_ = false;
		res = mean + spare_ * stddev;

		assert(!std::isnan(res) && !std::isinf(res)
			&& "Error: Generated value must be finite");
		return (res);
	}

	do {
		u = 2.0f * (static_cast<float>(std::rand())
			/ static_cast<float>(RAND_MAX)) - 1.0f;
		v = 2.0f * (static_cast<float>(std::rand())
			/ static_cast<float>(RAND_MAX)) - 1.0f;
		s = u * u + v * v;
	} while (s >= 1.0f || s <= epsilon);

	float factor = sqrt(-2.0f * log(s) / s);
	spare_ = v * factor;
	hasSpare_ = true;

	res = mean + stddev * u * factor;

	assert(!std::isnan(res) && !std::isinf(res)
		&& "Error: Generated value must be finite");

	return (res);
}

void	GaussianNoiseGenerator::initSeed() {
	std::srand(static_cast<unsigned int>(std::time(NULL)));
}

bool	GaussianNoiseGenerator::checkNumerics() {
	if (std::isnan(spare_) || std::isinf(spare_)) {
		return (0);
	}
	return (1);
}

GaussianNoiseGenerator::~GaussianNoiseGenerator() {
}
