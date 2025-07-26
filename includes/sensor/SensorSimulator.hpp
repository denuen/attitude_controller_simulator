#ifndef SENSORSIMULATOR_HPP
#define SENSORSIMULATOR_HPP

#include "../physics/Vector3f.hpp"
#include "../physics/RigidBodySimulator.hpp"
#include "GaussianNoise.hpp"

// Simulates IMU sensor readings with realistic drift accumulation and Gaussian noise.
class SensorSimulator {

	private:
		Vector3f						driftOmega;		// Accumulated angular velocity drift.
		Vector3f						driftAngles;	// Accumulated orientation drift.
		Vector3f						driftRate;		// Drift rate per second for each component.
		Vector3f						noiseStdDev;	// Standard deviation of Gaussian noise per component.

		RigidBodySimulator*				source;			// Pointer to the simulated rigid body.
		mutable GaussianNoiseGenerator	noiseGenerator;	// Noise generator (mutable for const read methods).

	public:
		// Constructs a sensor simulator for the given rigid body.
		SensorSimulator(RigidBodySimulator* source);

		// Copy constructor.
		SensorSimulator(const SensorSimulator& sensorSimulator);

		// Copy assignment operator.
		SensorSimulator&	operator=(const SensorSimulator& sensorSimulator);

		// Sets the standard deviation of Gaussian noise.
		inline void			setNoiseStdDev(const Vector3f& stddev) { noiseStdDev = stddev; }

		// Sets the rate of drift applied per second.
		inline void			setDriftRate(const Vector3f& rate) { driftRate = rate; }

		// Accumulates drift over time based on configured drift rates.
		void				update(float dt);

		// Reads angular velocity from source with applied drift and per-axis Gaussian noise.
		Vector3f			readAngularVelocity() const;

		// Reads Euler angles from source with applied drift and per-axis Gaussian noise.
		Vector3f			readOrientation() const;

		bool				checkNumerics() const;
		
		// Zeroes all drift accumulators and clears noise generator cached values.
		void				reset();

		// Destructor.
		~SensorSimulator();
};

#endif
