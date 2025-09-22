#ifndef SENSOR_SIMULATOR_HPP
#define SENSOR_SIMULATOR_HPP

#include "../physics/Vector3f.hpp"
#include "../physics/RigidBodySimulator.hpp"
#include "GaussianNoise.hpp"

// Simulates IMU sensor readings with realistic drift accumulation and Gaussian noise
class SensorSimulator {

	private:
		Vector3f						driftOmega_; // Accumulated angular velocity drift
		Vector3f						driftAngles_; // Accumulated orientation drift
		Vector3f						driftRate_; // Drift rate for each component (per second)
		Vector3f						noiseStdDev_; // Standard deviation of Gaussian noise per component

		RigidBodySimulator*				source_; // Pointer to the simulated rigid body
		mutable GaussianNoiseGenerator	noiseGenerator_; // Noise generator

	public:
		// Construct a sensor simulator for the given rigid body
		SensorSimulator(RigidBodySimulator* source);

		// Copy constructor
		SensorSimulator(const SensorSimulator& s);

		// Copy assignment operator
		SensorSimulator&	operator=(const SensorSimulator& s);

		// Sets the current accumulated angular velocity drift (rad/s)
		inline void			setDriftOmega(const Vector3f& drift) { driftOmega_ = drift; }

		// Sets the current accumulated orientation drift (rad)
		inline void			setDriftAngles(const Vector3f& drift) { driftAngles_ = drift; }

		// Sets the standard deviation of Gaussian noise per component
		inline void			setNoiseStdDev(const Vector3f& stddev) { noiseStdDev_ = stddev; }

		// Sets the rate of drift applied per second
		inline void			setDriftRate(const Vector3f& rate) { driftRate_ = rate; }

		// Returns the accumulated angular velocity drift (rad/s)
		inline Vector3f		getDriftOmega() const { return (driftOmega_); }

		// Returns the accumulated angles drift (rad)
		inline Vector3f		getDriftAngles() const { return (driftAngles_); }

		// Returns the configured drift rate (accumulation rate)
		inline Vector3f		getDriftRate() const { return (driftRate_); }

		// Returns the standard deviation used to generate the Gaussian noise
		inline Vector3f		getNoiseStdDev() const { return (noiseStdDev_); }

		// Returns the angular velocity measured by the sensor (rad/s)
		Vector3f			getMeasuredOmega() const;

		// Returns the angles measured by the sensor in (pitch, yaw, roll) order
		Vector3f			getMeasuredOrientation() const;

		// Accumulates drift over time based on configured drift rates
		void				update(float dt);

		/*
			Checks the correctness of all the required values of the module.
			Returns 1 if they are correct, 0 otherwise.
		*/
		bool				checkNumerics() const;

		// Resets all drift accumulators to zero and clears noise generator state
		void				reset();

		// Default destructor
		~SensorSimulator();

};

#endif
