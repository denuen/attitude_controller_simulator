#ifndef SENSORSIMULATOR_HPP
#define SENSORSIMULATOR_HPP

#include "Vector3f.hpp"
#include "RigidBodySimulator.hpp"

class SensorSimulator {

	private:
		Vector3f	driftOmega;
		Vector3f	drigtAngles;
		Vector3f	noiseStdDev;

		RigidBodySimulator*	source;

	public:
		SensorSimulator(RigidBodySimulator* source);
		SensorSimulator(const SensorSimulator& sensorSimulator);

		SensorSimulator&	operator=(const SensorSimulator& sensorSimulator);

		void	setNoiseStdDev(const Vector3f& stddev);
		void	setDriftRate(const Vector3f& rate);

		void	update(float dt);

		Vector3f	readAngularVelocity() const;
		Vector3f	readOrientation() const;

		void	reset();

		~SensorSimulator();
};


#endif
