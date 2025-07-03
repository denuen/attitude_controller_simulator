#ifndef SENSORSIMULATOR_HPP
#define SENSORSIMULATOR_HPP

#include "../physics/Vector3f.hpp"
#include "../physics/RigidBodySimulator.hpp"

class SensorSimulator {

	private:
		Vector3f			driftOmega;
		Vector3f			driftAngles;
		Vector3f			driftRate;
		Vector3f			noiseStdDev;

		RigidBodySimulator*	source;

	public:
		SensorSimulator(RigidBodySimulator* source);
		SensorSimulator(const SensorSimulator& sensorSimulator);

		SensorSimulator&	operator=(const SensorSimulator& sensorSimulator);

		inline void			setNoiseStdDev(const Vector3f& stddev) { noiseStdDev = stddev; }
		inline void			setDriftRate(const Vector3f& rate) { driftRate = rate; }

		void				update(float dt);

		Vector3f			readAngularVelocity() const;
		Vector3f			readOrientation() const;

		void				reset();

		~SensorSimulator();
};


#endif
