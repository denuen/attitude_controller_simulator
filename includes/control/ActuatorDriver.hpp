#ifndef ACTUATORDRIVER_HPP
#define ACTUATORDRIVER_HPP

#include "../physics/Vector3f.hpp"
#include "../physics/RigidBodySimulator.hpp"
#include <queue>

class ActuatorDriver {

	private:
		struct TimedCommand {
			Vector3f				torque;
			float					timeIssued;

			TimedCommand(const Vector3f& v, float time) : torque (v), timeIssued(time) {}
		};

		RigidBodySimulator*			rbs;
		std::queue<TimedCommand>	commandBuffer;
		float						delay;
		float						currentTime;

	public:
		ActuatorDriver();
		explicit ActuatorDriver(RigidBodySimulator* rbs, float delay);
		ActuatorDriver(const ActuatorDriver& actuatorDriver);

		ActuatorDriver&	operator=(const ActuatorDriver& actuatorDriver);

		void			setDelay(float delay);
		inline float	getDelay() const { return (delay); }
		inline size_t	getBufferedCommandCount() const { return (commandBuffer.size());}

		void			sendCommand(const Vector3f& torque);
		void			update(float dt);
		void			reset();

		~ActuatorDriver();
};


#endif
