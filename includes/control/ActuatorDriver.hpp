#ifndef ACTUATORDRIVER_HPP
#define ACTUATORDRIVER_HPP

#include "../physics/Vector3f.hpp"
#include "../physics/RigidBodySimulator.hpp"
#include <queue>
#include <cstddef>


// Simulates an actuator that applies torque commands to a rigid body after a fixed delay.
class ActuatorDriver {

	private:
		struct TimedCommand {
			Vector3f	torque;						// Torque vector to be applied to the rigid body.
			float		timeIssued;					// Timestamp when the command was issued.

			TimedCommand(const Vector3f& v, float time) : torque(v), timeIssued(time) {}
		};

		RigidBodySimulator*			rbs;			// Pointer to the target rigid body simulator.
		std::queue<TimedCommand>	commandBuffer;	// Queue of pending torque commands with timestamps.
		float						delay;			// Command execution delay in seconds.
		float						currentTime;	// Current simulation time for delay calculation.

	public:
		// Constructs an ActuatorDriver with zero delay and no associated rigid body.
		ActuatorDriver();

		// Constructs an ActuatorDriver with the given rigid body and delay in seconds.
		explicit ActuatorDriver(RigidBodySimulator* rbs, float delay);

		// Copy constructor.
		ActuatorDriver(const ActuatorDriver& actuatorDriver);

		// Copy assignment operator.
		ActuatorDriver&	operator=(const ActuatorDriver& actuatorDriver);

		// Sets the command delay in seconds.
		void			setDelay(float delay);

		// Returns the current command delay.
		inline float	getDelay() const { return delay; }

		// Returns the number of commands currently buffered.
		inline size_t	getBufferedCommandCount() const { return commandBuffer.size(); }

		// Queues a torque command to be applied after the delay.
		void			sendCommand(const Vector3f& torque);

		// Updates the internal clock and applies any commands whose delay has expired.
		void			update(float dt);

		// Clears all buffered commands and resets time tracking.
		void			reset();

		// Default destructor.
		~ActuatorDriver();
};

#endif
