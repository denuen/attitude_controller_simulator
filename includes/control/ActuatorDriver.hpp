#ifndef ACTUATOR_DRIVER_HPP
#define ACTUATOR_DRIVER_HPP

#include "../physics/RigidBodySimulator.hpp"
#include <ctime>
#include <queue>

// Simulates an actuator that applies torque commands to a rigid body after a 
// fixed delay
class ActuatorDriver {

	private:
		// Command structure
		struct TimedCommand {
			Vector3f	torque_; // Torque vector to be applied to the rigid body
			float		timeIssued_; // Timestamp when the command was issued

			TimedCommand(const Vector3f& v = Vector3f(), float t = 0.0f) :
				torque_(v), timeIssued_(t) {}
		};

		RigidBodySimulator*			rbs_; // Pointer to the target rigid body simulator
		std::queue<TimedCommand>	commandBuffer_; // Queue of pending timed commands
		float						delay_; // Command execution delay (in seconds)
		float						currentTime_; // Current simulation time
		float						maxTorqueMagnitude_; // Global magnitude cap (optional)
		Vector3f					maxTorquePerAxis_; // Per-axis torque limits (positive values) with ts
		TimedCommand				lastAppliedCommand_; // Timestamp at which the last command was applied
		
		// Clamps a value between -max and +max
		float	clamp(float value, float max);
	public:
		// Constructs an AD with delay = 0 and rbs = NULL
		ActuatorDriver();

		// Constructs an AD with the given rbs and delay (in seconds)
		explicit ActuatorDriver(RigidBodySimulator* rbs, float delay);

		// Copy constructor
		ActuatorDriver(const ActuatorDriver& a);
		
		// Copy assignment operator
		ActuatorDriver&	operator=(const ActuatorDriver& a);

		// Sets the command delay (in seconds)
		void			setDelay(float delay);

		// Sets per-axis maximum torque magnitudes (components must be positive)
		void			setMaxTorquePerAxis(Vector3f& maxTorques);

		// Sets a global cap on the torque magnitude (pass 0.0 to  disable) 
		void			setMaxTorqueMagnitude(float maxMag);

		// Returns the current commands delay
		inline float	getDelay() const { return (delay_); }

		// Returns the current time in the simulation
		inline float	getCurrentTime() const { return (currentTime_); }

		// Returns the number of commands currently buffered
		inline size_t	getBufferedCommandCount() const { return (commandBuffer_.size()); }

		// Returns the last torque that was applied to the rbs (post-saturation)
		inline Vector3f	getLastAppliedTorque() const { return (lastAppliedCommand_.torque_); }

		// Returns the timestamp of the last applied torque to the rbs
		inline float	getLastAppliedTorqueTs() const { return (lastAppliedCommand_.timeIssued_); }

		// Checks if any buffered command exceeds the given saturation limit
		bool			isBufferSaturated(float saturationLimit) const;

		// Queues a torque command to be applied after the delay
		void			sendCommand(const Vector3f& torque);

		// Updates the internal clock and applies any commands whose delay has expired
		void			update(float dt);

		/*
			Checks the correctness of all the required values of the module.
			Returns 1 if they are correct, 0 otherwise.
		*/
		bool			checkNumerics() const;

		// Clears all buffered commands and resets time tracking
		void			reset();

		// Default destructor
		~ActuatorDriver();
};

#endif

