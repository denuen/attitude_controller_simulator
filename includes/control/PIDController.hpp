#ifndef PIDCONTROLLER_HPP
#define PIDCONTROLLER_HPP

#include "../physics/Vector3f.hpp"
#include "PID.hpp"
#include <iostream>

// Controls a 3D system using three independent PID controllers for pitch, yaw, and roll.
class PIDController {

	private:
		PID	pidPitch;	// PID controller for pitch axis.
		PID	pidYaw;		// PID controller for yaw axis.
		PID	pidRoll;	// PID controller for roll axis.

	public:
		// Constructs a PIDController with zero gains on all axes.
		PIDController();

		// Constructs a PIDController with specified gains for each axis.
		explicit	PIDController(const Vector3f& kp, const Vector3f& ki, const Vector3f& kd);

		// Copy constructor.
		PIDController(const PIDController& pidController);

		// Copy assignment operator.
		PIDController&	operator=(const PIDController& pidController);

		// Sets new gain values for all three axes.
		void			setGains(const Vector3f& kp, const Vector3f& ki, const Vector3f& kd);

		// Sets the smoothing factor for derivative filtering on all axes.
		void			setSmoothing(const float alpha);

		bool			checkNumerics() const;
		// Computes three-axis torque vector based on attitude setpoint and current measurement.
		Vector3f		compute(const Vector3f& setpoint, const Vector3f& measure, float dt);

		// Resets integral and derivative states of all three PID controllers.
		void			reset(void);

		// Destructor.
		~PIDController();
};

#endif
