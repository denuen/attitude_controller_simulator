#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include "PID.hpp"
#include "../physics/Vector3f.hpp"

// Controls 3D system using three indipendent PID controllers
class PIDController {

	private:
		PID	pitch_; // PID Controller for pitch axis
		PID	yaw_;	// PID Controller for yaw axis
		PID	roll_;	// PID Controller for roll axis


	public:
		// Constructs a PID Controller with zero gains on all axes
		PIDController();

		/* Constructs a PID Controller with specified gains (contained in a Vector3f) for each axis
			x component stands for kp;
			y component stands for ki;
			z component stands for kd;
		*/
		PIDController(const Vector3f& pitchG, const Vector3f& yawG, const Vector3f& rollG);

		// Constructs a PID Controller with specified PID axes:
		PIDController(const PID& pitch, const PID& yaw, const PID& roll);

		// Copy constructor
		PIDController(const PIDController& p);

		// Copy assignment operator
		PIDController&	operator=(const PIDController& p);

		// Returns the PID Controller config for pitch axis
		inline PID	getPitch() const { return (pitch_); }

		// Returns the PID Controller config for yaw axis
		inline PID	getYaw() const { return (yaw_); }

		// Returns the PID Controller config for roll axis
		inline PID	getRoll() const { return (roll_); }

		// Sets new gain values for pitch axis
		void		setPitchGains(float kp, float ki, float kd);

		// Sets new gain values for yaw axis
		void		setYawGains(float kp, float ki, float kd);

		// Sets new gain values for roll axis
		void		setRollGains(float kp, float ki, float kd);

		// Sets new gain values for all the axes
		void		setAllGains(Vector3f& pitchGains, Vector3f& yawGains, Vector3f& rollGains);

		// Sets the smoothing factor for derivative filtering on all axes
		void		setSmoothing(float alpha);

		// Sets the anti-windup time constant for all the axes
		void		setAntiWindup(float tau);

		// Computes three-axis torque vector based on attitude setpoint and current measurement
		Vector3f	compute(Vector3f& setpoint, Vector3f& measure, float dt);

		/*
			Checks the correctness of all the required values of the module.
			Returns 1 if they are correct, 0 otherwise.
		*/
		bool		checkNumerics() const;

		// Resets integral and derivative states on all the PID Controllers
		void		reset();

		// Default destructor
		~PIDController();

};

#endif

