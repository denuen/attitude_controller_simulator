#ifndef PIDCONTROLLER_HPP
#define PIDCONTROLLER_HPP

#include "Vector3f.hpp"
#include "PID.hpp"

class PIDController {

	private:
		PID	pidPitch;
		PID	pidYaw;
		PID	pidRoll;

	public:
		PIDController();
		PIDController(const Vector3f& kp, const Vector3f& ki, const Vector3f& kd);
		PIDController(const PIDController& pidController);

		PIDController&	operator=(const PIDController& pidController);

		void	setGains(const Vector3f& kp, const Vector3f& ki, const Vector3f& kd);
		void	setSmoothing(const float alpha);

		Vector3f	compute(const Vector3f& setpoint, const Vector3f& measure, float dt);
		void		reset(void);

		~PIDController();
};



#endif
