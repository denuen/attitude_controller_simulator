#ifndef PID_HPP
#define PID_HPP

#include "Vector3f.hpp"

class PID {

	private:
		float	kp;
		float	ki;
		float	kd;
		float	integral;
		float	previousError;

		static const float	DEFAULT_INTEGRAL_MIN;
		static const float	DEFAULT_INTEGRAL_MAX;
		static const float	DEFAULT_OUTPUT_MIN;
		static const float	DEFAULT_OUTPUT_MAX;
		
	public:
		PID();
		PID(float kp, float ki, float kd);
		PID(const PID& pid);

		PID&	operator=(const PID& pid);

		inline const float&		getKp(void) const { return (kp); }
		inline const float&		getKi(void) const { return (ki); }
		inline const float&		getKd(void) const { return (kd); }
		inline const float& 	getIntegral(void) const { return (integral); }
		inline const float& 	getPrevErr(void) const { return (previousError); }

		void				setKp(const float kp);
		void				setKi(const float ki);
		void				setKd(const float kd);
		void				setGains(const float kp, const float ki, const float kd);

		float				compute(const float setpoint, const float measure, const float dt);
		void				reset(void);

		~PID();
};

#endif
