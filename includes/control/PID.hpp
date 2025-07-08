#ifndef PID_HPP
#define PID_HPP

#include "../physics/Vector3f.hpp"

// Proportional-Integral-Derivative controller with anti-windup and derivative filtering.
class PID {

	private:
		float				kp;						// Proportional gain coefficient (error amplification factor).
		float				ki;						// Integral gain coefficient (accumulated error correction rate).
		float				kd;						// Derivative gain coefficient (error rate damping factor).
		float				integral;				// Accumulated integral of error over time.
		float				previousError;			// Previous error value for derivative term calculation.
		float				filteredDerivative;		// Low-pass filtered derivative term to reduce noise.
		float				derivativeAlpha;		// Filter coefficient for derivative smoothing (0.0 to 1.0).

		static const float	DEFAULT_INTEGRAL_MIN;	// Lower saturation limit for integral windup protection.
		static const float	DEFAULT_INTEGRAL_MAX;	// Upper saturation limit for integral windup protection.
		static const float	DEFAULT_OUTPUT_MIN;		// Lower saturation limit for controller output.
		static const float	DEFAULT_OUTPUT_MAX;		// Upper saturation limit for controller output.

	public:
		// Constructs a PID controller with zero gains.
		PID();

		// Constructs a PID controller with specified gains.
		PID(float kp, float ki, float kd);

		// Copy constructor.
		PID(const PID& pid);

		// Copy assignment operator.
		PID&			operator=(const PID& pid);

		// Returns the proportional gain coefficient.
		inline float	getKp(void) const { return (kp); }

		// Returns the integral gain coefficient.
		inline float	getKi(void) const { return (ki); }

		// Returns the derivative gain coefficient.
		inline float	getKd(void) const { return (kd); }

		// Returns the current integral accumulator value.
		inline float	getIntegral(void) const { return (integral); }

		// Returns the previous error value used for derivative calculation.
		inline float	getPrevErr(void) const { return (previousError); }

		// Returns the alpha factor for derivative filtering (0.0 to 1.0).
		inline float	getDerivativeSmoothing(void) const { return (derivativeAlpha); }

		// Sets the proportional gain coefficient.
		void			setKp(const float kp);

		// Sets the integral gain coefficient.
		void			setKi(const float ki);

		// Sets the derivative gain coefficient.
		void			setKd(const float kd);

		// Sets all three gain coefficients simultaneously.
		void			setGains(const float kp, const float ki, const float kd);

		// Sets the alpha factor for derivative filtering to reduce noise sensitivity.
		void			setDerivativeSmoothing(float alpha);

		// Computes control output with integral clamping and derivative filtering applied.
		float			compute(const float setpoint, const float measure, const float dt);

		// Resets integral accumulator and previous error to zero.
		void			reset(void);

		// Destructor.
		~PID();
};

#endif
