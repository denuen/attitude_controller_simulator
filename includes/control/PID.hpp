#ifndef PID_HPP
#define PID_HPP

// Proportional-Integral-Derivative controller with anti-windup and derivative filtering
class PID {

	private:
		float				kp_; // Proportional gain coefficient (error amplification factor)
		float				ki_; // Integral gain coefficient (accumulated error correction rate)
		float				kd_; // Derivative gain coefficient (error rate damping factor)

		float				integral_; // Accumulated integral of error over time
		float				antiWindupTau_; // Time constant Tt used in back-calculation (seconds)

		float				previousError_; // Previous error value for derivative term calculation

		float				filteredDerivative_; // Low-pass filtered derivative term to reduce noice
		float				derivativeAlpha_; // Filtered coefficient for derivative smoothing [0.0 ; 1.0]

		// Fallback constant limits
		static const float	DEFAULT_INTEGRAL_MIN; // Lower saturation limit for integral windup protection
		static const float	DEFAULT_INTEGRAL_MAX; // Upper staturation limit fotr integral windup protection
		static const float	DEFAULT_OUTPUT_MIN; // Lower saturation limit for controller output
		static const float	DEFAULT_OUTPUT_MAX; // Upper saturation limit for controller output

	public:
		// Constructs a PID controller with zero gains
		PID();

		// Construct a PID controller with specified gains
		PID(float kp, float ki, float kd);

		// Copy constructor
		PID(const PID& pid);

		// Copy assignment operator
		PID&			operator=(const PID& pid);

		// Comparison operator (floating-point secure)
		bool			operator==(const PID& pid) const;

		// Comparison operator (floating-point secure)
		inline bool		operator!=(const PID& pid) const { return (!((*this) == pid)); }

		// Sets the proportional gain coefficient
		void			setKp(float kp);

		// Sets the integral gian coefficient
		void			setKi(float ki);

		// Sets the derivative gain coefficient
		void			setKd(float kd);

		// Sets all three gain coefficients simultaneously
		void			setGains(float kp, float ki, float kd);

		// Sets the alpha factor for derivative filtering to reduce noise sensitivity
		void			setDerivativeSmoothing(float alpha);

		// Sets the anti-windup time constant (requires tau > 0)
		void			setAntiWindupTau(float tau);

		// Returns the proportional gain coefficient
		inline float	getKp() const { return (kp_); }

		// Returns the integral gain coefficient
		inline float	getKi() const { return (ki_); }

		// Returns the derivative gain coefficient
		inline float	getKd() const { return (kd_); }

		// Returns the current integral accumulator value
		inline float	getIntegral() const { return (integral_); }

		// Returns the previous error value used for derivative calculation
		inline float	getPrevErr() const { return (previousError_); }

		// Returns the alpha factor used for derivative filtering [0.0 ; 1.0]
		inline float	getDerivativeSmoothing() const { return (derivativeAlpha_); }

		// Returns the anti-windup time constant (tau)
		inline float	getAntiWindupTau() const { return (antiWindupTau_); }

		// Computes control output with integral clamping and derivative filtering applied
		float			compute(const float setpoint, const float measure, const float dt);

		// Resets integral accumulator and previous error to zero
		void			reset();

		/*
			Checks the correctness of all the required values of the module.
			Returns 1 if they are correct, 0 otherwise
		*/
		bool			checkNumerics() const;

		~PID();
};

#endif
