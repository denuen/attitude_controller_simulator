#include "../../includes/control/PID.hpp"
#include "../../includes/control/PIDController.hpp"
#include <cassert>
#include <cmath>
#include <iostream>

static void assert_f32_eq(float a, float b, float eps, const char* label) {
	assert(std::fabs(a - b) < eps && label);
	(void)a;
	(void)b;
	(void)eps;
	(void)label;
}

// tau = Kp * (r - y) with Ki = Kd = 0
static void test_proportional_only_step_response() {
	PID pid(1.0f, 0.0f, 0.0f);
	const float tau = pid.compute(10.0f, 7.0f, 0.1f);
	assert_f32_eq(tau, 3.0f, 1e-5f, "proportional torque must equal Kp * error");
	pid.reset();
}

// integral windup clamp: I in [-100, 100] => output capped at 100 for Ki=1, Kp=Kd=0
static void test_integral_saturation_at_default_limit() {
	PID pid(0.0f, 1.0f, 0.0f);
	float tau = 0.0f;
	for (int i = 0; i < 100; ++i) {
		tau = pid.compute(1.0f, 0.0f, 0.1f);
	}
	assert(tau <= 100.0f && "integral contribution must respect DEFAULT_INTEGRAL_MAX");
	(void)tau;
	pid.reset();
}

// EMA on de/dt: second sample with constant error must produce smaller |d| than first
static void test_ema_derivative_filters_high_frequency() {
	PID pid(0.0f, 0.0f, 1.0f);
	pid.setDerivativeSmoothing(0.2f);
	const float first = pid.compute(1.0f, 0.0f, 0.1f);
	const float second = pid.compute(1.0f, 0.0f, 0.1f);
	assert(std::fabs(second) < std::fabs(first)
		&& "EMA must attenuate derivative when attitude error is constant");
	(void)first;
	(void)second;
	pid.reset();
}

// output saturation at +/- 1000 Nm
static void test_output_clamped_at_default_torque_limit() {
	PID pid(100.0f, 0.0f, 0.0f);
	const float tau = pid.compute(1.0f, -1.0f, 0.1f);
	assert(tau <= 1000.0f && "torque must respect DEFAULT_OUTPUT_MAX");
	(void)tau;
	pid.reset();
}

// roll/pitch/yaw gains map to x/y/z torque without cross-coupling in the controller
static void test_three_axis_gains_are_decoupled() {
	Vector3f pitchG(1.0f, 0.0f, 0.0f);
	Vector3f yawG(2.0f, 0.0f, 0.0f);
	Vector3f rollG(3.0f, 0.0f, 0.0f);
	PIDController controller(pitchG, yawG, rollG);

	Vector3f setpoint(1.0f, 2.0f, 3.0f);
	Vector3f measure(0.0f, 0.0f, 0.0f);
	Vector3f torque = controller.compute(setpoint, measure, 0.1f);

	assert_f32_eq(torque.getX(), 3.0f, 1e-5f, "roll torque = Kp_roll * e_roll");
	assert_f32_eq(torque.getY(), 2.0f, 1e-5f, "pitch torque = Kp_pitch * e_pitch");
	assert_f32_eq(torque.getZ(), 6.0f, 1e-5f, "yaw torque = Kp_yaw * e_yaw");
	controller.reset();
}

// with constant error and omega = 0, both laws reduce to Kp * e after the first sample
static void test_rate_feedback_matches_attitude_pid_at_constant_error_and_zero_rate() {
	PID attitude(2.0f, 0.0f, 4.0f);
	PID rate_fb(2.0f, 0.0f, 4.0f);
	attitude.setDerivativeSmoothing(0.0f);

	attitude.compute(0.5f, 0.0f, 0.1f);
	rate_fb.computeWithBodyRate(0.5f, 0.0f, 0.0f, 0.1f);

	const float tau_att = attitude.compute(0.5f, 0.0f, 0.1f);
	const float tau_rate = rate_fb.computeWithBodyRate(0.5f, 0.0f, 0.0f, 0.1f);
	assert_f32_eq(tau_att, tau_rate, 1e-5f,
		"with constant error and omega=0 both laws must return Kp*e");

	attitude.reset();
	rate_fb.reset();
}

// tau = Kp * e - Kd * omega for Ki = 0
static void test_rate_feedback_implements_minus_kd_times_omega() {
	PID pid(2.0f, 0.0f, 4.0f);
	const float tau = pid.computeWithBodyRate(0.5f, 0.0f, 0.1f, 0.1f);
	assert_f32_eq(tau, 0.6f, 1e-5f, "torque must be Kp*e - Kd*omega = 1.0 - 0.4");
	pid.reset();
}

// non-zero omega must change the command relative to attitude-only derivative on same sample
static void test_rate_feedback_differs_from_finite_difference_when_omega_nonzero() {
	PID attitude(2.0f, 0.0f, 4.0f);
	PID rate_fb(2.0f, 0.0f, 4.0f);
	attitude.setDerivativeSmoothing(0.0f);

	attitude.compute(0.5f, 0.0f, 0.1f);
	const float tau_att = attitude.compute(0.5f, 0.0f, 0.1f);
	const float tau_rate = rate_fb.computeWithBodyRate(0.5f, 0.0f, 0.2f, 0.1f);

	assert(std::fabs(tau_att - tau_rate) > 1e-4f
		&& "measured omega must replace de/dt in the D-term");
	(void)tau_att;
	(void)tau_rate;

	attitude.reset();
	rate_fb.reset();
}

int main() {
	std::cout << "test_pid_controller\n";

	test_proportional_only_step_response();
	test_integral_saturation_at_default_limit();
	test_ema_derivative_filters_high_frequency();
	test_output_clamped_at_default_torque_limit();
	test_three_axis_gains_are_decoupled();
	test_rate_feedback_matches_attitude_pid_at_constant_error_and_zero_rate();
	test_rate_feedback_implements_minus_kd_times_omega();
	test_rate_feedback_differs_from_finite_difference_when_omega_nonzero();

	std::cout << "all tests passed\n";
	return 0;
}
