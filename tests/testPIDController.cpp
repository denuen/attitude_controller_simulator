#include <iostream>
#include <cassert>
#include "../includes/PID.hpp"
#include "../includes/PIDController.hpp"

void assertAlmostEqual(float a, float b, float eps = 1e-5f) {
	assert(std::fabs(a - b) < eps && "Assertion failed: values not approximately equal");
}

void test_basic_pid_step_response() {
	PID pid(1.0f, 0.0f, 0.0f);

	float setpoint = 10.0f;
	float measure = 7.0f;
	float dt = 0.1f;

	float output = pid.compute(setpoint, measure, dt);
	assertAlmostEqual(output, 3.0f);

	pid.reset();
}

void test_integral_windup_clamping() {
	PID pid(0.0f, 1.0f, 0.0f); // solo integrale

	float output = 0.0f;
	for (int i = 0; i < 100; ++i)
		output = pid.compute(1.0f, 0.0f, 0.1f);

	assert(output <= 100.0f && "Integral clamping failed");
	pid.reset();
}

void test_derivative_filtering() {
	PID pid(0.0f, 0.0f, 1.0f); // solo derivativo
	pid.setDerivativeSmoothing(0.2f);

	float out1 = pid.compute(1.0f, 0.0f, 0.1f); // salto iniziale
	float out2 = pid.compute(1.0f, 0.0f, 0.1f); // deve essere smorzato

	assert(out2 < out1);
	pid.reset();
}

void test_output_clamping() {
	PID pid(100.0f, 0.0f, 0.0f);

	float out = pid.compute(1.0f, -1.0f, 0.1f); // errore = 2 → 200
	assert(out <= 1000.0f);
}

void test_vector3f_pid_controller() {
	Vector3f kp(1.0f, 2.0f, 3.0f);
	Vector3f ki(0.0f, 0.0f, 0.0f);
	Vector3f kd(0.0f, 0.0f, 0.0f);

	PIDController controller(kp, ki, kd);

	Vector3f setpoint(1.0f, 2.0f, 3.0f);
	Vector3f measure(0.0f, 0.0f, 0.0f);
	Vector3f torque = controller.compute(setpoint, measure, 0.1f);

	assertAlmostEqual(torque.getX(), 1.0f);
	assertAlmostEqual(torque.getY(), 4.0f);
	assertAlmostEqual(torque.getZ(), 9.0f);
	controller.reset();
}

int main() {
	std::cout << "Running PID tests...\n";

	test_basic_pid_step_response();
	test_integral_windup_clamping();
	test_derivative_filtering();
	test_output_clamping();
	test_vector3f_pid_controller();

	std::cout << "All PID tests passed.\n";
	return 0;
}
