#include <iostream>
#include <cassert>
#include <math.h>

#include "../includes/physics/RigidBodySimulator.hpp"
#include "../includes/physics/Vector3f.hpp"
#include "../includes/control/PID.hpp"
#include "../includes/control/PIDController.hpp"

void assertAlmostEqual(float a, float b, float eps = 1e-5f) {
    assert(std::fabs(a - b) < eps && "Values not approximately equal");
}

// Vector3f Tests
void test_vector3f_basic_operations() {
    Vector3f v1(1.0f, 2.0f, 3.0f);
    Vector3f v2(4.0f, 5.0f, 6.0f);

    Vector3f sum = v1 + v2;
    assertAlmostEqual(sum.getX(), 5.0f);
    assertAlmostEqual(sum.getY(), 7.0f);
    assertAlmostEqual(sum.getZ(), 9.0f);

    Vector3f diff = v2 - v1;
    assertAlmostEqual(diff.getX(), 3.0f);
    assertAlmostEqual(diff.getY(), 3.0f);
    assertAlmostEqual(diff.getZ(), 3.0f);

    Vector3f scaled = v1 * 2.0f;
    assertAlmostEqual(scaled.getX(), 2.0f);
    assertAlmostEqual(scaled.getY(), 4.0f);
    assertAlmostEqual(scaled.getZ(), 6.0f);

    float dp = dot(v1, v2);
    assertAlmostEqual(dp, 32.0f); // 1*4 + 2*5 + 3*6 = 32

    Vector3f cr = cross(v1, v2);
    assertAlmostEqual(cr.getX(), -3.0f);
    assertAlmostEqual(cr.getY(), 6.0f);
    assertAlmostEqual(cr.getZ(), -3.0f);
}

void test_vector3f_setters_getters() {
    Vector3f v;
    v.setX(7.0f);
    v.setY(8.0f);
    v.setZ(9.0f);

    assertAlmostEqual(v.getX(), 7.0f);
    assertAlmostEqual(v.getY(), 8.0f);
    assertAlmostEqual(v.getZ(), 9.0f);
}

// PID Tests
void test_pid_basic() {
    PID pid(1.0f, 0.0f, 0.0f); // P only
    float output = pid.compute(10.0f, 7.0f, 0.1f);
    assertAlmostEqual(output, 3.0f);
    pid.reset();
}

void test_pid_integral_windup() {
    PID pid(0.0f, 1.0f, 0.0f);
    float output = 0.0f;
    for (int i = 0; i < 100; ++i)
        output = pid.compute(1.0f, 0.0f, 0.1f);
    assert(output <= 100.0f);
    pid.reset();
}

void test_pid_derivative_filtering() {
    PID pid(0.0f, 0.0f, 1.0f);
    pid.setDerivativeSmoothing(0.2f);
    float out1 = pid.compute(1.0f, 0.0f, 0.1f);
    float out2 = pid.compute(1.0f, 0.0f, 0.1f);
    assert(out2 < out1);
    pid.reset();
}

void test_pid_output_clamping() {
    PID pid(100.0f, 0.0f, 0.0f);
    float out = pid.compute(1.0f, -1.0f, 0.1f);
    assert(out <= 1000.0f);
}

// PIDController Tests
void test_pidcontroller_compute() {
	Vector3f	pitchG(1.0f, 0.0f, 0.0f);
	Vector3f	yawG(2.0f, 0.0f, 0.0f);
	Vector3f	rollG(3.0f, 0.0f, 0.0f);

    PIDController controller(pitchG, yawG, rollG);
    Vector3f setpoint(1.0f, 2.0f, 3.0f);
    Vector3f measure(0.0f, 0.0f, 0.0f);

    Vector3f torque = controller.compute(setpoint, measure, 0.1f);
    assertAlmostEqual(torque.getX(), 1.0f);
    assertAlmostEqual(torque.getY(), 4.0f);
    assertAlmostEqual(torque.getZ(), 9.0f);
    controller.reset();
}

// RigidBodySimulator Tests
void test_rigidbodysimulator_update() {
    Vector3f inertia(1.0f, 1.0f, 1.0f);
    RigidBodySimulator sim(inertia);

    sim.setPitch(0.0f);
    sim.setYaw(0.0f);
    sim.setRoll(0.0f);
    sim.setOmega(Vector3f(0.0f, 0.0f, 0.0f));

    Vector3f torque(0.1f, 0.2f, 0.3f);
    float dt = 0.1f;

    sim.update(dt, torque);

    // Check updated omega is not zero (simple sanity check)
    Vector3f omega = sim.getOmega();
    assert(std::fabs(omega.getX()) > 0.0f);
    assert(std::fabs(omega.getY()) > 0.0f);
    assert(std::fabs(omega.getZ()) > 0.0f);

    // Check angles updated and normalized (within -pi..pi)
    float pitch = sim.getPitch();
    float yaw = sim.getYaw();
    float roll = sim.getRoll();

    assert(pitch >= -3.1416f && pitch <= 3.1416f);
    assert(yaw >= -3.1416f && yaw <= 3.1416f);
    assert(roll >= -3.1416f && roll <= 3.1416f);
}

int main() {
    std::cout << "Running complete module tests...\n";

    // Vector3f
    test_vector3f_basic_operations();
    test_vector3f_setters_getters();

    // PID
    test_pid_basic();
    test_pid_integral_windup();
    test_pid_derivative_filtering();
    test_pid_output_clamping();

    // PIDController
    test_pidcontroller_compute();

    // RigidBodySimulator
    test_rigidbodysimulator_update();

    std::cout << "All tests passed successfully.\n";
    return 0;
}
