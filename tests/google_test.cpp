#include <gtest/gtest.h>
#include "../includes/physics/Vector3f.hpp"
#include "../includes/control/PID.hpp"
#include "../includes/control/PIDController.hpp"
#include "../includes/physics/RigidBodySimulator.hpp"

// Vector3f
TEST(Vector3fTest, BasicOperations) {
	Vector3f v1(1, 2, 3);
	Vector3f v2(4, 5, 6);
	Vector3f sum = v1 + v2;
	EXPECT_FLOAT_EQ(sum.getX(), 5.0f);
	EXPECT_FLOAT_EQ(sum.getY(), 7.0f);
	EXPECT_FLOAT_EQ(sum.getZ(), 9.0f);
}

TEST(Vector3fTest, DotCross) {
	Vector3f v1(1, 2, 3);
	Vector3f v2(4, 5, 6);
	EXPECT_FLOAT_EQ(dot(v1, v2), 32.0f);

	Vector3f crossProduct = cross(v1, v2);
	EXPECT_FLOAT_EQ(crossProduct.getX(), -3.0f);
	EXPECT_FLOAT_EQ(crossProduct.getY(), 6.0f);
	EXPECT_FLOAT_EQ(crossProduct.getZ(), -3.0f);
}

// PID
TEST(PIDTest, StepResponse) {
	PID pid(1.0f, 0.0f, 0.0f);
	float out = pid.compute(10.0f, 7.0f, 0.1f);
	EXPECT_FLOAT_EQ(out, 3.0f);
}

TEST(PIDTest, Clamping) {
	PID pid(0.0f, 1.0f, 0.0f);
	for (int i = 0; i < 100; ++i)
		pid.compute(1.0f, 0.0f, 0.1f);
	EXPECT_LE(pid.compute(1.0f, 0.0f, 0.1f), 100.0f);
}

TEST(PIDTest, DerivativeFiltering) {
	PID pid(0.0f, 0.0f, 1.0f);
	pid.setDerivativeSmoothing(0.2f);
	float d1 = pid.compute(1.0f, 0.0f, 0.1f);
	float d2 = pid.compute(1.0f, 0.0f, 0.1f);
	EXPECT_LT(d2, d1);
}

// PIDController
TEST(PIDControllerTest, ComputeTorque) {
	Vector3f kp(1, 2, 3);
	PIDController ctrl(kp, Vector3f(), Vector3f());
	Vector3f torque = ctrl.compute(Vector3f(1,2,3), Vector3f(0,0,0), 0.1f);
	EXPECT_FLOAT_EQ(torque.getX(), 1.0f);
	EXPECT_FLOAT_EQ(torque.getY(), 4.0f);
	EXPECT_FLOAT_EQ(torque.getZ(), 9.0f);
}

// RigidBodySimulator
TEST(RBSTest, BasicUpdate) {
	RigidBodySimulator rbs(Vector3f(1.0f, 1.0f, 1.0f));
	rbs.setOmega(Vector3f(0.0f, 0.0f, 0.0f));
	rbs.setPitch(0.0f);
	rbs.setYaw(0.0f);
	rbs.setRoll(0.0f);
	rbs.update(0.1f, Vector3f(0.1f, 0.2f, 0.3f));
	EXPECT_NEAR(rbs.getPitch(), 0.001f, 1e-6);
	EXPECT_NEAR(rbs.getYaw(), 0.002f, 1e-6);
	EXPECT_NEAR(rbs.getRoll(), 0.003f, 1e-6);
}
