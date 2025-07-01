#include "RigidBodySimulator.hpp"
#include "Vector3f.hpp"
#include <gtest/gtest.h>
#include <cmath>

const float EPSILON = 1e-6f;

// Test 1: Null initial state with null torque -> nothing should change
TEST(RigidBodySimulatorTest, ZeroTorqueMaintainsState) {
	RigidBodySimulator sim(Vector3f(1.0f, 1.0f, 1.0f));
	sim.setOmega(Vector3f(0.0f, 0.0f, 0.0f));

	sim.update(0.1f, Vector3f(0.0f, 0.0f, 0.0f));

	EXPECT_NEAR(sim.getOmega().getX(), 0.0f, EPSILON);
	EXPECT_NEAR(sim.getPitch(), 0.0f, EPSILON);
}

// Test 2: Constant torque on X axis -> linear increment
TEST(RigidBodySimulatorTest, ConstantTorqueOnX) {
	RigidBodySimulator sim(Vector3f(1.0f, 1.0f, 1.0f));
	sim.setOmega(Vector3f(0.0f, 0.0f, 0.0f));

	Vector3f torque(1.0f, 0.0f, 0.0f);
	sim.update(1.0f, torque);

	EXPECT_NEAR(sim.getOmega().getX(), 1.0f, EPSILON);
	EXPECT_NEAR(sim.getPitch(), 1.0f, EPSILON);
	EXPECT_NEAR(sim.getOmega().getY(), 0.0f, EPSILON);
	EXPECT_NEAR(sim.getOmega().getZ(), 0.0f, EPSILON);
}

// Test 3: Angle normalization check
TEST(RigidBodySimulatorTest, AngleNormalization) {
	RigidBodySimulator sim(Vector3f(1.0f, 1.0f, 1.0f));
	sim.setPitch(10.0f);	// > π
	sim.setYaw(-10.0f);		// < -π
	sim.setRoll(0.0f);

	sim.update(0.1f, Vector3f(0.0f, 0.0f, 0.0f));

	EXPECT_GE(sim.getPitch(), -M_PI);
	EXPECT_LE(sim.getPitch(), M_PI);
	EXPECT_GE(sim.getYaw(), -M_PI);
	EXPECT_LE(sim.getYaw(), M_PI);
}

// Test 4: NonNull gyroscopic term
TEST(RigidBodySimulatorTest, GyroscopicCouplingEffect) {
	RigidBodySimulator sim(Vector3f(2.0f, 1.0f, 3.0f));
	sim.setOmega(Vector3f(1.0f, 2.0f, 3.0f));
	Vector3f torque(0.0f, 0.0f, 0.0f);

	sim.update(0.1f, torque);

	// La componente x di omega dovrebbe cambiare a causa del termine giroscopico
	EXPECT_GT(std::fabs(sim.getOmega().getX()), EPSILON);
}

// Test 5: Inverse inertia check
TEST(RigidBodySimulatorTest, InverseInertiaComputation) {
	RigidBodySimulator sim(Vector3f(4.0f, 2.0f, 1.0f));

	EXPECT_FLOAT_EQ(sim.getInverseInertia().getX(), 0.25f);
	EXPECT_FLOAT_EQ(sim.getInverseInertia().getY(), 0.5f);
	EXPECT_FLOAT_EQ(sim.getInverseInertia().getZ(), 1.0f);
}

int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
