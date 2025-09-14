#include <gtest/gtest.h>
#include <math.h>
#include "../includes/control/PID.hpp"
#include "../includes/control/PIDController.hpp"
#include "../includes/control/ActuatorDriver.hpp"
#include "../includes/physics/Vector3f.hpp"
#include "../includes/physics/RigidBodySimulator.hpp"
#include "../includes/sensor/SensorSimulator.hpp"
#include "../includes/sensor/GaussianNoise.hpp"

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
	PIDController	ctrl(Vector3f(1, 0.0f, 0.0f), Vector3f(2, 0.0f, 0.0f), Vector3f(3, 0.0f, 0.0f));
	Vector3f		tmp1(1, 2, 3);
	Vector3f		tmp2;
	
	Vector3f torque = ctrl.compute(tmp1, tmp2, 0.1f);
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
	EXPECT_NEAR(rbs.getPitch(), 0.002f, 1e-6);
	EXPECT_NEAR(rbs.getYaw(), 0.003f, 1e-6);
	EXPECT_NEAR(rbs.getRoll(), 0.001f, 1e-6);
}

// SensorSimulator Tests
TEST(SensorSimulatorTest, BasicConstruction) {
	RigidBodySimulator rbs(Vector3f(1.0f, 1.0f, 1.0f));
	SensorSimulator sensor(&rbs);

	// Test basic reading without drift or noise
	Vector3f omega = sensor.readAngularVelocity();
	Vector3f orientation = sensor.readOrientation();

	// Should read the same values as the source (since drift and noise are zero)
	EXPECT_FLOAT_EQ(omega.getX(), rbs.getOmega().getX());
	EXPECT_FLOAT_EQ(omega.getY(), rbs.getOmega().getY());
	EXPECT_FLOAT_EQ(omega.getZ(), rbs.getOmega().getZ());

	EXPECT_FLOAT_EQ(orientation.getX(), rbs.getPitch());
	EXPECT_FLOAT_EQ(orientation.getY(), rbs.getYaw());
	EXPECT_FLOAT_EQ(orientation.getZ(), rbs.getRoll());
}

TEST(SensorSimulatorTest, DriftAccumulation) {
	RigidBodySimulator rbs(Vector3f(1.0f, 1.0f, 1.0f));
	SensorSimulator sensor(&rbs);

	// Set drift rate: 0.1 rad/s per second on each axis
	Vector3f driftRate(0.1f, 0.2f, 0.3f);
	sensor.setDriftRate(driftRate);

	// Read initial values
	Vector3f initialOmega = sensor.readAngularVelocity();
	Vector3f initialOrientation = sensor.readOrientation();

	// Update drift for 1 second
	sensor.update(1.0f);

	// Read values after drift accumulation
	Vector3f driftedOmega = sensor.readAngularVelocity();
	Vector3f driftedOrientation = sensor.readOrientation();

	// Check that drift has been applied (approximately, since noise is also present)
	// The difference should be close to the drift rate
	EXPECT_NEAR(driftedOmega.getX() - initialOmega.getX(), 0.1f, 0.01f);
	EXPECT_NEAR(driftedOmega.getY() - initialOmega.getY(), 0.2f, 0.01f);
	EXPECT_NEAR(driftedOmega.getZ() - initialOmega.getZ(), 0.3f, 0.01f);

	EXPECT_NEAR(driftedOrientation.getX() - initialOrientation.getX(), 0.1f, 0.01f);
	EXPECT_NEAR(driftedOrientation.getY() - initialOrientation.getY(), 0.2f, 0.01f);
	EXPECT_NEAR(driftedOrientation.getZ() - initialOrientation.getZ(), 0.3f, 0.01f);
}

TEST(SensorSimulatorTest, NoiseApplication) {
	RigidBodySimulator rbs(Vector3f(1.0f, 1.0f, 1.0f));
	SensorSimulator sensor(&rbs);

	// Set significant noise standard deviation
	Vector3f noiseStdDev(0.1f, 0.1f, 0.1f);
	sensor.setNoiseStdDev(noiseStdDev);

	// Take multiple readings and verify they are different (due to noise)
	Vector3f reading1 = sensor.readAngularVelocity();
	Vector3f reading2 = sensor.readAngularVelocity();
	Vector3f reading3 = sensor.readAngularVelocity();

	// With noise, readings should be different
	bool hasVariation = (reading1.getX() != reading2.getX()) ||
						(reading1.getY() != reading2.getY()) ||
						(reading1.getZ() != reading2.getZ()) ||
						(reading2.getX() != reading3.getX()) ||
						(reading2.getY() != reading3.getY()) ||
						(reading2.getZ() != reading3.getZ());

	EXPECT_TRUE(hasVariation);

	// Test orientation noise as well
	Vector3f orient1 = sensor.readOrientation();
	Vector3f orient2 = sensor.readOrientation();

	bool hasOrientationVariation = (orient1.getX() != orient2.getX()) ||
									(orient1.getY() != orient2.getY()) ||
									(orient1.getZ() != orient2.getZ());

	EXPECT_TRUE(hasOrientationVariation);
}

TEST(SensorSimulatorTest, ResetFunctionality) {
	RigidBodySimulator rbs(Vector3f(1.0f, 1.0f, 1.0f));
	SensorSimulator sensor(&rbs);

	// Set drift rate and noise
	Vector3f driftRate(0.1f, 0.2f, 0.3f);
	Vector3f noiseStdDev(0.05f, 0.05f, 0.05f);
	sensor.setDriftRate(driftRate);
	sensor.setNoiseStdDev(noiseStdDev);

	// Accumulate drift
	sensor.update(2.0f);

	// Read values with drift
	Vector3f driftedOmega = sensor.readAngularVelocity();
	Vector3f driftedOrientation = sensor.readOrientation();

	// Reset the sensor
	sensor.reset();

	// Read values after reset
	Vector3f resetOmega = sensor.readAngularVelocity();
	Vector3f resetOrientation = sensor.readOrientation();

	// Values should be much closer to the original source values
	// (allowing for small noise variations)
	EXPECT_NEAR(resetOmega.getX(), rbs.getOmega().getX(), 0.2f);
	EXPECT_NEAR(resetOmega.getY(), rbs.getOmega().getY(), 0.2f);
	EXPECT_NEAR(resetOmega.getZ(), rbs.getOmega().getZ(), 0.2f);

	EXPECT_NEAR(resetOrientation.getX(), rbs.getPitch(), 0.2f);
	EXPECT_NEAR(resetOrientation.getY(), rbs.getYaw(), 0.2f);
	EXPECT_NEAR(resetOrientation.getZ(), rbs.getRoll(), 0.2f);
}

TEST(SensorSimulatorTest, CopyConstructorAndAssignment) {
	RigidBodySimulator rbs(Vector3f(1.0f, 1.0f, 1.0f));
	SensorSimulator sensor1(&rbs);

	// Configure sensor1
	Vector3f driftRate(0.1f, 0.2f, 0.3f);
	Vector3f noiseStdDev(0.05f, 0.05f, 0.05f);
	sensor1.setDriftRate(driftRate);
	sensor1.setNoiseStdDev(noiseStdDev);
	sensor1.update(1.0f);

	// Test copy constructor
	SensorSimulator sensor2(sensor1);

	// Both sensors should give similar readings (within noise tolerance)
	Vector3f omega1 = sensor1.readAngularVelocity();
	Vector3f omega2 = sensor2.readAngularVelocity();

	// The drift should be the same, but noise will be different
	// Check that they're in a reasonable range
	EXPECT_NEAR(omega1.getX(), omega2.getX(), 0.3f);
	EXPECT_NEAR(omega1.getY(), omega2.getY(), 0.3f);
	EXPECT_NEAR(omega1.getZ(), omega2.getZ(), 0.3f);

	// Test assignment operator
	SensorSimulator sensor3(&rbs);
	sensor3 = sensor1;

	Vector3f omega3 = sensor3.readAngularVelocity();
	EXPECT_NEAR(omega1.getX(), omega3.getX(), 0.3f);
	EXPECT_NEAR(omega1.getY(), omega3.getY(), 0.3f);
	EXPECT_NEAR(omega1.getZ(), omega3.getZ(), 0.3f);
}

TEST(SensorSimulatorTest, RealisticIMUSimulation) {
	// Initialize Gaussian noise generator
	GaussianNoiseGenerator::initSeed();

	RigidBodySimulator rbs(Vector3f(1.0f, 2.0f, 3.0f));
	SensorSimulator imu(&rbs);

	// Set realistic IMU parameters
	// Typical gyroscope noise: 0.01 rad/s
	// Typical gyroscope bias drift: 0.001 rad/s per second
	Vector3f gyroNoise(0.01f, 0.01f, 0.01f);
	Vector3f gyroDrift(0.001f, 0.001f, 0.001f);

	imu.setNoiseStdDev(gyroNoise);
	imu.setDriftRate(gyroDrift);

	// Set initial conditions
	rbs.setPitch(0.1f);
	rbs.setYaw(0.2f);
	rbs.setRoll(0.3f);
	rbs.setOmega(Vector3f(0.5f, 0.6f, 0.7f));

	// Simulate IMU readings over 10 seconds
	float dt = 0.1f;
	int steps = 100; // 10 seconds

	Vector3f lastOmega = imu.readAngularVelocity();
	Vector3f lastOrientation = imu.readOrientation();

	for (int i = 0; i < steps; ++i) {
		// Update physics
		rbs.update(dt, Vector3f(0.01f, 0.02f, 0.03f));

		// Update sensor drift
		imu.update(dt);

		// Read sensor values
		Vector3f currentOmega = imu.readAngularVelocity();
		Vector3f currentOrientation = imu.readOrientation();

		// Verify readings are reasonable (not NaN or infinite)
		EXPECT_FALSE(std::isnan(currentOmega.getX()));
		EXPECT_FALSE(std::isnan(currentOmega.getY()));
		EXPECT_FALSE(std::isnan(currentOmega.getZ()));

		EXPECT_FALSE(std::isnan(currentOrientation.getX()));
		EXPECT_FALSE(std::isnan(currentOrientation.getY()));
		EXPECT_FALSE(std::isnan(currentOrientation.getZ()));

		// Verify readings are within reasonable bounds
		EXPECT_LT(abs(currentOmega.getX()), 10.0f);
		EXPECT_LT(abs(currentOmega.getY()), 10.0f);
		EXPECT_LT(abs(currentOmega.getZ()), 10.0f);

		lastOmega = currentOmega;
		lastOrientation = currentOrientation;
	}

	// After 10 seconds, drift should be approximately 0.01 rad/s
	Vector3f finalOmega = imu.readAngularVelocity();
	Vector3f realOmega = rbs.getOmega();

	// The difference should be roughly the accumulated drift plus some noise
	float driftAccumulated = 0.001f * 10.0f; // 0.01 rad/s
	EXPECT_NEAR(finalOmega.getX() - realOmega.getX(), driftAccumulated, 0.05f);
	EXPECT_NEAR(finalOmega.getY() - realOmega.getY(), driftAccumulated, 0.05f);
	EXPECT_NEAR(finalOmega.getZ() - realOmega.getZ(), driftAccumulated, 0.05f);
}

TEST(SensorSimulatorTest, ZeroNoiseConfiguration) {
	RigidBodySimulator rbs(Vector3f(1.0f, 1.0f, 1.0f));
	SensorSimulator sensor(&rbs);

	// Set zero noise standard deviation
	Vector3f zeroNoise(0.0f, 0.0f, 0.0f);
	sensor.setNoiseStdDev(zeroNoise);

	// Multiple readings should be identical (no noise)
	Vector3f reading1 = sensor.readAngularVelocity();
	Vector3f reading2 = sensor.readAngularVelocity();
	Vector3f reading3 = sensor.readAngularVelocity();

	EXPECT_FLOAT_EQ(reading1.getX(), reading2.getX());
	EXPECT_FLOAT_EQ(reading1.getY(), reading2.getY());
	EXPECT_FLOAT_EQ(reading1.getZ(), reading2.getZ());

	EXPECT_FLOAT_EQ(reading2.getX(), reading3.getX());
	EXPECT_FLOAT_EQ(reading2.getY(), reading3.getY());
	EXPECT_FLOAT_EQ(reading2.getZ(), reading3.getZ());
}

// GaussianNoiseGenerator Tests
TEST(GaussianNoiseGeneratorTest, BasicGeneration) {
	GaussianNoiseGenerator::initSeed();
	GaussianNoiseGenerator generator;

	// Test basic generation
	float value1 = generator.generate(0.0f, 1.0f);
	float value2 = generator.generate(0.0f, 1.0f);

	// Values should be different (very unlikely to be the same)
	EXPECT_NE(value1, value2);

	// Values should be finite
	EXPECT_TRUE(isfinite(value1));
	EXPECT_TRUE(isfinite(value2));
}

TEST(GaussianNoiseGeneratorTest, MeanAndStdDev) {
	GaussianNoiseGenerator::initSeed();
	GaussianNoiseGenerator generator;

	// Generate many samples and check statistical properties
	const int numSamples = 10000;
	float mean = 5.0f;
	float stddev = 2.0f;

	float sum = 0.0f;
	float sumSquares = 0.0f;

	for (int i = 0; i < numSamples; ++i) {
		float value = generator.generate(mean, stddev);
		sum += value;
		sumSquares += value * value;
	}

	float actualMean = sum / numSamples;
	float actualVariance = (sumSquares / numSamples) - (actualMean * actualMean);
	float actualStdDev = sqrt(actualVariance);

	// Check that mean and standard deviation are close to expected values
	EXPECT_NEAR(actualMean, mean, 0.1f);
	EXPECT_NEAR(actualStdDev, stddev, 0.1f);
}

TEST(GaussianNoiseGeneratorTest, ZeroStdDev) {
	GaussianNoiseGenerator generator;

	// When stddev is 0, should always return the mean
	float mean = 3.14f;
	float value1 = generator.generate(mean, 0.0f);
	float value2 = generator.generate(mean, 0.0f);

	EXPECT_FLOAT_EQ(value1, mean);
	EXPECT_FLOAT_EQ(value2, mean);
}

TEST(GaussianNoiseGeneratorTest, ResetFunctionality) {
	GaussianNoiseGenerator generator1;
	GaussianNoiseGenerator generator2;

	// Generate some values
	generator1.generate(0.0f, 1.0f);
	generator2.generate(0.0f, 1.0f);

	// Reset one generator
	generator1.reset();

	// After reset, the internal state should be cleared
	// This should be hard to test directly, but at least no crashes should occur
	float value = generator1.generate(0.0f, 1.0f);
	EXPECT_TRUE(isfinite(value));
}

// ActuatorDriver Tests
TEST(ActuatorDriverTest, BasicConstruction) {
	RigidBodySimulator rbs(Vector3f(1.0f, 1.0f, 1.0f));
	ActuatorDriver actuator(&rbs, 0.1f);
	// Test basic properties
	EXPECT_FLOAT_EQ(actuator.getDelay(), 0.1f);
	EXPECT_EQ(actuator.getBufferedCommandCount(), static_cast<size_t>(0));

	// Test default constructor
	ActuatorDriver defaultActuator;
	EXPECT_FLOAT_EQ(defaultActuator.getDelay(), 0.0f);
	EXPECT_EQ(defaultActuator.getBufferedCommandCount(), static_cast<size_t>(0));
}

TEST(ActuatorDriverTest, CommandBuffering) {
	RigidBodySimulator rbs(Vector3f(1.0f, 1.0f, 1.0f));
	ActuatorDriver actuator(&rbs, 0.2f);
	// Send multiple commands
	actuator.sendCommand(Vector3f(1.0f, 0.0f, 0.0f));
	EXPECT_EQ(actuator.getBufferedCommandCount(), static_cast<size_t>(1));

	actuator.sendCommand(Vector3f(0.0f, 1.0f, 0.0f));
	EXPECT_EQ(actuator.getBufferedCommandCount(), static_cast<size_t>(2));

	actuator.sendCommand(Vector3f(0.0f, 0.0f, 1.0f));
	EXPECT_EQ(actuator.getBufferedCommandCount(), static_cast<size_t>(3));
}

TEST(ActuatorDriverTest, DelayedExecution) {
	RigidBodySimulator rbs(Vector3f(1.0f, 1.0f, 1.0f));
	ActuatorDriver actuator(&rbs, 0.1f);

	rbs.setPitch(0.0f);
	rbs.setYaw(0.0f);
	rbs.setRoll(0.0f);
	rbs.setOmega(Vector3f(0.0f, 0.0f, 0.0f));

	// Send a command
	Vector3f testTorque(0.1f, 0.2f, 0.3f);
	actuator.sendCommand(testTorque);
	// Command should be buffered
	EXPECT_EQ(actuator.getBufferedCommandCount(), static_cast<size_t>(1));

	// Update for less than delay time - command should not execute
	actuator.update(0.05f);
	EXPECT_EQ(actuator.getBufferedCommandCount(), static_cast<size_t>(1));

	// Update for remaining delay time - command should execute
	actuator.update(0.05f);
	EXPECT_EQ(actuator.getBufferedCommandCount(), static_cast<size_t>(0));

	// Verify that torque was applied by checking angular velocity change
	Vector3f omega = rbs.getOmega();
	EXPECT_GT(abs(omega.getX()), 0.0f);
	EXPECT_GT(abs(omega.getY()), 0.0f);
	EXPECT_GT(abs(omega.getZ()), 0.0f);
}

TEST(ActuatorDriverTest, ZeroDelayExecution) {
	RigidBodySimulator rbs(Vector3f(1.0f, 1.0f, 1.0f));
	ActuatorDriver actuator(&rbs, 0.0f);

	rbs.setPitch(0.0f);
	rbs.setYaw(0.0f);
	rbs.setRoll(0.0f);
	rbs.setOmega(Vector3f(0.0f, 0.0f, 0.0f));

	// Send a command
	Vector3f testTorque(0.1f, 0.2f, 0.3f);
	actuator.sendCommand(testTorque);

	// With zero delay, command should execute immediately on next update
	actuator.update(0.01f);
	EXPECT_EQ(actuator.getBufferedCommandCount(), static_cast<size_t>(0));

	// Verify that torque was applied
	Vector3f omega = rbs.getOmega();
	EXPECT_GT(abs(omega.getX()), 0.0f);
	EXPECT_GT(abs(omega.getY()), 0.0f);
	EXPECT_GT(abs(omega.getZ()), 0.0f);
}

TEST(ActuatorDriverTest, MultipleDelayedCommands) {
	RigidBodySimulator rbs(Vector3f(1.0f, 1.0f, 1.0f));
	ActuatorDriver actuator(&rbs, 0.1f);

	// Send commands at different times
	actuator.sendCommand(Vector3f(1.0f, 0.0f, 0.0f));
	actuator.update(0.02f);  // currentTime = 0.02

	actuator.sendCommand(Vector3f(0.0f, 1.0f, 0.0f));
	actuator.update(0.02f);  // currentTime = 0.04

	actuator.sendCommand(Vector3f(0.0f, 0.0f, 1.0f));
	actuator.update(0.02f);  // currentTime = 0.06

	// Should have 3 commands buffered
	EXPECT_EQ(actuator.getBufferedCommandCount(), static_cast<size_t>(3));

	// Update to execute first command (0 + 0.1 = 0.1, currentTime will be 0.1)
	actuator.update(0.04f);  // currentTime = 0.1
	EXPECT_EQ(actuator.getBufferedCommandCount(), static_cast<size_t>(2));

	// Update to execute second command (0.02 + 0.1 = 0.12, currentTime will be 0.12)
	actuator.update(0.02f);  // currentTime = 0.12
	EXPECT_EQ(actuator.getBufferedCommandCount(), static_cast<size_t>(1));

	// Update to execute third command (0.04 + 0.1 = 0.14, currentTime will be 0.14)
	actuator.update(0.02f);  // currentTime = 0.14
	EXPECT_EQ(actuator.getBufferedCommandCount(), static_cast<size_t>(0));
}

TEST(ActuatorDriverTest, DelayModification) {
	RigidBodySimulator rbs(Vector3f(1.0f, 1.0f, 1.0f));
	ActuatorDriver actuator(&rbs, 0.1f);

	// Test initial delay
	EXPECT_FLOAT_EQ(actuator.getDelay(), 0.1f);

	// Change delay
	actuator.setDelay(0.2f);
	EXPECT_FLOAT_EQ(actuator.getDelay(), 0.2f);

	// Test that new delay is applied
	actuator.sendCommand(Vector3f(1.0f, 0.0f, 0.0f));
	actuator.update(0.1f);
	EXPECT_EQ(actuator.getBufferedCommandCount(), static_cast<size_t>(1)); // Command should still be buffered

	actuator.update(0.1f);
	EXPECT_EQ(actuator.getBufferedCommandCount(), static_cast<size_t>(0)); // Now command should execute
}

TEST(ActuatorDriverTest, ResetFunctionality) {
	RigidBodySimulator rbs(Vector3f(1.0f, 1.0f, 1.0f));
	ActuatorDriver actuator(&rbs, 0.1f);

	// Send multiple commands and advance time
	actuator.sendCommand(Vector3f(1.0f, 0.0f, 0.0f));
	actuator.sendCommand(Vector3f(0.0f, 1.0f, 0.0f));
	actuator.sendCommand(Vector3f(0.0f, 0.0f, 1.0f));
	actuator.update(0.05f);

	// Verify commands are buffered
	EXPECT_EQ(actuator.getBufferedCommandCount(), static_cast<size_t>(3));

	// Reset should clear all commands
	actuator.reset();
	EXPECT_EQ(actuator.getBufferedCommandCount(), static_cast<size_t>(0));

	// After reset, new commands should work normally
	actuator.sendCommand(Vector3f(1.0f, 1.0f, 1.0f));
	EXPECT_EQ(actuator.getBufferedCommandCount(), static_cast<size_t>(1));
}

TEST(ActuatorDriverTest, CopyConstructorAndAssignment) {
	RigidBodySimulator rbs(Vector3f(1.0f, 1.0f, 1.0f));
	ActuatorDriver actuator1(&rbs, 0.1f);

	// Configure actuator1
	actuator1.sendCommand(Vector3f(1.0f, 0.0f, 0.0f));
	actuator1.sendCommand(Vector3f(0.0f, 1.0f, 0.0f));
	actuator1.update(0.05f);

	// Test copy constructor
	ActuatorDriver actuator2(actuator1);
	EXPECT_FLOAT_EQ(actuator2.getDelay(), 0.1f);
	EXPECT_EQ(actuator2.getBufferedCommandCount(), static_cast<size_t>(2));

	// Test assignment operator
	ActuatorDriver actuator3;
	actuator3 = actuator1;
	EXPECT_FLOAT_EQ(actuator3.getDelay(), 0.1f);
	EXPECT_EQ(actuator3.getBufferedCommandCount(), static_cast<size_t>(2));
}

TEST(ActuatorDriverTest, RealisticControlLoop) {
	RigidBodySimulator rbs(Vector3f(1.0f, 2.0f, 3.0f));
	ActuatorDriver actuator(&rbs, 0.05f); // 50ms delay

	rbs.setPitch(0.0f);
	rbs.setYaw(0.0f);
	rbs.setRoll(0.0f);
	rbs.setOmega(Vector3f(0.0f, 0.0f, 0.0f));

	// Simulate a control loop with 10ms timestep for 1 second
	float dt = 0.01f;
	int steps = 100;

	for (int i = 0; i < steps; ++i) {
		// Generate control commands (simple proportional control)
		Vector3f targetAttitude(0.1f, 0.2f, 0.3f);
		Vector3f currentAttitude(rbs.getPitch(), rbs.getYaw(), rbs.getRoll());
		Vector3f error = targetAttitude - currentAttitude;
		Vector3f controlTorque = error * 0.5f; // Simple P controller

		// Send command through actuator
		actuator.sendCommand(controlTorque);

		// Update actuator (applies delayed commands)
		actuator.update(dt);

		// Update physics simulation
		rbs.update(dt, Vector3f(0.0f, 0.0f, 0.0f));

		// Verify system stability
		Vector3f omega = rbs.getOmega();
		EXPECT_LT(abs(omega.getX()), 10.0f);
		EXPECT_LT(abs(omega.getY()), 10.0f);
		EXPECT_LT(abs(omega.getZ()), 10.0f);

		// Verify angles are reasonable
		EXPECT_LT(abs(rbs.getPitch()), 5.0f);
		EXPECT_LT(abs(rbs.getYaw()), 5.0f);
		EXPECT_LT(abs(rbs.getRoll()), 5.0f);
	}

	// After 1 second, the system should be moving towards the target
	Vector3f finalAttitude(rbs.getPitch(), rbs.getYaw(), rbs.getRoll());
	Vector3f targetAttitude(0.1f, 0.2f, 0.3f);

	// The attitude should be closer to target than initial (which was zero)
	EXPECT_LT(abs(finalAttitude.getX() - targetAttitude.getX()), 0.1f);
	EXPECT_LT(abs(finalAttitude.getY() - targetAttitude.getY()), 0.2f);
	EXPECT_LT(abs(finalAttitude.getZ() - targetAttitude.getZ()), 0.3f);
}

TEST(ActuatorDriverTest, HighFrequencyCommands) {
	RigidBodySimulator rbs(Vector3f(1.0f, 1.0f, 1.0f));
	ActuatorDriver actuator(&rbs, 0.1f);

	// Send many commands rapidly
	for (int i = 0; i < 100; ++i) {
		actuator.sendCommand(Vector3f(0.01f, 0.01f, 0.01f));
		actuator.update(0.001f); // 1ms updates
	}
	// Should have accumulated many commands
	EXPECT_GT(actuator.getBufferedCommandCount(), static_cast<size_t>(0));

	// Update for delay time to execute all commands
	actuator.update(0.2f);
	EXPECT_EQ(actuator.getBufferedCommandCount(), static_cast<size_t>(0));
}
