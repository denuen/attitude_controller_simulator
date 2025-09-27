#include "../../includes/manager/SimulationManager.hpp"
#include <cassert>
#include <iostream>

void test_simulation_manager_initialization() {
	std::cout << "Testing SimulationManager initialization...\n";

	SimulationManager sim;

	// Check initial state
	assert(sim.getState() == STATE_UNINITIALIZED);
	assert(sim.getLastError() == ERR_SUCCESS);
	assert(sim.getCurrentTime() == 0.0f);

	// Test with invalid file first (should fail)
	ErrorCode result = sim.loadConfiguration("nonexistent.xml");
	assert(result != ERR_SUCCESS);
	assert(sim.getState() == STATE_UNINITIALIZED);

	// Test with valid configuration file
	result = sim.loadConfiguration("simulation_input/config_normal_case.xml");
	assert(result == ERR_SUCCESS);

	// Suppress unused variable warning in release mode
	(void)result;

	std::cout << "Initialization test passed\n";
}

void test_simulation_manager_safety_limits() {
	std::cout << "Testing SimulationManager safety limits...\n";

	SimulationManager sim;

	// Test valid safety limits
	ErrorCode result = sim.setSafetyLimits(5.0f, 50.0f, 0.001f);
	assert(result == ERR_SUCCESS);

	// Test invalid safety limits
	result = sim.setSafetyLimits(-1.0f, 50.0f, 0.001f); // negative max rate
	assert(result == ERR_CNF_OUT_OF_RANGE);

	result =
		sim.setSafetyLimits(5.0f, -50.0f, 0.001f); // negative control limit
	assert(result == ERR_CNF_OUT_OF_RANGE);

	// Suppress unused variable warning in release mode
	(void)result;

	std::cout << "Safety limits test passed\n";
}

void test_simulation_manager_state_transitions() {
	std::cout << "Testing SimulationManager state transitions...\n";

	SimulationManager sim;

	// Initialize
	ErrorCode result =
		sim.loadConfiguration("simulation_input/config_normal_case.xml");
	assert(result == ERR_SUCCESS);

	// Try to pause without running (should fail)
	result = sim.pause();
	assert(result == ERR_SYS_INVALID_STATE);

	// Try to resume without pausing (should fail)
	result = sim.resume();
	assert(result == ERR_SYS_INVALID_STATE);

	// Suppress unused variable warning in release mode
	(void)result;

	std::cout << "State transitions test passed\n";
}

void test_simulation_manager_short_run() {
	std::cout << "Testing SimulationManager short simulation run...\n";

	SimulationManager sim;

	// Initialize and set reasonable limits
	sim.loadConfiguration("simulation_input/config_normal_case.xml");
	sim.setSafetyLimits(10.0f, 100.0f, 0.01f);

	// Run a very short simulation (should complete without errors)
	ErrorCode result = sim.run(0.1f); // 0.1 seconds
	assert(result == ERR_SUCCESS);
	assert(sim.getState() == STATE_COMPLETED);
	assert(sim.getCurrentTime() >= 0.09f && sim.getCurrentTime() <= 0.11f);

	// Suppress unused variable warning in release mode
	(void)result;

	std::cout << "Short simulation run test passed\n";
}

int main() {
	std::cout << "Running SimulationManager tests...\n\n";

	test_simulation_manager_initialization();
	test_simulation_manager_safety_limits();
	test_simulation_manager_state_transitions();
	test_simulation_manager_short_run();

	std::cout << "\nAll SimulationManager tests passed!\n";
	return 0;
}
