# Main application
NAME_MAIN				= attitude_simulator

# Test executables (buildable individually)
NAME_TEST_PID			= test_pid_rbs_vec3f
NAME_TEST_CONTROLLER	= test_pid_controller
NAME_TEST_INPUTPARSER	= test_input_parser
NAME_TEST_SIMMANAGER	= test_simulation_manager
NAME_GTEST				= test_gtest

CXX						= g++
CXXFLAGS				= -Wall -Wextra -Werror -std=c++98
CXXFLAGS_GTEST			= -Wall -Wextra -Werror -std=c++17

ifeq ($(RELEASE), 1)
	CXXFLAGS += -O2 -DNDEBUG
	CXXFLAGS_GTEST += -O2 -DNDEBUG
endif

SRC_DIR					?= src
TEST_DIR				?= tests
OBJ_DIR					?= obj
INC_DIR					?= includes
INPUT_DIR				?= simulation_input
OUTPUT_DIR				?= simulation_output

UNAME					:= $(shell uname)

ifeq ($(UNAME), Darwin)
	# macOS
	GTEST_INC	= -I/opt/homebrew/include
	GTEST_LIB	= -L/opt/homebrew/lib -lgtest -lgtest_main -pthread
	TINYXML_INC	= -I/opt/homebrew/opt/tinyxml/include
	TINYXML_LIB	= -L/opt/homebrew/opt/tinyxml/lib -ltinyxml
else
	# Linux
	GTEST_INC	= -I/usr/include
	GTEST_LIB	= -lgtest -lgtest_main -pthread
	TINYXML_INC = -I/usr/include
	TINYXML_LIB = -L/usr/lib64 -ltinyxml
endif

INCLUDES				= -I $(INC_DIR)
SRC_CONTROL				= $(wildcard $(SRC_DIR)/control/*.cpp)
SRC_PHYSICS				= $(wildcard $(SRC_DIR)/physics/*.cpp)
SRC_SENSOR				= $(wildcard $(SRC_DIR)/sensor/*.cpp)
SRC_IO					= $(wildcard $(SRC_DIR)/io/*.cpp)
SRC_MANAGER				= $(wildcard $(SRC_DIR)/manager/*.cpp)

OBJ_CONTROL				= $(SRC_CONTROL:$(SRC_DIR)/%.cpp=$(OBJ_DIR)/%.o)
OBJ_PHYSICS				= $(SRC_PHYSICS:$(SRC_DIR)/%.cpp=$(OBJ_DIR)/%.o)
OBJ_SENSOR				= $(SRC_SENSOR:$(SRC_DIR)/%.cpp=$(OBJ_DIR)/%.o)
OBJ_IO					= $(SRC_IO:$(SRC_DIR)/%.cpp=$(OBJ_DIR)/%.o)
OBJ_MANAGER				= $(SRC_MANAGER:$(SRC_DIR)/%.cpp=$(OBJ_DIR)/%.o)

OBJ_ALL					= $(OBJ_CONTROL) $(OBJ_PHYSICS) $(OBJ_SENSOR) $(OBJ_IO) $(OBJ_MANAGER)

# Main target: builds only the main application
all: $(NAME_MAIN)

# Build all executables (main + tests)
all-tests: $(NAME_MAIN) $(NAME_TEST_PID) $(NAME_TEST_CONTROLLER) $(NAME_TEST_INPUTPARSER) $(NAME_TEST_SIMMANAGER) $(NAME_GTEST)

$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)
	mkdir -p $(OBJ_DIR)/control
	mkdir -p $(OBJ_DIR)/physics
	mkdir -p $(OBJ_DIR)/sensor
	mkdir -p $(OBJ_DIR)/io
	mkdir -p $(OBJ_DIR)/manager

$(OUTPUT_DIR):
	mkdir -p $(OUTPUT_DIR)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(TINYXML_INC) -c $< -o $@

$(NAME_TEST_PID): $(OBJ_CONTROL) $(OBJ_PHYSICS) $(TEST_DIR)/test_pid_rbs_vec3f.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(OBJ_CONTROL) $(OBJ_PHYSICS) $(TEST_DIR)/test_pid_rbs_vec3f.cpp -o $@

$(NAME_TEST_CONTROLLER): $(OBJ_CONTROL) $(OBJ_PHYSICS) $(TEST_DIR)/control/test_pid_controller.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(OBJ_CONTROL) $(OBJ_PHYSICS) $(TEST_DIR)/control/test_pid_controller.cpp -o $@

$(NAME_TEST_INPUTPARSER): $(OBJ_IO) $(OBJ_PHYSICS) $(TEST_DIR)/io/test_input_parser.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(TINYXML_INC) $(OBJ_IO) $(OBJ_PHYSICS) $(TEST_DIR)/io/test_input_parser.cpp $(TINYXML_LIB) -o $@

$(NAME_TEST_SIMMANAGER): $(OBJ_ALL) $(TEST_DIR)/manager/test_simulation_manager.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(TINYXML_INC) $(OBJ_ALL) $(TEST_DIR)/manager/test_simulation_manager.cpp $(TINYXML_LIB) -o $@

$(NAME_MAIN): $(OBJ_ALL) main.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(TINYXML_INC) $(OBJ_ALL) main.cpp $(TINYXML_LIB) -o $@

$(NAME_GTEST): $(OBJ_ALL) $(TEST_DIR)/gtest/test_gtest.cpp
	$(CXX) $(CXXFLAGS_GTEST) $(INCLUDES) $(GTEST_INC) $(TINYXML_INC) $(OBJ_ALL) $(TEST_DIR)/gtest/test_gtest.cpp $(GTEST_LIB) $(TINYXML_LIB) -o $@

clean:
	@if [ -d "$(OBJ_DIR)" ]; then \
		rm -rf $(OBJ_DIR); \
		echo "Object files removed"; \
	else \
		echo "No object files to clean"; \
	fi

clean-output:
	@if [ -d "$(OUTPUT_DIR)" ]; then \
		rm -rf $(OUTPUT_DIR)/*; \
		echo "Output files removed from $(OUTPUT_DIR)"; \
	else \
		echo "No output directory to clean"; \
	fi

fclean: clean clean-output
	@EXECUTABLES="$(NAME_MAIN) $(NAME_TEST_PID) $(NAME_TEST_CONTROLLER) $(NAME_TEST_INPUTPARSER) $(NAME_TEST_SIMMANAGER) $(NAME_GTEST)"; \
	FOUND=false; \
	for exe in $$EXECUTABLES; do \
		if [ -f "$$exe" ]; then \
			rm -f "$$exe"; \
			echo "$$exe removed"; \
			FOUND=true; \
		fi; \
	done; \
	if [ "$$FOUND" = false ]; then \
		echo "All executables already clean"; \
	fi


re: fclean all


# Release builds (optimized)
release: release-main

release-main:
	@echo "Performing release build of main application ($(NAME_MAIN))"
	$(MAKE) $(NAME_MAIN) RELEASE=1

release-all:
	@echo "Performing release build of all executables"
	$(MAKE) all-tests RELEASE=1

release-target:
	@if [ -z "$(TARGET)" ]; then \
		echo "Usage: make release-target TARGET=<target_name>"; \
		echo "Available targets: $(NAME_MAIN) $(NAME_TEST_PID) $(NAME_TEST_CONTROLLER) $(NAME_TEST_INPUTPARSER) $(NAME_TEST_SIMMANAGER) $(NAME_GTEST)"; \
		exit 1; \
	fi
	@echo "Performing release build of target: $(TARGET)"
	$(MAKE) $(TARGET) RELEASE=1


help:
	@echo "Usage: make [options] [target] ...\n"
	@echo "Main Targets:"
	@echo "  all                     Build main application (attitude_simulator)"
	@echo "  all-tests               Build main application + all test executables"
	@echo ""
	@echo "Individual Test Targets:"
	@echo "  test_pid_rbs_vec3f      Build test executable for modules: PID, RigidBodySimulator, Vector3f"
	@echo "  test_pid_controller     Build test executable for PIDController module"
	@echo "  test_input_parser       Build test executable for InputParser module"
	@echo "  test_simulation_manager Build test executable for SimulationManager module"
	@echo "  test_gtest              Build GoogleTest executable with all module tests"
	@echo ""
	@echo "Cleanup Targets:"
	@echo "  clean                   Remove object files"
	@echo "  clean-output            Remove simulation output files ($(OUTPUT_DIR)/*)"
	@echo "  fclean                  Remove object files, executables, and output files"
	@echo "  re                      Clean and rebuild main application"
	@echo ""
	@echo "Release Targets (Optimized Builds):"
	@echo "  release                 Build main application with optimizations (default)"
	@echo "  release-main            Build main application with optimizations"
	@echo "  release-all             Build all executables with optimizations"
	@echo "  release-target TARGET=<name>  Build specific target with optimizations"
	@echo ""
	@echo "Other Targets:"
	@echo "  help                    Show this help message"
	@echo ""
	@echo "Directory Structure:"
	@echo "  Input configs:  $(INPUT_DIR)/"
	@echo "  Output files:   $(OUTPUT_DIR)/"

.PHONY: all all-tests clean clean-output fclean re release release-main release-all release-target help
