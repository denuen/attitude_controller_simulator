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

# Python virtual environment for the visualization package and its tests
VENV_DIR				?= .venv
VENV_PY					:= $(VENV_DIR)/bin/python

# Use the venv interpreter when it exists, otherwise fall back to the system
# python3 (override explicitly with 'make <target> PYTHON=...')
PYTHON					?= $(if $(wildcard $(VENV_PY)),$(VENV_PY),python3)

ifeq ($(RELEASE), 1)
	CXXFLAGS += -O2 -DNDEBUG
	CXXFLAGS_GTEST += -O2 -DNDEBUG
endif

SRC_DIR					?= src
TEST_DIR				?= tests
OBJ_DIR					?= .obj
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

HASH					:= \#

# Compiles a minimal snippet to verify the tinyxml header is reachable
TINYXML_CHECK			= printf '$(HASH)include <tinyxml.h>\nint main(){ return (0); }\n' | $(CXX) $(TINYXML_INC) -x c++ -c -o /dev/null - > /dev/null 2>&1

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

TINYXML_STAMP			= $(OBJ_DIR)/.tinyxml_ok

# Main target: builds only the main application
all: $(TINYXML_STAMP) $(NAME_MAIN)

# Build all executables (main + tests)
all-tests: $(TINYXML_STAMP) $(NAME_MAIN) $(NAME_TEST_PID) $(NAME_TEST_CONTROLLER) $(NAME_TEST_INPUTPARSER) $(NAME_TEST_SIMMANAGER) $(NAME_GTEST)

$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)
	mkdir -p $(OBJ_DIR)/control
	mkdir -p $(OBJ_DIR)/physics
	mkdir -p $(OBJ_DIR)/sensor
	mkdir -p $(OBJ_DIR)/io
	mkdir -p $(OBJ_DIR)/manager

$(OUTPUT_DIR):
	mkdir -p $(OUTPUT_DIR)

$(TINYXML_STAMP): | $(OBJ_DIR)
	@$(MAKE) --no-print-directory check-tinyxml
	@touch $@

# Verify tinyxml is available, installing it automatically if it is missing
check-tinyxml:
	@if $(TINYXML_CHECK); then \
		echo "tinyxml: found"; \
	else \
		echo "tinyxml: not found, attempting automatic installation..."; \
		$(MAKE) install-tinyxml; \
		if $(TINYXML_CHECK); then \
			echo "tinyxml: installed successfully"; \
		else \
			echo "ERROR: tinyxml is still unavailable after installation."; \
			echo "  macOS:          brew install tinyxml"; \
			echo "  Debian/Ubuntu:  sudo apt-get install libtinyxml-dev"; \
			echo "  Fedora/RHEL:    sudo dnf install tinyxml-devel"; \
			echo "  Arch:           sudo pacman -S tinyxml"; \
			exit 1; \
		fi; \
	fi

# Install tinyxml through the platform package manager
install-tinyxml:
ifeq ($(UNAME), Darwin)
	@if command -v brew > /dev/null 2>&1; then \
		brew install tinyxml; \
	else \
		echo "ERROR: Homebrew not found. Install it from https://brew.sh, then run 'brew install tinyxml'."; \
		exit 1; \
	fi
else
	@if command -v apt-get > /dev/null 2>&1; then \
		sudo apt-get update && sudo apt-get install -y libtinyxml-dev; \
	elif command -v dnf > /dev/null 2>&1; then \
		sudo dnf install -y tinyxml-devel; \
	elif command -v yum > /dev/null 2>&1; then \
		sudo yum install -y tinyxml-devel; \
	elif command -v pacman > /dev/null 2>&1; then \
		sudo pacman -S --noconfirm tinyxml; \
	elif command -v zypper > /dev/null 2>&1; then \
		sudo zypper install -y tinyxml-devel; \
	else \
		echo "ERROR: no supported package manager found. Please install the tinyxml development files manually."; \
		exit 1; \
	fi
endif

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp | $(OBJ_DIR) $(TINYXML_STAMP)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(TINYXML_INC) -c $< -o $@

$(NAME_TEST_PID): $(OBJ_CONTROL) $(OBJ_PHYSICS) $(TEST_DIR)/test_pid_rbs_vec3f.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(OBJ_CONTROL) $(OBJ_PHYSICS) $(TEST_DIR)/test_pid_rbs_vec3f.cpp -o $@

$(NAME_TEST_CONTROLLER): $(OBJ_CONTROL) $(OBJ_PHYSICS) $(TEST_DIR)/control/test_pid_controller.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(OBJ_CONTROL) $(OBJ_PHYSICS) $(TEST_DIR)/control/test_pid_controller.cpp -o $@

$(NAME_TEST_INPUTPARSER): $(OBJ_IO) $(OBJ_PHYSICS) $(TEST_DIR)/io/test_input_parser.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(TINYXML_INC) $(OBJ_IO) $(OBJ_PHYSICS) $(TEST_DIR)/io/test_input_parser.cpp $(TINYXML_LIB) -o $@

$(NAME_TEST_SIMMANAGER): $(OBJ_ALL) $(TEST_DIR)/manager/test_simulation_manager.cpp | $(OUTPUT_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(TINYXML_INC) $(OBJ_ALL) $(TEST_DIR)/manager/test_simulation_manager.cpp $(TINYXML_LIB) -o $@

$(NAME_MAIN): $(OBJ_ALL) main.cpp | $(OUTPUT_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(TINYXML_INC) $(OBJ_ALL) main.cpp $(TINYXML_LIB) -o $@

$(NAME_GTEST): $(OBJ_ALL) $(TEST_DIR)/gtest/test_gtest.cpp
	$(CXX) $(CXXFLAGS_GTEST) $(INCLUDES) $(GTEST_INC) $(TINYXML_INC) $(OBJ_ALL) $(TEST_DIR)/gtest/test_gtest.cpp $(GTEST_LIB) $(TINYXML_LIB) -o $@

# Create the virtual environment if it is missing, then install (or refresh)
# the Python requirements into it
venv:
	@if [ ! -d "$(VENV_DIR)" ]; then \
		echo "Creating virtual environment in $(VENV_DIR)/..."; \
		python3 -m venv $(VENV_DIR); \
	else \
		echo "Virtual environment $(VENV_DIR)/ already exists"; \
	fi
	@$(VENV_PY) -m pip --version > /dev/null 2>&1 || $(VENV_PY) -m ensurepip --upgrade
	@echo "Installing Python requirements from requirements.txt..."
	@$(VENV_PY) -m pip install -r requirements.txt

# Run the Python visualization test suite (requires the deps in requirements.txt)
test-visualization:
	$(PYTHON) -m unittest discover -s $(TEST_DIR)/visualization -t . -v

clean: clean-pyc
	@if [ -d "$(OBJ_DIR)" ]; then \
		rm -rf $(OBJ_DIR); \
		echo "Object files removed"; \
	else \
		echo "No object files to clean"; \
	fi

clean-pyc:
	@find . -path './$(VENV_DIR)' -prune -o -path './.git' -prune -o \
		-type d -name '__pycache__' -exec rm -rf {} + 2>/dev/null; \
	find . -path './$(VENV_DIR)' -prune -o -path './.git' -prune -o \
		-type f \( -name '*.pyc' -o -name '*.pyo' \) -exec rm -f {} + 2>/dev/null; \
	echo "Python bytecode caches removed"

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
	@echo "  test-visualization      Run the Python visualization test suite"
	@echo ""
	@echo "Python Environment Targets:"
	@echo "  venv                    Create $(VENV_DIR)/ if missing and install requirements.txt"
	@echo ""
	@echo "Cleanup Targets:"
	@echo "  clean                   Remove object files (and Python bytecode caches)"
	@echo "  clean-pyc               Remove Python bytecode caches (__pycache__, *.pyc)"
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
	@echo "Dependency Targets:"
	@echo "  check-tinyxml           Verify tinyxml is available, installing it if missing"
	@echo "  install-tinyxml         Install tinyxml via the platform package manager"
	@echo ""
	@echo "Other Targets:"
	@echo "  help                    Show this help message"
	@echo ""
	@echo "Directory Structure:"
	@echo "  Input configs:  $(INPUT_DIR)/"
	@echo "  Output files:   $(OUTPUT_DIR)/"

.PHONY: all all-tests venv test-visualization check-tinyxml install-tinyxml clean clean-pyc clean-output fclean re release release-main release-all release-target help
