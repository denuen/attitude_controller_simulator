NAME_TEST_PID			= test_pid_rbs_vec3f
NAME_TEST_CONTROLLER	= test_pid_controller
NAME_TEST_INPUTPARSER	= test_input_parser
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

UNAME					:= $(shell uname)

ifeq ($(UNAME), Darwin)
	# macOS
	GTEST_INC	= -I/opt/homebrew/include
	GTEST_LIB	= -L/opt/homebrew/lib -lgtest -lgtest_main -pthread
	TINYXML_INC	= -I/opt/homebrew/Cellar/tinyxml/2.6.2/include
	TINYXML_LIB	= -L/opt/homebrew/Cellar/tinyxml/2.6.2/lib -ltinyxml
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


all: $(NAME_TEST_PID) $(NAME_TEST_CONTROLLER) $(NAME_TEST_INPUTPARSER) $(NAME_GTEST)

$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)
	mkdir -p $(OBJ_DIR)/control
	mkdir -p $(OBJ_DIR)/physics
	mkdir -p $(OBJ_DIR)/sensor
	mkdir -p $(OBJ_DIR)/io
	mkdir -p $(OBJ_DIR)/manager

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(TINYXML_INC) -c $< -o $@

$(NAME_TEST_PID): $(OBJ_CONTROL) $(OBJ_PHYSICS) $(TEST_DIR)/test_pid_rbs_vec3f.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(OBJ_CONTROL) $(OBJ_PHYSICS) $(TEST_DIR)/test_pid_rbs_vec3f.cpp -o $@

$(NAME_TEST_CONTROLLER): $(OBJ_CONTROL) $(OBJ_PHYSICS) $(TEST_DIR)/control/test_pid_controller.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(OBJ_CONTROL) $(OBJ_PHYSICS) $(TEST_DIR)/control/test_pid_controller.cpp -o $@

$(NAME_TEST_INPUTPARSER): $(OBJ_IO) $(OBJ_PHYSICS) $(TEST_DIR)/io/test_input_parser.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(TINYXML_INC) $(OBJ_IO) $(OBJ_PHYSICS) $(TEST_DIR)/io/test_input_parser.cpp $(TINYXML_LIB) -o $@

$(NAME_GTEST): $(OBJ_ALL) $(TEST_DIR)/gtest/test_gtest.cpp
	$(CXX) $(CXXFLAGS_GTEST) $(INCLUDES) $(GTEST_INC) $(TINYXML_INC) $(OBJ_ALL) $(TEST_DIR)/gtest/test_gtest.cpp $(GTEST_LIB) $(TINYXML_LIB) -o $@

clean:
	@if [ -d "$(OBJ_DIR)" ]; then \
		rm -rf $(OBJ_DIR); \
		echo "Object files removed"; \
	else \
		echo "No object files to clean"; \
	fi

fclean: clean
	@if [ ! -f "$(NAME_TEST_PID)" ] && [ ! -f "$(NAME_TEST_CONTROLLER)" ] && [ ! -f "$(NAME_TEST_INPUTPARSER)" ] && [ ! -f "$(NAME_GTEST)" ]; then \
		echo "All executables already clean"; \
	fi
	@if [ -f "$(NAME_TEST_PID)" ]; then \
		rm -f $(NAME_TEST_PID); \
		echo "$(NAME_TEST_PID) removed"; \
	fi
	@if [ -f "$(NAME_TEST_CONTROLLER)" ]; then \
		rm -f $(NAME_TEST_CONTROLLER); \
		echo "$(NAME_TEST_CONTROLLER) removed"; \
	fi
	@if [ -f "$(NAME_TEST_INPUTPARSER)" ]; then \
		rm -f $(NAME_TEST_INPUTPARSER); \
		echo "$(NAME_TEST_INPUTPARSER) removed"; \
	fi
	@if [ -f "$(NAME_GTEST)" ]; then \
		rm -f $(NAME_GTEST); \
		echo "$(NAME_GTEST) removed"; \
	fi


re: fclean all


release:
	@if [ -z "$(TARGET)" ]; then \
		echo "Usage: make release TARGET=<target_name>"; \
		exit 1; \
	fi
	@echo "Performing release build of target: $(TARGET)"
	$(MAKE) $(TARGET) RELEASE=1


help:
	@echo "Usage: make [options] [target] ...\n"
	@echo "Targets:"
	@echo "  all                    Build all test executables"
	@echo "  test_pid_rbs_vec3f     Build test executable for modules: PID, RigidBodySimulator, Vector3f"
	@echo "  test_pid_controller    Build test executable for PIDController module"
	@echo "  test_input_parser      Build test executable for InputParser module"
	@echo "  test_gtest             Build GoogleTest executable with all module tests"
	@echo "  clean                  Remove object files"
	@echo "  fclean                 Remove object files and executables"
	@echo "  re                     Clean and rebuild all targets"
	@echo "  release TARGET=<name>  Build target with release optimizations and without assert controls on variables"
	@echo "  help                   Show this help message"

.PHONY: all clean fclean re release help
