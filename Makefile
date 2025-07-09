NAME_TEST_PID			= test_pid_rbs_vec
NAME_TEST_CONTROLLER	= test_pidcontroller
NAME_GTEST				= gtest

CXX						= g++
CXXFLAGS				= -Wall -Wextra -Werror -std=c++98
CXXFLAGS_GTEST			= -Wall -Wextra -Werror -std=c++17

SRC_DIR					= src
TEST_DIR				= tests
OBJ_DIR					= obj
INC_DIR					= includes

UNAME					:= $(shell uname)

ifeq ($(UNAME), Darwin)
	# macOS
	GTEST_INC	= -I/opt/homebrew/include
	GTEST_LIB	= -L/opt/homebrew/lib -lgtest -lgtest_main -pthread
else
	# Linux
	GTEST_INC	= -I/usr/include
	GTEST_LIB	= -lgtest -lgtest_main -pthread
endif

INCLUDES				= -I $(INC_DIR)
SRC_CONTROL				= $(wildcard $(SRC_DIR)/control/*.cpp)
SRC_PHYSICS				= $(wildcard $(SRC_DIR)/physics/*.cpp)
SRC_SENSOR				= $(wildcard $(SRC_DIR)/sensor/*.cpp)

SRC						= $(SRC_CONTROL) $(SRC_PHYSICS) $(SRC_SENSOR)

OBJ_CONTROL				= $(SRC_CONTROL:$(SRC_DIR)/%.cpp=$(OBJ_DIR)/%.o)
OBJ_PHYSICS				= $(SRC_PHYSICS:$(SRC_DIR)/%.cpp=$(OBJ_DIR)/%.o)
OBJ_SENSOR				= $(SRC_SENSOR:$(SRC_DIR)/%.cpp=$(OBJ_DIR)/%.o)

OBJ						= $(OBJ_CONTROL) $(OBJ_PHYSICS) $(OBJ_SENSOR)


all: $(NAME_TEST_PID) $(NAME_TEST_CONTROLLER) $(NAME_GTEST)

$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)
	mkdir -p $(OBJ_DIR)/control
	mkdir -p $(OBJ_DIR)/physics
	mkdir -p $(OBJ_DIR)/sensor

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

$(NAME_TEST_PID): $(OBJ) $(TEST_DIR)/test_PID_RBS_Vec.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(OBJ) $(TEST_DIR)/test_PID_RBS_Vec.cpp -o $@

$(NAME_TEST_CONTROLLER): $(OBJ) $(TEST_DIR)/testPIDController.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(OBJ) $(TEST_DIR)/testPIDController.cpp -o $@

$(NAME_GTEST): $(OBJ) $(TEST_DIR)/google_test.cpp
	$(CXX) $(CXXFLAGS_GTEST) $(INCLUDES) $(GTEST_INC) $(OBJ) $(TEST_DIR)/google_test.cpp $(GTEST_LIB) -o $@

clean:
	@if [ -d "$(OBJ_DIR)" ]; then \
		rm -rf $(OBJ_DIR); \
		echo "Object files removed"; \
	else \
		echo "No object files to clean"; \
	fi

fclean: clean
	@if [ ! -f "$(NAME_TEST_PID)" ] && [ ! -f "$(NAME_TEST_CONTROLLER)" ] && [ ! -f "$(NAME_GTEST)" ]; then \
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
	@if [ -f "$(NAME_GTEST)" ]; then \
		rm -f $(NAME_GTEST); \
		echo "$(NAME_GTEST) removed"; \
	fi

re: fclean all


.PHONY: all clean fclean re
