NAME        = attitude_controller
TEST_NAME   = test_attitude_controller
TEST_PID_RBS_VEC_NAME = test_pid_rbs_vec
TEST_PIDCONTROLLER_NAME = test_pidcontroller
GTEST_ALL_MODULES_NAME = gtest_all_modules

CXX         = c++
CXXFLAGS    = -Wall -Wextra -Werror -std=c++98
INCLUDES    = -I includes

SRCS        = src/Vector3f.cpp src/RigidBodySimulator.cpp src/PID.cpp src/PIDController.cpp main.cpp
OBJS        = $(SRCS:.cpp=.o)

TEST_FLAGS  = -std=c++17 -I/opt/homebrew/include -L/opt/homebrew/lib -lgtest -lgtest_main -pthread

all: $(NAME)

$(NAME): $(OBJS) | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $(OBJS) -o $@

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

$(BIN_DIR):
	mkdir -p $(BIN_DIR)


$(TEST_PID_RBS_VEC_NAME): src/Vector3f.cpp src/RigidBodySimulator.cpp src/PID.cpp src/PIDController.cpp tests/test_PID_RBS_Vec.cpp | $(BIN_DIR)
	$(CXX) -Wall -Wextra -Werror -std=c++98 $(INCLUDES) tests/test_PID_RBS_Vec.cpp src/*.cpp -o $@

$(TEST_PIDCONTROLLER_NAME): src/Vector3f.cpp src/RigidBodySimulator.cpp src/PID.cpp src/PIDController.cpp tests/testPIDController.cpp | $(BIN_DIR)
	$(CXX) -Wall -Wextra -Werror -std=c++98 $(INCLUDES) tests/testPIDController.cpp src/*.cpp -o $@

$(GTEST_ALL_MODULES_NAME): tests/google_test.cpp src/*.cpp | $(BIN_DIR)
	$(CXX) $(INCLUDES) $(TEST_FLAGS) tests/google_test.cpp src/*.cpp -o $@

# Default test target
test: $(TEST_PID_RBS_VEC_NAME) $(TEST_PIDCONTROLLER_NAME)

clean:
	rm -f $(OBJS)

fclean: clean
	rm -f $(NAME) $(TEST_PID_RBS_VEC_NAME) $(TEST_PIDCONTROLLER_NAME) $(GTEST_ALL_MODULES_NAME)

re: fclean all
