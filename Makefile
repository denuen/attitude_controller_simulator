TEST_PID_RBS_VEC = test_pid_rbs_vec
TEST_PIDCONTROLLER = test_pidcontroller
GTEST_ALL_MODULES = gtest_all_modules

CXX = c++
CXXFLAGS = -Wall -Wextra -Werror -std=c++98
CXXFLAGS_GTEST = -Wall -Wextra -Werror -std=c++17

INCLUDES = -I includes
GTEST_INC = -I/opt/homebrew/include
GTEST_LIB = -L/opt/homebrew/lib -lgtest -lgtest_main -pthread

SRC_DIR = src
TEST_DIR = tests

SRC_CONTROL = $(wildcard $(SRC_DIR)/control/*.cpp)
SRC_PHYSICS = $(wildcard $(SRC_DIR)/physics/*.cpp)
SRC_SENSOR = $(wildcard $(SRC_DIR)/sensor/*.cpp)

SRC = $(SRC_CONTROL) $(SRC_PHYSICS) $(SRC_SENSOR)
OBJ = $(SRC:.cpp=.o)

all: $(TEST_PID_RBS_VEC) $(TEST_PIDCONTROLLER) $(GTEST_ALL_MODULES)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

$(TEST_PID_RBS_VEC): $(SRC) $(TEST_DIR)/test_PID_RBS_Vec.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@

$(TEST_PIDCONTROLLER): $(SRC) $(TEST_DIR)/testPIDController.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@

$(GTEST_ALL_MODULES): $(SRC) $(TEST_DIR)/google_test.cpp
	$(CXX) $(CXXFLAGS_GTEST) $(INCLUDES) $(GTEST_INC) $(GTEST_LIB) $^ -o $@

clean:
	rm -f $(OBJ)

fclean: clean
	rm -f $(TEST_PID_RBS_VEC) $(TEST_PIDCONTROLLER) $(GTEST_ALL_MODULES)

re: fclean all
