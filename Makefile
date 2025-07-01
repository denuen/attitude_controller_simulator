NAME		= attitude_controller
TEST_NAME	= test_attitude_controller

CXX			= c++
CXXFLAGS	= -Wall -Wextra -Werror -std=c++98
INCLUDES	= -I includes

SRCS		= src/Vector3f.cpp src/RigidBodySimulator.cpp main.cpp
OBJS		= $(SRCS:.cpp=.o)

TEST_SRCS	= tests/testRigidBodySimulator.cpp src/Vector3f.cpp src/RigidBodySimulator.cpp
TEST_FLAGS	= -std=c++17 -I/opt/homebrew/include -L/opt/homebrew/lib -lgtest -lgtest_main -pthread

all: $(NAME)

$(NAME): $(OBJS)
	$(CXX) $(CXXFLAGS) $(OBJS) -o $(NAME)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

test:
	$(CXX) $(TEST_SRCS) $(INCLUDES) $(TEST_FLAGS) -o $(TEST_NAME)

clean:
	rm -f $(OBJS)

fclean: clean
	rm -f $(NAME) $(TEST_NAME)

re: fclean all
