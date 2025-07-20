# Compiler settings
CXX := g++
CXXFLAGS :=  -std=c++20 -g -pedantic -Wall -Wextra  -Wshadow -Wconversion -Wunreachable-code #-fsanitize=address  # -Werror
LDFLAGS := -I/usr/include/eigen3
TARGET := my_eigen_program.out

# Auto-detect all .cpp files in the current directory
SRCS := $(wildcard src/*.cpp)
OBJS := $(SRCS:.cpp=.o)

# Default target
all: $(TARGET)

# Build the target executable
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) $(OBJS) -o $(TARGET) $(LDFLAGS)

# Compile each .cpp file into .o
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@


# Clean up
clean:
	rm -rf $(OBJS) $(TARGET)

clean-o:
	rm -rf $(OBJS)

.PHONY: all clean-o my_program
	make clean-o