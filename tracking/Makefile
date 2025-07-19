# Compiler settings
CXX := g++
CXXFLAGS :=  -std=c++20 -g -pedantic -Wall -Wextra  -Wshadow -Wconversion -Wunreachable-code #-fsanitize=address  # -Werror
LDFLAGS := -I/usr/include/eigen3
TARGET := my_eigen_program

# Auto-detect all .cpp files in the current directory
SRCS := $(wildcard src/*.cpp)
OBJS := $(SRCS:.cpp=.o)

# Default target
all: $(TARGET)
	@$(MAKE) clean-o

# Build the target executable
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) $(OBJS) -o $(TARGET) $(LDFLAGS)

# Compile each .cpp file into .o
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@


# Clean up
clean:
	rm -f $(OBJS) $(TARGET)

clean-o:
	rm -f $(OBJS)

.PHONY: all clean-o clean
