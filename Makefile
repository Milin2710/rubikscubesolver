CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall -Wextra
TARGET = rubiks_solver

SRCS = cube.cpp pdb.cpp solver.cpp main.cpp
OBJS = $(SRCS:.cpp=.o)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

test: $(TARGET)
	./$(TARGET) test

clean:
	rm -f $(OBJS) $(TARGET) *.pdb

.PHONY: all test clean
