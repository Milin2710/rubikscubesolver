CXX      = g++
CXXFLAGS = -std=c++17 -O2 -Wall -Wextra
LDFLAGS  =

SRCS = cube.cpp pdb.cpp solver.cpp main.cpp
OBJS = $(SRCS:.cpp=.o)
TARGET = rubiks_solver

.PHONY: all clean test

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

# Dependencies
cube.o:   cube.cpp cube.h
pdb.o:    pdb.cpp pdb.h cube.h
solver.o: solver.cpp solver.h cube.h pdb.h
main.o:   main.cpp cube.h pdb.h solver.h

test: $(TARGET)
	./$(TARGET) test

clean:
	rm -f $(OBJS) $(TARGET) *.pdb
