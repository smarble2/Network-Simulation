CXX = g++
CXXFLAGS = -Wall -g

simulator: nodes.h simulator.cpp
	$(CXX) $(CXXFLAGS) nodes.h simulator.cpp -o simulator


run:
	./simulator
val:
	valgrind ./test
clean:
	rm simulator
	rm *.o
	rm *~
