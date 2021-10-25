all: test test2 test-com

test: test.cpp Eigen.h
	g++ -I/usr/include/eigen3 test.cpp -lyaml-cpp -o test

test2: test2.cpp
	g++ -I/usr/include/eigen3 test2.cpp -lyaml-cpp -o test2

test-com: test-com.c kbhit.c
	cc -o test-com test-com.c kbhit.c

clean:
	rm test test2 test-com
