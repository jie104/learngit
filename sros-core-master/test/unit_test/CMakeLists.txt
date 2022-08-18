cmake_minimum_required(VERSION 3.10)
project(unit_test)


set(CMAKE_CXX_COMPILER g++)
add_definitions("-Wall -std=c++11") 

add_executable(unit_test
	gtest_main.cpp
	samples/sample1.cpp
	samples/sample1_unittest.cpp
	samples/sample2.cpp
	samples/sample2_unittest.cpp
	samples/sample3_unittest.cpp
	samples/sample4.cpp
	samples/sample4_unittest.cpp
	samples/sample5_unittest.cpp
	samples/sample6_unittest.cpp
	samples/sample8_unittest.cpp
)



target_link_libraries(unit_test -lgtest -lgmock -lpthread)