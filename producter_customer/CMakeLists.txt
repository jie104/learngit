cmake_minimum_required(VERSION 3.19)
project(producter_customer)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11  -pthread" )


add_executable(producter_customer producter_customer.cpp)