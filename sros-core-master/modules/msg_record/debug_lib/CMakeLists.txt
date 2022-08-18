cmake_minimum_required(VERSION 2.8)
project(debug_lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

aux_source_directory(. MODULE_SRCS)

set(MODULE_SRCS
        base_record_function.cpp
        base_record_function.h
        circle_func_array.hpp
        record_function_operator.hpp)
add_library(debug_lib ${MODULE_SRCS})

add_executable(debug_lib_test func_record_sample.cpp)
target_link_libraries(debug_lib_test debug_lib glog boost_system boost_thread pthread)
