cmake_minimum_required(VERSION 3.0)

cmake_policy(SET CMP0048 NEW)
project(sros_test VERSION 0.0.1 LANGUAGES CXX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

include_directories(../..)

find_package(PkgConfig)
pkg_search_module(GTEST REQUIRED gtest_main)

include(CTest)

# TODO(pengjiali): 将下面提取成函数
add_executable(async_condition_variable_test async_condition_variable_test.cpp)
target_link_libraries(async_condition_variable_test ${GTEST_LDFLAGS})
target_compile_options(async_condition_variable_test PUBLIC ${GTEST_CFLAGS})
add_test(NAME async_condition_variable_test COMMAND async_condition_variable_test)

add_executable(sick_security_unit_test sick_security_unit_test.cpp)
target_link_libraries(sick_security_unit_test ${GTEST_LDFLAGS} boost_system)
target_compile_options(sick_security_unit_test PUBLIC ${GTEST_CFLAGS})
add_test(NAME sick_security_unit_test COMMAND sick_security_unit_test)