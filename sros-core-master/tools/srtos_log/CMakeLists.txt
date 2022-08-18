cmake_minimum_required(VERSION 2.8)
project(sros_srtos_log)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC")

add_executable(srtos_log spi_srtos.c)
target_link_libraries(pthread)
