cmake_minimum_required(VERSION 2.8)
project(tools-init)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_executable(init init.cpp)

target_link_libraries(init SQLiteCpp sqlite3 dl pthread)
