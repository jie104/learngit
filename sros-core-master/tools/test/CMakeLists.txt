cmake_minimum_required(VERSION 2.8)
project(test_socket_can)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_executable(test_socket_can test_socket_can.cpp)
target_link_libraries(test_socket_can pthread glog boost_system)

add_executable(test_serial_port_echo test_serial_port_echo.cpp)
target_link_libraries(test_serial_port_echo pthread boost_system)

add_executable(test_socket_can_echo test_socket_can_echo.cpp)
target_link_libraries(test_socket_can_echo pthread glog boost_system)


if(CMAKE_CROSSCOMPILING)
    # arm交叉编译
else()
endif()
