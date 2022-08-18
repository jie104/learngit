cmake_minimum_required(VERSION 3.5)
project(module-bag
        LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE "Release")

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++17" COMPILER_SUPPORTS_CXX17)
check_cxx_compiler_flag("-std=c++14" COMPILER_SUPPORTS_CXX14)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)

if(COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has c++17 support.")
elseif(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_STANDARD 14)
    set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has c++14 support.")
elseif(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_STANDARD 11)
    set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has c++11 support.")
else()
    # message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(INCLUDE_DIR ${CMAKE_CURRENT_LIST_DIR}/bag/include)

# set(NONE_SROS ON)

if(${NONE_SROS})
    get_filename_component(SROS_DIR ${CMAKE_CURRENT_LIST_DIR} DIRECTORY)
    get_filename_component(SROS_DIR ${SROS_DIR} DIRECTORY)
    # find_package(glog REQUIRED)
    find_package(OpenCV REQUIRED)

    include_directories(
        ${SROS_DIR}
        ${OpenCV_INCLUDE_DIRS}
    )
    
    file(GLOB SRC ${SROS_DIR}/core/util/*)
    add_library(util SHARED
        ${SRC}
    )

    link_libraries(
        util
        # glog::glog
        glog
        pthread
        ${OpenCV_LIBS}
    )


endif()

add_subdirectory(message_registration)
add_subdirectory(bag)


if(NOT NONE_SROS)

    add_library(${PROJECT_NAME} STATIC "" ../laser/standard_lidar_protocol/oradar_laser_protocol.hpp)
    target_sources(${PROJECT_NAME}
        PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/sros_bag_module.cpp
    )

    target_include_directories(${PROJECT_NAME}
        PUBLIC
        ${INCLUDE_DIR}
    )

    target_link_libraries(${PROJECT_NAME}
        PUBLIC
        msg_bag
    )

endif()