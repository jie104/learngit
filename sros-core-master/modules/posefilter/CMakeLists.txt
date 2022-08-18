cmake_minimum_required(VERSION 3.5)
project(module-posemanager)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NONE_SROS)
    get_filename_component(SROS_DIR ${CMAKE_CURRENT_LIST_DIR} DIRECTORY)
    get_filename_component(SROS_DIR ${SROS_DIR} DIRECTORY)
    include_directories(
    ${SROS_DIR}
)
endif()

add_subdirectory(concise_odometry)

if(NOT NONE_SROS)
    add_library(${PROJECT_NAME} 
        PoseManagerModule.cpp motion_state_detector.cpp )

    target_link_libraries(${PROJECT_NAME}
        PUBLIC
        concise_odometry
    )
endif()


