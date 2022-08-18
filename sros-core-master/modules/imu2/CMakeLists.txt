cmake_minimum_required(VERSION 3.5)

project(module-imu2)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++17" COMPILER_SUPPORT_CXX17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

if(${NONE_SROS})
    get_filename_component(SROS_DIR ${CMAKE_CURRENT_LIST_DIR} DIRECTORY)
    get_filename_component(SROS_DIR ${SROS_DIR} DIRECTORY)
    include_directories(
        ${SROS_DIR}
    )
    
    file(GLOB SRC ${SROS_DIR}/core/util/*)
    add_library(util SHARED
        ${SRC}
    )

endif()

add_library(imu_handle "")

target_sources(imu_handle
    PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/imu_module.cpp
)

target_link_libraries(imu_handle
    PUBLIC
    # bag_driver 
    )

if(NONE_SROS)
    target_include_directories(imu_handle
    PUBLIC
    ${SROS_DIR}
    )
    target_link_libraries(imu_handle
        PUBLIC
        util
    )
endif()

if(NOT NONE_SROS)
    add_library(${PROJECT_NAME} "")

    target_sources(${PROJECT_NAME}
        PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/imu_module.cpp
    )

    target_link_libraries(${PROJECT_NAME}
        imu_handle
    )
endif()
