cmake_minimum_required(VERSION 2.8.3)
project(livox_module)

add_subdirectory(sdk_core)

include_directories(
    modules/perception/sensor/livox/sdk_core/include
)

## Add cmake target dependencies of the executable
## same as for the library above
sr_library(module-livox_midxx
    SRCS
    livox_midxx_module.cpp
    lds_lidar.cpp

    HDRS
    livox_midxx_module.h
    lds_lidar.h

    DEPENDS
    module-device
    )

target_link_libraries(module-livox_midxx
    ${PROJECT_NAME}_static
    )
