cmake_minimum_required(VERSION 2.8)
project(sros-thirty-party)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#include_directories(${CMAKE_CURRENT_LIST_DIR}/SQLiteCpp/include/)

add_subdirectory(SQLiteCpp)

#add_subdirectory(gflags-2.2.1)

add_subdirectory(stb_image_write)

#add_subdirectory(json)

add_subdirectory(libmodbus)

set(MODULE_DMTX
        )
if(ENABLE_VISION)
#add_subdirectory(dmtx)
set(MODULE_DMTX
        ${MODULE_DMTX}
#        dmtx_lib)
    )

endif()
sr_combined_library(sros-thirty-party
        SRCS
        sqlite3
        SQLiteCpp
#            gflags
        stb_image_write
        libmodbus
#        ${MODULE_DMTX}
        )
