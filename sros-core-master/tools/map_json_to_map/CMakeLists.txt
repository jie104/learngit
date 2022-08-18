cmake_minimum_required(VERSION 2.8)
project(sros_map_json_to_map)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC")

add_executable(sros_map_json_to_map map_json_to_map.cpp)

if(CHIP_VERSION STREQUAL "nxp")
    target_link_libraries(sros_map_json_to_map
        sros-core
        core_core
        src-sdk
        sros-modules
        sros-thirty-party
        SQLiteCpp
        sqlite3
        pthread
        dl
        boost_thread
        boost_system
        boost_filesystem
        glog
        libpng12.so
        )
else()
    target_link_libraries(sros_map_json_to_map
        sros-core
        core_core
        src-sdk
        sros-modules
        sros-thirty-party
        SQLiteCpp
        sqlite3
        pthread
        dl
        boost_thread
        boost_system
        boost_filesystem
        glog
        libpng16.so
        )
endif()
