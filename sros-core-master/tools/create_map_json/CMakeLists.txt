cmake_minimum_required(VERSION 2.8)
project(sros_create_map_json)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_executable(sros_create_map_json create_map_json.cpp)

target_link_libraries(sros_create_map_json
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
        )
