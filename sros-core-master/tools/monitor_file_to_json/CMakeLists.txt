cmake_minimum_required(VERSION 2.8)
project(sros_monitor_file_to_json)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

execute_process(COMMAND bash ${CMAKE_SOURCE_DIR}/tools/monitor_file_to_json/create_protocol_linux.sh)

set(TOOL_FILES
    monitor_protobuf_full.pb.h
    monitor_protobuf_full.pb.cc
    protobuf_transform.h
    protobuf_transform.cpp
    parse.h
    parse.cpp
    monitor_file_to_json.cpp
    )

add_executable(sros_monitor_file_to_json ${TOOL_FILES})

target_link_libraries(sros_monitor_file_to_json
        pthread
        boost_system
        boost_filesystem
        protobuf
#        core_proto
        )
