

sr_library(module-network_server
        SRCS
            server/connection.cpp
            server/connection_manager.cpp
            server/server.cpp
            client/client.cpp
            icmp/ping.cpp
            real_time_detect_info.cpp
        HDRS
            server/connection.h
            server/connection_manager.h
            server/server.h
            client/client.h
            icmp/ping.hpp
            real_time_detect_info.h
        DEPENDS
            core_proto
            protobuf-lite
        )

sr_library(module-network
        SRCS
            network_module.cpp
        HDRS
            network_module.h
        DEPENDS
            module-network_server
        )
