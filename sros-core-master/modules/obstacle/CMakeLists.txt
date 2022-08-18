
project(module-obstacle)

sr_library(module-obstacle
        SRCS
            obstacle_module.cpp
            sensor_manager/sensor_process_alg.cpp
            rack_oba_filter_singleton.cpp
        HDRS
            obstacle_module.h
            sensor_manager/sensor_process_alg.h
            rack_oba_filter_singleton.h
        DEPENDS
        )
