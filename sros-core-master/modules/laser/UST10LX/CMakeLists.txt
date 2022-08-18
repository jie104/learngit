
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/urg_c/src URG_DRIVER_SRCS)

sr_library(module-laser_UST10LX
        SRCS
            ust10lx_module.cpp
            urg_c_wrapper.cpp
            ${URG_DRIVER_SRCS}
        HDRS
            ust10lx_module.h
            urg_c_wrapper.h
        )
