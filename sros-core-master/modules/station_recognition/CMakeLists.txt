
add_subdirectory(recog_processor)

sr_library(module-station_recognition
        SRCS
            station_recognition_module.cpp
        HDRS
            station_recognition_module.h
        DEPENDS
            feature_recognition
        )
