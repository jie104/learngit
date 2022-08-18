
aux_source_directory(. MODULE_SRCS)
sr_library(module-location
        SRCS
            location_processor.cpp
            location_module.cpp
            loc_processor/loc_processor_factory.cpp
            loc_processor/lmk_loc_processor.cpp
            loc_processor/slam_loc_processor.cpp
        HDRS
            location_processor.h
            location_module.h
            loc_processor/pgv_processor.hpp
            loc_processor/loc_processor_factory.h
            loc_processor/lmk_loc_processor.h
            loc_processor/slam_loc_processor.h
        DEPENDS
        )
