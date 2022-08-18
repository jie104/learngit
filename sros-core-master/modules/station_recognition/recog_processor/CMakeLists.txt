
add_subdirectory(angle_recog)
add_subdirectory(feat_recog)

sr_library(feature_recognition
        SRCS
            base_recog_processor.cpp
            recog_processor_factory.cpp
        HDRS
            base_recog_processor.hpp
            rack_lmk_processor.hpp
            angle_recog_processor.hpp
            charging_pile_recog_processor.hpp
            recog_processor_factory.h
        DEPENDS
            feat_recognition
            angle_recognition
        )
