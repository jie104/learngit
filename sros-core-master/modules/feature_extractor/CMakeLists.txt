
aux_source_directory(. MODULE_SRCS)

sr_library(module-feature_extractor
        SRCS
            feature_extractor_module.cpp
        HDRS
            feature_extractor_module.h
        DEPENDS
        )