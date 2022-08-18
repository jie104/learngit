
sr_library(module-main
        SRCS
            main_module.cpp
            path_checker.cpp
            pose_checker.cpp
        HDRS
            main_module.h
            path_checker.h
            pose_checker.h
        DEPENDS
            src-sdk
        )

