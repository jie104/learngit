cmake_minimum_required(VERSION 2.8)
project(module-action_controller)

aux_source_directory(. MODULE_SRCS)

sr_library(module-action_controller
        SRCS
            ${MODULE_SRCS}
        )
