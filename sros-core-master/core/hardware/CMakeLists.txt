aux_source_directory(. HARDWARE_SRCS)
sr_library(core_hardware
        SRCS
        ${HARDWARE_SRCS}
        )