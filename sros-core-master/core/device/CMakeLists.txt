aux_source_directory(. DEVICE_SRCS)
sr_library(core_device
        SRCS
            ${DEVICE_SRCS}
        )
