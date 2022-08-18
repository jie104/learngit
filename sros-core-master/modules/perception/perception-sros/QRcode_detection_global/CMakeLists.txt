cmake_minimum_required(VERSION 2.8)

set(SUB_DETECTOR_NAME QRcode_detector_global)
set(SUB_DETECTOR_DESC "Perception library QRcode detect global.")

add_library(${SUB_DETECTOR_NAME}
        QRcode_detection_global.cpp
    )

target_include_directories(${SUB_DETECTOR_NAME} PUBLIC
    ${PERCEPTION_INCLUDE_DIR}
)

set_target_properties(${SUB_DETECTOR_NAME}
    PROPERTIES VERSION ${PERCEPTION_VERSION}
    SOVERSION ${ver_major}.${ver_minor}
    )
