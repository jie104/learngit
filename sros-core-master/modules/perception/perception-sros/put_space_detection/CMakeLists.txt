cmake_minimum_required(VERSION 2.8)

set(SUB_DETECTOR_NAME put_space_detector)
set(SUB_DETECTOR_DESC "Perception library custom put space detect.")

add_library(${SUB_DETECTOR_NAME}
    put_space_detection.cpp
    ../detector_base/detection_base.cpp
    )

target_include_directories(${SUB_DETECTOR_NAME} PRIVATE
    ${PERCEPTION_INCLUDE_DIR}
    )

target_link_libraries(${SUB_DETECTOR_NAME} PRIVATE
        libput_space_detector)


set_target_properties(${SUB_DETECTOR_NAME}
    PROPERTIES VERSION ${PERCEPTION_VERSION}
    SOVERSION ${ver_major}.${ver_minor}
    )