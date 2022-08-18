cmake_minimum_required(VERSION 2.8)

set(SUB_DETECTOR_NAME handcart_detector)
set(SUB_DETECTOR_DESC "Perception library handcart detect.")

add_library(${SUB_DETECTOR_NAME}
    handcart_detection.cpp
    ../detector_base/detection_base.cpp
    )

add_dependencies(${SUB_DETECTOR_NAME} handcart_detector)

target_include_directories(${SUB_DETECTOR_NAME} PUBLIC
    ${PERCEPTION_INCLUDE_DIR}
)

target_link_libraries(${SUB_DETECTOR_NAME} PRIVATE
    libhandcart_detector
    )

set_target_properties(${SUB_DETECTOR_NAME}
    PROPERTIES VERSION ${PERCEPTION_VERSION}
    SOVERSION ${ver_major}.${ver_minor}
    )
