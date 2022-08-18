cmake_minimum_required(VERSION 2.8)

set(PERCEPTION_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/lib)
message("PERCEPTION_INCLUDE_DIR" = ${PERCEPTION_INCLUDE_DIR})

add_subdirectory(card_detection)
add_subdirectory(circle_detection)
add_subdirectory(custom_region_detection)
add_subdirectory(put_space_detection)
add_subdirectory(handcart_detection)
add_subdirectory(QRcode_detection)
add_subdirectory(QRcode_detection_global)
add_subdirectory(decards_detection)
