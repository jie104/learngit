
add_library(concise_odometry "")

target_sources(concise_odometry
    PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/concise_odometry.cpp
)

target_include_directories(concise_odometry
    PUBLIC
    $<BUILD_INTERFACE:${INCLUDE_BASE_DIR}>
    $<BUILD_INTERFACE:${INCLUDE_BASE_DIR}/msgs>
    $<BUILD_INTERFACE:${INCLUDE_BASE_DIR}/concise_odometry>
)

# target_link_libraries(concise_odometry
#     atom_imu_reader
#     lins_driver
#     asensing_driver
# )