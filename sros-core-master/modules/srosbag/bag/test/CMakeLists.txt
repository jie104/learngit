

add_executable(test
  test.cpp
)

target_include_directories(test
    PUBLIC
    ${BAG_INCLUDE_DIR}
)

target_link_libraries(test
  PUBLIC
  msg_bag
)

# add_executable(test_camera
#         test_camera.cpp)
# target_include_directories(test_camera
#         PUBLIC
#         ${BAG_INCLUDE_DIR})
# target_link_libraries(test_camera
#         PUBLIC
#         msg_bag)