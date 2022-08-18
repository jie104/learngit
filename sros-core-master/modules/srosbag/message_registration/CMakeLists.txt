


add_library(products  "")

# file(GLOB SRC *.cpp)

target_sources(products
    PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/message_registration.cpp
)

target_include_directories(products
    PUBLIC
    $<BUILD_INTERFACE:${INCLUDE_DIR}>
)