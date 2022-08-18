

add_library(my_factory STATIC "")

target_sources(my_factory
    PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/factory.cpp
)

target_include_directories(my_factory
    PUBLIC
    $<BUILD_INTERFACE:${BAG_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${BAG_INCLUDE_DIR}/products>
)