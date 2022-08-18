add_library(asensing_driver "")

target_sources(asensing_driver
        PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/asensing_driver.cpp
)

target_include_directories(asensing_driver 
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/3rd>
        $<BUILD_INTERFACE:${IMU_BASE_DIR}> 
)  
     

target_link_libraries(asensing_driver
        PUBLIC
        boost_filesystem
)

target_compile_options(asensing_driver
    PUBLIC
    -fPIC
)