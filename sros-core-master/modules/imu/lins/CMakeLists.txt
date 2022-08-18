add_library(lins_driver "")

add_subdirectory(3rd/lins_imu_reader)

target_sources(lins_driver
        PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/lins_driver.cpp
)

target_include_directories(lins_driver 
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/3rd>
        $<BUILD_INTERFACE:${IMU_BASE_DIR}>
        )   

target_link_libraries(lins_driver
        PUBLIC
        lins_imu_reader
)

target_compile_options(lins_driver
    PUBLIC
    -fPIC
)
