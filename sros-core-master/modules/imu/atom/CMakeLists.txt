

add_library(atom_driver "")

add_subdirectory(3rd/atom_imu_reader)

target_sources(atom_driver
#     PUBLIC
    PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/atom_driver.cpp
)

target_include_directories(atom_driver 
        PUBLIC    
        $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/3rd>
        $<BUILD_INTERFACE:${IMU_BASE_DIR}>
        )  
        

target_link_libraries(atom_driver
        PUBLIC
        atom_imu_reader
)

target_compile_options(atom_driver
    PUBLIC
    -fPIC
)
