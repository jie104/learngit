add_library(bag_driver "")

target_sources(bag_driver
    PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/bag_driver.cpp
)

target_include_directories(bag_driver 
        PUBLIC    
        $<BUILD_INTERFACE:${IMU_BASE_DIR}>
        )  
        

target_link_libraries(bag_driver
        PUBLIC
)

target_compile_options(atom_driver
    PUBLIC
    -fPIC
)
