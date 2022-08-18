project(atom_imu_reader)

add_compile_options(-fPIC)

add_library(atom_imu_reader protocal_wrapper.c atomprotocol.c com.c imu_receive.cpp)
target_link_libraries(atom_imu_reader pthread glog)