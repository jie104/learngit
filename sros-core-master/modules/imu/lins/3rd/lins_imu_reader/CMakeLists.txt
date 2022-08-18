project(lins_imu_reader)

add_compile_options(-fPIC)

add_library(lins_imu_reader api_uart.c sr_ls_imu.c)
target_link_libraries(lins_imu_reader pthread glog)