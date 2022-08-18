cmake_minimum_required(VERSION 3.0)

cmake_policy(SET CMP0048 NEW)
project(sros_test VERSION 0.0.1 LANGUAGES CXX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fext-numeric-literals")

include_directories(../..)
include_directories(../../thirty-party/SQLiteCpp/include/)

find_package(PkgConfig)
pkg_search_module(GTEST REQUIRED gtest_main)

include(CTest)

# 新建test可以参考这个
#add_executable(gtest_helper gtest_helper.cpp)
#target_link_libraries(gtest_helper ${GTEST_LDFLAGS})
#target_compile_options(gtest_helper PUBLIC ${GTEST_CFLAGS})
#add_test(NAME gtest_helper COMMAND gtest_helper)

add_executable(async_condition_variable_test async_condition_variable_test.cpp)
target_link_libraries(async_condition_variable_test ${GTEST_LDFLAGS})
target_compile_options(async_condition_variable_test PUBLIC ${GTEST_CFLAGS})
add_test(NAME async_condition_variable_test COMMAND async_condition_variable_test)

#if (CMAKE_CROSSCOMPILING)
add_executable(src_test src_test.cpp ../../modules/security/security_state_handle.cpp)
target_link_libraries(src_test
        ${GTEST_LDFLAGS}
        sros-core
        src-sdk
        boost_thread
        boost_system
        boost_filesystem
        glog
        sros-thirty-party
        dl
        )
target_compile_options(src_test PUBLIC ${GTEST_CFLAGS})
add_test(NAME src_test COMMAND src_test)
#endif ()

add_executable(src_usart_test src_usart_test.cpp ../../modules/security/security_state_handle.cpp)
target_link_libraries(src_usart_test
        ${GTEST_LDFLAGS}
        sros-core
        src-sdk
        boost_thread
        boost_system
        boost_filesystem
        glog
        sros-thirty-party
        dl
        )
target_compile_options(src_usart_test PUBLIC ${GTEST_CFLAGS})
add_test(NAME src_usart_test COMMAND src_usart_test)


add_executable(time_unit_test time_unit_test.cpp ../../core/util/time.cpp)
target_link_libraries(time_unit_test ${GTEST_LDFLAGS})
target_compile_options(time_unit_test PUBLIC ${GTEST_CFLAGS})
add_test(NAME time_unit_test COMMAND time_unit_test)

if (CMAKE_CROSSCOMPILING)
else ()
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
    set(CMAKE_AUTOUIC ON)
    set(CMAKE_AUTOMOC ON)
    set(CMAKE_AUTORCC ON)
    set(CMAKE_CXX_STANDARD 11)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    # 需要设置系统中Qt cmake的路径 export PATH=$PATH:/home/john/Qt5.9.8/5.9.8/gcc_64/lib/cmake/Qt5/:/home/john/Qt5.9.8/5.9.8/gcc_64/lib/cmake/Qt5Widgets/
    find_package(Qt5 COMPONENTS Widgets Gui REQUIRED)
    add_executable(navigation_on_net_test
            navigation_on_net_test.cpp
            ../../modules/navigation/navigation_on_net.cpp
            ../../core/linear_algebra_func.cpp
            ../../core/navigation_path.cpp
            ../../core/util/utils.cpp
            ../../core/util/time.cpp
            navigation_info_peep.cpp)
    target_link_libraries(navigation_on_net_test
            ${GTEST_LDFLAGS}
            boost_system
            boost_filesystem
            glog
            Qt5::Widgets
            Qt5::Gui
            )
    target_compile_options(navigation_on_net_test PUBLIC ${GTEST_CFLAGS})
    add_test(NAME navigation_on_net_test COMMAND navigation_on_net_test)
endif ()

add_executable(json_test json_test.cpp)
target_link_libraries(json_test
        boost_thread
        boost_system
        boost_filesystem
        )
target_link_libraries(json_test ${GTEST_LDFLAGS})
target_compile_options(json_test PUBLIC ${GTEST_CFLAGS})
add_test(NAME json_test COMMAND json_test)

add_executable(distribution_plot_test distribution_plot_test.cpp
        ../../core/util/distribution_plot.cpp)
target_link_libraries(distribution_plot_test ${GTEST_LDFLAGS})
target_compile_options(distribution_plot_test PUBLIC ${GTEST_CFLAGS})
add_test(NAME distribution_plot_test COMMAND distribution_plot_test)

add_executable(user_log_test user_log_test.cpp)
target_link_libraries(user_log_test
        ${GTEST_LDFLAGS}
        sros-core
        boost_thread
        boost_system
        boost_filesystem
        glog
        sros-thirty-party
        dl
        )
target_compile_options(user_log_test PUBLIC ${GTEST_CFLAGS})
add_test(NAME user_log_test COMMAND user_log_test)

add_executable(path_checker_test
        path_checker_test.cpp ../../modules/main/path_checker.cpp ../../modules/security/security_state_handle.cpp)
target_link_libraries(path_checker_test
        ${GTEST_LDFLAGS}
        sros-core
        src-sdk
        boost_thread
        boost_system
        boost_filesystem
        glog
        sros-thirty-party
        dl
        )
target_compile_options(path_checker_test PUBLIC ${GTEST_CFLAGS})
add_test(NAME path_checker_test COMMAND path_checker_test)

add_executable(fault_center_test fault_center_test.cpp  ../../modules/security/security_state_handle.cpp)
target_link_libraries(fault_center_test
        ${GTEST_LDFLAGS}
        sros-core
        src-sdk
        boost_thread
        boost_system
        boost_filesystem
        glog
        sros-thirty-party
        dl
        )
target_compile_options(fault_center_test PUBLIC ${GTEST_CFLAGS})
add_test(NAME fault_center_test COMMAND fault_center_test)


add_executable(async_serial_test async_serial_test.cpp)
target_link_libraries(async_serial_test
        ${GTEST_LDFLAGS}
        sros-core
        boost_thread
        boost_system
        boost_filesystem
        glog
        sros-thirty-party
        dl
        )
target_compile_options(async_serial_test PUBLIC ${GTEST_CFLAGS})
add_test(NAME async_serial_test COMMAND async_serial_test)