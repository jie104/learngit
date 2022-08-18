cmake_minimum_required(VERSION 2.8)
project(sros-core)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#set(SRC_SDK_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../src-sdk/lib)


include(GetGitRevisionDescription)
include(GetCompilerInfoString)

get_git_head_revision(GIT_VERSION, GIT_VERSION_SHA1)
get_compiler_info_string(COMPILER_INFO)

string(SUBSTRING ${GIT_VERSION_SHA1} 0 7 GIT_VERSION_SHA1_SHORT)

configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/version.h.in
        ${CMAKE_CURRENT_SOURCE_DIR}/version.h)


sr_library(core_util
        SRCS
        util/md5.cpp
        util/pose_saver.cpp
        util/time.cpp
        util/timer.cpp
        util/gpio_sysfs.cpp
        util/utils.cpp
        util/distribution_plot.cpp
        HDRS
        util/md5.h
        util/pose_saver.h
        util/time.h
        util/timer.h
        util/gpio_sysfs.h
        util/utils.h
        util/queue.h
        util/json.h
        util/async_condition_variable.hpp
        util/read_write_lock_container.hpp
        util/distribution_plot.h
        )

sr_library(core_msg
        SRCS
        msg/base_msg.h
        msg/notification_msg.hpp
        msg/laser_scan_msg.hpp
        msg/str_msg.hpp
        msg/SlamStateMsg.h
        msg/SlamCommandMsg.h
        msg/PoseStampedMsg.h
        #            msg/network_task_msg.hpp
        msg/common_state_msg.hpp
        msg/common_command_msg.hpp
        msg/path_msg.hpp
        msg/ObstacleMsg.hpp
        msg/command_msg.hpp
        msg/parameter_msg.hpp
        msg/usart_data_msg.hpp
        msg/sonar_data_msg.hpp
        msg/common_msg.hpp
        msg/image_msg.hpp
        msg/huawei_comm_data_msg.hpp
        msg/can_data_msg.hpp
        msg/modbus_register_msg.hpp
        msg/emergency_msg.h
        msg/hmi_msg.hpp

        # odom fusion
        msg/imu_msg.hpp
        msg/std_msg.hpp
        msg/odometry_msg.hpp
        msg/geometry_msg.hpp
        )

sr_library(core_core
        SRCS
        core.h
        module.cpp
        module.h
        msg_bus.cpp
        msg_bus.h
        msg_queue.cpp
        msg_queue.h
        pose.h
        module_manager.cpp
        module_manager.h
        map_manager.cpp
        map_info.hpp
        log_helper.hpp
        control_mutex.hpp
        state.h
        state.cpp
        navigation_path.cpp
        navigation_path.h
        user_manager.cpp
        user_manager.h
        settings.cpp
        settings.h
        settings_db.cpp
        settings_db.h
        src.cpp
        src.h
        version.h
        session_manager.cpp
        session_manager.h
        temp/global_bag_config.cpp
        temp/global_bag_config.h
        db/db.h
        db/db.cpp
        linear_algebra_func.cpp
        linear_algebra_func.h
        logger.cpp
        logger.h
        user_log.cpp
        user_log.h
        exec_error.hpp
        fault_center.cpp
        fault_center.h
        fault_code.h
        dump.h
        dump.cpp
        DEPENDS
        boost_filesystem
        src-sdk
        )

sr_library(core_task
        SRCS
        task/task.cpp
        task/action_task.cpp
        task/movement_task.cpp
        task/task_manager.cpp
        HDRS
        task/task.h
        task/action_task.h
        task/movement_task.h
        task/task_manager.h
        )

sr_library(core_mission
        SRCS
        mission/dataModel/abstract_step.cpp
        mission/dataModel/step_begin.cpp
        mission/dataModel/step_end.cpp
        mission/dataModel/step_register.cpp
        mission/dataModel/step_action.cpp
        mission/dataModel/step_move.cpp
        mission/dataModel/step_set_reg.cpp
        mission/dataModel/step_wait_reg_update.cpp
        mission/dataModel/step_wait_time.cpp
        mission/dataModel/step_pause.cpp
        mission/dataModel/step_decision.cpp
        mission/dataModel/step_mission.cpp
        mission/dataModel/step_parallel.cpp
        mission/mission_manager.cpp
        mission/mission_info.cpp
        mission/mission_storage.cpp
        mission/mission_instance.cpp
        HDRS
        mission/constant.h
        mission/dataModel/abstract_step.h
        mission/dataModel/step_begin.h
        mission/dataModel/step_end.h
        mission/dataModel/step_register.h
        mission/dataModel/step_action.h
        mission/dataModel/step_move.h
        mission/dataModel/step_set_reg.h
        mission/dataModel/step_wait_reg_update.h
        mission/dataModel/step_wait_time.h
        mission/dataModel/step_pause.h
        mission/dataModel/step_decision.h
        mission/dataModel/step_mission.h
        mission/dataModel/step_parallel.h
        mission/mission_manager.h
        mission/mission_info.h
        mission/mission_storage.h
        mission/mission_instance.h
        )

sr_library(core_modbus
        SRCS
        modbus/register_admin.cpp
        modbus/modbus_action_controller.cpp
        modbus/modbus_synchronizer.cpp
        HDRS
        modbus/sr_modbus_address.h
        modbus/register_admin.h
        modbus/modbus_action_controller.h
        modbus/modbus_synchronizer.h
        )

sr_library(core_log
        SRCS
        log/run_logger.cpp
        HDRS
        log/run_logger.h
        )

sr_library(core_rack
        SRCS
        rack/rack_operator_instance.cpp
        HDRS
        rack/rack_operator_instance.hpp
        )

sr_library(core_schedule
        SRCS
        schedule/abstrace_schedule.cpp
        schedule/timer_schedule.cpp
        schedule/register_schedule.cpp
        schedule/schedule_admin.cpp
        HDRS
        schedule/constant.h
        schedule/abstrace_schedule.h
        schedule/timer_schedule.h
        schedule/register_schedule.h
        schedule/schedule_admin.h
        )

add_subdirectory(usart)

add_subdirectory(monitor)

add_subdirectory(device)

add_subdirectory(hardware)

add_subdirectory(proto)

sr_combined_library(sros-core
        SRCS
        core_util
        core_msg
        core_core
        core_task
        core_modbus
        core_mission
        core_schedule
        core_usart
        core_monitor
        #            core_file_manager
        core_log
        core_device
        core_hardware
        core_proto
        core_rack
        )
