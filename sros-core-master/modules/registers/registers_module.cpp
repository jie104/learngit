/**
 * @file registers_module
 *
 * @author pengjiali
 * @date 2/13/20.
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#include "registers_module.h"
#include <ifaddrs.h>
#include "core/fault_center.h"
#include "core/log/run_logger.h"
#include "core/mission/mission_manager.h"
#include "core/settings.h"
#include "core/state.h"
#include "core/task/task_manager.h"
#include "core/version.h"

namespace sros {

void RegistersModule::run() {
    LOG(INFO) << "RegistersModule 正在初始化...";

    reg_admin_ = core::RegisterAdmin::getInstance();

    subscribeTopic("TIMER_100MS", CALLBACK(&RegistersModule::onTimer_100ms));
    subscribeTopic("TIMER_5S", CALLBACK(&RegistersModule::onTimer_5s));

    reg_admin_->setLock();
    // 设置版本号
    setInputRegister16(IRA_SYSTEM_MAJOR_VERSION, SROS_MAJOR_VERSION);
    setInputRegister16(IRA_SYSTEM_MINOR_VERSION, SROS_MINOR_VERSION);
    setInputRegister16(IRA_SYSTEM_PATCH_VERSION, SROS_PATCH_VERSION);
    reg_admin_->unlock();

    dispatch();
}

void RegistersModule::onTimer_100ms(core::base_msg_ptr m) {
    reg_admin_->setLock();
    updateAllDiscreteInputs();
    updateAllInputRegister();
    reg_admin_->unlock();
}

void RegistersModule::onTimer_5s(core::base_msg_ptr m) {
    // 设置ip
    std::string ip_address = getIPaddress();
    if (ip_address_ != ip_address) {
        ip_address_ = ip_address;
        g_state.ip_addr.set(ip_address);

        std::stringstream ip_stream(ip_address);
        uint16_t num = 0;
        char dot;

        reg_admin_->setLock();
        ip_stream >> num;
        setInputRegister16(IRA_IP_1, num);

        ip_stream >> dot;
        ip_stream >> num;
        setInputRegister16(IRA_IP_2, num);

        ip_stream >> dot;
        ip_stream >> num;
        setInputRegister16(IRA_IP_3, num);

        ip_stream >> dot;
        ip_stream >> num;
        setInputRegister16(IRA_IP_4, num);
        reg_admin_->unlock();
    }
}

void RegistersModule::updateAllDiscreteInputs() {
    // NOTE: 数据在tab_input_bits中存储的格式是： 每一Bit存储一位，具体参见modbus::response_io_status()函数
    auto setDiscreteInputFun = [&](uint16_t address, bool enabled) {
        uint16_t value = 0;
        if (enabled) {
            value = 1;
        }
        reg_admin_->setDiscreteInput(address, value, false);
    };
    setDiscreteInputFun(DIA_EMERGENCY_STATE, g_state.isEmergency());
    setDiscreteInputFun(DIA_EMERGENCY_RECOVERABLE_STATE, g_state.isEmergencyRecoverable());
    setDiscreteInputFun(DIA_BREAK_SWITCH_STATE, g_state.isBreakSwitchON());
    setDiscreteInputFun(DIA_CHARGE_STATE, g_state.isChargeState());
    setDiscreteInputFun(DIA_POWER_SAVE_MODE, g_state.isPowerSaveMode());
    setDiscreteInputFun(DIA_SLOWDOWN_FOR_OBSTACLE_STATE, g_state.isSlowdownForObstacle());
    setDiscreteInputFun(DIA_PAUSED_FOR_OBSTACLE_STATE, g_state.isPausedForObstacle());
    setDiscreteInputFun(DIA_READY_FOR_NEW_MOVEMENT_TASK, g_state.ready_for_new_movement_task);

    setDiscreteInputFun(DIA_DI_0_STATE, g_state.gpio_input & 0x01);
    setDiscreteInputFun(DIA_DI_1_STATE, g_state.gpio_input & 0x02);
    setDiscreteInputFun(DIA_DI_2_STATE, g_state.gpio_input & 0x04);
    setDiscreteInputFun(DIA_DI_3_STATE, g_state.gpio_input & 0x08);
    setDiscreteInputFun(DIA_DI_4_STATE, g_state.gpio_input & 0x10);
    setDiscreteInputFun(DIA_DI_5_STATE, g_state.gpio_input & 0x20);
    setDiscreteInputFun(DIA_DI_6_STATE, g_state.gpio_input & 0x40);
    setDiscreteInputFun(DIA_DI_7_STATE, g_state.gpio_input & 0x80);

    setDiscreteInputFun(DIA_DO_0_STATE, g_state.gpio_output & 0x01);
    setDiscreteInputFun(DIA_DO_1_STATE, g_state.gpio_output & 0x02);
    setDiscreteInputFun(DIA_DO_2_STATE, g_state.gpio_output & 0x04);
    setDiscreteInputFun(DIA_DO_3_STATE, g_state.gpio_output & 0x08);
    setDiscreteInputFun(DIA_DO_4_STATE, g_state.gpio_output & 0x10);
    setDiscreteInputFun(DIA_DO_5_STATE, g_state.gpio_output & 0x20);
    setDiscreteInputFun(DIA_DO_6_STATE, g_state.gpio_output & 0x40);
    setDiscreteInputFun(DIA_DO_7_STATE, g_state.gpio_output & 0x80);

    const int ACTION_ID_WAIT_COMMAND = 131;
    auto action_task = sros::core::TaskManager::getInstance()->getActionTask();
    bool waiting_let_go =
        action_task && action_task->isSlaveRunning() && (action_task->getActionID() == ACTION_ID_WAIT_COMMAND);
    setDiscreteInputFun(DIA_WAITING_LET_GO, waiting_let_go);
    setDiscreteInputFun(DIA_FLEET_MODE, (g_state.fleet_mode == sros::core::FLEET_MODE_ONLINE));
}

// 更新所有输入寄存器的数据
void RegistersModule::updateAllInputRegister() {
    //    LOG(INFO) << "ModbusModule::updateAllInputRegister()";

    setInputRegister16(IRA_SYS_STATE, g_state.sys_state);
    setInputRegister16(IRA_LOCATION_STATE, g_state.location_state);
    setPoseToRegister(src_sdk->getCurPose());

    auto movement_task = sros::core::TaskManager::getInstance()->getMovementTask();
    if (movement_task) {
        setInputRegister16(IRA_MOVEMENT_STATE_ABANDONED, movement_task->getState());
        setInputRegister16(IRA_MOVEMENT_TASK_STATE, movement_task->getState());
    }
    auto action_task = sros::core::TaskManager::getInstance()->getActionTask();
    if (action_task) {
        setInputRegister16(IRA_ACTION_TASK_STATE, action_task->getState());
        setInputRegister32(IRA_ACTION_RESULT_ABANDONED, action_task->getActionResultValue());
    }
    setInputRegister16(IRA_STATION_NO, g_state.station_no);
    setInputRegister16(IRA_MANUAL_CONTROL_STATE, g_state.operation_state);
    setInputRegister16(IRA_SPEED_FOR_X_STATE, g_src_state.cur_v);
    setInputRegister16(IRA_SPEED_FOR_Y_STATE, 0);  // 对应于横着走的车，暂时保留
    setInputRegister16(IRA_SPEED_FOR_YAW_STATE, g_src_state.cur_w);
    setInputRegister16(IRA_GPIO_INPUT, g_state.gpio_input);
    setInputRegister16(IRA_GPIO_OUTPUT, g_state.gpio_output);
    setInputRegister32(IRA_HARDWARE_ERROR_CODE, g_state.hardware_error_code);
    setInputRegister32(IRA_SYSTEM_LAST_ERROR, g_state.laster_error_code);

    // NOTE： int16转为uint16再转为int16符号不会被丢失！
    // 例如：电流可能小于0，单位为int16，modbus存储的寄存器为uint16，
    // 当主机读取电流对应的地址时转换成了int16，该值的符号并没有丢失
    setInputRegister16(IRA_BATTERY_VOLTAGE, g_state.battery_voltage);
    setInputRegister16(IRA_BATTERY_ELECTRICITY, g_state.battery_current);
    setInputRegister16(IRA_BATTERY_TEMPERATURE, g_state.battery_temperature);
    setInputRegister16(IRA_BATTERY_ESTIMATED_USABLE_TIME, g_state.battery_remain_time);
    setInputRegister16(IRA_BATTERY_PERCENTAGE, g_state.battery_percentage);
    setInputRegister16(IRA_BATTERY_STATE, g_state.battery_state);
    setInputRegister16(IRA_BATTERY_USE_CYCLES, g_state.battery_use_cycles);
    setInputRegister16(IRA_BATTERY_NOMINAL_CAPACITY, g_state.battery_nominal_capacity);
    setInputRegister16(IRA_VOLUME, g_state.cur_volume);

    auto &run_logger = sros::core::RunLogger::getInstance();
    setInputRegister32(IRA_TOTAL_MILEAGE, run_logger.getTotalMileage());
    setInputRegister32(IRA_POWERON_TIME, g_src_state.total_poweron_time);
    setInputRegister32(IRA_POWER_CYCLE, run_logger.getTotalBootTimes());
    setInputRegister32(IRA_SYSTEM_TIME, (uint32_t)sros::core::util::get_timestamp_in_s());

    // NOTE: ip、版本号等不变的地址，在初始化的时候就写入了

    // 下视摄像头偏差
    auto offset = g_state.cur_down_camera_offset.get();
    setInputRegister32(IRA_DOWN_PGV_ID, offset.id);
    setInputRegister32(IRA_DOWN_PGV_X, static_cast<int>(offset.x * 1000));
    setInputRegister32(IRA_DOWN_PGV_Y, static_cast<int>(offset.y * 1000));
    setInputRegister32(IRA_DOWN_PGV_YAW, static_cast<int>(offset.yaw * 1000));

    auto cur_map_name = g_state.getCurMapName();
    uint16_t map_name_16 = 0x00;
    switch (cur_map_name.size()) {
        case 0: {
            break;
        }
        case 1: {
            map_name_16 += cur_map_name.at(0) << 8;
            break;
        }
        default: {
            map_name_16 += cur_map_name.at(0) << 8;
            map_name_16 += cur_map_name.at(1);
            break;
        }
    }
    setInputRegister16(IRA_CUR_MAP, map_name_16);
    setInputRegister32(IRA_CUR_MOVEMENT_NO, sros::core::TaskManager::getInstance()->getMovementTaskNo());
    setInputRegister32(IRA_CUR_ACTION_NO, sros::core::TaskManager::getInstance()->getActionTaskNo());

    auto fault_list = sros::core::FaultCenter::getInstance()->getFaultList();
    for (int i = 0; i < 5; ++i) {
        if (fault_list->size() > i) {
            setInputRegister32(IRA_HARDWARE_ERROR_CODE_1 + 2 * i, fault_list->at(i)->id);
        } else {
            setInputRegister32(IRA_HARDWARE_ERROR_CODE_1 + 2 * i, 0);
        }
    }

    // 任务状态信息
    auto mission_manager = sros::core::MissionManager::getInstance();
    setInputRegister32(IRA_RUNNING_MISSION, mission_manager->getCurrentRootMissionId());
    setInputRegister16(IRA_MISSION_STATE, (uint16_t)(mission_manager->getCurrentMissionState()));
    setInputRegister16(IRA_MISSION_RESULT, (uint16_t)(mission_manager->getCurrentMissionResult()));

    // 移动任务状态
    if (movement_task) {
        setInputRegister16(IRA_MOVEMENT_STATE, movement_task->getState());
        setInputRegister32(IRA_MOVEMENT_NO, movement_task->getTaskNo());
        setInputRegister16(IRA_MOVEMENT_DST_STATION, movement_task->getCurDstStation());
        setInputRegister16(IRA_MOVEMENT_CUR_PATH, g_src_state.cur_path_no);
        setInputRegister16(IRA_MOVEMENT_RESULT, movement_task->getTaskResult());
        setInputRegister32(IRA_MOVEMENT_RESULT_VALUE, movement_task->getFailedCode());
    }

    // 动作任务
    if (action_task) {
        setInputRegister16(IRA_ACTION_STATE, action_task->getState());
        setInputRegister32(IRA_ACTION_NO, action_task->getTaskNo());
        setInputRegister32(IRA_ACTION_ID, action_task->getActionID());
        setInputRegister32(IRA_ACTION_PARAM_0, action_task->getActionParam());
        setInputRegister32(IRA_ACTION_PARAM_1, action_task->getActionParam1());
        setInputRegister16(IRA_ACTION_RESULT, action_task->getTaskResult());
        setInputRegister32(IRA_ACTION_RESULT_VALUE, action_task->getActionResultValue());
    }

    //硬件设备状态
    auto dm = sros::device::DeviceManager::getInstance();
    auto device_map = dm->getDeviceList();

    uint8_t cur_dev_state = 0x3F;

    for (const auto &it : *device_map) {
        const auto &device = it.second;
        auto dev_name = device->getName();

        if(dev_name == "lidar_OMD30M" && !device->isOk()) {
            cur_dev_state &= (~ sros::core::HardwareDevice::HD_LASER_NAV);
        } else if(dev_name == sros::device::DEVICE_CAMERA_D435 && !device->isOk()) {
            cur_dev_state &= (~ sros::core::HardwareDevice::HD_CAMERA_D435);
        } else if(dev_name == sros::device::DEVICE_LIDAR_LIVOX && !device->isOk()) {
            cur_dev_state &= (~ sros::core::HardwareDevice::HD_CAMERA_O3D);
        } else if(dev_name == sros::device::DEVICE_UST_LIDAR_LEFT && !device->isOk()) {
            cur_dev_state &= (~ sros::core::HardwareDevice::HD_UST_LIDAR_LEFT);
        } else if(dev_name == sros::device::DEVICE_UST_LIDAR_BACK && !device->isOk()) {
            cur_dev_state &= (~ sros::core::HardwareDevice::HD_UST_LIDAR_BACK);
        } else if(dev_name == sros::device::DEVICE_UST_LIDAR_RIGHT && !device->isOk()) {
            cur_dev_state &= (~ sros::core::HardwareDevice::HD_UST_LIDAR_RIGHT);
        }
    }

    setInputRegister32(IRA_DEVICE_STATE, cur_dev_state);

}

void RegistersModule::setInputRegister16(uint16_t addr, uint16_t value, bool lock) {
    reg_admin_->setInputRegister16(addr, value, lock);
}

void RegistersModule::setInputRegister32(uint16_t addr, int32_t value, bool lock) {
    reg_admin_->setInputRegister32(addr, value, lock);
}

void RegistersModule::setInputRegister32(uint16_t addr, uint32_t value, bool lock) {
    reg_admin_->setInputRegister32(addr, value, lock);
}

void RegistersModule::setPoseToRegister(const core::Pose &pose) {
    int pose_x = pose.x() * 1000;
    setInputRegister32(IRA_POSE, pose_x);

    int pose_y = pose.y() * 1000;
    setInputRegister32(IRA_POSE + 2, pose_y);

    int pose_yaw = pose.yaw() * 1000;
    setInputRegister32(IRA_POSE + 4, pose_yaw);

    setInputRegister16(IRA_RELIABILITY_STATE,
                       static_cast<int>(pose.confidence() * 10000));  // 可信度先写死为10000，表示100% （保留）
}

std::string RegistersModule::getIPaddress() {
    auto ip_addr = getIPaddressFromFile();

    // 先尝试从文件中获取ip，如果没有，再从网卡获取
    if (ip_addr.empty()) {
        ip_addr = getIPaddressFromAdapter();
    }

    return ip_addr;
}

std::string RegistersModule::getIPaddressFromAdapter() {
    struct ifaddrs *ifa;
    std::array<char, 16> ip_address;

    std::string local_ip_address = "0.0.0.0";

    if (getifaddrs(&ifa) == 0) {
        auto &s = sros::core::Settings::getInstance();
        std::string interface = s.getValue<std::string>("network.communication_interface", "eth0");

        for (auto ifa_it = ifa; ifa_it != NULL; ifa_it = ifa_it->ifa_next) {
            struct sockaddr_in *sin = (struct sockaddr_in *)ifa_it->ifa_addr;
            if (sin && sin->sin_family != AF_INET) continue;
            if (sin && strncmp(ifa_it->ifa_name, interface.c_str(), interface.size()) == 0) {
                unsigned char *b = (unsigned char *)(&sin->sin_addr.s_addr);
                snprintf(&ip_address.front(), ip_address.size(), "%u.%u.%u.%u", b[0], b[1], b[2], b[3]);
                local_ip_address = &ip_address.front();
            }
        }
    }

    freeifaddrs(ifa);

    return local_ip_address;
}

std::string RegistersModule::getIPaddressFromFile() {
    std::ifstream t("/tmp/nodes_client_nat_ip");
    std::stringstream buffer;
    buffer << t.rdbuf();
    std::string contents(buffer.str());

    //    LOG(INFO) << "GOT_IP: " << contents;

    return contents;
}
}  // namespace sros
