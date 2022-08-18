//
// Created by caoyan on 2020-11-12.
//

#include "src_sdk_do.h"

#include <glog/logging.h>
#include <boost/chrono/duration.hpp>
#include <boost/thread/thread.hpp>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>

#include "core/device/device_manager.h"
#include "core/state.h"
#include "core/util/timer.h"

#include "core/logger.h"
#include "core/settings.h"
#include "core/util/utils.h"
#include "core/hardware/SRC.h"
#include "core/msg_bus.h"
#include "core/msg/command_msg.hpp"
#include "protocol/src_protocol.h"
#include "protocol/sros_state_msg.hpp"
#include "core/src.h"
#include "core/fault_center.h"
#include "core/fault_code.h"

using namespace std;
using namespace src;
using namespace sros::device;

//初始化静态变量
sdk::PoseCallbackFunc_t sdk::SrcSdkDo::pose_callback_f_ = NULL ;
sdk::OptPoseCallbackFunc_t sdk::SrcSdkDo::opt_pose_callback_f_ = NULL;
sdk::StateCallbackFunc_t sdk::SrcSdkDo::state_callback_f_ = NULL;
sdk::VelocityCallbackFunc_t sdk::SrcSdkDo::velocity_callback_f_ = NULL;
sdk::ActionStateCallbackFunc_t sdk::SrcSdkDo::action_state_callback_f_ = NULL;
sdk::MonitorStateCallbackFunc_t sdk::SrcSdkDo::monitor_state_callback_f_ = NULL;
sdk::PathFinishCallbackFunc_t sdk::SrcSdkDo::path_finish_callback_f_ = NULL;
sdk::PathAbortedCallbackFunc_t sdk::SrcSdkDo::path_aborted_callback_f_ = NULL;
sdk::PathPausedCallbackFunc_t sdk::SrcSdkDo::path_paused_callback_f_ = NULL;
sdk::USARTDataCallbackFunc_t sdk::SrcSdkDo::usart_data_callback_f_ = NULL;
sdk::UpgradeCallbackFunc_t sdk::SrcSdkDo::src_upgrade_callback_f_ = NULL;
sdk::PoseTimeoutCallbackFunc_t sdk::SrcSdkDo::pose_timeout_callback_f_ = NULL;

sdk::SrcSdkDo::SrcSdkDo()
    : is_path_finished_(false),
      is_pose_available_(false),
      src_version_(0),
      odo_version_(0),
      command_msg_seq_no_(1),
      last_pose_timestamp_(0),
      last_src_pose_serial_(0),
      cur_pose_() {}

sdk::SrcSdkDo::~SrcSdkDo() {}

bool sdk::SrcSdkDo::upgradeRequest(const std::string &upgrade_bin_file_path) {
    if (!src_upgrade_.setUpgradeFilePath(upgrade_bin_file_path)) {
        return false;
    }
    src_sdk->sendCommandMsg(COMMAND_UPGRADE_REQUEST);

    return true;
}

void sdk::SrcSdkDo::upgradSrcTest() {
    CommandMsg_ptr m = std::make_shared<CommandMsg>();
    m->setCommand(COMMAND_UPGRADE_TEST);
    m->setParam0(0);
    m->setParam1(0);
    m->setParam2(0);
    m->setParam3(0);

    m->setSeqNO(command_msg_seq_no_);

    src_sdk->sendMsg(m);

    command_msg_seq_no_ += 1;  // 自增序列号
}

const sros::core::Pose &sdk::SrcSdkDo::getCurPose() {
    return cur_pose_;
}

bool sdk::SrcSdkDo::isPathFinished() {
    return is_path_finished_;
}

void sdk::SrcSdkDo::resetFault() {
    is_reset_fault_ = true;
    src_sdk->sendCommandMsg(COMMAND_RESET_FAULT);
}

void sdk::SrcSdkDo::setPoseAvailable(bool available) { is_pose_available_ = available; }

const SRCState &sdk::SrcSdkDo::getSRCState() { return state_; }

void sdk::SrcSdkDo::handleSecurityMsg(const std::vector<uint8_t> &data) {
    auto &s = sros::core::Settings::getInstance();
    if (s.getValue<std::string>("main.vehicle_controller_type", "VC300") != "VC400" &&
        s.getValue<int>("main.security_unit_type", 1) != 2) {
        return;
    }
    
    // src安全单元触发源已扩充到4个字节
    SecurityMsg_ptr msg = make_shared<SecurityMsg>();
    msg->src_length = 4;
    msg->rawData(data);
    msg->decode();
    notify_queue_.put(msg);
    // security_state_handle_.setNewState((sros::core::EmergencyState)msg->safety_state, msg->emergency_src);
}

void sdk::SrcSdkDo::setNewStateByNotify() {
    while(true) {
        try{
            auto mm = notify_queue_.get();
            auto msg = dynamic_pointer_cast<SecurityMsg>(mm);
            security_state_handle_.setNewState((sros::core::EmergencyState)msg->safety_state, msg->emergency_src);
        } catch(...) {
            LOG(INFO) << "setNewStateError";
        }
    }
}

void sdk::SrcSdkDo::handleIAPRequest(const std::vector<uint8_t> &data) {
    // LOG(INFO) <<"recv data size =  "<<data.size()<<" "<<std::hex<< (int)data[0]<<" "<<(int)data[1]<<"
    // "<<(int)data[2];
    src_upgrade_.onIAPRequest(data);
}

sros::device::Device_ptr sdk::SrcSdkDo::getDevice(const string &device_name, bool create_if_not_exist) const {
    auto dm = sros::device::DeviceManager::getInstance();

    auto device = dm->getDeviceByName(device_name);

    DeviceID device_id = DEVICE_ID_UNDEFINED;

    if (!device && create_if_not_exist) {
        if (device_name == DEVICE_MOTOR_1) {
            device_id = DEVICE_ID_MC_MOTOR_1;
            device = createDevice<Motor>(device_name, device_id, DEVICE_COMM_INTERFACE_TYPE_CAN_2, DEVICE_MOUNT_SRC);
        } else if (device_name == DEVICE_MOTOR_2) {
            device_id = DEVICE_ID_MC_MOTOR_2;
            device = createDevice<Motor>(device_name, device_id, DEVICE_COMM_INTERFACE_TYPE_CAN_2, DEVICE_MOUNT_SRC);
        } else if (device_name == DEVICE_MOTOR_3) {
            device_id = DEVICE_ID_AC_MOTOR_1;
            device = createDevice<Motor>(device_name, device_id, DEVICE_COMM_INTERFACE_TYPE_CAN_2, DEVICE_MOUNT_SRC);
        } else if (device_name == DEVICE_MOTOR_4) {
            device_id = DEVICE_ID_AC_MOTOR_2;
            device = createDevice<Motor>(device_name, device_id, DEVICE_COMM_INTERFACE_TYPE_CAN_2, DEVICE_MOUNT_SRC);
        } else if (device_name == DEVICE_IMU) {
            device_id = DEVICE_ID_IMU;
            device = dm->registerDevice(device_name, device_id, DEVICE_COMM_INTERFACE_TYPE_RS232_2, DEVICE_MOUNT_SRC);
        } else if (device_name == DEVICE_PGV_UP) {
            device_id = DEVICE_ID_PGV_UP;
            device = dm->registerDevice(device_name, device_id, DEVICE_COMM_INTERFACE_TYPE_RS232_2, DEVICE_MOUNT_SRC);
        } else if (device_name == DEVICE_PGV_DOWN) {
            device_id = DEVICE_ID_PGV_DOWN;
            device = dm->registerDevice(device_name, device_id, DEVICE_COMM_INTERFACE_TYPE_RS232_2, DEVICE_MOUNT_SRC);
        } else if (device_name == DEVICE_WALK_1) {
            device_id = DEVICE_ID_WALK_1;
            device = createDevice<Encoder>(device_name, device_id, DEVICE_COMM_INTERFACE_TYPE_CAN_2, DEVICE_MOUNT_SRC);
        } else if (device_name == DEVICE_WALK_2) {
            device_id = DEVICE_ID_WALK_2;
            device = createDevice<Encoder>(device_name, device_id, DEVICE_COMM_INTERFACE_TYPE_CAN_2, DEVICE_MOUNT_SRC);
        } else if (device_name == DEVICE_WALK_3) {
            device_id = DEVICE_ID_WALK_3;
            device = createDevice<Encoder>(device_name, device_id, DEVICE_COMM_INTERFACE_TYPE_CAN_2, DEVICE_MOUNT_SRC);
        } else if (device_name == DEVICE_ROTATE_1) {
            device_id = DEVICE_ID_ROTATE_1;
            device = createDevice<Encoder>(device_name, device_id, DEVICE_COMM_INTERFACE_TYPE_CAN_2, DEVICE_MOUNT_SRC);
        } else if (device_name == DEVICE_ROTATE_2) {
            device_id = DEVICE_ID_ROTATE_2;
            device = createDevice<Encoder>(device_name, device_id, DEVICE_COMM_INTERFACE_TYPE_CAN_2, DEVICE_MOUNT_SRC);
        } else if (device_name == DEVICE_ROTATE_3) {
            device_id = DEVICE_ID_ROTATE_3;
            device = createDevice<Encoder>(device_name, device_id, DEVICE_COMM_INTERFACE_TYPE_CAN_2, DEVICE_MOUNT_SRC);
        } else if (device_name == DEVICE_PULL_ROPE_1) {
            device_id = DEVICE_ID_PULL_ROPE_1;
            device = createDevice<Encoder>(device_name, device_id, DEVICE_COMM_INTERFACE_TYPE_CAN_2, DEVICE_MOUNT_SRC);
        } else if (device_name == DEVICE_PULL_ROPE_2) {
            device_id = DEVICE_ID_PULL_ROPE_2;
            device = createDevice<Encoder>(device_name, device_id, DEVICE_COMM_INTERFACE_TYPE_CAN_2, DEVICE_MOUNT_SRC);
        } else if (device_name == DEVICE_PULL_SWAY_1) {
            device_id = DEVICE_ID_PULL_SWAY_1;
            device = createDevice<Encoder>(device_name, device_id, DEVICE_COMM_INTERFACE_TYPE_CAN_2, DEVICE_MOUNT_SRC);
        } else if (device_name == DEVICE_PULL_SWAY_2) {
            device_id = DEVICE_ID_PULL_SWAY_2;
            device = createDevice<Encoder>(device_name, device_id, DEVICE_COMM_INTERFACE_TYPE_CAN_2, DEVICE_MOUNT_SRC);
        } else if (device_name == DEVICE_IOEXTEND_1353) {
            device_id = DEVICE_ID_IOEXTEND_1353;
            device = createDevice<IOExtend>(device_name, device_id, DEVICE_COMM_INTERFACE_TYPE_CAN_2, DEVICE_MOUNT_SRC);
        }

        if (!device) {
            LOG(ERROR) << "device [" << device_name << "] register failed!";
        }
        device->setStateOK();  // src 上传说有这个设备，那就是连上了
    }

    return device;
}

void sdk::SrcSdkDo::updateDeviceState(const string &device_name, uint8_t state, uint32_t error_no) const {
    auto device = getDevice(device_name, true);

    if (device) {
        switch ((DeviceState)state) {
            case sros::device::DEVICE_NONE: {
                device->setStateNone();
                break;
            }
            case sros::device::DEVICE_OK: {
                device->setStateOK();
                break;
            }
            case sros::device::DEVICE_INITIALIZING: {
                device->setStateInitialization();
                break;
            }
            case sros::device::DEVICE_OFF: {
                device->setStateOff();
                break;
            }
            case sros::device::DEVICE_ERROR: {
                device->setStateError(error_no);
                break;
            }
            case sros::device::DEVICE_ERROR_OPEN_FAILED: {
                device->setStateOpenFailed();
                break;
            }
            case sros::device::DEVICE_ERROR_TIMEOUT: {
                //增加错误码打印
                if(device->getState() != sros::device::DEVICE_ERROR_TIMEOUT){
                    LOG(ERROR) << "device_name:"<<device_name<<",state:"<<state<<",error_no:"<<error_no;
                }

                device->setStateTimeout(error_no);
                break;
            }
            case sros::device::DEVICE_ERROR_INITIAL: {
                device->setStateInitialFailed();
                break;
            }
            default: {
                LOG(ERROR) << "unreachable! state is " << (int)state;
            }
        }
        device->updateAliveTime();
    }
}

void sdk::SrcSdkDo::updateDeviceSerialNo(const string &name, const string &serial_no) const {
    auto device = getDevice(name, true);

    if (device) {
        device->setSerialNo(serial_no);
        LOG(INFO) << "updateDeviceSerialNo() " << name << " -> " << serial_no;
    }
}

void sdk::SrcSdkDo::logSendMonitor(std::ostringstream& oss) {
    auto log_msg = std::make_shared<sros::core::StrMsg>("MONITOR_PRINT_LOG");
    log_msg->data = oss.str();
    sros::core::MsgBus::sendMsg(log_msg);
}

// 处理Src心跳机制（100ms）
void sdk::SrcSdkDo::dealSrcHeartBeat() 
{
    while (true) {
        SrosStateMsg_Ptr m = std::make_shared<SrosStateMsg>();
        m->heart_beat = sros::core::util::get_time_in_ms();
        m->encode();
        src_sdk->sendMsg(m->rawData());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void sdk::SrcSdkDo::dealSrcPoseTimeout() {
    static bool last_src_pose_trigger = false;
    static uint64_t mark_system_timestamp = 0;

    static bool is_delay_deal_pose_time = false;

    static bool last_is_emergency = false;
    static bool last_is_break_switch_on = false;
    static bool last_is_reset_fault = false;
    static bool last_is_power_save_mode = false;

    bool cur_is_emergency = g_state.isEmergency();
    bool cur_is_break_switch_on = g_state.isBreakSwitchON();
    bool cur_is_reset_fault = is_reset_fault_;
    bool cur_is_power_save_mode = g_state.isPowerSaveMode();

    //do it
    uint64_t last_pose_timestamp = last_pose_timestamp_;
    if(last_pose_timestamp == 0) {
        return;
    }
    auto cur_timestamp = sros::core::util::get_time_in_ms();

    //LOG(INFO) << "l_e:" << last_is_emergency << ",c_e:" << cur_is_emergency
    //          << ";l_b:" << last_is_break_switch_on << ",c_b:" << cur_is_break_switch_on
    //          << ";l_r:" << last_is_reset_fault << ",c_r:" << cur_is_reset_fault
    //          << ";l_p:" << cur_is_power_save_mode << ",c_p:" << cur_is_power_save_mode
    //          << ";curtime:" << cur_timestamp << ",potime:"<< last_pose_timestamp
    //          << ",dfff:" << (cur_timestamp - last_pose_timestamp);

    bool trigger_emergency_flag = false;

    //logic
    if(is_delay_deal_pose_time) {

        //LOG(INFO) << "curtime:" << cur_timestamp << ",marktime:" << mark_system_timestamp
        //          << ",diff:" << (cur_timestamp-mark_system_timestamp)
        //          << ",posetime:" << last_pose_timestamp << ",diff:" << (cur_timestamp - last_pose_timestamp);

        if(cur_timestamp - mark_system_timestamp > 10000) {
            //超时1秒，未收到src pose
            if (cur_timestamp - last_pose_timestamp > 1000) {

                LOG(INFO) << "curtime:" << cur_timestamp << ",marktime:" << mark_system_timestamp
                          << ",diff:" << (cur_timestamp-mark_system_timestamp)
                          << ",posetime:" << last_pose_timestamp << ",diff:" << (cur_timestamp - last_pose_timestamp);

                //当前pose还是超过1秒
                trigger_emergency_flag = true;
                is_delay_deal_pose_time = false;
                last_src_pose_trigger = true;

                LOG(ERROR) << "recover, but src pose timeout, send CMD_TRIGGER_EMERGENCY again";

            } else {
                //当前pose时间在1秒内，正常清除所有标志
                is_delay_deal_pose_time = false;
                last_src_pose_trigger = false;
                LOG(INFO) << "recover, src pose normal";
            }
        }
    } else {
        if(!last_src_pose_trigger) {
            //超时1秒，未收到src pose
            if(cur_timestamp - last_pose_timestamp > 1000) {

                LOG(INFO) << "curtime:" << cur_timestamp << ",posetime:" << last_pose_timestamp
                          << ",diff:" << (cur_timestamp - last_pose_timestamp);

                trigger_emergency_flag = true;
                last_src_pose_trigger = true;
                LOG(ERROR) << "src pose timeout, send CMD_TRIGGER_EMERGENCY";
            }
        }
    }

    //trigger emergency
    if(trigger_emergency_flag) {
        sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_ODO_TIMEOUT);
        //g_state.sros_emergency_src = g_state.sros_emergency_src | sros::core::SROS_EMERGENCY_ODO_TIMEOUT;
    } else {
        //sros::core::FaultCenter::getInstance()->removeFault(sros::core::FAULT_CODE_ODO_TIMEOUT);
        //g_state.sros_emergency_src = g_state.sros_emergency_src & (~sros::core::SROS_EMERGENCY_ODO_TIMEOUT);
    }


    //解除急停、解除抱闸、发送解除电机故障、退出低功耗时，才需要延时判断pose超时
    if((last_is_emergency && !cur_is_emergency)
       || (last_is_break_switch_on && !cur_is_break_switch_on)
       || (!last_is_reset_fault && cur_is_reset_fault)
       || (last_is_power_save_mode && !cur_is_power_save_mode)) {

        is_delay_deal_pose_time = true;
        mark_system_timestamp = sros::core::util::get_time_in_ms();
        LOG(INFO) << "state change, delay to deal pose time";
    }

    //change state
    last_is_emergency = cur_is_emergency;
    last_is_break_switch_on = cur_is_break_switch_on;
    last_is_reset_fault = cur_is_reset_fault;
    last_is_power_save_mode = cur_is_power_save_mode;

    is_reset_fault_ = false;
}

bool sdk::NotifyQueue::put(src::BaseMsg_ptr item) {
    try {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!item) {
            LOG(ERROR) << "Error: Item is empty!!!";
            return false;
        }
        queue_.push_back(item);
        lock.unlock();

        cond_.notify_one();
    } catch (const std::system_error& ex) {
        LOG(ERROR) << ex.code() << '\n';
        LOG(ERROR) << ex.code().message() << '\n';
        LOG(ERROR) << ex.what() << '\n';
    } catch (const std::exception &e) {
        LOG(ERROR) << e.what() << '\n';
    } catch (...) {
        LOG(ERROR) << "UNNKOWE error!" << '\n';
    }
    return true;
}

src::BaseMsg_ptr sdk::NotifyQueue::get() {
    std::unique_lock<std::mutex> lock(mutex_);

    while (queue_.empty()) {
        cond_.wait(lock);
    }
    src::BaseMsg_ptr item = queue_.front();
    if (item != nullptr) {
        queue_.pop_front();
    }

    return item;
}

