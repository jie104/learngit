/**
 * @file vsc_module.h
 *
 * @author lhx
 * @date 17-02-17.
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#include "vsc_module.h"

#include <cmath>
#include <thread>

#include "core/device/device_manager.h"
#include "core/exec_error.hpp"
#include "core/fault_center.h"
#include "core/msg/common_command_msg.hpp"
#include "core/msg/command_msg.hpp"
#include "core/msg/common_msg.hpp"
#include "core/msg/common_state_msg.hpp"
#include "core/msg/notification_msg.hpp"
#include "core/msg/sonar_data_msg.hpp"
#include "core/msg/usart_data_msg.hpp"
#include "core/settings.h"
#include "core/src.h"
#include "core/state.h"

using namespace std;
using namespace sros::core;
using namespace sros::device;

namespace vsc {

static const int ACTION_CHARGE = 0x4E;

static const int CHARGE_TASK_ONLY_OPEN = 1;
static const int CHARGE_TASK_ONLY_CLOSE = 2;
static const int CHARGE_TASK_UNTIL_MAINTENANCE = 3;
static const int CHARGE_TASK_UNTIL_PERCENTAGE = 11;
static const int CHARGE_TASK_UNTIL_TIMEOUT = 21;

VSCModule::VSCModule()
    : Module("VSCModule"),
      enable_action_controller_(false),
      enable_safety_unit_(true),
      enable_power_unit_(true),
      enable_sonar_device_(false),
      enable_hmi_unit_(true),
      send_request_safety_(false),
      send_request_power_(false),
      request_cancel_charge_action_(false),
      timer_cnt_(0),
      safety_action_(SUA_NONE),
      power_action_(PUA_NONE),
      last_timer_send_data(0),
      cur_break_sw_state_(sros::core::BREAK_SW_NA) {}

VSCModule::~VSCModule() {}

void VSCModule::run() {
    auto &s = sros::core::Settings::getInstance();
    bool enable_vsc_module = (s.getValue<string>("main.enable_vsc", "False") == "True");
    if (!enable_vsc_module) {
        stop();
        return;
    }

    enable_action_controller_ = (s.getValue<std::string>("main.enable_action_controller", "False") == "True");
    enable_safety_unit_ = (s.getValue<std::string>("main.enable_safety_unit", "True") == "True");
    enable_power_unit_ = (s.getValue<std::string>("main.enable_power_unit", "True") == "True");
    enable_sonar_device_ = (s.getValue<std::string>("main.enable_sonar_device", "False") == "True");
    enable_hmi_unit_ = (s.getValue<std::string>("main.enable_hmi_unit", "True") == "True");
    charge_detect_timeout_ = s.getValue<int>("vsc.power_charge_detect_timeout", 15000);
    minute_of_mean_current_ = s.getValue<int>("battery.minute_of_mean_current", 5);
    if (minute_of_mean_current_ > 180) {
        minute_of_mean_current_ = 180;
        s.setValue("battery.minute_of_mean_current", minute_of_mean_current_);
    } else if (minute_of_mean_current_ < 5) {
        minute_of_mean_current_ = 5;
        s.setValue("battery.minute_of_mean_current", minute_of_mean_current_);
    }

    subscribeTopic("DEBUG_CMD", CALLBACK(&VSCModule::onDebugCmdMsg));
    subscribeTopic("ACTION_CMD", CALLBACK(&VSCModule::onActionCmdMsg));
    subscribeTopic("USART_DATA_SEND", CALLBACK(&VSCModule::onSendUsartDataMsg));
//    subscribeTopic("TIMER_50MS", CALLBACK(&VSCModule::onTimer_50ms));
    subscribeTopic("TIMER_20S", CALLBACK(&VSCModule::onTimer_20s));

    subscribeTopic("TOPIC_VSC_COMMAND", CALLBACK(&VSCModule::onVSCCommandMsg));
    subscribeTopic("TOPIC_UPDATA_VSC", CALLBACK(&VSCModule::onUpdateVscMsg));    // 升级VSC300

    vsc_device_ = sros::device::DeviceManager::getInstance()->registerDevice(
        DEVICE_VSC, DEVICE_ID_VSC, DEVICE_COMM_INTERFACE_TYPE_RS232_3, DEVICE_MOUNT_HARDWARE);
    CHECK(vsc_device_);

    battery_device_ =
        createDevice<Battery>(DEVICE_BATTERY, DEVICE_ID_PMU, DEVICE_COMM_INTERFACE_TYPE_RS232_3, DEVICE_MOUNT_HARDWARE);
    CHECK(battery_device_);

    initUsartDevice();

    vsc_upgrade_.init(connection_ptr_);
    vsc_upgrade_.setUpgradeCallback(boost::bind(&VSCModule::onVSCUpgradeResult, this, _1));

    std::thread t(&VSCModule::scheduleTask, this);
    t.detach();
    dispatch();
}

void VSCModule::onUpdateVscMsg(sros::core::base_msg_ptr m) {
    auto msg = std::dynamic_pointer_cast<sros::core::CommonMsg>(m);
    const std::string &upgrade_file_path = msg->str_0_;
    LOG(INFO) << "start upgrade... upgrade_file_path = " << upgrade_file_path;
    if (!vsc_upgrade_.upgradeRequest(upgrade_file_path)) {  // 请求升级失败
        auto msg_reply = std::make_shared<sros::core::CommonMsg>("TOPIC_UPDATA_IAP_REPLY");
        msg_reply->int_0_ = -1;  // 代表升级失败
        sendMsg(msg_reply);
        return;
    }
}

void VSCModule::onVSCUpgradeResult(int result)
{
    LOG(INFO) << "onVSCUpgradeResult " << result;
    auto upgrade_msg = std::make_shared<sros::core::CommonMsg>("TOPIC_UPDATA_VSC_RESULT");
    upgrade_msg->int_0_ = result;
    sendMsg(upgrade_msg);
}

void VSCModule::scheduleTask() {
    uint32_t i = 0;
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        if(!vsc_upgrade_.isInUpgrade()) {
            onTimer_50ms(nullptr);
        }
    }
}

void VSCModule::initUsartDevice() {
    // 绑定SRC串口数据
    src_sdk->setUSARTDataCallback(boost::bind(&VSCModule::onRecvSRCUsartData, this, _1));

    // 初始化VC300的LMNS通信串口
    auto &s = sros::core::Settings::getInstance();
    string device_name = s.getValue<std::string>("vsc.comm_device", "/dev/ttyTHS3");
    auto baud_rate = s.getValue<unsigned int>("vsc.comm_baud_rate", 9600);

    string device_type = s.getValue<string>("vsc.comm_device_type", "RS485");

    bool enable_rs485_mode = (device_type == "RS485");

    connection_ptr_ = make_shared<usart::Connection<usart::FrameV1<>>>();
    connection_ptr_->setHwDevName(DEVICE_VSC);
    connection_ptr_->setRecvDataCallback(boost::bind(&VSCModule::onRecvUsartData, this, _1));

    if (!connection_ptr_->connect(device_name, baud_rate, enable_rs485_mode)) {
        if (vsc_device_) {
            vsc_device_->setStateOpenFailed();
            battery_device_->setStateOpenFailed();
        }
    } else {
        vsc_device_->setStateOK();
        battery_device_->setStateOK();
    }

    //初始化期间延时发送串口数据（会激活src超时检测），解决重启软急停问题
    boost::this_thread::sleep_for(boost::chrono::milliseconds(10000));

    // 主动请求VSC300版本信息
    vector<uint8_t> request_vsc_info = {0x00, 0x01};
    sendUsartData(request_vsc_info);
}

void VSCModule::responseCommand(const sros::core::CommandMsg_ptr &msg) {
    auto response_msg = std::make_shared<CommandMsg>(msg->source, "TOPIC_CMD_RESPONSE");
    response_msg->command = msg->command;
    response_msg->session_id = msg->session_id;
    response_msg->req_seq = msg->req_seq;
    response_msg->result_state = msg->result_state;
    response_msg->result_code = msg->result_code;
//    LOG(INFO) << "Response command : command = " << response_msg->command
//              << " result state = " << response_msg->result_state << " result code = " << response_msg->result_code
//              << " seq = " << response_msg->req_seq << " session id = " << response_msg->session_id;
    sendMsg(response_msg);
}

void VSCModule::waitForFinishBatteryMaintenanceActionTask() {
    auto action_task = sros::core::TaskManager::getInstance()->getActionTask();

    if (!action_task || action_task->getActionParam() != CHARGE_TASK_UNTIL_MAINTENANCE) {
//        LOG(WARNING) << "waitForFinishBatteryMaintenanceActionTask(): No active action task";
        return;
    }

    TaskManager::getInstance()->setActionStart();

    // detect charge signal
    // charge state should turn to CHARGING after 10000ms if there is nothing wrong
    boost::this_thread::sleep_for(boost::chrono::milliseconds(charge_detect_timeout_));

    if (g_state.battery_state != sros::core::BATTERY_CHARGING) {
        SET_ACTION_EXEC_FAILED(ERROR_CODE_CHARGE_FAILED_NO_CHARGE_DOCK,
                               "Battery state is not CHARGING after 15000ms, battery maintenance action task failed");
        return;
    }

    auto &s = sros::core::Settings::getInstance();
    int soc_th_percentage = s.getValue<int>("battery.battery_maintenance_percentage", 98);
    float finish_current = s.getValue<float>("battery.battery_maintenance_finish_current", 2.0);


    while (!request_cancel_charge_action_) {
        boost::this_thread::sleep_for(boost::chrono::milliseconds(1000));

        // check if detach from charge dock
        if (g_state.battery_state != sros::core::BATTERY_CHARGING) {
            if(g_state.battery_percentage >= soc_th_percentage && fabs(g_state.battery_current) < finish_current*1000 ) {
                LOG(INFO) << "Battery percentage(" << static_cast<int>(g_state.battery_percentage)
                          << ") >= battery_maintenance_percentage(" << soc_th_percentage
                          << "), battery_current("<<fabs(g_state.battery_current)
                          << ") <= battery_maintenance_finish_current("<<finish_current*1000
                          << "), battery maintenance action finished.";
                TaskManager::getInstance()->setActionFinishSucceed(g_state.battery_percentage);

                // 到达目标电量后停止充电
                power_action_ = PUA_STOP_CHARGE;
                send_request_power_ = true;
//                LOG(INFO) << "请求向电源单元发送“关闭自动充电”指令";

            } else {
//                LOG(INFO) << "battery_percentage(" << g_state.battery_percentage
//                          << ") < battery_maintenance_percentage(" << soc_th_percentage
//                          << "), battery maintenance action task failed, detach from charge dock";
                power_action_ = PUA_STOP_CHARGE;
                send_request_power_ = true;
//                LOG(INFO) << "请求向电源单元发送“关闭自动充电”指令";

                SET_ACTION_EXEC_FAILED(ERROR_CODE_CHARGE_TO_PERCENT_FAILED_DETACH_FROM_CHARGE_DOCK,
                                       "battery maintenance action failed, detach from charge dock");
            }
            break;
        }
    }

    if (request_cancel_charge_action_) {
//        LOG(INFO) << " >> request cancel battery maintenance action !! << ";

        power_action_ = PUA_STOP_CHARGE;
        send_request_power_ = true;

        // FIXME(pengjiali): 重构vsc时：此处执行机构还没结束，但任务结束了，现在改起来太麻烦
        TaskManager::getInstance()->setActionFinishCanceled();

        request_cancel_charge_action_ = false;
    }

}

void VSCModule::waitForFinishChargeActionTaskUntilPercentage() {
    auto action_task = sros::core::TaskManager::getInstance()->getActionTask();

    if (!action_task || action_task->getActionParam() != CHARGE_TASK_UNTIL_PERCENTAGE) {
//        LOG(WARNING) << "waitForFinishChargeActionTaskUntilPercentage(): No active action task";
        return;
    }

    TaskManager::getInstance()->setActionStart();

    // detect charge signal
    // charge state should turn to CHARGING after 10000ms if there is nothing wrong
    boost::this_thread::sleep_for(boost::chrono::milliseconds(charge_detect_timeout_));

    if (g_state.battery_state != sros::core::BATTERY_CHARGING) {
        SET_ACTION_EXEC_FAILED(ERROR_CODE_CHARGE_FAILED_NO_CHARGE_DOCK,
                               "Battery state is not CHARGING after 15000ms, charge action task failed");
        return;
    }

    while (!request_cancel_charge_action_) {
        boost::this_thread::sleep_for(boost::chrono::milliseconds(1000));

        auto target_percentage = (uint16_t)action_task->getActionParam1();
        if (g_state.battery_percentage >= target_percentage) {
            // charge action finished
//            LOG(INFO) << "Battery percentage is " << static_cast<int>(g_state.battery_percentage)
//                      << ", charge action finished.";
            TaskManager::getInstance()->setActionFinishSucceed(g_state.battery_percentage);

            // 到达目标电量后停止充电
            power_action_ = PUA_STOP_CHARGE;
            send_request_power_ = true;
//            LOG(INFO) << "请求向电源单元发送“关闭自动充电”指令";

            break;
        }

        // check if detach from charge dock
        if (g_state.battery_state != sros::core::BATTERY_CHARGING) {
//            LOG(INFO) << "Charge action failed, detach from charge dock";
            power_action_ = PUA_STOP_CHARGE;
            send_request_power_ = true;
//            LOG(INFO) << "请求向电源单元发送“关闭自动充电”指令";

            SET_ACTION_EXEC_FAILED(ERROR_CODE_CHARGE_TO_PERCENT_FAILED_DETACH_FROM_CHARGE_DOCK,
                                   "Charge action failed, detach from charge dock");
            break;
        }
    }

    if (request_cancel_charge_action_) {
//        LOG(INFO) << " >> request cancel charge action !! << ";

        power_action_ = PUA_STOP_CHARGE;
        send_request_power_ = true;

        // FIXME(pengjiali): 重构vsc时：此处执行机构还没结束，但任务结束了，现在改起来太麻烦
        TaskManager::getInstance()->setActionFinishCanceled();

        request_cancel_charge_action_ = false;
    }
}

/**
 *
 * -----------------------------------------
 * | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 |
 * -----------------------------------------
 * | # | s | # | a | # | s | # | p | # | s | # | a | # | s | # | a | # | s | # | o |
 * -----------------------------------------
 *
 */
void VSCModule::onTimer_50ms(sros::core::base_msg_ptr msg) {
    timer_cnt_++;

    uint8_t type = 0;

    auto x = timer_cnt_ % 20;
    if (x == 19) {
        if(!is_recv_vsc_version_) {
            // 主动请求VSC300版本信息
            vector<uint8_t> request_vsc_info = {0x00, 0x01};
            sendUsartData(request_vsc_info);
        }
    }

    // 分时查询各个模块状态
    if ((x == 1 || x == 5 || x == 9 || x == 13 || x == 17) && enable_safety_unit_) {  // 查询安全单元状态
        auto &s = sros::core::Settings::getInstance();
        if (s.getValue<std::string>("main.vehicle_controller_type", "VC300") != "VC300") {
            return;
        }

        //实时更新sros急停触发源给vsc
        if(is_send_sros_emergency_src_) {
            vector<uint8_t> payload;
            payload.push_back(0x09);  // 安全单元
            payload.push_back(0x14);  // SROS软件触发源

            if(g_state.sros_emergency_src > 0) {
                payload.push_back(1);  // 不可恢复状态
            } else {
                payload.push_back(0);  // 不可恢复状态
            }

            payload.push_back((g_state.sros_emergency_src >> 24) & 0xff);  // 触发源
            payload.push_back((g_state.sros_emergency_src >> 16) & 0xff);  // 触发源
            payload.push_back((g_state.sros_emergency_src >> 8) & 0xff);  // 触发源
            payload.push_back(g_state.sros_emergency_src & 0xff);  // 触发源

            sendUsartData(payload);
            //LOG(INFO) << "g_state.sros_emergency_src: " << g_state.sros_emergency_src;
        }

        if (send_request_safety_) {
            vector<uint8_t> payload;
            payload.push_back(0x09);  // 安全单元

            if (safety_action_ == SUA_TRIGGER) {
//                LOG(INFO) << "VSC: 向安全单元发送“触发急停”指令";
                payload.push_back(0x15);  // 触发急停
                payload.push_back(0x01);  // 触发源默认为0x01
            } else {
//                LOG(INFO) << "VSC: 向安全单元发送“解除急停”指令";
                payload.push_back(0x11);  // 解除急停
            }

            sendUsartData(payload);

            send_request_safety_ = false;
        }
        type = 0x09;
    } else if ((x == 3 || x == 15) && enable_hmi_unit_) {  // 查询人机界面
        updateHMIState();
        return;
    } else if ((x == 7) && enable_power_unit_) {  // 查询电源单元状态
        if (send_request_power_) {
            vector<uint8_t> payload;
            payload.push_back(0x08);  // 电源单元

            if (power_action_ == PUA_START_CHARGE) {
//                LOG(INFO) << "USART: 向电源单元发送“启动自动充电”指令";
                payload.push_back(0x15);  // 启动自动充电
                //                boost::thread t(boost::bind(&VSCModule::waitForFinishCharge, this));
            } else if (power_action_ == PUA_STOP_CHARGE) {
//                LOG(INFO) << "USART: 向电源单元发送“关闭自动充电”指令";
                payload.push_back(0x17);  // 关闭自动充电
            } else if (power_action_ == PUA_ENTER_POWER_SAVE_MODE) {
//                LOG(INFO) << "USART: 向电源单元发送“进入低功耗模式”指令";
                payload.push_back(0x11);  // 进入低功耗模式
            } else if (power_action_ == PUA_EXIT_POWER_SAVE_MODE) {
//                LOG(INFO) << "USART: 向电源单元发送“退出低功耗模式”指令";
                payload.push_back(0x13);  // 退出低功耗模式
            } else {
//                LOG(WARNING) << "VSC: 电源单元指令错误";
                send_request_power_ = false;
                return;
            }

            sendUsartData(payload);
            send_request_power_ = false;
        }
        type = 0x08;
        // NOTE:超声波被废弃
        //    } else if (
        //        (x % 4 == 0) &&
        //        enable_sonar_device_) {  // 查询超声单元
        //                                 //
        //                                 FIXME(pengjiali):此处超声雷达从100ms查一次改为200ms查一次，没有经过测试，有机会要测试一下
        //        type = 0xA1;
    } else if (x == 11) {  // 查询充电桩状态
        vector<uint8_t> payload;
        payload.push_back(0x08);  // 电源单元
        payload.push_back(0x07); 
        sendUsartData(payload);
        return ;
    }else if (timer_cnt_ % 100 == 19) { // 5s 查一次
        type = 0xA2;
    } else {
        return;
    }

    // 构造需要发送的payload
    vector<uint8_t> payload;
    payload.push_back(type);
    payload.push_back(0x05);

    sendUsartData(payload);
}

void VSCModule::onTimer_20s(sros::core::base_msg_ptr msg) {
    static int i = 0;
    ++i;
    if (i % 3 == 0) {  // 每分钟触发一次
        estimationBatteryRemainTime();
    }
}

void VSCModule::tempRemoveForkliftErr() {

    FaultCenter::getInstance()->removeFault(FAULT_CODE_FORK_LIFT_FAULT);
    FaultCenter::getInstance()->removeFault(FAULT_CODE_GOODS_NOT_DETECTED);
    FaultCenter::getInstance()->removeFault(FAULT_CODE_GOODS_POS_YAW_DEVIATE);
    FaultCenter::getInstance()->removeFault(FAULT_CODE_GOODS_POS_X_DEVIATE_FORWARD);
    FaultCenter::getInstance()->removeFault(FAULT_CODE_GOODS_POS_X_DEVIATE_BACK);

    FaultCenter::getInstance()->removeFault(ERROR_CODE_ACTION_FORK_ADJUST_LEN_NOT_ENOUGH);
    FaultCenter::getInstance()->removeFault(ERROR_CODE_ACTION_FORK_MOVE_LEN_NOT_ENOUGH);

    FaultCenter::getInstance()->removeFault(FAULT_CODE_GOODS_POS_Y_DEVIATE);
    FaultCenter::getInstance()->removeFault(FAULT_CODE_LOAD_NO_DOCKING);
    FaultCenter::getInstance()->removeFault(FAULT_CODE_FORK_FAIL_PICKUP_GOODS);
    FaultCenter::getInstance()->removeFault(FAULT_CODE_BACKFORK_MOVE_NOT_REACH);

    FaultCenter::getInstance()->removeFault(FAULT_CODE_GOODS_DETECTED);
    FaultCenter::getInstance()->removeFault(FAULT_CODE_UNLOAD_MOVE_NOT_REACH);
    FaultCenter::getInstance()->removeFault(FAULT_CODE_LOAD_PALLET_BOUNCE);
    FaultCenter::getInstance()->removeFault(FAULT_CODE_FORK_FAIL_PICKDOWN_GOODS);
    
    FaultCenter::getInstance()->removeFault(sros::core::FAULT_CODE_ODO_TIMEOUT);
    FaultCenter::getInstance()->removeFault(sros::core::FAULT_CODE_MOTOR_SLIP);
    FaultCenter::getInstance()->removeFault(sros::core::FAULT_CODE_COUPLING_BREAK);
    FaultCenter::getInstance()->removeFault(sros::core::FAULT_CODE_FORK_HEIGHT_DEVIATE);
    FaultCenter::getInstance()->removeFault(sros::core::FAULT_CODE_AUTO_CHARGE_BREAK);
    FaultCenter::getInstance()->removeFault(sros::core::FAULT_CODE_EMERGENCY_LEFT_TRIGGER_FORKTIP);
    FaultCenter::getInstance()->removeFault(sros::core::FAULT_CODE_EMERGENCY_RIGHT_TRIGGER_FORKTIP);
}

void VSCModule::onDebugCmdMsg(sros::core::base_msg_ptr msg) {
    auto m = dynamic_pointer_cast<sros::core::CommandMsg>(msg);
    auto session_id = m->session_id;
    auto &s = sros::core::Settings::getInstance();

    auto getControlMutexFun = [&]() {
        if (!g_state.control_mutex.get(session_id)) {
            throw EXEC_ERROR(ERROR_CODE_CONTROL_MUTEX_IS_LOCKED, "locker:", g_state.control_mutex.getLockerSessionId(),
                             "current:", session_id);
        }
    };

    try {
        switch (m->command) {
            // 解除急停
            case sros::core::CMD_CANCEL_EMERGENCY: {
                if (s.getValue<std::string>("main.vehicle_controller_type", "VC300") != "VC300") {
                    return;
                }
                LOGGER(INFO, CMD_HANDER) << "Handling CMD_CANCEL_EMERGENCY command";

                getControlMutexFun();

                if (g_state.emergency_state != sros::core::STATE_EMERGENCY_RECOVERABLE) {
                    // 如果当前状态不是“可恢复急停”，则不予处理
                    throw EXEC_ERROR(ERROR_CODE_CANCEL_EMERGENCY_CAN_NOT_RECOVER,
                                     "emergency_state is STATE_EMERGENCY_TRIGER, can't recover");
                }

                tempRemoveForkliftErr();

                // 请求向安全单元发送“解除急停”指令
                safety_action_ = SUA_RECOVER;
                send_request_safety_ = true;
                m->result_state = RESPONSE_PROCESSING;
                break;
            }
            case sros::core::CMD_TRIGGER_EMERGENCY: {
                if (s.getValue<std::string>("main.vehicle_controller_type", "VC300") != "VC300") {
                    return;
                }
                LOGGER(INFO, CMD_HANDER) << "Handling CMD_TRIGGER_EMERGENCY command";

                getControlMutexFun();

//                if (g_state.emergency_state == sros::core::STATE_EMERGENCY_RECOVERABLE ||
//                    g_state.emergency_state == sros::core::STATE_EMERGENCY_TRIGER) {
//                    // 如果当前状态是“急停”，直接返回成功！
//                    m->result_state = RESPONSE_OK;
//                    break;
//                }

                // 请求向安全单元发送“急停触发”指令
                safety_action_ = SUA_TRIGGER;
                send_request_safety_ = true;
                m->result_state = RESPONSE_PROCESSING;

                break;
            }
            case sros::core::CMD_ENABLE_AUTO_CHARGE: {
                LOGGER(INFO, CMD_HANDER) << "Handling CMD_ENABLE_AUTO_CHARGE command";

                getControlMutexFun();

                if (TaskManager::getInstance()->isMovementTaskRunning()) {
                    throw EXEC_ERROR(ActionErrorStartIndex + ERROR_CODE_CHARGE_FAILED_MOVEMENT_RUNNING,
                                     "Movement task is running!");
                }

                // 请求向电源单元发送“启动自动充电”指令
                power_action_ = PUA_START_CHARGE;
                send_request_power_ = true;
                request_cancel_charge_action_ = false;

                m->result_state = RESPONSE_PROCESSING;
                break;
            }
            case sros::core::CMD_STOP_CHARGE: {
                LOGGER(INFO, CMD_HANDER) << "Handling CMD_STOP_CHARGE command";

                getControlMutexFun();

                // 请求向电源单元发送“关闭充电”指令
                power_action_ = PUA_STOP_CHARGE;
                send_request_power_ = true;
                request_cancel_charge_action_ = true;

                m->result_state = RESPONSE_PROCESSING;
                break;
            }
            case sros::core::CMD_ENTER_POWER_SAVE_MODE: {
                LOGGER(INFO, CMD_HANDER) << "Handling CMD_ENTER_POWER_SAVE_MODE command";

                getControlMutexFun();

                // 请求向电源单元发送“进入低功耗模式”指令
                power_action_ = PUA_ENTER_POWER_SAVE_MODE;
                send_request_power_ = true;

                m->result_state = RESPONSE_PROCESSING;
                break;
            }
            case sros::core::CMD_EXIT_POWER_SAVE_MODE: {
                LOGGER(INFO, CMD_HANDER) << "Handling CMD_EXIT_POWER_SAVE_MODE command";

                getControlMutexFun();

                // 请求向电源单元发送“退出低功耗模式”指令
                power_action_ = PUA_EXIT_POWER_SAVE_MODE;
                send_request_power_ = true;

                m->result_state = RESPONSE_PROCESSING;
                break;
            }
            case sros::core::CMD_NEW_ACTION_TASK:
            case sros::core::CMD_CANCEL_ACTION_TASK: {
                // in onActionCmdMsg process
                return;
            }
            default: {
                return;
                break;
            }
        }
    } catch (const ExecError &e) {
        // 命令处理出错的情况
        m->result_state = RESPONSE_FAILED;
        m->result_code = e.errorCode();
        responseCommand(m);
        return;
    }

    responseCommand(m);
}

void VSCModule::onActionCmdMsg(sros::core::base_msg_ptr msg) {
    auto mm = dynamic_pointer_cast<sros::core::CommandMsg>(msg);
    auto session_id = mm->session_id;

    try {
        switch (mm->command) {
            case sros::core::CMD_NEW_ACTION_TASK: {
                auto action_task = mm->action_task;
                if (!action_task) {
                    LOGGER(WARNING, CMD_HANDER) << "action task is NULL";

                    return;
                }

                auto action_id = action_task->getActionID();

                if (action_id != 0x4F && action_id != ACTION_CHARGE) {
                    // 不在处理范围内
                    return;  // action_controller_module中回复了，此处不需要回复
                }

                sros::core::TaskManager::getInstance()->updateActionTask(action_task);

                if (action_id == 0x4F) {
                    auto action_param0 = action_task->getActionParam();
                    LOGGER(INFO, CMD_HANDER) << "Is enable prevent fall down:" << action_param0;

                    vector<uint8_t> payload;
                    payload.push_back(0x09);  // 安全单元
                    payload.push_back(0x13);  // 启用/禁用跌落急停
                    payload.push_back(action_param0);  // 如果Enable=0，禁用跌落急停功能，如果Enable=1，启用跌落急停功能

                    sendUsartData(payload);

                    TaskManager::getInstance()->setActionFinishSucceed();

                } else if (action_id == ACTION_CHARGE) {
                    handleChargeActionTask(action_task);
                }
                mm->result_state = RESPONSE_OK;
                break;
            }
            case sros::core::CMD_CANCEL_ACTION_TASK: {
                auto action_task = sros::core::TaskManager::getInstance()->getActionTask();
                if (!action_task) {
                    LOGGER(WARNING, CMD_HANDER) << "action task is NULL";
                    return;
                }

                if (action_task->isSlaveRunning() || action_task->isWaitForStart()) {
                    int action_id = action_task->getActionID();

                    if (action_id == ACTION_CHARGE) {
                        // 设置取消标志位
                        request_cancel_charge_action_ = true;
                        sros::core::TaskManager::getInstance()->setActionInCancel();

                        LOGGER(WARNING, CMD_HANDER) << "cancel ACTION_CHARGE, set cancel state";
                        mm->result_state = RESPONSE_OK;
                        break;
                    }
                }
                return;
                break;
            }
            default: {
                return;
            }
        }
    } catch (const ExecError &e) {
        // 命令处理出错的情况
        mm->result_state = RESPONSE_FAILED;
        mm->result_code = e.errorCode();
        responseCommand(mm);
        return;
    }

    responseCommand(mm);
}

void VSCModule::onVSCCommandMsg(sros::core::base_msg_ptr msg) {
    auto m = dynamic_pointer_cast<sros::core::StrMsg>(msg);

    if (m->data == "DISABLE_VSC_HMI") {
//        LOG(INFO) << "onVSCCommandMsg(): DISABLE_VSC_HMI";
        is_vsc_hmi_disabled_ = true;
    }
}

void VSCModule::handleChargeActionTask(sros::core::ActionTask_ptr action_task) {
    LOG(INFO) << "charge action: 78,"<<action_task->getActionParam()<<","<<action_task->getActionParam1();
    auto charge_task_type = action_task->getActionParam();

    auto checkStartChangeConditionFunc = [&]() {
      if (TaskManager::getInstance()->isMovementTaskRunning()) {
          SET_ACTION_EXEC_FAILED(ERROR_CODE_CHARGE_FAILED_MOVEMENT_RUNNING, "Movement task is running!");
          throw EXEC_ERROR(ActionErrorStartIndex + ERROR_CODE_CHARGE_FAILED_MOVEMENT_RUNNING,
                           "Movement task is running!");
      }
      if (g_state.isManualControl()) {
          SET_ACTION_EXEC_FAILED(ERROR_CODE_CHARGE_FAILED_CURRENT_MANUAL_CONTROL, "In manual control!");
          throw EXEC_ERROR(ActionErrorStartIndex + ERROR_CODE_CHARGE_FAILED_CURRENT_MANUAL_CONTROL,
                           "In manual control!");
      }
    };

    if (charge_task_type == CHARGE_TASK_ONLY_OPEN) {
        checkStartChangeConditionFunc();
        TaskManager::getInstance()->setActionStart();

        if (g_state.battery_state == sros::core::BATTERY_CHARGING) {
            TaskManager::getInstance()->setActionFinishSucceed();
            return;
        }

        // 开始启动充电
        power_action_ = PUA_START_CHARGE;
        send_request_power_ = true;
        request_cancel_charge_action_ = false;
//        LOG(INFO) << "请求向电源单元发送“启动自动充电”指令";

        thread t([this, action_task]() {
            for (auto i = 0; i < charge_detect_timeout_; i += 500) {
                boost::this_thread::sleep_for(boost::chrono::milliseconds(500));
                if (g_state.battery_state == sros::core::BATTERY_CHARGING) {
                    break;
                }
            }

            if (g_state.battery_state == sros::core::BATTERY_CHARGING) {
//                LOG(INFO) << "CHARGE_TASK_ONLY_OPEN: OK";
                TaskManager::getInstance()->setActionFinishSucceed();
            } else {
                SET_ACTION_EXEC_FAILED(ERROR_CODE_CHARGE_TO_PERCENT_FAILED_NO_CHARGE_DOCK,
                                       "Charge action failed, no charge dock");
            }
        });
        t.detach();

    } else if (charge_task_type == CHARGE_TASK_ONLY_CLOSE) {
        // 到达目标电量后停止充电
        power_action_ = PUA_STOP_CHARGE;
        send_request_power_ = true;
        request_cancel_charge_action_ = false;
//        LOG(INFO) << "请求向电源单元发送“关闭自动充电”指令";

        thread t([this, action_task]() {
            for (auto i = 0; i < charge_detect_timeout_; i += 500) {
                boost::this_thread::sleep_for(boost::chrono::milliseconds(500));
                if (g_state.battery_state == sros::core::BATTERY_NO_CHARGING) {
                    break;
                }
            }

            if (g_state.battery_state == sros::core::BATTERY_NO_CHARGING) {
//                LOG(INFO) << "CHARGE_TASK_ONLY_CLOSE: OK";
                TaskManager::getInstance()->setActionFinishSucceed();
            } else {
                SET_ACTION_EXEC_FAILED(ERROR_CODE_STOP_CHARGE_FAILED,
                                       "Failed to finish charging, it may be charging with manual charger");
            }
        });
        t.detach();

    } else if (charge_task_type == CHARGE_TASK_UNTIL_PERCENTAGE) {
        checkStartChangeConditionFunc();

        auto target_percentage = action_task->getActionParam1();

//        LOG(INFO) << "Start charge until percentage >= " << target_percentage;

        if (g_state.battery_percentage >= target_percentage) {
            // 当前电量高于目标电量，直接结束充电任务
//            LOG(INFO) << "current battery percentage >= target_percentage "
//                      << static_cast<int>(g_state.battery_percentage);

            TaskManager::getInstance()->setActionFinishSucceed(g_state.battery_percentage);
            return;
        } else {
            // 开始启动充电
            power_action_ = PUA_START_CHARGE;
            send_request_power_ = true;
            request_cancel_charge_action_ = false;
//            LOG(INFO) << "请求向电源单元发送“启动自动充电”指令";
        }

        thread t([this]() { waitForFinishChargeActionTaskUntilPercentage(); });
        t.detach();
    } else if(charge_task_type == CHARGE_TASK_UNTIL_MAINTENANCE) {
        checkStartChangeConditionFunc();
//        LOG(INFO) << "Start battery maintenance";

        //开始启动充电
        power_action_ = PUA_START_CHARGE;
        send_request_power_ = true;
        request_cancel_charge_action_ = false;
//        LOG(INFO) << "请求向电源单元发送“启动自动充电”指令";

        thread t([this]() { waitForFinishBatteryMaintenanceActionTask(); });
        t.detach();

    } else {
        SET_ACTION_EXEC_FAILED(ERROR_CODE_CHARGE_PARAM_INVALID, "invalid charge_task_type:", charge_task_type);
    }
}

/**
 * 接收SRC通过src-sdk发送过来的串口数据帧
 * 按照FrameV1结构进行解析
 * @param frame
 */
void VSCModule::onRecvSRCUsartData(std::vector<uint8_t> frame) {
    //LOG(INFO) << "-----------------data--------------------\n";
    if (frame.size() < 6) {
//        LOG(WARNING) << "onRecvSRCUsartData() frame length < 6";
        return;
    }

    auto msg = std::make_shared<sros::core::UsartDataMsg>("USART_DATA_RECV");
    msg->raw_data = frame;

    if (!msg->decodeRawData()) {
        // 如果数据解析错误，则终止处理该数据
//        LOG(WARNING) << "onRecvSRCUsartData() frame check fail";
        return;
    }

    vector<uint8_t> payload;
    for (int i = 2; i < frame.size() - 1; i++) {
        payload.push_back(frame[i]);
    }

    // 处理VSC相关
    onRecvUsartData(payload);

    // 转发SRC的数据帧到usart-module
    sendMsg(msg);
}

/**
 * 处理串口接收到的有效数据（去除了帧头帧尾）
 * @param data
 */
void VSCModule::onRecvUsartData(const vector<uint8_t> &data) {
    if (data.empty()) {
//        LOG(ERROR) << "VSC module read empty!";
        return;
    }

    uint8_t type = data[0];

    DLOG(INFO) << "==> onRecvUsartData: type = " << static_cast<int>(type) << ", " << data.size();

    if (vsc_device_) {
        vsc_device_->keepAlive();
    }

    // 根据type不同为msg设置不同的topic
    if (type == 0x00) {  // VSC
        handleVSCData(data);
    } else if (type == 0x06 && enable_action_controller_) {  // 动作控制器
        handleActionControllerData(data);
    } else if (type == 0x08 && enable_power_unit_) {  // 电源单元数据
        handlePowerUnitData(data);
    } else if (type == 0x09 && enable_safety_unit_) {  // 安全单元
        handleSafetyUnitData(data);
    } else if (type == 0xA1 && enable_sonar_device_) {  // 超声单元
        handleSonarUnitData(data);
    } else if (type == 0xA2) {  // 温湿度、风扇
        handleSensorData(data);
    } else if(type == MSG_IAP) {   // 处理升级过程
        vsc_device_->setModelNo("vsc300_bootloader"); // 当设备本次升级失败,会自动恢复上一次升级的版本,此时Matrix硬件状态中会有显示
        vsc_device_->setStateOK();
        vsc_upgrade_.handleIAPRequest(data);
    }
}

void VSCModule::sendUsartData(const vector<uint8_t> &data) {

    std::lock_guard<std::mutex> lg(lock_mutex_);

    // 通过VC300串口发送
    if (connection_ptr_) {
        connection_ptr_->sendData(data);

        //增加时间戳记录发送心跳是否超时
        auto cur_time = sros::core::util::get_time_in_ms();
//        LOG(INFO) << "sendUsartData get_time_in_ms:" << cur_time;
        if (0 == last_timer_send_data)
        {
            last_timer_send_data = cur_time;
        }
        else
        {
            if ((cur_time - last_timer_send_data) > 500)
            {
                LOG(INFO) << "Request the heart beat of security module time out!";
            }
            last_timer_send_data = cur_time;
        }
    } else {
//        LOG(ERROR) << "connection_ptr_ 为空";
    }

    // 构造Msg通过SRC串口发送
    //    auto m = make_shared<sros::core::UsartDataMsg>("USART_DATA_SEND");
    //    m->type = data[0];
    //    m->command = data[1];
    //    m->data.clear();
    //
    //    onSendUsartDataMsg(m);
}

/**
 * 将UsartDataMsg通过SRC串口发送出去
 * @param msg
 */
void VSCModule::onSendUsartDataMsg(sros::core::base_msg_ptr msg) {
    auto m = std::dynamic_pointer_cast<sros::core::UsartDataMsg>(msg);
    m->generateRawData();  // 生成完整数据帧

    //    src_sdk->sendUsartData(m->raw_data);
}

void VSCModule::updateNavigationState(sros::core::NavigationState state) {
    auto mm = make_shared<sros::core::CommonStateMsg<sros::core::NavigationState>>("NAV_STATE");
    mm->state = state;
    sendMsg(mm);
}

void VSCModule::handleSafetyUnitData(const vector<uint8_t> &data) {
    auto &s = sros::core::Settings::getInstance();
    if (s.getValue<std::string>("main.vehicle_controller_type", "VC300") != "VC300") {
        return;
    }

    uint8_t command = data[1];

    if (command == 0x06) {       // 状态返回
        if (data.size() != 6) {  // 数据段长度错误
//            LOG(WARNING) << "USART Data length error";
            return;
        }

        uint8_t safety_state = data[2];
        uint8_t error_code = data[3];
        uint16_t emergency_src = (data[4] << 8) + data[5];
//        LOG(INFO) << "handleSafetyUnitData safety_state: " << std::hex << (int)safety_state << ", emergency_src: 0x"
//                  << std::hex << emergency_src << ", get_time_in_ms:" << sros::core::util::get_time_in_ms();
        security_state_handle_.setNewState((sros::core::EmergencyState)safety_state, emergency_src);
    }
}

std::string convertBMSDateStr(uint16_t d) {
    // 根据BMS提供的协议解析
    int day = d & 0x1f;
    int month = (d >> 5) & 0x0f;
    int year = 2000 + (d >> 9);

    stringstream ss;
    ss << year << "/" << month << "/" << day;
    return ss.str();
}

void VSCModule::handlePowerUnitData(const vector<uint8_t> &data) {
    uint8_t command = data[1];


    if (command == 0x06) {                                                  // 状态返回
        if (data.size() != 14 && data.size() != 18 && data.size() != 26) {  // 数据段长度错误
//            LOG(WARNING) << "USART Data length error： " << data.size();
            return;
        }

        uint8_t power_state = data[0 + 2];
        uint8_t power_error_code = data[1 + 2];
        uint8_t charge_state = data[2 + 2];
        uint8_t charge_current = (data[3 + 2] << 8) + data[4 + 2];
        uint8_t charge_station_state = data[5 + 2];
        uint8_t battery_percentage = data[6 + 2];
        uint16_t battery_voltage = (data[7 + 2] << 8) + data[8 + 2];
        int16_t battery_current = (data[9 + 2] << 8) + data[10 + 2];
        uint8_t battery_temperature = data[11 + 2];

        uint16_t battery_state_word = 0;
        uint16_t battery_date_word = 0;
        if (data.size() >= 18) {  // 扩展了电池状态字及生产日期
            battery_state_word = (data[12 + 2] << 8) + data[13 + 2];
            battery_date_word = (data[14 + 2] << 8) + data[15 + 2];
        }

        static auto old_battery_state_word = -1;
        if (old_battery_state_word != battery_state_word) {
            battery_device_->setInfo("state: " + std::to_string(battery_state_word));
            old_battery_state_word = battery_state_word;
        }

        uint16_t battery_remain_capacity = 0;   // 电池剩余容量（mAh)
        uint16_t battery_nominal_capacity = 0;  // 电池标称容量（mAh)
        uint16_t battery_use_cycles = 0;        // 电池循环次数
        if (data.size() >= 26) {
            battery_remain_capacity = (data[16 + 2] << 8) + data[17 + 2];
            battery_nominal_capacity = (data[18 + 2] << 8) + data[19 + 2];
            battery_use_cycles = (data[20 + 2] << 8) + data[21 + 2];
        }

        if (battery_device_ && battery_date_word != 0) {
            // 将电池生产日期作为批次型号
            battery_device_->setModelNo(convertBMSDateStr(battery_date_word));
        }

        const int POWER_STATE_INITIALIZING = 0x00;
        const int POWER_STATE_DEFAULT = 0x01;
        const int POWER_STATE_SLEEP = 0x02;
        const int POWER_STATE_ERROR = 0x03;
        const int BATTERY_CHARGING = 0x02;

        if (battery_percentage <= 100) {
            //电量跳变检测
            if((fabs(g_state.battery_percentage - battery_percentage) > 2) && g_state.battery_percentage != 101){
                LOG(INFO) << "last percentage:" << static_cast<int>(g_state.battery_percentage) << ",cur percentage:" << static_cast<int>(battery_percentage);
            }
            g_state.battery_percentage = battery_percentage;
        } else {
//                LOG(WARNING) << "ERROR battery_percentage: " << battery_percentage;
        }

        // 根据电源单元协议，充电时current为正，放电时为负
        // 上层应用使用时理解与此相反，故将电流取负数
        g_state.battery_current = -battery_current;
        g_state.battery_voltage = battery_voltage;
        g_state.battery_temperature = battery_temperature;

        g_state.battery_remain_capacity = battery_remain_capacity;
        g_state.battery_nominal_capacity = battery_nominal_capacity;
        g_state.battery_use_cycles = battery_use_cycles;

        BatteryState new_battery_state = BATTERY_NA;
        if (charge_state == BATTERY_CHARGING) {
            new_battery_state = sros::core::BATTERY_CHARGING;
        } else {
            new_battery_state = sros::core::BATTERY_NO_CHARGING;
        }

        if (new_battery_state != g_state.battery_state) {
            //减速触发的信号不做充电状态切换
            ActionTask_ptr action_task = sros::core::TaskManager::getInstance()->getActionTask();

            if(TaskManager::getInstance()->isMovementTaskRunning() && g_state.battery_state == BATTERY_NO_CHARGING){
                LOG(INFO) << "AGV is moving. can not change battery state";
            } else{
                LOGGER(INFO, SROS) << "Charge state changed! New state is\ "
                                    << (new_battery_state == sros::core::BATTERY_CHARGING ? "BATTERY_CHARGING"
                                                                                        : "BATTERY_NO_CHARGING");
                g_state.battery_state = new_battery_state;
            }
        }

        if (power_state == POWER_STATE_DEFAULT || power_state == POWER_STATE_SLEEP) {
            if (battery_device_) {
                battery_device_->keepAlive();
            }

            auto cur_power_state = sros::core::POWER_NA;
            if (power_state == POWER_STATE_DEFAULT) {
                cur_power_state = sros::core::POWER_NORMAL;
            } else if (power_state == POWER_STATE_SLEEP) {
                cur_power_state = sros::core::POWER_SAVE_MODE;
            }

            auto fault_center = sros::core::FaultCenter::getInstance();
            if (g_state.power_state == sros::core::POWER_NORMAL && cur_power_state == sros::core::POWER_SAVE_MODE) {
                // 进入低功耗模式
//                LOGGER(INFO, SROS) << "PowerState: NORMAL -> POWER_SAVE_MODE";
                src_sdk->emergencyPause(EMERGENCY_PAUSE_LOW_POWER);
                fault_center->addFault(
                    FAULT_CODE_POWER_SAVE_MODE, []() -> bool { return true; },
                    [&]() {
                        auto d_msg = std::make_shared<sros::core::CommandMsg>(getName());
                        d_msg->command = sros::core::CMD_EXIT_POWER_SAVE_MODE;
                        sendMsg(d_msg);
                    });
            }
            if (g_state.power_state == sros::core::POWER_SAVE_MODE && cur_power_state == sros::core::POWER_NORMAL) {
                // 退出低功耗模式
                // 重新初始化电机等
//                LOGGER(INFO, SROS) << "PowerState: POWER_SAVE_MODE -> NORMAL";
                src_sdk->emergencyContinue(EMERGENCY_PAUSE_LOW_POWER);
                fault_center->removeFault(FAULT_CODE_POWER_SAVE_MODE);
            }

            g_state.power_state = cur_power_state;


            // FIXME: 此处依赖handlePowerUnitData()函数的调用周期为1s，若周期变化，此段代码会出问题
            if (g_state.battery_state == sros::core::BATTERY_NO_CHARGING) {
                recent_discharge_battery_current_.push_back(g_state.battery_current);
                if (recent_discharge_battery_current_.size() > minute_of_mean_current_ * 60) {
                    recent_discharge_battery_current_.pop_front();
                }
            }

            // 第一次获取到电池数据，估算一下剩余时间
            static bool is_first_get_battery_data = true;
            if (is_first_get_battery_data) {
                is_first_get_battery_data = false;
                estimationBatteryRemainTime();
            }

            // 获取解抱闸状态
            if (power_error_code == 0x01) {
                cur_break_sw_state_ = sros::core::BREAK_SW_ON;
            } else {
                cur_break_sw_state_ = sros::core::BREAK_SW_OFF;
            }

            fault_center->track(sros::core::FAULT_CODE_BREAK_SWITCH_ON,
                                [&]() { return cur_break_sw_state_ == sros::core::BREAK_SW_ON; });

            if (g_state.break_sw_state == sros::core::BREAK_SW_OFF && cur_break_sw_state_ == sros::core::BREAK_SW_ON) {
                // 解抱闸开关: 关闭 -> 打开
                // 向SRC发送急停暂停
//                LOGGER(INFO, SROS) << "Break switch state change: OFF -> ON";
//                LOG(INFO) << "Set src memergency pause for break switch ON!";
                src_sdk->emergencyPause(EMERGENCY_PAUSE_BREAK_SWITCH);
            }
            if (g_state.break_sw_state == sros::core::BREAK_SW_ON && cur_break_sw_state_ == sros::core::BREAK_SW_OFF) {
                // 解抱闸开关: 打开 -> 关闭
                // 向SRC发送急停继续，初始化电机驱动器
//                LOGGER(INFO, SROS) << "Break switch state change: ON -> OFF";
//                LOG(INFO) << "Set src emergency continue, last emergency pause source is break switch ON!";
                src_sdk->emergencyContinue(EMERGENCY_PAUSE_BREAK_SWITCH);
            }

            // 更新系统状态的解抱闸开关状态
            g_state.break_sw_state = cur_break_sw_state_;
        } else if (power_state == POWER_STATE_ERROR) {
            if (battery_device_) {
                 if(power_error_code == 0x0004){
                    battery_device_->setStateTimeout();
                }else{
                   battery_device_->setStateError(power_error_code);
                }
                g_state.power_state = sros::core::POWER_NA;
            }
        }
    } else if(command == 0x08) {     //充电桩信息
        //charge_station  简称CS 
        static bool last_cs_connect_state = false;
        static bool last_cs_output_state = false;
        static uint8_t last_cs_state_flag = 0;
        bool cs_connect_state = static_cast<int>(data[0 + 2]);
        bool cs_output_state = static_cast<int>(data[1 + 2]);
        int16_t cs_output_current  = (data[2 + 2] << 8) + data[3 + 2];
        int16_t cs_output_voltage  = (data[4 + 2] << 8) + data[5 + 2];
        uint8_t cs_state_flag = data[6 + 2];

        if(data.size() == 10){
            if( last_cs_connect_state != cs_connect_state || last_cs_output_state != cs_output_state || last_cs_state_flag != cs_state_flag) {
                std::stringstream stream;
                for(int i = 0; i < 10; ++i) {
                    stream << std::setfill ('0') << std::setw(2) << std::hex << (int)data[i];
                }
                std::string hexString = stream.str();
                LOGGER(INFO, SROS) <<  "charge_station hexStr: " << hexString;

                LOGGER(INFO, SROS) << "Charging station connect state:" << (cs_connect_state ? "ON":"OFF");
                LOGGER(INFO, SROS) << "Charging station output state:" << (cs_output_state ? "ON":"OFF");
                LOG(INFO) << "cs_output_current:" << cs_output_current << ",cs_output_voltage:" << cs_output_voltage;
                last_cs_connect_state = cs_connect_state;
                last_cs_output_state = cs_output_state;
                last_cs_state_flag = cs_state_flag;
            
            }
        }else if(data.size() == 26) {
            //电池信息
            static bool last_control_flag = false;
            static bool last_bms_state = false;
            static bool last_old_bms_state = false;
            int16_t max_allow_charge_voltage  = (data[8 + 2] << 8) + data[9 + 2];    //最高允许充电端电压
            int16_t max_allow_charge_current  = (data[10 + 2] << 8) + data[11 + 2];
            bool control_flag = static_cast<int>(data[12 + 2]);
            bool bms_state = static_cast<int>(data[13 + 2]);
            bool old_bms_state = static_cast<int>(data[14 + 2]);   //上一次错误状态
            int16_t single_highest  = (data[16 + 2] << 8) + data[17 + 2];
            int16_t single_lowest  = (data[18 + 2] << 8) + data[19 + 2];
            int soc = static_cast<int>(data[20 + 2]); 
            int temperature = static_cast<int>(data[21 + 2]);
            int16_t voltage_high_byte  = (data[22 + 2] << 8) + data[23 + 2];

            if(last_cs_connect_state != cs_connect_state || last_cs_output_state != cs_output_state || last_cs_state_flag != cs_state_flag     
                    || last_control_flag != control_flag || last_bms_state != bms_state ) {
                
                std::stringstream stream;
                for(int i = 0; i < 10; ++i) {
                    stream << std::setfill ('0') << std::setw(2) << std::hex << (int)data[i];
                }
                std::string hexString = stream.str();
                LOGGER(INFO, SROS) <<  "charge_station hexStr: " << hexString;                      

                //电池的
                std::stringstream stream1;
                for(int i = 10; i < 18; ++i) {
                    stream1 << std::setfill ('0') << std::setw(2) << std::hex << (int)data[i];
                }
                std::string hexString1 = stream1.str();
                LOGGER(INFO, SROS) <<  "BMS1 hexStr: " << hexString1;

                std::stringstream stream2;
                for(int i = 18; i < 26; ++i) {
                    stream2 << std::setfill ('0') << std::setw(2) << std::hex << (int)data[i];
                }
                std::string hexString2 = stream2.str();
                LOGGER(INFO, SROS) <<  "BMS2 hexStr: " << hexString2;

                LOGGER(INFO, SROS) << "Charging station connect state:" << (cs_connect_state ? "ON":"OFF");
                LOGGER(INFO, SROS) << "Charging station output state:" << (cs_output_state ? "ON":"OFF");
                LOG(INFO) << "cs_output_current:" << cs_output_current << ",cs_output_voltage:" << cs_output_voltage;
                
                last_cs_connect_state = cs_connect_state;
                last_cs_output_state = cs_output_state;
                last_cs_state_flag = cs_state_flag;   
                
                if(last_control_flag != control_flag){
                    LOGGER(INFO, SROS) << "control_flag:" << (last_control_flag ? "CLOSE":"OPEN") << " => " << (control_flag ? "CLOSE":"OPEN");
                    last_control_flag = control_flag;
                }
                if(last_bms_state != bms_state){
                    LOGGER(INFO, SROS) << "bms_state:" << (last_bms_state ? "ERROR":"NORMAL") << " => " << (bms_state ? "ERROR":"NORMAL");
                    last_bms_state = bms_state;
                }
                if(last_old_bms_state != old_bms_state){
                    LOGGER(INFO, SROS) << "old_bms_state:" << (last_old_bms_state ? "ERROR":"NORMAL") << " => " << (old_bms_state ? "ERROR":"NORMAL");
                    last_old_bms_state = old_bms_state;
                }
                
        
            }
        }else {
            LOG(ERROR) <<  "illegal number of bytes，data size: " << data.size() ;

        }          
       
    }
}

void VSCModule::estimationBatteryRemainTime() {
    auto &s = sros::core::Settings::getInstance();

    // 电池的实际最大容量，随着电池的使用次数增加，电池的实际容量会随之减少
    double max_capacity_percentage = s.getValue<int>("battery.battery_max_capacity_percentage", 100);
    auto history_mean_current = s.getValue<int>("battery.agv_mean_current", 2000);  // 存在数据库中的历史平均电流
    double nominal_capacity = s.getValue<int>("battery.battery_nominal_capacity", 33000);  // 电池容量

    auto n = s.getValue<double>("battery.peukert_exponent", 1.1);  // Peukert's exponent
    auto lambda = s.getValue<double>("battery.lambda_new_current_add_to_history_mean_current",
                                     0.01);  // 新的平均电流加入历史平均电流的比例

    const int ULTRA_LOW_BATTERY_PERCENT = 5;  // 超低电量的百分比，此时电池是由vsc来控制，剩余时间要减去这个

    int recent_mean_current = 0;  // 最近的平均电流
    if (g_state.battery_state == sros::core::BATTERY_CHARGING) {
        // 当在充电的时候，按照历史功耗来估算
        recent_mean_current = history_mean_current;
    } else {
        if (recent_discharge_battery_current_.empty()) {
            return;
        }

        auto sum =
            std::accumulate(recent_discharge_battery_current_.begin(), recent_discharge_battery_current_.end(), 0);

        if (recent_discharge_battery_current_.size() == minute_of_mean_current_ * 60) {
            recent_mean_current = sum / recent_discharge_battery_current_.size();
            auto new_history_mean_current = lambda * recent_mean_current + (1 - lambda) * history_mean_current;

//            LOG(INFO) << "[BATTERY] update battery.agv_mean_current from " << history_mean_current << " to "
//                      << new_history_mean_current;

            s.setTmpValue("battery.agv_mean_current", std::to_string(new_history_mean_current));  // 更新历史平均电流
            history_mean_current = new_history_mean_current;
        } else {
            // 若当前的平均电流不够就拿历史的平均电流来凑
            sum += history_mean_current * (minute_of_mean_current_ * 60 - recent_discharge_battery_current_.size());
            recent_mean_current = sum / (minute_of_mean_current_ * 60);
        }
    }

    double remain_capacity = 0;                  // 剩余电池容量
    if (g_state.battery_remain_capacity != 0) {  // 能从电池中获取标称容量数据
        if (nominal_capacity != g_state.battery_nominal_capacity) {
//            LOG(INFO) << "[BATTERY] update battery.battery_nominal_capacity from " << nominal_capacity << " to "
//                      << g_state.battery_nominal_capacity;

            nominal_capacity = g_state.battery_nominal_capacity;
            s.setValue("battery.battery_nominal_capacity", g_state.battery_nominal_capacity);
        }

        if (g_state.battery_percentage == 100) {
            auto new_max_capacity_percentage = g_state.battery_remain_capacity / g_state.battery_nominal_capacity;

//            LOG(INFO) << "update battery.battery_max_capacity_percentage from " << max_capacity_percentage << " to "
//                      << new_max_capacity_percentage;

            s.setValue("battery.battery_max_capacity_percentage", new_max_capacity_percentage);
        }
        remain_capacity = g_state.battery_remain_capacity;
    } else {
        // 根据当前电量百分比估算剩余容量
        remain_capacity = nominal_capacity * g_state.battery_percentage / 100 * max_capacity_percentage / 100;
    }

    int R = nominal_capacity / history_mean_current;  // 预计能使用多久时间
    uint32_t remain_time_min = R * std::pow((remain_capacity / R), n) * 60 / pow(recent_mean_current, n);

    uint32_t low_battery_time_min =
        R * std::pow((nominal_capacity * ULTRA_LOW_BATTERY_PERCENT / 100 / R), n) * 60 / pow(recent_mean_current, n);

    g_state.battery_remain_time = remain_time_min > low_battery_time_min ? (remain_time_min - low_battery_time_min) : 0;

    // REMOVEME: v4.8.0，前期测试用，后期可以删除
//    LOG_EVERY_N(INFO, 5) << "g_state.battery_remain_time: " << g_state.battery_remain_time;
//    LOG_EVERY_N(INFO, 5) << "remain_time_min: " << remain_time_min;
//    LOG_EVERY_N(INFO, 5) << "low_battery_time_min: " << low_battery_time_min;
//    LOG_EVERY_N(INFO, 5) << "battery_remain_capacity: " << remain_capacity;
//    LOG_EVERY_N(INFO, 5) << "recent_mean_current: " << recent_mean_current;
//    LOG_EVERY_N(INFO, 5) << "minute_of_mean_current_: " << minute_of_mean_current_;
}

void VSCModule::handleActionControllerData(const vector<uint8_t> &data) {
    // TODO(nobody): 实现AC处理逻辑
}

void VSCModule::handleSonarUnitData(const vector<uint8_t> &data) {
    const int SONAR_SENSOR_NUM = 16;

    uint8_t command = data[1];

    if (command == 0x06) {                                // 状态返回
        if (data.size() != (2 + 2 + SONAR_SENSOR_NUM)) {  // 数据段长度错误
//            LOG(WARNING) << "USART Data length error";
            return;
        }

        uint16_t sonar_device = data[0 + 2] << 8 + data[1 + 2];

        // 构造DistanceDataMsg发送
        auto mm = std::make_shared<sros::core::DistanceDataMsg>("SONAR_DATA");
        mm->distances.clear();

        const uint8_t INFINITY_DISTANCE = 250;

        for (int i = 0; i < SONAR_SENSOR_NUM && i < data.size(); i++) {
            uint32_t d = (uint32_t)data[2 + 2 + i] * 10;  // 单位从cm转换为mm
            if (d >= INFINITY_DISTANCE) {
                d = 0xffffffff;  // 无穷远
            }

            mm->distances.push_back(d);
        }

        sendMsg(mm);
    }
}

void VSCModule::handleSensorData(const vector<uint8_t> &data) {
    uint8_t command = data[1];

    if (command == 0x06) {       // 状态返回
//        LOG(INFO) << data;
        if (data.size() != 11) {  // 数据段长度错误
//            LOG(ERROR) << "USART Data length error";
            return;
        }
        g_state.box_temperature_max = (data[2] << 8) + data[3];
        g_state.box_temperature_min = (data[4] << 8) + data[5];
        g_state.box_humidity_max = (data[6] << 8) + data[7];
        g_state.box_humidity_min = (data[8] << 8) + data[9];
        g_state.fan_switch_state = data[10] == 0 ? false : true;
    }
}

void VSCModule::updateHMIState() {
    enum HMISystemState {
        HMI_HARDWARE_BOOTING = 0x01,
        HMI_SYSTEM_INITIALIZING = 0x02,
        HMI_SYSTEM_READY = 0x10,
        HMI_RUNNING = 0x20,
        HMI_SLOWDOWN = 0x22,
        HMI_BLOCKED = 0x24,
        HMI_BATTERY_LOW = 0x30,
        HMI_BATTERY_CHARGING = 0x32,
        HMI_EMERGENCY_STOP = 0xA0,
        HMI_SYSTEM_FAULT = 0xB0,  // FAULT状态不由SROS下发，VSC自动生成
        HMI_SYSTEM_ERROR = 0xC0,

        HMI_VSC_HMI_DISABLED = 0xFF,  // 禁用VSC HMI功能
    };

    enum HMIMotionState {
        HMI_STILL = 0x01,
        HMI_FORWARD = 0x10,
        HMI_BACKWARD = 0x11,
        HMI_LEFTWARD = 0x12,
        HMI_RIGHTWARD = 0x13,
        HMI_MOTION_EMERGENCY_STOP = 0x20,
    };

    HMISystemState hmi_sys_state = HMI_SYSTEM_INITIALIZING;

    using namespace sros::core;

    switch (g_state.sys_state) {
        case SYS_STATE_INITIALING:
        case SYS_STATE_START_LOCATING:
            hmi_sys_state = HMI_SYSTEM_INITIALIZING;
            break;
        case SYS_STATE_IDLE:
        case SYS_STATE_TASK_NAV_NO_WAY:
            hmi_sys_state = HMI_SYSTEM_READY;
            break;
        case SYS_STATE_TASK_NAV_INITIALING:
        case SYS_STATE_TASK_NAV_FINDING_PATH:
        case SYS_STATE_TASK_NAV_WAITING_FINISH:
        case SYS_STATE_TASK_NAV_REFINDING_PATH:
        case SYS_STATE_TASK_PATH_NAV_INITIALING:
        case SYS_STATE_TASK_PATH_WAITING_FINISH:
            hmi_sys_state = HMI_RUNNING;
            break;
        case SYS_STATE_TASK_NAV_WAITING_FINISH_SLOW:
        case SYS_STATE_TASK_PATH_WAITING_FINISH_SLOW:
            hmi_sys_state = HMI_SLOWDOWN;
            break;
        case SYS_STATE_TASK_NAV_PAUSED:
        case SYS_STATE_TASK_PATH_PAUSED:
            hmi_sys_state = HMI_BLOCKED;
            break;
        default:
            hmi_sys_state = HMI_SYSTEM_READY;
            break;
    }
    if (g_state.in_manual_obstacle_paused) {
        hmi_sys_state = HMI_BLOCKED;
    }

    // 如果定位不成功，则设置HMI状态为初始化中
    if (g_state.location_state != LOCATION_STATE_RUNNING) {
        hmi_sys_state = HMI_SYSTEM_INITIALIZING;
    }

    auto &s = sros::core::Settings::getInstance();
    auto low_battery_threshold = s.getValue<int>("main.low_battery_threshold", 15);

    auto fault_center = sros::core::FaultCenter::getInstance();
    fault_center->track(sros::core::FAULT_CODE_BATTERY_IS_TOO_LOW, [&]() {
        return g_state.battery_percentage < low_battery_threshold && !g_state.isChargeState();
    });

    if (g_state.battery_percentage < low_battery_threshold) {
        hmi_sys_state = HMI_BATTERY_LOW;
    }

    if (g_state.battery_state == BATTERY_CHARGING) {
        hmi_sys_state = HMI_BATTERY_CHARGING;
    }

    if (g_state.emergency_state == STATE_EMERGENCY_TRIGER || g_state.emergency_state == STATE_EMERGENCY_RECOVERABLE) {
        hmi_sys_state = HMI_EMERGENCY_STOP;
    }

    if (g_state.sys_state == SYS_STATE_ERROR || g_state.sys_state == SYS_STATE_HARDWARE_ERROR) {
        hmi_sys_state = HMI_SYSTEM_ERROR;
    }

    HMIMotionState hmi_motion_state = HMI_STILL;

    // 根据src上传的movement_state得到运动状态
    if (g_src_state.movement_state == MOVEMENT_STILL) {
        hmi_motion_state = HMI_STILL;
    }
    if (g_src_state.movement_state == MOVEMENT_GO_FORWARD) {
        hmi_motion_state = HMI_FORWARD;
    }
    if (g_src_state.movement_state == MOVEMENT_GO_BACKWARD) {
        hmi_motion_state = HMI_BACKWARD;
    }
    if (g_src_state.movement_state == MOVEMENT_TURN_LEFT) {
        hmi_motion_state = HMI_LEFTWARD;
    }
    if (g_src_state.movement_state == MOVEMENT_TURN_RIGHT) {
        hmi_motion_state = HMI_RIGHTWARD;
    }

    if (g_state.emergency_state == STATE_EMERGENCY_TRIGER || g_state.emergency_state == STATE_EMERGENCY_RECOVERABLE) {
        hmi_motion_state = HMI_MOTION_EMERGENCY_STOP;
    }

    if (is_vsc_hmi_disabled_) {
        // 禁用VSC HMI功能
        hmi_sys_state = HMI_VSC_HMI_DISABLED;
    }

    vector<uint8_t> payload;
    payload.push_back(0x05);  // HMI
    payload.push_back(0x11);  // 更新状态

    // uint8_t hmi_speaker_volume = g_state.speaker_volume;

    payload.push_back((uint8_t)(hmi_sys_state));
    payload.push_back((uint8_t)(hmi_motion_state));
    payload.push_back((uint8_t)(g_state.cur_volume * 28 / 100));  // 根据协议，最高音量为28

    sendUsartData(payload);
}

void VSCModule::handleVSCData(const vector<uint8_t> &data) {
    uint8_t command = data[1];

    if (command == 0x02 && data.size() >= 14) {  // 版本信息返回
        uint32_t serial_no = ((uint32_t)data[0 + 2] << 24) + ((uint32_t)data[1 + 2] << 16) +
                             ((uint32_t)data[2 + 2] << 8) + ((uint32_t)data[3 + 2]);

        uint32_t sw_version_no = ((uint32_t)data[0 + 6] << 24) + ((uint32_t)data[1 + 6] << 16) +
                                 ((uint32_t)data[2 + 6] << 8) + ((uint32_t)data[3 + 6]);

        uint32_t hw_version_no = ((uint32_t)data[0 + 10] << 24) + ((uint32_t)data[1 + 10] << 16) +
                                 ((uint32_t)data[2 + 10] << 8) + ((uint32_t)data[3 + 10]);

        int major_version = sw_version_no / (1000 * 1000);
        int minor_version = (sw_version_no / 1000) % 1000;
        int patch_version = sw_version_no % 1000;

        string serial_no_str = to_string(serial_no);
        string sw_version_no_str =
            to_string(major_version) + "." + to_string(minor_version) + "." + to_string(patch_version);
        string hw_version_no_str = to_string(hw_version_no);

        //VSC 版本 A.B.C： B>= 8 根据扩充的安全单元0X14触发源中Byte0的值来决定是否可恢复。
        if (minor_version >= 8) {
            is_send_sros_emergency_src_ = true;
        }

        if (vsc_device_) {
            vsc_device_->setSerialNo(serial_no_str);
            vsc_device_->setVersionNo(sw_version_no_str);
            vsc_device_->setModelNo(hw_version_no_str);
//            LOG(INFO) << "VSC version: " << sw_version_no_str
//                      << ", hardware version: " << hw_version_no_str
//                      << ", is_send_sros_emergency_src: " << is_send_sros_emergency_src_;
        }
        vsc_upgrade_.checkUpgradeResult(sw_version_no); // 检查自研硬件是否升级成功: 判断版本号
        is_recv_vsc_version_ = true;
    }
}

} /* namespace vsc */
