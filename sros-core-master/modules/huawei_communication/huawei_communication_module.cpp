
//
// Created by john on 18-9-27.
//

#include "huawei_communication_module.h"

#include <memory>
#include <glog/logging.h>

#include "core/device/can_interface.h"
#include "core/map_manager.h"
#include "core/msg/can_data_msg.hpp"
#include "core/msg/command_msg.hpp"
#include "core/msg/huawei_comm_data_msg.hpp"
#include "core/msg/notification_msg.hpp"
#include "core/settings.h"
#include "core/src.h"
#include "core/state.h"
#include "core/task/task_manager.h"

using namespace sros::core;
using namespace sros::device;
using namespace std;

namespace huawei {

HuaweiCommModule::HuaweiCommModule() : sros::CommunicationModule("HuaweiComm") {}

bool HuaweiCommModule::subClassRunPrepare() {
    LOG(INFO) << "HuaweiCommModule module start running";

    auto &s = sros::core::Settings::getInstance();
    low_battery_threshold_ = s.getValue<int>("main.low_battery_threshold", 20);

    enabled_ = (s.getValue<std::string>("main.enable_huawei_communication", "False") == "True");

    if (!enabled_) {
        LOG(INFO) << "HuaweiCommModule module stop running(disable)";
        stop();
        return false;
    }

    server_ptr_ = std::make_shared<SimpleServer>(5003);
    lc100_a_ = createDevice<LC100>(DEVICE_LC100, DEVICE_ID_LC100, DEVICE_COMM_INTERFACE_TYPE_CAN_1,
                                   std::make_shared<CanInterface>(0x300));
    lc100_b_ = createDevice<LC100>(DEVICE_LC100_2, DEVICE_ID_LC100_2, DEVICE_COMM_INTERFACE_TYPE_CAN_1,
                                   std::make_shared<CanInterface>(0x302));

    if (!lc100_a_ || !lc100_b_) {
        LOG(INFO) << "HuaweiCommModule module stop lc100 create failed!";
        stop();
        return false;
    }

    bool enable_pgv_rectify = (s.getValue<std::string>("main.enable_pgv_rectify", "False") == "True");
    if (!enable_pgv_rectify) {
        LOG(INFO) << "HuaweiCommModule module stop 请开启pgv矫正功能";
        stop();
        return false;
    }

    bool enable_qrcode_location = (s.getValue<std::string>("main.enable_qrcode_location", "False") == "True");
    if (!enable_qrcode_location) {
        LOG(INFO) << "HuaweiCommModule module stop 请开启二维码辅助定位功能";
        stop();
        return false;
    }

    boost::thread t(boost::bind(&HuaweiCommModule::runServer, this));

    subscribeTopic("HUAWEI_TEST_LINE", CALLBACK(&HuaweiCommModule::onNewMsg));
    subscribeTopic("TIMER_200MS", CALLBACK(&HuaweiCommModule::onTimer_200ms));

    LOG(INFO) << "HuaweiCommModule: 启动";

    return true;
}

void HuaweiCommModule::onTimer_200ms(sros::core::base_msg_ptr m) {
    if (is_action_run_) {
        return;
    }

    // 检测定位错误是否要回复
    if (is_location_error_) {
        if (g_state.station_no != 0) {  // 当前站点不为0
            auto checkPgvInfoFun = [&]() {
                std::vector<int> results = src_sdk->getCurPgvInfo();

                if (results.empty()) {
                    return false;
                }

                int cur_pgv_id = g_state.cur_down_camera_offset.get().id;
                int last_pgv_id = 0;
                auto station = MapManager::getInstance()->getStation(g_state.station_no);
                if (station && !station.dmcode_id.empty()) {
                    DMCodeOffset offset(station.dmcode_id, station.dmcode_offset.x / 100, station.dmcode_offset.y / 100,
                                        station.dmcode_offset.yaw);
                    last_pgv_id = offset.id;
                }

                if (cur_pgv_id == 0) {  // 防止到了一个没有pgv的站点，此时历史id和当前id都为0，而解除错误
                    return false;
                }

                if (cur_pgv_id != last_pgv_id) {
                    LOG(INFO) << "HuaweiCommModule current pgv id not equal to last pgv id!";
                    return false;
                }

                return true;
            };

            if (checkPgvInfoFun() &&
                g_state.isLocateSucceed()) {  // 若pvg的信息对的上了，而且还是定位成功的，就解除定位出错的问题
                is_location_error_ = false;
            }
        }

        return;
    }

    updateGoodsInfo();  // 若没有动作的情况下获取传感器信息检测是否出现异常

    updateLEDState();
}

void HuaweiCommModule::onNewMsg(sros::core::base_msg_ptr msg) {
    //    LOG(INFO) << "onNewMsg";
    auto mm = std::dynamic_pointer_cast<sros::core::HuaweiCommMsg>(msg);

    SimpleSession_Ptr session_ptr = mm->session_ptr;
    CommMsg &comm_msg = session_ptr->getMsg();
    const std::vector<uint8_t> &request_data = comm_msg.getData();  // 请求的数据的数据
    std::vector<uint8_t> response_data;                             // 需要正确回复的数据

    auto setCommonError = [&]() {
        if (g_state.battery_percentage < low_battery_threshold_) {
            comm_msg.setErrorCode(ERROR_LOW_BATTERY);
            return true;
        }
        if (g_state.isEmergency()) {
            comm_msg.setErrorCode(ERROR_EMERGENCY_TRIGGERED);
            return true;
        }
        if (g_state.isBreakSwitchON()) {
            comm_msg.setErrorCode(ERROR_BREAK_SWITCH_TRIGGERED);
            return true;
        }
        if (g_state.isLocationError()) {
            comm_msg.setErrorCode(ERROR_LOCATION_NOT_RUNNING);
            return true;
        }
        if (g_state.isNavNoWay()) {
            comm_msg.setErrorCode(FAILED_NAV_NO_WAY);
            return true;
        }
        if (is_location_error_) {
            comm_msg.setErrorCode(FAILED_LACATION_ERROR);
            return true;
        }
        return false;
    };

    switch (comm_msg.command_) {
        case (int)CommMsg::CMD::INITIALIZE: {
            if (setCommonError()) {
                break;
            }
            if (!g_state.isSystemIDLE()) {
                comm_msg.setErrorCode(ERROR_SYSTEM_BUSY);
            }

            if (!comm_msg.isError()) {
                response_data.push_back(0x00);
                comm_msg.setData(response_data);
            }
            break;
        }
        case (int)CommMsg::CMD::QUERY_STATE: {
            if (setCommonError()) {
                break;
            }
            if (line_goods_info_ == GOODS_INFO_ERROR) {
                comm_msg.setErrorCode(FAILED_SENSOR_ERROR);
                break;
            }

            if (!comm_msg.isError()) {
                response_data.resize(2);
                response_data[0] = (g_state.isSystemIDLE() && !is_agv_run_ && !is_action_run_ && !is_pgv_run_)
                                       ? 0x00
                                       : 0x01;  // 链板线在动也看做时正在运动
                response_data[1] = line_goods_info_;
                comm_msg.setData(response_data);
            }
            break;
        }
        case (int)CommMsg::CMD::QUERY_POSITION: {
            response_data.resize(1);
            response_data[0] = g_state.station_no;
            comm_msg.setData(response_data);
            break;
        }
        case (int)CommMsg::CMD::PARAM_SETTING: {
            switch (request_data.front()) {
                case 0x00: {  // 设置ip 暂不提供
                    response_data.push_back(0x00);
                    comm_msg.setData(response_data);
                    break;
                }
                case 0x01: {  // 设置AGV速度
                    if (request_data.back() < 1 || request_data.back() > 5 || request_data[1] != 0x00) {
                        comm_msg.setErrorCode(ERROR_REQUEST_DATA);
                        break;
                    }

                    int level = request_data.back() * 20;

                    if (setSpeedLevel(level)) {
                        response_data.push_back(0x00);
                        comm_msg.setData(response_data);
                    } else {
                        comm_msg.setErrorCode(ERROR_FAILED_EXEC);
                    }

                    break;
                }
                default: {
                    LOG(ERROR) << "HuaweiCommModule: UNREACHEBLE! " << response_data.front();
                    comm_msg.setErrorCode(ERROR_REQUEST_DATA);
                    break;
                }
            }

            break;
        }
        case (int)CommMsg::CMD::IO_SETTING: {
            bool is_succeed = false;
            switch (request_data[1]) {
                case 0x00: {  // 关闭
                    switch (request_data[0]) {
                        case FRIST_LINE_ROTATE_REVERSE:
                        case FRIST_LINE_ROTATE_FORWARD: {
                            // 停止第一排转动
                            is_succeed = startNewCoexistAction(task_no_, 3, 33, 0);

                            break;
                        }
                        case SECOND_LINE_ROTATE_REVERSE:
                        case SECOND_LINE_ROTATE_FORWARD: {
                            // 停止第二排转动
                            is_succeed = startNewCoexistAction(task_no_, 3, 36, 0);
                            break;
                        }
                        default: {
                            LOG(ERROR) << "HuaweiCommModule: UNREACHEBLE! " << response_data.front();
                            comm_msg.setErrorCode(ERROR_REQUEST_DATA);
                            break;
                        }
                    }
                    break;
                }
                case 0x01: {  // 开启
                    switch (request_data[0]) {
                        case FRIST_LINE_ROTATE_REVERSE: {
                            is_succeed = startNewAction(task_no_, 3, 32, 0);
                            break;
                        }
                        case FRIST_LINE_ROTATE_FORWARD: {
                            is_succeed = startNewAction(task_no_, 3, 31, 0);
                            break;
                        }
                        case SECOND_LINE_ROTATE_REVERSE: {
                            is_succeed = startNewAction(task_no_, 3, 35, 0);
                            break;
                        }
                        case SECOND_LINE_ROTATE_FORWARD: {
                            is_succeed = startNewAction(task_no_, 3, 34, 0);

                            break;
                        }
                        default: {
                            LOG(ERROR) << "HuaweiCommModule: UNREACHEBLE! " << response_data.front();
                            comm_msg.setErrorCode(ERROR_REQUEST_DATA);
                            break;
                        }
                    }
                    break;
                }
                default: {
                    LOG(ERROR) << "HuaweiCommModule: UNREACHEBLE! " << response_data.front();
                    comm_msg.setErrorCode(ERROR_REQUEST_DATA);
                    break;
                }
            }

            if (is_succeed) {
                response_data.push_back(0x00);
                comm_msg.setData(response_data);
            } else {
                comm_msg.setErrorCode(ERROR_FAILED_EXEC);
            }
            break;
        }
        case (int)CommMsg::CMD::MOVE_TO_POSITION: {
            if (isSystemError()) {
                comm_msg.setErrorCode(ERROR_SYSTEM);
                break;
            }
            if (!g_state.isSystemIDLE()) {
                comm_msg.setErrorCode(ERROR_SYSTEM_BUSY);
                break;
            }

            sros::core::StationNo_t station_no = request_data[0];
            LOG(INFO) << "移动到站点：" << station_no;

            if (moveToStation(movement_task_no_, station_no, OBSTACLE_AVOID_WAIT)) {
                is_agv_run_ = true;
                response_data.push_back(0x00);
                comm_msg.setData(response_data);
            } else {
                comm_msg.setErrorCode(ERROR_FAILED_EXEC);
            }
            break;
        }
        case (int)CommMsg::CMD::LOAD_MATERIAL: {
            if (isSystemError()) {
                comm_msg.setErrorCode(ERROR_SYSTEM);
                break;
            }
            if (!g_state.isSystemIDLE()) {
                comm_msg.setErrorCode(ERROR_SYSTEM_BUSY);
                break;
            }
            if (request_data[1] == 0x01 &&
                (FIRST_LINE_HAS_GOODS == line_goods_info_ || line_goods_info_ == BOTH_LINE_HAS_GOODS)) {
                comm_msg.setErrorCode(ERROR_ERROR_HAS_GOODS_GET_LOAD_CMD);
                break;
            }
            if (request_data[1] == 0x02 &&
                (SECOND_LINE_HAS_GOODS == line_goods_info_ || line_goods_info_ == BOTH_LINE_HAS_GOODS)) {
                comm_msg.setErrorCode(ERROR_ERROR_HAS_GOODS_GET_LOAD_CMD);
                break;
            }

            bool is_succeed = false;
            if (!is_action_run_) {
                LOG(INFO) << request_data.size();
                if (request_data[0] == 0x11) {  // 向左
                    switch (request_data[1]) {
                        case 0x01: {  // 第一个
                            is_succeed = startNewAction(task_no_, 3, 3, 1);
                            LOG(INFO) << "左边第一个上料！";

                            break;
                        }
                        case 0x02: {  // 第二个
                            is_succeed = startNewAction(task_no_, 3, 3, 2);
                            LOG(INFO) << "左边第二个上料！";

                            break;
                        }
                        case 0x03: {  // 两个都要取货 暂不提供
                                      //                            is_succeed = startNewAction(task_no_, 3, 3, 1) &&
                                      //                            startNewAction(task_no_, 3, 3, 2);
                            LOG(INFO) << "UNRACHABLE";
                            break;
                        }
                    }
                } else if (request_data[0] == 0x12) {  // 向右
                    switch (request_data[1]) {
                        case 0x01: {  // 第一个
                            is_succeed = startNewAction(task_no_, 3, 1, 1);
                            LOG(INFO) << "右边第一个上料！";

                            break;
                        }
                        case 0x02: {  // 第二个
                            is_succeed = startNewAction(task_no_, 3, 1, 2);
                            LOG(INFO) << "右边第二个上料！";

                            break;
                        }
                        case 0x03: {  // 两个都要取货
                                      //                            is_succeed = startNewAction(task_no_, 1, 3, 1) &&
                                      //                            startNewAction(task_no_, 1, 3, 2);
                            LOG(INFO) << "UNRACHABLE";
                            break;
                        }
                    }
                }
            }

            if (is_succeed) {
                is_action_run_ = true;
                response_data.push_back(0x00);
                comm_msg.setData(response_data);
            } else {
                comm_msg.setErrorCode(ERROR_FAILED_EXEC);
            }
            break;
        }
        case (int)CommMsg::CMD::UNLOAD_MATERIAL: {
            if (isSystemError()) {
                comm_msg.setErrorCode(ERROR_SYSTEM);
                LOG(INFO) << "系统错误！";
                break;
            }
            if (!g_state.isSystemIDLE()) {
                comm_msg.setErrorCode(ERROR_SYSTEM_BUSY);
                LOG(INFO) << "系统忙！";
                break;
            }
            if (request_data[1] == 0x01 &&
                (NO_GOODS == line_goods_info_ || SECOND_LINE_HAS_GOODS == BOTH_LINE_HAS_GOODS)) {
                comm_msg.setErrorCode(ERROR_ERROR_NO_GOODS_GET_UNLOAD_CMD);
                break;
            }
            if (request_data[1] == 0x02 &&
                (NO_GOODS == line_goods_info_ || FIRST_LINE_HAS_GOODS == BOTH_LINE_HAS_GOODS)) {
                comm_msg.setErrorCode(ERROR_ERROR_NO_GOODS_GET_UNLOAD_CMD);
                break;
            }

            bool is_succeed = false;
            if (!is_action_run_) {
                if (request_data[0] == 0x11) {  // 向左
                    switch (request_data[1]) {
                        case 0x01: {  // 第一个
                            is_succeed = startNewAction(task_no_, 3, 4, 1);
                            LOG(INFO) << "左边第一个下料！";

                            break;
                        }
                        case 0x02: {  // 第二个
                            is_succeed = startNewAction(task_no_, 3, 4, 2);
                            LOG(INFO) << "左边第二个下料！";

                            break;
                        }
                        case 0x03: {  // 两个都要取货
                                      //                            is_succeed = startNewAction(task_no_, 3, 4, 1) &&
                                      //                            startNewAction(task_no_, 3, 4, 2);
                            LOG(INFO) << "UNRACHABLE";
                            break;
                        }
                    }
                } else if (request_data[0] == 0x12) {  // 向右
                    switch (request_data[1]) {
                        case 0x01: {  // 第一个
                            is_succeed = startNewAction(task_no_, 3, 2, 1);
                            LOG(INFO) << "右边第一个下料！";

                            break;
                        }
                        case 0x02: {  // 第二个
                            is_succeed = startNewAction(task_no_, 3, 2, 2);
                            LOG(INFO) << "右边第二个下料！";

                            break;
                        }
                        case 0x03: {  // 两个都要取货
                                      //                            is_succeed = startNewAction(task_no_, 1, 2, 1) &&
                                      //                            startNewAction(task_no_, 1, 2, 2);
                            LOG(INFO) << "UNRACHABLE";
                            break;
                        }
                    }
                }
            }

            if (is_succeed) {
                is_action_run_ = true;
                response_data.push_back(0x00);
                comm_msg.setData(response_data);
            } else {
                comm_msg.setErrorCode(ERROR_FAILED_EXEC);
            }
            break;
        }
        case (int)CommMsg::CMD::MOVE_TO_ORIGIN: {
            if (moveToStation(movement_task_no_, 1, OBSTACLE_AVOID_WAIT)) {
                is_agv_run_ = true;

                response_data.push_back(0x00);
                comm_msg.setData(response_data);
            } else {
                comm_msg.setErrorCode(ERROR_FAILED_EXEC);
            }
            break;
        }
        case (int)CommMsg::CMD::RESERVED: {
            comm_msg.setErrorCode(ERROR_REQUEST_CMD);
            break;
        }
        case (int)CommMsg::CMD::QUERY_ALARM: {
            int error_num = 0;
            auto addAlarmFun = [&](uint16_t alarm) {
                ++error_num;
                response_data.resize(error_num * 2 + 1);
                response_data[0] = error_num;
                response_data[(error_num - 1) * 2 + 1] = (alarm >> 8) & 0xFF;
                response_data[(error_num - 1) * 2 + 2] = (alarm >> 0) & 0xFF;
            };
            if (g_state.isPausedForObstacle()) {
                addAlarmFun(WARNING_PAUSED_FOR_OBSTACAL);
                comm_msg.setData(response_data);
            }
            if (g_state.battery_percentage < 30) {
                addAlarmFun(WARNING_LOW_BATTERY);
                comm_msg.setData(response_data);
            }
            if (error_num == 0) {  // 没有出错的情况
                response_data.push_back(0x00);
                comm_msg.setData(response_data);
            }
            break;
        }
        default: {
            LOG(ERROR) << "HuaweiCommModule: UNREACHEBLE! " << comm_msg.command_;
            comm_msg.setErrorCode(ERROR_REQUEST_CMD);
        }
    }

    session_ptr->itIsTimeToResponse();
}

void HuaweiCommModule::recvDataMsgCallback(SimpleSession_Ptr session_ptr) {
    // 其他线程发送的

    auto msg = std::make_shared<sros::core::HuaweiCommMsg>();
    msg->session_ptr = session_ptr;

    sendMsg(msg);
}

bool HuaweiCommModule::isSystemError() const {  // 执行各种动作前先要检测是否有系统错误存在，若存在就不能运行
    if (g_state.battery_percentage < low_battery_threshold_) {
        return true;
    }
    if (g_state.isEmergency()) {
        return true;
    }
    if (g_state.isBreakSwitchON()) {
        return true;
    }
    if (g_state.isLocationError()) {
        return true;
    }
    if (is_location_error_) {
        return true;
    }

    return false;
}

void HuaweiCommModule::runServer() {
    server_ptr_->set_msg_callback_func(boost::bind(&HuaweiCommModule::recvDataMsgCallback, this, _1));
    server_ptr_->run();
}

void HuaweiCommModule::updateGoodsInfo() {
    int eu100_input_value = 0;
    if (!src_sdk->getParameter(0x1300, eu100_input_value)) {
        return;  // 获取状态失败
    }
    //    LOG(INFO) << "HuaweiCommModule: eu100_input_value ===" << eu100_input_value;
    if (eu100_input_value & FRIST_LINE_LEFT_SENSOR || eu100_input_value & FRIST_LINE_RIGHT_SENSOR ||
        eu100_input_value & SECOND_LINE_LEFT_SENSOR ||
        eu100_input_value & SECOND_LINE_RIGHT_SENSOR) {  // 若两边的传感器出现了被挡住的情况，直接报错
        LOG(INFO) << "HuaweiCommModule: eu100_input_value ===" << (uint16_t)eu100_input_value;

        // 获取当前的Task信息
        auto cur_task = sros::core::TaskManager::getInstance()->getMovementTask();
        if (line_goods_info_ != GOODS_INFO_ERROR && cur_task &&
            cur_task->isRunning()) {  // 若当前有移动任务，然后检测到传感器状态为异常，需要暂停移动任务
            LOG(INFO) << "HuaweiCommModule: set pause";
            srcPause();
        }

        line_goods_info_ = GOODS_INFO_ERROR;
        LOG(INFO) << "sensor error!";
        return;
    } else {  // 传感器没有异常的情况
        auto cur_task = sros::core::TaskManager::getInstance()->getMovementTask();
        if (line_goods_info_ == GOODS_INFO_ERROR && cur_task &&
            cur_task->isRunning()) {  // 上一次链板线出错了,人工手动恢复后继续走
            srcContinue();
        }
    }

    bool has_goods_frist_line = (eu100_input_value & FRIST_LINE_MIDDLE_SENSOR) != 0;
    bool has_goods_second_line = (eu100_input_value & SECOND_LINE_MIDDLE_SENSOR) != 0;

    if (has_goods_frist_line && has_goods_second_line) {
        line_goods_info_ = BOTH_LINE_HAS_GOODS;
    } else if (has_goods_frist_line) {
        line_goods_info_ = FIRST_LINE_HAS_GOODS;
    } else if (has_goods_second_line) {
        line_goods_info_ = SECOND_LINE_HAS_GOODS;
    } else {
        line_goods_info_ = NO_GOODS;
    }

    //    LOG(INFO) << "line_goods_info_ : " << line_goods_info_;
}

void HuaweiCommModule::onMoveTaskFinishedNotify(sros::core::NotificationMsg_ptr msg) {
    const auto &movement_task = msg->movement_task;
    if (movement_task->isSetDownCameraOffset() && movement_task->getTaskNo() == movement_task_no_) {
        is_agv_run_ = false;

        // 检测pgv信息是否合法，当移动到一个站点后，若当前站点的历史pgv和当前站点的pgv对不上，就报错
        auto checkPgvInfoFun = [&]() {
            // 由于可能会出现获取pgv_info 失败的情况，所以此处尝试多次
            std::vector<int> results;
            for (int i = 0; i < 3; ++i) {  // 尝试次数
                results = src_sdk->getCurPgvInfo();
                if (!results.empty()) {
                    break;
                }
                usleep(100 * 1000);
            }

            if (results.empty()) {
                LOG(ERROR) << "HuaweiCommModule getCurPgvInfo error!";
                return false;
            }

            int cur_pgv_id = g_state.cur_down_camera_offset.get().id;
            int last_pgv_id = movement_task->getDownCameraOffset().id;
            if (cur_pgv_id != last_pgv_id) {
                LOG(INFO) << "HuaweiCommModule current pgv id not equal to last pgv id!";
                return false;
            }

            return true;
        };

        if (!checkPgvInfoFun()) {  // 检测pgv信息没通过了
            is_location_error_ = true;
            return;
        }

        if (startNewAction(pgv_adjust_task_no_, 7, 1, 0)) {
            is_pgv_run_ = true;
        }
    }
}

void HuaweiCommModule::onActionTaskFinishedNotify(sros::core::NotificationMsg_ptr msg) {
    auto action_task = msg->action_task;
    if (action_task->getTaskNo() == pgv_adjust_task_no_) {
        LOG(INFO) << "pgv_adjust_task finished";
        is_pgv_run_ = false;
    } else if (action_task->getTaskNo() == task_no_) {
        LOG(INFO) << "goods_task finished";

        updateGoodsInfo();  // 上货不管成功还是失败都需要查询一遍链板线是是否有货

        is_action_run_ = false;
    } else {
        LOG(ERROR) << "HuaweiCommModule UNREACHEBLE";
    }
}

void HuaweiCommModule::updateLEDState() {
    // 左转向
    if (g_src_state.movement_state == MOVEMENT_TURN_LEFT) {
        lc100_b_->setLed(LED_1, LED_BLUE, LED_STATIC);
    } else {
        lc100_b_->setLed(LED_1, LED_OFF, LED_STATIC);
    }

    // 右转向
    if (g_src_state.movement_state == MOVEMENT_TURN_RIGHT) {
        lc100_b_->setLed(LED_3, LED_BLUE, LED_STATIC);
    } else {
        lc100_b_->setLed(LED_3, LED_OFF, LED_STATIC);
    }

    // 是否有障碍
    bool is_blocked = false;
    if (g_state.sys_state == SYS_STATE_TASK_PATH_PAUSED || g_state.sys_state == SYS_STATE_TASK_NAV_PAUSED) {
        is_blocked = true;
        lc100_b_->setLed(LED_2, LED_RED, LED_STATIC);
    } else if (g_state.sys_state == SYS_STATE_TASK_NAV_WAITING_FINISH_SLOW ||
               g_state.sys_state == SYS_STATE_TASK_PATH_WAITING_FINISH_SLOW) {
        is_blocked = true;
        lc100_b_->setLed(LED_2, LED_YELLOW, LED_STATIC);
    } else {
        lc100_b_->setLed(LED_2, LED_GREEN, LED_STATIC);
    }

    //     电量
    if (g_state.battery_percentage < low_battery_threshold_) {
        lc100_a_->setLed(LED_3, LED_RED, LED_BREATH, 2000);
    } else {
        lc100_a_->setLed(LED_3, LED_GREEN, LED_BREATH, 2000);
    }

    // 运行 故障 低电量
    if (g_state.battery_percentage < low_battery_threshold_) {
        lc100_a_->setLed(LED_2, LED_RED, LED_STATIC);
    } else if (g_state.isEmergency() || g_state.sys_state == SYS_STATE_ERROR || g_state.isLocationError()) {
        lc100_a_->setLed(LED_2, LED_BLUE, LED_STATIC);
    } else {
        lc100_a_->setLed(LED_2, LED_GREEN, LED_STATIC);
    }

    // 状态
    if (g_state.isEmergency() || g_state.sys_state == SYS_STATE_ERROR || g_state.isLocationError() ||
        g_state.battery_percentage < low_battery_threshold_ || g_state.isBreakSwitchON()) {
        lc100_a_->setLed(LED_1, LED_RED, LED_STATIC);
    } else if (is_blocked || g_state.sys_state == SYS_STATE_INITIALING) {
        lc100_a_->setLed(LED_1, LED_YELLOW, LED_STATIC);
    } else {
        lc100_a_->setLed(LED_1, LED_GREEN, LED_STATIC);
    }
}

bool HuaweiCommModule::sendCanMsg(uint32_t id, const std::vector<uint8_t> &data) {
    auto msg = std::make_shared<sros::core::CanDataMsg>("TOPIC_MSG_TO_CAN");
    msg->can_id = id;
    msg->data = std::move(data);
    sendMsg(msg);

    return true;
}
}  // namespace huawei