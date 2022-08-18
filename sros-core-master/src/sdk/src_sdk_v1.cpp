//
// Created by caoyan on 2020-11-12
//

#include "src_sdk_v1.h"
#include "core/device/device_manager.h"
#include "core/modbus/register_admin.h"
#include "core/src.h"
#include "core/util/utils.h"
#include "protocol/all_msg.h"
#include "protocol/sros_state_msg.hpp"
#include "core/fault_center.h"
#include "core/fault_code.h"

using namespace src;
using namespace sros::device;
using namespace sros::core;

namespace sdk {

SrcSdkV1::SrcSdkV1() : SrcSdkDo() {}

void SrcSdkV1::init() {
    std::thread t(&SrcSdkV1::checker, this);
    boost::thread(boost::bind(&SrcSdkV1::dealSrcHeartBeat, this));
    boost::thread(boost::bind(&SrcSdkV1::setNewStateByNotify, this));
    t.detach();

    initMotorSerialNo();
}

void SrcSdkV1::checker() {
    uint32_t i = 0;
    auto &s = sros::core::Settings::getInstance();
    bool enable_detect_manual_btn_signal = (s.getValue<std::string>("main.detect_manual_btn_signal", "False") == "True");
    bool enable_1353 = (s.getValue<std::string>("main.enable_1353", "False") == "True");

    while (true) {
        if(i % 10 == 0) {
            dealSrcPoseTimeout();

            if (i % 50 == 0) {
                updateActionControlDebugInfo();
                getSrcDebugInfo();
            }
            
            if(enable_detect_manual_btn_signal) {
                updateControlMode();
            }
        }

        if(i % 2 == 0) {
            updateForkHeight();
        }

        if(i % 3 == 0) {
            if(enable_1353) {
                update1353IO();
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        i++;
    }
}

void SrcSdkV1::executeAction(uint32_t no, uint16_t id, int16_t param0, int16_t param1, int16_t param2) {
    LOG(INFO) << "####### Action-> no " << no << " id " << (int)id << ", "
              << " p0 " << param0 << ", "
              << " p1 " << param1 << ", "
              << " p2 " << param2 << ", ";

    uint32_t cmd_param0 = no;
    uint32_t cmd_param1 = id;
    uint32_t cmd_param2 = (((uint32_t)param0) << 16) + param1;
    uint32_t cmd_param3 = param2;

    sendCommandMsgDo(COMMAND_EXECUTE_ACTION, cmd_param0, cmd_param1, cmd_param2, cmd_param3);
}

void SrcSdkV1::sendCommandMsgDo(COMMAND_t command, int32_t param0, int32_t param1, int32_t param2, int32_t param3) {
    LOG(INFO) << "@@@@@@@ COMMAND: command_id: " << command << ", p0 = " << param0 << ", p1 = " << param1
              << ", p2 = " << param2 << ", seq = " << command_msg_seq_no_;

    src::CommandMsg_ptr m = std::make_shared<src::CommandMsg>();
    m->setCommand(command);
    m->setParam0(param0);
    m->setParam1(param1);
    m->setParam2(param2);
    m->setParam3(param3);

    m->setSeqNO(command_msg_seq_no_);

    src_sdk->sendMsg(m);

    command_msg_seq_no_ += 1;  // 自增序列号
}

/**
 * 将SROS提供的NavigationPath转换为PATH_t,并发送给SRC
 */
void SrcSdkV1::setPaths(sros::core::NavigationPathi_vector paths) {
    std::vector<PATH_t> src_paths;
    PATH_t p;
    uint8_t no = 1;

    if (paths.size() >= 128) {
        LOG(ERROR) << "SRC::setPaths(): path size is > 128!";
        return;
    }

    for (auto item : paths) {
        p.type = (uint8_t)item.type_;
        p.sx = (int32_t)item.sx_;  // 单位为mm
        p.sy = (int32_t)item.sy_;
        p.ex = (int32_t)item.ex_;
        p.ey = (int32_t)item.ey_;
        p.cx = (int32_t)item.cx_;
        p.cy = (int32_t)item.cy_;
        p.dx = (int32_t)item.dx_;
        p.dy = (int32_t)item.dy_;
        p.angle = (int32_t)(item.rotate_angle_ * 10);  // 角度值单位为 1/10000 rad
        p.radius = (int32_t)item.radius_;

        p.max_v = (int16_t)item.limit_v_;
        p.mav_w = (int16_t)item.limit_w_;

        p.direction = (uint8_t)item.direction_;

        LOG(INFO) << "=> p.direction = " << (int)p.direction << ", p.max_v = " << p.max_v << ", p.max_w = " << p.mav_w;

        p.no = (uint8_t)no;
        p.over = OVER_NORMAL;

        src_paths.push_back(p);

        if (no % PathMsg::MAX_PATH_NUM == 0 || no == paths.size()) {
            if (no == paths.size()) {  // 最后一条路径
                src_paths[src_paths.size() - 1].over = OVER_LAST;
            }

            PathMsg_ptr m = std::make_shared<PathMsg>();
            m->setPaths(src_paths);
            m->encode();

            // connection_->sendData(m->rawData());
            src_sdk->sendMsg(m->rawData());

            src_paths.clear();

            boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
        }

        no++;
    }
}

// v 单位 mm/s , w 单位 1/1000 rad/s
void sdk::SrcSdkV1::setVelocity(int v, int w) {
    if (v > 2000 || w > M_PI * 1000)  // 限制最大速度与最大角速度
        return;

    src_sdk->sendCommandMsg(COMMAND_SET_VELOCITY, v, w * 10);  // SRC角度值单位为 1/10000 rad
}

void SrcSdkV1::sendVelocityBack(sros::core::Velocity velocity) {
    VelocityMsg_ptr m = std::make_shared<VelocityMsg>();
    m->setVx(FLOAT2INT32_M(velocity.vx()));
    m->setVy(FLOAT2INT32_M(velocity.vy()));
    m->setVtheta(FLOAT2INT32_RAD(velocity.vtheta()));

    src_sdk->sendMsg(m);
}

void sdk::SrcSdkV1::sendPoseBack(sros::core::Pose pose) {
    auto cur_timestamp = sros::core::util::get_time_in_ms();
    cur_pose_ = pose;  // 选择融合后的pose作为自身pose

    PoseMsg_ptr m = std::make_shared<PoseMsg>();
    m->setX(FLOAT2INT32_M(pose.x()));
    m->setY(FLOAT2INT32_M(pose.y()));
    m->setYaw((int16_t)FLOAT2INT32_RAD(pose.yaw()));  // 角度值单位为 1/10000 rad

    m->setZ(is_pose_available_ ? 1 : 0);  // 融合pose可用时z=1,否则z=0

    src_sdk->sendMsg(m);
    pose_send_back_time_plot_.shooting(sros::core::util::get_time_in_ms() - cur_timestamp);
}

void sdk::SrcSdkV1::handleOptPoseMsg(const std::vector<uint8_t> &data) {
    OptPoseMsg_ptr msg = make_shared<OptPoseMsg>();
    msg->rawData(data);
    msg->decode();

    Pose p(Location(msg->getX() / CONST_FLOAT_1000, msg->getY() / CONST_FLOAT_1000, msg->getZ() / CONST_FLOAT_1000),
           Rotation(msg->getYaw() / 10000.0, msg->getPitch() / 10000.0, msg->getRoll() / 10000.0));
    p.timestamp() = msg->getTimestamp();

    if (opt_pose_callback_f_) {
        opt_pose_callback_f_(p);
    }
}

void SrcSdkV1::handleStateMsg(const std::vector<uint8_t> &data) {
    StateMsg_ptr msg = make_shared<StateMsg>();
    msg->rawData(data);
    msg->decode();

    state_ = msg->getSRCState();

    {  // 括号不要去掉
       //        std::lock_guard<std::recursive_mutex> lg(pause_mutex);
       //        if (sros_pause_ && state_.src_state != STATE_PATH_PAUSED) {
       //            LOG(ERROR) << "SROS paused, but SRC not pause!!! src in " << (int)state_.src_state
       //                       << " state, try pause again!!!";
       //            pauseMovement(0);
       //        }
    }

    if (state_callback_f_ && src_sdk->isInitProtoOk()) {
        state_callback_f_(state_);
    }

    if (g_src_state.src_state != STATE_PATH_OUT && state_.src_state == STATE_PATH_OUT) {
//        LOG(WARNING) << "****** SRC PATH OUT !";  // 取消创建地图，选择不保存就会必现
//        LOG(WARNING) << numberListToStr(data.begin(), data.end());
    }

    g_src_state.src_state = state_.src_state;
    g_src_state.src_state_error_reason = state_.error_reason;
    if (g_src_state.cur_path_no != state_.path_no) {
//        LOG(INFO) << "current path no change: " << (int)g_src_state.cur_path_no << " -> " << (int)state_.path_no;
        std::ostringstream oss;
        oss << "current path no change: " << (int)g_src_state.cur_path_no << " -> " << (int)state_.path_no;
        logSendMonitor(oss);
        g_src_state.cur_path_no = state_.path_no;
    }
    if (g_src_state.cur_checkpoint != state_.reserved_2) {
//        LOG(INFO) << "checkpoint change: " << (int)g_src_state.cur_checkpoint << " -> " << (int)state_.reserved_2;
        std::ostringstream oss;
        oss << "checkpoint change: " << (int)g_src_state.cur_checkpoint << " -> " << (int)state_.reserved_2;
        logSendMonitor(oss);
        g_src_state.cur_checkpoint = state_.reserved_2;
    }
    static uint8_t reserved_3_old = 0;
    if (reserved_3_old != state_.reserved_3) {
//        LOG(INFO) << "reserved_3 change: " << (int)reserved_3_old << " -> " << (int)state_.reserved_3;
        std::ostringstream oss;
        oss << "reserved_3 change: " << (int)reserved_3_old << " -> " << (int)state_.reserved_3;
        logSendMonitor(oss);
        reserved_3_old = state_.reserved_3;
    }
    if (src_version_ >= 4004000) {  // src版本需要大于等于v4.4.0
        g_src_state.is_waitting_checkpoint = (bool)state_.reserved_3;
    }
    g_src_state.cur_remain_distance = state_.path_remain_distance;
    g_src_state.cur_remain_time = state_.path_remain_time;
    g_src_state.cur_total_distance = state_.path_total_distance;
    g_src_state.cur_v = state_.v_x;
    g_src_state.cur_w = state_.w;  // 此处角度值单位为 1/1000 rad

    g_src_state.movement_state = state_.movement_state;

    if (g_state.action_unit == sros::core::ACTION_UNIT_SRC) {
        if (g_state.multi_load_state != state_.reserved_0) {
            g_state.multi_load_state = state_.reserved_0;
//            LOG(INFO) << "multi_load_state: " << g_state.multi_load_state;
        }
        sros::core::LoadState new_load_state = state_.reserved_0 == 0 ? sros::core::LOAD_FREE : sros::core::LOAD_FULL;
        if (g_state.load_state != new_load_state) {
            g_state.load_state = new_load_state;

            if (new_load_state == sros::core::LOAD_FULL) {
//                LOG(INFO) << "Load state changed: LOAD_FREE -> LOAD_FULL";
                std::ostringstream oss;
                oss << "Load state changed: LOAD_FREE -> LOAD_FULL";
                logSendMonitor(oss);
            } else {
//                LOG(INFO) << "Load state changed: LOAD_FULL -> LOAD_FREE";
                std::ostringstream oss;
                oss << "Load state changed: LOAD_FULL -> LOAD_FREE";
                logSendMonitor(oss);
            }
        }
    }

    //    LOG_EVERY_N(INFO, 10) << "****** SRCState "
    //              << "src_state " << (int) state_.src_state << ", "
    //              << "movement_state " << (int) state_.movement_state << ", "
    //              << "error_reason " << (int) state_.error_reason << ", "
    //              << "reserved_0 " << (int) state_.reserved_0 << ", "
    //              << "reserved_1 " << (int) state_.reserved_1 << ", "
    //              << "reserved_2 " << (int) state_.reserved_2 << ", "
    //              << "reserved_3 " << (int) state_.reserved_3 << ", "
    //            ;
}

void SrcSdkV1::handleVelocityMsg(const std::vector<uint8_t> &data) {
    //            LOG(INFO) << "[sdk] got MSG_STATE";
    VelocityMsg_ptr msg = make_shared<VelocityMsg>();
    msg->rawData(data);
    msg->decode();

    if (velocity_callback_f_) {
        velocity_callback_f_(sros::core::Velocity(msg->getVx() / 1000.0, 0, msg->getVtheta() / 10000.0));
    }
}

void SrcSdkV1::handleSignalMsg(const std::vector<uint8_t> &data) {
    auto msg = make_shared<SignalMsg>();
    msg->rawData(data);
    msg->decode();

    auto new_signal = msg->getSignal();

    static int old_signal = 0;

//    LOG_IF(INFO, old_signal != (int)new_signal) << "handle src MSG_SIGNAL Msg！ signal_: " << (int)new_signal;

    if (old_signal != (int)new_signal) {
        std::ostringstream oss;
        oss << "handle src MSG_SIGNAL Msg！ signal_: " << (int)new_signal;
        logSendMonitor(oss);
    }

    old_signal = (int)new_signal;

    if (new_signal == SIGNAL_PATH_FINISHED && path_finish_callback_f_) {
        path_finish_callback_f_();
        std::ostringstream oss;
        oss << "==============> onSRCPathFinish";
        logSendMonitor(oss);
    } else if (new_signal == SIGNAL_PATH_ABORTED && path_aborted_callback_f_) {
//        path_aborted_callback_f_();
        std::ostringstream oss;
        oss << "==============> onSRCPathAborted";
        logSendMonitor(oss);
    } else if (new_signal == SIGNAL_PATH_PAUSED && path_paused_callback_f_) {
//        path_paused_callback_f_();
        std::ostringstream oss;
        oss << "==============> onSRCPathPaused";
        logSendMonitor(oss);
    } else if (new_signal == SIGNAL_UPGRADE_SUCCESS && src_upgrade_callback_f_) {
        // NOTE: 老版本的src收到升级请求后会进入此函数
        src_upgrade_.setUpgradeResult(1);
        if (!src_upgrade_.isInUpgrade()) {
            src_upgrade_callback_f_(1);
        }
    } else if (new_signal == SIGNAL_UPGRADE_FAILED && src_upgrade_callback_f_) {
        src_upgrade_.setUpgradeResult(0);
        if (!src_upgrade_.isInUpgrade()) {
            src_upgrade_callback_f_(0);
        } else if (new_signal == SIGNAL_POSE_TIMEOUT && pose_timeout_callback_f_) {
            pose_timeout_callback_f_();
            std::ostringstream oss;
            oss << "SRC pose timeout, force update pause state!";
            logSendMonitor(oss);
        }
    } else if (new_signal == SIGNAL_MOTOR_SLIP) {
        sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_MOTOR_SLIP);
    } else if (new_signal == SIGNAL_COUPLING_BREAK) {
        sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_COUPLING_BREAK);
    }
}

void SrcSdkV1::handleInfoMsg(const std::vector<uint8_t> &data) {
    auto msg = make_shared<InfoMsg>();
    msg->rawData(data);
    msg->decode();

    if (src_version_ != msg->version_no) {
        src_version_ = msg->version_no;
        src_version_str_ = msg->versionStr();
        LOGGER(INFO, SROS) << "SRC version: " << src_version_str_;
    }

    g_src_state.total_mileage = msg->total_mileage;
    g_src_state.total_power_cycle = msg->total_power_cycle;
    g_src_state.total_poweron_time = msg->total_poweron_time;

    g_src_state.m1_status = msg->m1_status;
    g_src_state.m1_status_code = msg->m1_status_code;
    g_src_state.m1_mileage = msg->m1_mileage;

    g_src_state.m2_status = msg->m2_status;
    g_src_state.m2_status_code = msg->m2_status_code;
    g_src_state.m2_mileage = msg->m2_mileage;

    g_src_state.m3_status = msg->m3_status;
    g_src_state.m3_status_code = msg->m3_status_code;
    g_src_state.m3_mileage = msg->m3_mileage;

    g_src_state.m4_status = msg->m4_status;
    g_src_state.m4_status_code = msg->m4_status_code;
    g_src_state.m4_mileage = msg->m4_mileage;

    g_src_state.imu_status = msg->device0_status;
    g_src_state.pgv1_status = msg->device1_status;
    g_src_state.pgv2_status = msg->device2_status;
    g_src_state.manual_controller_status = msg->device3_status;

    g_src_state.reserved_field = msg->reserved_field;

    updateDeviceState(DEVICE_MOTOR_1, msg->m1_status, msg->m1_status_code);
    updateDeviceState(DEVICE_MOTOR_2, msg->m2_status, msg->m2_status_code);
    updateDeviceState(DEVICE_MOTOR_3, msg->m3_status, msg->m3_status_code);
    updateDeviceState(DEVICE_MOTOR_4, msg->m4_status, msg->m4_status_code);

    updateDeviceState(DEVICE_IMU, msg->device0_status);
    updateDeviceState(DEVICE_PGV_DOWN, msg->device1_status);
    updateDeviceState(DEVICE_PGV_UP, msg->device2_status);

    sros::device::VirtualDevice_ptr src_device = src_sdk->getSrcVirtualDevicePtr();
    if (src_device) {
        src_device->setStateOK();
        src_device->setVersionNo(src_version_str_);
    }
}

void sdk::SrcSdkV1::handlePoseMsg(const std::vector<uint8_t> &data) {
    const uint64_t MAX_POSE_TIMESTAMP_THRESHOLD = 30;        // 30ms
    int64_t sync_time = sros::core::util::get_time_in_us();  //获取到里程计时,本地的同步时间戳
    PoseMsg_ptr msg = make_shared<PoseMsg>();
    msg->rawData(data);
    msg->decode();

    // static uint32_t last_src_timestamp = 0; // 从src上传的pose时间戳
    // LOG_EVERY_N(INFO, 50) << "=> Pose " << msg->getTimestamp() << " : " << msg->getX() << ", " << msg->getY();

    auto cur_timestamp = sros::core::util::get_time_in_ms();
    static DistributionPlot src_pose_interval_plot("SRC上监控两次Pose间隔(ms)", 1);
    static DistributionPlot receive_pose_sros_plot("两次收到SRC上传Pose的sros时间间隔(ms)", 1);
    // static DistributionPlot receive_pose_src_plot("两次收到SRC上传Pose的timestamp间隔", 10);
    static DistributionPlot algorithm_handle_time_plot("算法融合pose和回传pose的时间间隔之和(ms)", 1);
    static DistributionPlot last_src_pose_serial_plot("SRC上传pose序列号间隔", 1);

    auto cur_src_pose_serial = msg->getTimestamp();
    if (last_src_pose_serial_ != 0) {
        last_src_pose_serial_plot.shooting(cur_src_pose_serial - last_src_pose_serial_);
    }
    last_src_pose_serial_ = cur_src_pose_serial;

    if (last_pose_timestamp_ != 0) {
        receive_pose_sros_plot.shooting(cur_timestamp - last_pose_timestamp_);
        // receive_pose_src_plot.shooting(msg->getTimestamp() - last_src_timestamp);
        src_pose_interval_plot.shooting(msg->getRoll());
    }

    // if (cur_timestamp - last_pose_timestamp_ > MAX_POSE_TIMESTAMP_THRESHOLD) {
    //     LOG(WARNING) << "=> Pose timeout: " << cur_timestamp - last_pose_timestamp_ << "ms";
    // }

    last_pose_timestamp_ = cur_timestamp;
    // last_src_timestamp = msg->getTimestamp();

    Pose p(Location(msg->getX() / CONST_FLOAT_1000, msg->getY() / CONST_FLOAT_1000, msg->getZ() / CONST_FLOAT_1000),
           Rotation(msg->getYaw() / CONST_FLOAT_10000, msg->getPitch() / CONST_FLOAT_10000, 0));
    p.timestamp() = msg->getTimestamp();
    p.synctimestamp() = sync_time;
    if (pose_callback_f_) {
        pose_callback_f_(p);
        algorithm_handle_time_plot.shooting(sros::core::util::get_time_in_ms() - cur_timestamp);
    }

//    if (receive_pose_sros_plot.shoot_count() % (6000 * 5) == 0) {
//        LOG(INFO) << src_pose_interval_plot;
//        LOG(INFO) << receive_pose_sros_plot;
//        LOG(INFO) << last_src_pose_serial_plot;
//        // LOG(INFO) << receive_pose_src_plot;
//        LOG(INFO) << algorithm_handle_time_plot;
//        LOG(INFO) << pose_send_back_time_plot_;
//}
}

void SrcSdkV1::handleActionStateMsg(const std::vector<uint8_t> &data) {
    // LOG(INFO) << "[sdk] got MSG_ACTION_STATE";
    auto msg = make_shared<ActionStateMsg>();
    msg->rawData(data);
    msg->decode();

    action_state_ = msg->getSRCActionState();

    sros::core::TaskStateStruct task_state_struct = srcActionStateToTaskStateStruct(action_state_);

    if (action_state_callback_f_) {
        action_state_callback_f_(task_state_struct);
    }

    //    LOG(INFO) << "%%%%%%% ActionState -> "
    //              << "no " << action_state_.action_no << ", "
    //              << "result " << action_state_.action_result << ", "
    //              << "result_value " << action_state_.action_result_value << ", "
    //              << "reserved_0 " << action_state_.action_reserved_0 << ", "
    //              << "reserved_1 " << action_state_.action_reserved_1;
}

void SrcSdkV1::handleMonitorStateMsg(const std::vector<uint8_t> &data) {
    //    LOG(INFO) << "[sdk] got MSG_MONITOR_STATE";
    auto msg = make_shared<MonitorStateMsg>();
    msg->rawData(data);
    msg->decode();

    auto monitor_state = msg->getSRCMonitorState();

    if (monitor_state_callback_f_) {
        monitor_state_callback_f_(monitor_state);
    }
}

void SrcSdkV1::handleCommandMsg(const std::vector<uint8_t> &data) {
    auto msg = make_shared<src::CommandMsg>();
    msg->rawData(data);
    msg->decode();

    // TODO 记录command到日志中，用于判断src是否收到指令
//    LOG(INFO) << "$$$$$$$ COMMAND_ACK: command_id: " << msg->getCommand() << ", "
//              << "p0 = " << msg->getParam0() << ", "
//              << "p1 = " << msg->getParam1() << ", "
//              << "p2 = " << msg->getParam2() << ", "
//              << "seq = " << msg->getSeqNO();
}

void SrcSdkV1::handleWWDGMsg(const vector<uint8_t> &data) {
    uint16_t data_0 = (data[1] << 8) + data[2];
    uint16_t data_1 = (data[3] << 8) + data[4];
    uint16_t main_freq = (data[5] << 8) + data[6];
    uint16_t core_fault_status = (data[7] << 8) + data[8];
    uint16_t last_src_position = (data[9] << 8) + data[10];

//    LOGGER(ERROR, DEVICE) << "WWDG: src crash!!! "
//                          << " data0:0x" << std::setfill('0') << std::setw(4) << std::hex << data_0 << " data1:0x"
//                          << std::setfill('0') << std::setw(4) << std::hex << data_1 << " main_freq:0x"
//                          << std::setfill('0') << std::setw(4) << std::hex << main_freq << " core_fault_status:0x"
//                          << std::setfill('0') << std::setw(4) << std::hex << core_fault_status
//                          << " last_src_position:0x" << std::setfill('0') << std::setw(4) << std::hex
//                          << last_src_position;

    std::ostringstream oss;
    oss << "WWDG: src crash!!! "
        << " data0:0x" << std::setfill('0') << std::setw(4) << std::hex << data_0 << " data1:0x"
        << std::setfill('0') << std::setw(4) << std::hex << data_1 << " main_freq:0x"
        << std::setfill('0') << std::setw(4) << std::hex << main_freq << " core_fault_status:0x"
        << std::setfill('0') << std::setw(4) << std::hex << core_fault_status
        << " last_src_position:0x" << std::setfill('0') << std::setw(4) << std::hex
        << last_src_position;
    logSendMonitor(oss);

}

sros::core::TaskStateStruct SrcSdkV1::srcActionStateToTaskStateStruct(const SRCActionState &action_state) const {
    sros::core::TaskStateStruct task_state_struct;

    task_state_struct.no_ = action_state.action_no;
    switch (action_state.action_result) {
        case SRC_ACTION_RESULT_RUNNING: {
            task_state_struct.state_ = TASK_RUNNING;
            break;
        }
        case SRC_ACTION_RESULT_OK: {
            task_state_struct.state_ = TASK_FINISHED;
            task_state_struct.result_ = TASK_RESULT_OK;
            task_state_struct.result_value_ = action_state.action_result_value;
            break;
        }
        case SRC_ACTION_RESULT_FAILED: {
            task_state_struct.state_ = TASK_FINISHED;
            task_state_struct.result_ = TASK_RESULT_FAILED;
            task_state_struct.result_value_ = action_state.action_result_value;
            break;
        }
        case SRC_ACTION_RESULT_CANCELED: {
            task_state_struct.state_ = TASK_FINISHED;
            task_state_struct.result_ = TASK_RESULT_CANCELED;
            task_state_struct.result_value_ = action_state.action_result_value;
            break;
        }
        case SRC_ACTION_RESULT_PAUSED: {
            task_state_struct.state_ = TASK_PAUSED;
            task_state_struct.result_value_ = action_state.action_result_value;
            break;
        }
        default: {
//            LOG(ERROR) << "unsupport action_state.action_result: " << action_state.action_result;
            break;
        }
    }

    return task_state_struct;
}
void SrcSdkV1::initMotorSerialNo() {
    const uint16_t ADDR_MOTOR_1_SERIAL_NO = 0x1100;
    const uint16_t ADDR_MOTOR_1_DRIVER_SERIAL_NO = 0x1108;
    const uint16_t ADDR_MOTOR_2_SERIAL_NO = 0x1120;
    const uint16_t ADDR_MOTOR_2_DRIVER_SERIAL_NO = 0x1128;
    const uint16_t ADDR_MOTOR_3_SERIAL_NO = 0x1140;
    const uint16_t ADDR_MOTOR_3_DRIVER_SERIAL_NO = 0x1148;
    const uint16_t ADDR_MOTOR_4_SERIAL_NO = 0x1160;
    const uint16_t ADDR_MOTOR_4_DRIVER_SERIAL_NO = 0x1168;

    const auto SERIAL_STR_LEN = 32;

    auto m1_serial_no = src_sdk->getParametersString(ADDR_MOTOR_1_SERIAL_NO, SERIAL_STR_LEN);
    auto m1_driver_serial_no = src_sdk->getParametersString(ADDR_MOTOR_1_DRIVER_SERIAL_NO, SERIAL_STR_LEN);
    updateDeviceSerialNo(DEVICE_MOTOR_1, m1_serial_no + "/" + m1_driver_serial_no);

    auto m2_serial_no = src_sdk->getParametersString(ADDR_MOTOR_2_SERIAL_NO, SERIAL_STR_LEN);
    auto m2_driver_serial_no = src_sdk->getParametersString(ADDR_MOTOR_2_DRIVER_SERIAL_NO, SERIAL_STR_LEN);
    updateDeviceSerialNo(DEVICE_MOTOR_2, m2_serial_no + "/" + m2_driver_serial_no);

    auto m3_serial_no = src_sdk->getParametersString(ADDR_MOTOR_3_SERIAL_NO, SERIAL_STR_LEN);
    auto m3_driver_serial_no = src_sdk->getParametersString(ADDR_MOTOR_3_DRIVER_SERIAL_NO, SERIAL_STR_LEN);
    updateDeviceSerialNo(DEVICE_MOTOR_3, m3_serial_no + "/" + m3_driver_serial_no);

    auto m4_serial_no = src_sdk->getParametersString(ADDR_MOTOR_4_SERIAL_NO, SERIAL_STR_LEN);
    auto m4_driver_serial_no = src_sdk->getParametersString(ADDR_MOTOR_4_DRIVER_SERIAL_NO, SERIAL_STR_LEN);
    updateDeviceSerialNo(DEVICE_MOTOR_4, m4_serial_no + "/" + m4_driver_serial_no);
}

bool SrcSdkV1::setUpCameraDetectInfo(int valid, int x, int y, int yaw, int dm_code_info) {

    if(g_state.is_forbid_send_up_svc100_to_src) {
        return false;
    }

    if (valid) {
//        LOG(INFO) << "setUpCameraDetectInfo valid: " << valid << ", " << dm_code_info << "(" << x << ", " << y << ", "
//                  << yaw / 10 << ")";
    }
    return src_sdk->setParameters(0x1240, {valid, x, y, yaw / 10, dm_code_info});
}

bool SrcSdkV1::setDownCameraDetectInfo(int valid, int x, int y, int yaw, int dm_code_info) {
    if(g_state.is_forbid_send_down_svc100_to_src) {
        return false;
    }

    if (valid) {
//        LOG(INFO) << "setDownCameraDetectInfo valid: " << valid << ", " << dm_code_info << "(" << x << ", " << y << ", "
//                  << yaw << ")";
    }
    return src_sdk->setParameters(0x1250, {valid, x, y, yaw, dm_code_info});
}

bool SrcSdkV1::setTargetDMCodeOffset(int pgv_id, double x, double y, double yaw) {
    const uint16_t ADDR_PGV_EXEC_INFO = 0x1224;

    std::vector<int> values{pgv_id, (int)(10000 * x), (int)(10000 * y), (int)(10000 * yaw)};
//    LOG(INFO) << "Target DMCode offset: " << numberListToStr(values.cbegin(), values.cend());
    return src_sdk->setParameters(ADDR_PGV_EXEC_INFO, values);
}

bool SrcSdkV1::setTractorDockingDMCodeOffset(double x, double y, double yaw, double deviate) {
    const uint16_t ADDR_TRACTOR_DOCKING = 0x1700;

    std::vector<int> values{(int)(10000 * x), (int)(10000 * y), (int)(10000 * yaw), (int)(10000 * deviate)};
//    LOG_EVERY_N(INFO, 100) << "Tractor Docking DMCode offset: " << numberListToStr(values.cbegin(), values.cend());
    return src_sdk->setParameters(ADDR_TRACTOR_DOCKING, values);
}

bool SrcSdkV1::setPgvExecInfo(double x, double y, double yaw, int enable) {
    const uint16_t ADDR_PGV_EXEC_INFO = 0x1228;
    int pose_x = int(x * 1000);
    int pose_y = int(y * 1000);
    int pose_yaw = int(yaw / 3.1415f * 1800);

    // src v4.7.7版本修改接收四个寄存器参数
    if (src_version_ >= 4007007) {
        std::vector<int> values{pose_x, pose_y, pose_yaw, enable};
        return src_sdk->setParameters(ADDR_PGV_EXEC_INFO, values);
    } else {
        std::vector<int> values{pose_x, pose_y, pose_yaw};
        return src_sdk->setParameters(ADDR_PGV_EXEC_INFO, values);
    }
}

std::vector<int> SrcSdkV1::getSrcDebugInfo() {
    const uint16_t ADDR_MONITOR_SRC_STATUS = 0x1540;
    const int register_count = 10;  // 寄存器个数

    auto results = src_sdk->getParameters(ADDR_MONITOR_SRC_STATUS, register_count, 50, true);
    if (results.size() != register_count) {
        if (!results.empty()) {
            LOG(ERROR) << "getParameters 返回的数据有问题！！！ size: " << results.size();
            for (auto ret : results) {
                LOG(ERROR) << ret;
            }
        }
        LOG(WARNING) << "获取调试信息失败！";
        return std::vector<int>(register_count, 0);
    }

    static std::vector<int> src_state(register_count, 0);
    //auto enable_src_debug = (s.getValue<std::string>("debug.enable_src_debug", "False") == "True");
    //if (enable_src_debug) {
    bool is_same = true;
    auto it_old = src_state.cbegin();
    auto it_new = results.cbegin();
    while (it_old != src_state.cend()) {
        if (*it_old != *it_new) {
            is_same = false;
            LOG(INFO) << "SRC the " << std::distance(src_state.cbegin(), it_old) << "th register changed! "
                      << *it_old << " => " << *it_new;
        }
        ++it_old;
        ++it_new;
    }
    if (!is_same) {
        LOG(INFO) << "old: " << numberListToStr(src_state.cbegin(), src_state.cend());
        LOG(INFO) << "new: " << numberListToStr(results.cbegin(), results.cend());
        src_state = results;
    }

    return results;
}

void SrcSdkV1::updateActionControlDebugInfo() {
    const uint16_t SRC_ADDR_ACTION_DEBUG_INFO = 0x5000;
    const int register_count = 4;  // 寄存器个数

    auto data = src_sdk->getParameters(SRC_ADDR_ACTION_DEBUG_INFO, register_count);
    if (data.empty()) {
//        LOG(ERROR) << "读到数据为空！！！";
        return;
    }

    // LOG(INFO) << "src_actc_debug_info: "<<numberListToStr(data.cbegin(), data.cend());

    std::vector<uint16_t> data_16(data.size());
    for (int i = 0; i < data.size(); ++i) {
        data_16[i] = data[i] & 0xFFFF;
    }
    // LOG(INFO) << "write src_actc_debug_info: "<<numberListToStr(data_16.cbegin(), data_16.cend());

    RegisterAdmin::getInstance()->writeRelativeInputRegisters(sros::SRC_ACTION_DEBUG_INFO_1, data_16);
}

void SrcSdkV1::update1353IO() {
    const uint16_t ADDR_1353 = 0x1451;
    int io_1353 = 0;
    if(!src_sdk->getParameter(ADDR_1353, io_1353)){
        LOG(WARNING) << "get1353IO timeout!";
    }    
    if (g_state.io_1353 != io_1353) {
        LOG(INFO) << "io_1353 changed: 0x" << std::hex << g_state.io_1353 << " -> 0x" << std::hex << io_1353;
        g_state.io_1353 = io_1353;
    }
}

void SrcSdkV1::updateForkHeight() {

    const uint16_t ADDR_FORK_HEIGHT_ENCODER = 0x1411;

    auto results = src_sdk->getParameters(ADDR_FORK_HEIGHT_ENCODER, 1);
    if (results.size() != 1) {
        if (!results.empty()) {
            LOG(ERROR) << "getParameters 返回的数据有问题！！！ size: " << results.size();
            for (auto ret : results) {
                LOG(ERROR) << ret;
            }
        }
        return;
    }

    g_state.fork_height_encoder = results.at(0) / 1000.0;
}

void SrcSdkV1::updateControlMode() {

    auto results = src_sdk->getParameters(SRTOS_ADDR_CONTROL_TYPE, 1);
    if (results.size() != 1) {
        if (!results.empty()) {
            LOG(ERROR) << "getControlMode 返回的数据有问题！！！ size: " << results.size();
            for (auto ret : results) {
                LOG(ERROR) << ret;
            }
        }
        return;
    }

    ManualBtnState mode = (enum ManualBtnState)(results.at(0));
    src_sdk->handlManualSignal(mode);
}

void SrcSdkV1::subRecvUartDataHandle(const std::vector<uint8_t> &data) {
    uint8_t type = data[0];

    switch (type) {
        case MSG_STATE: {
            handleStateMsg(data);
            break;
        }
        case MSG_INFO: {
            handleInfoMsg(data);
            break;
        }
        case MSG_ACTION_STATE: {
            handleActionStateMsg(data);
            break;
        }
        case MSG_MONITOR_STATE: {
            handleMonitorStateMsg(data);
            break;
        }
        case MSG_COMMAND: {
            handleCommandMsg(data);
            break;
        }
        case MSG_WWDG: {
            handleWWDGMsg(data);
            break;
        }
        default:
            break;
    }
}

}  // namespace sdk
