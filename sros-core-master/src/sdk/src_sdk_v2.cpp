//
// Created by caoyan on 2020-11-12
//

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/sem.h>
#include <sys/time.h>
#include <unistd.h>
#include <sys/utsname.h>
#include "src_sdk_v2.h"
#include "core/device/device_manager.h"
#include "core/device/virtual_device.hpp"
#include "core/modbus/register_admin.h"
#include "core/settings.h"
#include "core/util/utils.h"
#include "src/sdk/protocol/src_protocol.h"
#include "core/src.h"
#include "src/sdk/protocol/sros_state_msg.hpp"
#include "src/sdk/protocol/parameter_msg.hpp"
#include "core/fault_center.h"

using namespace src;
using namespace sros::core;
using namespace sros::device;

namespace sdk {

SrcSdkV2::SrcSdkV2():SrcSdkDo() {}

SrcSdkV2::~SrcSdkV2() {}

void SrcSdkV2::init() {
    auto &s = sros::core::Settings::getInstance();

	bool enableSrtosLog = (s.getValue<std::string>("debug.enable_srtos_log", "True") == "True");
	if(enableSrtosLog) {
		if(access("/dev/spidev0.0", F_OK)) {
			LOG(WARNING) << "WARNING: CPU kernel don't support srtos log translate.";
		} else {
            struct utsname kernel_info;
            if(uname(&kernel_info) == 0) {
                std::string version = kernel_info.release;
                std::string curVer = version.substr(0,6);
                std::string maxVer = "4.14.0";
                if(curVer.compare(maxVer) < 0) {
                    LOG(WARNING) << "WARNING: CPU kernel version too old, don't support srtos log translate - " << curVer;
                }
                LOG(INFO) << "!!!!!spi log ready. kernel version: " << curVer;
            }

            if(access("/sros/tool/srtos_log", F_OK)) {
                LOG(WARNING) << "WARNING: /sros/tool/srtos_log no exist.";
            } else {
                system("/sros/tool/srtos_log &");
                spi_log_ = std::make_shared<sdkSrtos::SrcSdkSpiLog>();
                boost::thread(boost::bind(&SrcSdkV2::handler_src_log, this));
                LOG(INFO) << "!!!!!SRTOS log enable";
            }           
        }
	} else {
		LOG(INFO) << "!!!!!SRTOS log close";
	}

    if(!setSrcConfig()){
        LOG(ERROR) << "setSrcConfig failed.";
    }
    src_sdk->sendCommandMsg(COMMAND_LAUNCH);  // 初始化成功

    // 轮询SRC寄存器状态，并同步到SROS端
    boost::thread(boost::bind(&SrcSdkV2::syncState, this));

    // 处理SROS实时下发心跳给SRC
    SECURITY_UNIT_TYPE security_unit_type = (SECURITY_UNIT_TYPE)(s.getValue<int>("main.security_unit_type", 1));
    if (security_unit_type == SECURITY_UNIT_TYPE_SRC) {
        boost::thread(boost::bind(&SrcSdkV2::dealSrcHeartBeat, this));
    }

    // 处理SRC实时上报的急停触发源
    boost::thread(boost::bind(&SrcSdkV2::setNewStateByNotify, this));
}

void SrcSdkV2::handler_src_log() {
    while(1) {
        spi_log_->srtos_log_loop();
        usleep(50000);
    }
}

// v 单位 mm/s , w 单位 1/1000 rad/s
void sdk::SrcSdkV2::setVelocity(int v, int w) {
    if (v > 2000 || w > M_PI * 1000)  // 限制最大速度与最大角速度
        return;

    src_sdk->sendCommandMsg(COMMAND_SET_VELOCITY, v * CONST_INT_10, w * 10);  // SRC角度值单位为 1/10000 rad
}

void SrcSdkV2::sendCommandMsgDo(COMMAND_t command, int32_t param0, int32_t param1, int32_t param2, int32_t param3) {
    return sendCommandMsgDo(command, param0, param1, param2, param3, 0);
}

void SrcSdkV2::sendCommandMsgDo(COMMAND_t command, int32_t param0, int32_t param1, int32_t param2, int32_t param3, int32_t param4) {

    if (state_.src_system_state == SYS_INVALID || state_.src_system_state == SYS_INITING) {
        if (command != COMMAND_CONNECT && command != COMMAND_LAUNCH && command != COMMAND_UPGRADE &&
            command != COMMAND_UPGRADE_TEST && command != COMMAND_CPU_RESET && command != COMMAND_SET_IO_OUPUT) {
            // 在初始化过程中，不允许发送
            return;
        }
    }

    LOG(INFO) << "@@@@@@@ COMMAND: command_id: 0x" << std::hex << command << ", p0 = " << param0 << ", p1 = " << param1
              << ", p2 = " << param2 << ", p3 = " << param3 << ", p4 = " << param4 << ", seq = " << command_msg_seq_no_;

    // 基于当前串口通讯机制在嵌套调用时逻辑冲突的问题，对触发急停和解除急停2个必现点进行处理（后续需重构串口通讯模块）
    std::vector<int> datas = {command, param0, param1, param2, param3, param4, (int)command_msg_seq_no_, 0};
    bool bEmergency = isEmergencyCmd(command);
    // if (bEmergency)
    // {
    //     // 非阻塞式，不等待结果返回
    //     src::ParameterMsg_ptr m = std::make_shared<src::ParameterMsg>();
    //     m->setValues(SRTOS_ADDR_CMD, datas);
    //     src_sdk->sendMsg(m);
    // }
    // else
    // {
    //     //阻塞式，等待结果返回
    src_sdk->setParameters(SRTOS_ADDR_CMD, datas);
    // }

    command_msg_seq_no_ += 1;  // 自增序列号
    return;
}

bool SrcSdkV2::isEmergencyCmd(COMMAND_t command)
{
    if (command == COMMAND_EMERGENCY_PAUSE ||
        command == COMMAND_EMERGENCY_CANCEL ||
        command == COMMAND_EMERGENCY_RECOVER ||
        command == COMMAND_TRIGGER_EMERGENCY ||
        command == COMMAND_PAUSE)
    {
        return true;
    }
    return false;
}

void SrcSdkV2::setPaths(sros::core::NavigationPathi_vector paths) {
    std::vector<PATH_t> src_paths;
    PATH_t p;
    uint8_t no = 1;

    if (paths.size() >= 128) {
        LOG(ERROR) << "SRC::setPaths(): path size is > 128!";
        return;
    }

    for (auto item : paths) {
        p.type = (uint8_t)item.type_;
        p.sx = (int32_t)INT_UNIT_TO_SRTOS_UNIT(item.sx_);  // 单位为mm
        p.sy = (int32_t)INT_UNIT_TO_SRTOS_UNIT(item.sy_);
        p.ex = (int32_t)INT_UNIT_TO_SRTOS_UNIT(item.ex_);
        p.ey = (int32_t)INT_UNIT_TO_SRTOS_UNIT(item.ey_);
        p.cx = (int32_t)INT_UNIT_TO_SRTOS_UNIT(item.cx_);
        p.cy = (int32_t)INT_UNIT_TO_SRTOS_UNIT(item.cy_);
        p.dx = (int32_t)INT_UNIT_TO_SRTOS_UNIT(item.dx_);
        p.dy = (int32_t)INT_UNIT_TO_SRTOS_UNIT(item.dy_);
        p.angle = (int32_t)INT_UNIT_TO_SRTOS_UNIT(item.rotate_angle_);  // 角度值单位为 1/10000 rad
        p.radius = (int32_t)INT_UNIT_TO_SRTOS_UNIT(item.radius_);

        p.max_v = (int16_t)INT_UNIT_TO_SRTOS_UNIT(item.limit_v_);
        p.mav_w = (int16_t)INT_UNIT_TO_SRTOS_UNIT(item.limit_w_);

        p.direction = (uint8_t)item.direction_;

        LOG(INFO) << "=> p.direction = " << (int)p.direction << ", p.max_v = " << p.max_v << ", p.max_w = " << p.mav_w;

        p.no = (uint8_t)no;
        p.over = OVER_NORMAL;

        src_paths.push_back(p);
        //PathMsg::MAX_PATH_NUM
        if (no % 1 == 0 || no == paths.size()) {
            if (no == paths.size()) {  // 最后一条路径
                src_paths[src_paths.size() - 1].over = OVER_LAST;
            }

            PathMsg_ptr m = std::make_shared<PathMsg>();
            m->setPaths(src_paths);
            m->encode();

            //connection_->sendData(m->rawData());
            src_sdk->sendMsg(m->rawData());
            src_paths.clear();

            //boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
            boost::this_thread::sleep_for(boost::chrono::milliseconds(3));
        }

        no++;
    }
}

// void SrcSdkV2::setPaths(sros::core::NavigationPathi_vector paths) {
//     std::vector<PATH_t> src_paths;
//     PATH_t p;
//     uint8_t no = 0;

//     if (paths.size() >= 128) {
//         LOG(ERROR) << "SRC::setPaths(): path size is > 128!";
//         return;
//     }

//     for (auto p : paths) {
//         no++;
//         LOG(INFO) << p;
//         std::vector<int> datas = {
//             INT_UNIT_TO_SRTOS_UNIT(p.sx_),
//             INT_UNIT_TO_SRTOS_UNIT(p.sy_),
//             INT_UNIT_TO_SRTOS_UNIT(p.ex_),
//             INT_UNIT_TO_SRTOS_UNIT(p.ey_),
//             INT_UNIT_TO_SRTOS_UNIT(p.cx_),
//             INT_UNIT_TO_SRTOS_UNIT(p.cy_),
//             INT_UNIT_TO_SRTOS_UNIT(p.dx_),
//             INT_UNIT_TO_SRTOS_UNIT(p.dy_),
//             (int32_t)INT_UNIT_TO_SRTOS_UNIT(p.radius_),  // FIXME(pengjiali): srtos 测试下p.radius_到底有没有乘以1000
//             (int32_t)INT_UNIT_TO_SRTOS_UNIT(p.rotate_angle_),
//             (int32_t)INT_UNIT_TO_SRTOS_UNIT(p.limit_v_),
//             (int32_t)INT_UNIT_TO_SRTOS_UNIT(p.limit_w_),
//             no,
//             p.type_,
//             no == paths.size() ? OVER_LAST : OVER_NORMAL,
//             p.direction_,
//             0,
//             0,
//             0,
//             0};
//         src_sdk->setParameters(SRTOS_ADDR_SET_PATHS, datas);
//     }
// }

bool SrcSdkV2::setUpCameraDetectInfo(int valid, int x, int y, int yaw, int dm_code_info) {
    return src_sdk->setParameters(SRTOS_ADDR_SVC_UP_STATUS, {valid, x, y, yaw, dm_code_info, (int)sros::core::util::get_time_in_ms(), 0, 0});
}

bool SrcSdkV2::setDownCameraDetectInfo(int valid, int x, int y, int yaw, int dm_code_info) {
    return src_sdk->setParameters(SRTOS_ADDR_SVC_DOWN_STATUS, {valid, x, y, yaw, dm_code_info, (int)sros::core::util::get_time_in_ms(), 0, 0});
}

void SrcSdkV2::sendVelocityBack(sros::core::Velocity velocity) {
    std::vector<int> datas = {0, FLOAT_UNIT_TO_SRTOS_UNIT(velocity.vx()), FLOAT_UNIT_TO_SRTOS_UNIT(velocity.vy()),
                              FLOAT_UNIT_TO_SRTOS_UNIT(velocity.vtheta())};

    VelocityMsg_ptr m = std::make_shared<VelocityMsg>();
    m->setVx(FLOAT2INT32_RAD(velocity.vx()));
    m->setVy(FLOAT2INT32_RAD(velocity.vy()));
    m->setVtheta(FLOAT2INT32_RAD(velocity.vtheta()));
    src_sdk->sendMsg(m);
}

void sdk::SrcSdkV2::sendPoseBack(sros::core::Pose pose) {
    auto cur_timestamp = sros::core::util::get_time_in_ms();
    cur_pose_ = pose;  // 选择融合后的pose作为自身pose

    PoseMsg_ptr m = std::make_shared<PoseMsg>();
    m->setX(FLOAT2INT32_RAD(pose.x()));
    m->setY(FLOAT2INT32_RAD(pose.y()));
    m->setYaw((int16_t)FLOAT2INT32_RAD(pose.yaw()));  // 角度值单位为 1/10000 rad

    m->setZ(is_pose_available_ ? 1 : 0);  // 融合pose可用时z=1,否则z=0

    src_sdk->sendMsg(m);
    pose_send_back_time_plot_.shooting(sros::core::util::get_time_in_ms() - cur_timestamp);
}

void sdk::SrcSdkV2::handleOptPoseMsg(const std::vector<uint8_t> &data) {
    OptPoseMsg_ptr msg = make_shared<OptPoseMsg>();
    msg->rawData(data);
    msg->decode();

    Pose p(Location(msg->getX() / CONST_FLOAT_10000, msg->getY() / CONST_FLOAT_10000, msg->getZ() / CONST_FLOAT_10000),
           Rotation(msg->getYaw() / 10000.0, msg->getPitch() / 10000.0, msg->getRoll() / 10000.0));
    p.timestamp() = msg->getTimestamp();

    if (opt_pose_callback_f_) {
        opt_pose_callback_f_(p);
    }
}

void sdk::SrcSdkV2::handleSignalMsg(const std::vector<uint8_t> &data) {
    auto msg = make_shared<SignalMsg>();
    msg->rawData(data);
    msg->decode();

    auto new_signal = msg->getSignal();

    static int old_signal = 0;

    LOG_IF(INFO, old_signal != (int)new_signal) << "handle src MSG_SIGNAL Msg！ signal_: " << (int)new_signal;

    old_signal = (int)new_signal;

    if (new_signal == SIGNAL_UPGRADE_SUCCESS && src_upgrade_callback_f_) {
        // NOTE: 老版本的src收到升级请求后会进入此函数
        src_upgrade_.setUpgradeResult(1);
        LOG(INFO) << src_upgrade_.isInUpgrade();
        if (!src_upgrade_.isInUpgrade()) {
            src_upgrade_callback_f_(1);
        }
    } else if (new_signal == SIGNAL_UPGRADE_FAILED && src_upgrade_callback_f_) {
        src_upgrade_.setUpgradeResult(0);
        LOG(INFO) << src_upgrade_.isInUpgrade();
        if (!src_upgrade_.isInUpgrade()) {
            src_upgrade_callback_f_(0);
        }
    }
}

void sdk::SrcSdkV2::handlePoseMsg(const std::vector<uint8_t> &data) {
    const uint64_t MAX_POSE_TIMESTAMP_THRESHOLD = 30;        // 30ms
    int64_t sync_time = sros::core::util::get_time_in_us();  //获取到里程计时,本地的同步时间戳
    PoseMsg_ptr msg = make_shared<PoseMsg>();
    msg->rawData(data);
    msg->decode();

    //static uint32_t last_src_timestamp = 0; // 从src上传的pose时间戳
    //LOG_EVERY_N(INFO, 50) << "=> Pose " << msg->getTimestamp() << " : " << msg->getX() << ", " << msg->getY();

    auto cur_timestamp = sros::core::util::get_time_in_ms();
    static DistributionPlot src_pose_interval_plot("SRC上监控两次Pose间隔(ms)", 1);
    static DistributionPlot receive_pose_sros_plot("两次收到SRC上传Pose的sros时间间隔(ms)", 1);
    //static DistributionPlot receive_pose_src_plot("两次收到SRC上传Pose的timestamp间隔", 10);
    static DistributionPlot algorithm_handle_time_plot("算法融合pose和回传pose的时间间隔之和(ms)", 1);
    static DistributionPlot last_src_pose_serial_plot("SRC上传pose序列号间隔", 1);

    auto cur_src_pose_serial = msg->getTimestamp();
    if(last_src_pose_serial_ != 0) {
        last_src_pose_serial_plot.shooting( cur_src_pose_serial - last_src_pose_serial_);
    }
    last_src_pose_serial_ = cur_src_pose_serial;

    if (last_pose_timestamp_ != 0) {
        receive_pose_sros_plot.shooting(cur_timestamp - last_pose_timestamp_);
        //receive_pose_src_plot.shooting(msg->getTimestamp() - last_src_timestamp);
        src_pose_interval_plot.shooting(msg->getRoll());
    }

    //    if (cur_timestamp - last_pose_timestamp_ > MAX_POSE_TIMESTAMP_THRESHOLD) {
    //        LOG(WARNING) << "=> Pose timeout: " << cur_timestamp - last_pose_timestamp_ << "ms";
    //    }

    last_pose_timestamp_ = cur_timestamp;
    //last_src_timestamp = msg->getTimestamp();

    Pose p(Location(msg->getX() / CONST_FLOAT_10000, msg->getY() / CONST_FLOAT_10000, msg->getZ() / CONST_FLOAT_10000),
           Rotation(msg->getYaw() / CONST_FLOAT_10000, msg->getPitch() / CONST_FLOAT_10000, 0));
    p.timestamp() = msg->getTimestamp();
    p.synctimestamp() = sync_time;
    if (pose_callback_f_) {
        pose_callback_f_(p);
        algorithm_handle_time_plot.shooting(sros::core::util::get_time_in_ms() - cur_timestamp);
    }

    if (receive_pose_sros_plot.shoot_count() % (6000 * 5) == 0) {
        LOG(INFO) << src_pose_interval_plot;
        LOG(INFO) << receive_pose_sros_plot;
        LOG(INFO) << last_src_pose_serial_plot;
        //LOG(INFO) << receive_pose_src_plot;
        LOG(INFO) << algorithm_handle_time_plot;
        LOG(INFO) << pose_send_back_time_plot_;
    }
}

bool SrcSdkV2::setTargetDMCodeOffset(int pgv_id, double x, double y, double yaw) {
    std::vector<int> values{pgv_id, FLOAT_UNIT_TO_SRTOS_UNIT(x), FLOAT_UNIT_TO_SRTOS_UNIT(y),
                            FLOAT_UNIT_TO_SRTOS_UNIT(yaw)};
    LOG(INFO) << "Target DMCode offset: " << numberListToStr(values.cbegin(), values.cend());
    return src_sdk->setParameters(SRTOS_ADDR_PGV_OFFSET_X_POSITION, values);
}

bool SrcSdkV2::setTractorDockingDMCodeOffset(double x, double y, double yaw, double deviate) {
    const uint16_t ADDR_TRACTOR_DOCKING = 0x2419;

    std::vector<int> values{(int)(10000 * x), (int)(10000 * y), (int)(10000 * yaw), (int)(10000 * deviate)};
    LOG_EVERY_N(INFO, 100) << "Tractor Docking DMCode offset: " << numberListToStr(values.cbegin(), values.cend());
    return src_sdk->setParameters(ADDR_TRACTOR_DOCKING, values);
}

bool SrcSdkV2::setPgvExecInfo(double x, double y, double yaw, int enable) {
    std::vector<int> values{ FLOAT_UNIT_TO_SRTOS_UNIT(x), FLOAT_UNIT_TO_SRTOS_UNIT(y), INT_UNIT_TO_SRTOS_UNIT(yaw), enable, 0};
    return src_sdk->setParameters(SRTOS_ADDR_SHELF_X_POSITION, values);
}


void SrcSdkV2::syncState() {
    //updateStaticSystemState();

    uint64_t i = 1;
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        if (src_upgrade_.isInUpgrade()) {
            continue;
        }
        
        // 100ms
        if ((i++ % 2) == 0) {
            updateSystemState();
            updateSportControlState();
            updateActionControlState();
            updateDeviceState();
            getSrcDebugInfo();
        }

        // 50ms
        updateActionControlCurState();
    }
}

void SrcSdkV2::updateStaticSystemState() {
    auto data = src_sdk->getParameters(SRTOS_ADDR_HW_VERSION, 18);
    if (data.empty()) {
        LOG(ERROR) << "读到数据为空！！！";
        return;
    }

    sros::device::VirtualDevice_ptr src_device = src_sdk->getSrcVirtualDevicePtr();

    src_device->setModelNo(convertInt32ListToStr(data.cbegin(), data.cbegin() + 2) +
                            "(kernal:" + versionUint2Str(data[7]) + ")");
    src_device->setVersionNo(versionUint2Str(data.at(2)));
    src_device->setSerialNo(convertInt32ListToStr(data.cbegin() + 3, data.cbegin() + 7));

    std::time_t build_time = data[15];
    uint32_t fm_boot_version = data[8];
    src_version_ = data[9];

    src_version_str_ = "SRTOS_" + versionUint2Str(src_version_) + "(" + int_to_hex(data[10]) + ")[" +
                       convertInt32ListToStr(data.begin() + 11, data.begin() + 15) + "]" +
                       std::asctime(std::localtime(&build_time)) + "(" + versionUint2Str(fm_boot_version) + ")";

    LOG(INFO) << "src_version_str_: " << src_version_str_;
    src_device->setVersionNo(src_version_str_);

    src_device->setStateOK();
}

std::vector<int> SrcSdkV2::getSrcDebugInfo() {
    static struct timeval tlast = {0,0};
    struct timeval tcur;
    const int register_count = 16;  // 寄存器个数

    gettimeofday(&tcur, NULL);
    auto results = src_sdk->getParameters(SRTOS_ADDR_MONITOR_INFO, register_count);
    //LOG(INFO) << numberListToStr(data.cbegin(), data.cend());
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

    bool is_same = true;
    auto it_old = src_state.cbegin();
    auto it_new = results.cbegin();
    while (it_old != src_state.cend()) {
        if (*it_old != *it_new) {
            is_same = false;
            LOG(INFO) << "SRTOS monitor register(ms): main_while_tick | mc_while_tick | ac_while_tick | sros2srtos_tick |" 
                    << "srtos2sros_tick | m2can_tick | can2m_tick | sros2pose_tick | pose2sros_tick | reserved";
            LOG(INFO) << "SRTOS the " << std::distance(src_state.cbegin(), it_old) << "th register changed! "
                      << *it_old << " => " << *it_new;
        }
        ++it_old;
        ++it_new;
    }
    if (!is_same) {
        // LOG(INFO) << "old: " << numberListToStr(src_state.cbegin(), src_state.cend());
        LOG(INFO) << "SRTOS new value: " << numberListToStr(results.cbegin(), results.cend());
        src_state = results;
    }
    
    long delta_time = 1000L * (tcur.tv_sec - tlast.tv_sec ) + (tcur.tv_usec - tlast.tv_usec)/1000L; //ms
    if(delta_time >= 1000){
       LOG(INFO) << "SROS timeout get SRTOS debug info, delta_time= "<< delta_time<<" ms";
    }
    tlast = tcur;
    return results;
}

void SrcSdkV2::updateSystemState() {
    auto data = src_sdk->getParameters(SRTOS_ADDR_REQ_SEQ, 16);
    if (data.empty()) {
        LOG(ERROR) << "读到数据为空！！！";
        return;
    }

    state_.src_system_state = (SRC_SYSTEM_STATE)data[2];
    state_.src_system_error_code = data[3];
    g_src_state.total_power_cycle = data[6];
    g_src_state.total_poweron_time = data[7];
    g_src_state.total_mileage = data[8];
    g_src_state.total_device_count = data[9];
    state_.gpio_input = data[10];
    state_.gpio_output = data[11];

    //update src system state and error code to Matrix
    sros::device::VirtualDevice_ptr src_device = src_sdk->getSrcVirtualDevicePtr();
    switch ((SRC_SYSTEM_STATE)state_.src_system_state) {
        case SYS_INVALID: {
            src_device->setStateNone();
            break;
        }
        case SYS_INITING: {
            src_device->setStateInitialization();
            break;
        }
        case SYS_IDLE: {
            src_device->setStateOK();
            break;
        }
        case SYS_BUSY: {
            src_device->setStateOK();
            break;
        }
        case SYS_ERROR: {
            src_device->setStateError(state_.src_system_error_code);
            break;
        }
        default: {
            LOG(ERROR) << "unreachable! state is " << state_.src_system_state;
        }
    }
    src_device->updateAliveTime();
}

bool SrcSdkV2::setSrcConfig() {
    auto setConfigToSRCFunc = [&](std::string class_name, int config_start_id, int src_start_register_addr) -> bool {
        auto configs = Settings::getInstance().getItemInfoListOfClass(class_name);
        if (configs.empty()) {
            LOG(ERROR) << "configs empty";
            return false;
        }

        std::vector<int> values(configs.back().id - config_start_id + 1);
        for (auto i = 0; i < configs.size(); ++i) {
            int value = 0;
            try {
                value = std::stoi(configs[i].value);
            } catch (std::exception &e) {
                LOG(ERROR) << e.what();
                return false;
            }
            values[configs[i].id - config_start_id] = value;
        }

        if(!src_sdk->setParameters(src_start_register_addr, values)){
            LOG(ERROR) << "setParameters failed";
            return false;
        }
        return true;
    };

    LOG(INFO) << "setSrcConfig 1";
    bool bRet = setConfigToSRCFunc("srtos", SRTOS_CONFIG_START_ID, SRTOS_ADDR_SYSTEM_CONFIG);
    if(!bRet){
        LOG(ERROR) << "srtos setParameters failed";
        return bRet;
    }

    LOG(INFO) << "setSrcConfig 2";
    bRet = setConfigToSRCFunc("mc", SRTOS_CONFIG_MC_ID, SRTOS_ADDR_MC_CONFIG);
    if(!bRet){
        LOG(ERROR) << "mc setParameters failed";
        return bRet;
    }

    LOG(INFO) << "setSrcConfig 3";
    bRet = setConfigToSRCFunc("ac", SRTOS_CONFIG_AC_ID, SRTOS_ADDR_AC_CONFIG);
    if(!bRet){
        LOG(ERROR) << "ac setParameters failed";
        return bRet;
    }
    return bRet;
}
void SrcSdkV2::updateSportControlState() {
    auto data = src_sdk->getParameters(SRTOS_ADDR_TASK_MC_ID, 21);
    if (data.empty()) {
        return;
    }

    // 更新状态功能
    auto update_task_state = [=](TaskState _state) {
        LOG(INFO) << "update_task_state : " << _state;
        if (_state == TASK_PAUSED) {
            if (path_paused_callback_f_) {
                path_paused_callback_f_();
            }
        } else if (_state == TASK_FINISHED) {
            src_sdk->offRunSportControlFlag();
            LOG(INFO) << "TASK_FINISHED : " << state_.mc_result;
            switch (state_.mc_result) {
                case TASK_RESULT_OK: {
                    if (path_finish_callback_f_) {
                        path_finish_callback_f_();
                    }
                    break;
                }
                case TASK_RESULT_CANCELED:
                case TASK_RESULT_FAILED: {
                    if (path_aborted_callback_f_) {
                        path_aborted_callback_f_();
                    }
                    break;
                }
                default: {
                    break;
                }
            }
        } else if (_state == TASK_RUNNING) { 
            if(state_.mc_result == TASK_RESULT_FAILED) {
                if(state_.mc_result_code == 7)  {  //ERR_CODE_PATH_OUT 偏离路网
                    LOG(ERROR) <<"ERR_CODE_PATH_OUT";

                }else if(state_.mc_result_code == 17) { //ERR_CODE_JAM 驱动轮卡死
                    LOG(ERROR) <<"ERR_CODE_JAM";
                    FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_MOTOR_SLIP);
                    src_sdk->cancleTask();

                }
            }
        }
    };

    // 路网导航移动任务状态
    auto path_mc_task_no = data[0];
    auto path_mc_task_state = data[1];
    if (state_.mc_task_state != path_mc_task_state || state_.mc_task_no != path_mc_task_no || state_.mc_result_code != data[3]) {
        state_.mc_task_state = (TaskState)path_mc_task_state;
        state_.mc_task_no = path_mc_task_no;
        state_.mc_result = (TaskResult)data[2];
        state_.mc_result_code = data[3];
        update_task_state(state_.mc_task_state);
    }

    // 动作移动任务状态(7,1,0)
    auto ac_mc_task_no = data[4];
    auto ac_mc_task_state = data[5];
    if (state_.ac_mc_task_state != ac_mc_task_state || state_.ac_mc_task_no != ac_mc_task_no) {        
        auto ac_task_state = (TaskState)ac_mc_task_state;
        state_.ac_mc_task_no = ac_mc_task_no;
        state_.ac_mc_result = (TaskResult)data[6];
        state_.ac_mc_result_code = data[7];
        TaskStateStruct task_state_struct(state_.ac_mc_task_no, ac_task_state, state_.ac_mc_result, state_.ac_mc_result_code);
        state_.ac_mc_task_state = ac_task_state;
        if (action_state_callback_f_) {
            action_state_callback_f_(task_state_struct);
        }
    }

    state_.src_state = (SRC_STATE_t)data[SRTOS_ADDR_MC_MODE - SRTOS_ADDR_TASK_MC_ID + 1];
    g_src_state.src_state = state_.src_state;
    state_.v_x = SRTOS_UNIT_TO_INT_UNIT(data[SRTOS_ADDR_MC_MODE - SRTOS_ADDR_TASK_MC_ID + 3]);
    state_.v_y = SRTOS_UNIT_TO_INT_UNIT(data[SRTOS_ADDR_MC_MODE - SRTOS_ADDR_TASK_MC_ID + 4]);
    state_.w = SRTOS_UNIT_TO_INT_UNIT(data[SRTOS_ADDR_MC_MODE - SRTOS_ADDR_TASK_MC_ID + 5]);
    if (state_.path_no != data[SRTOS_ADDR_MC_MODE - SRTOS_ADDR_TASK_MC_ID + 6]) {
        LOG(INFO) << "current path no change: " << (int)g_src_state.cur_path_no << " -> "
                  << data[SRTOS_ADDR_MC_MODE - SRTOS_ADDR_TASK_MC_ID + 6];
        state_.path_no = data[SRTOS_ADDR_MC_MODE - SRTOS_ADDR_TASK_MC_ID + 6];
        g_src_state.cur_path_no = state_.path_no;
    }
    state_.path_remain_time = data[SRTOS_ADDR_MC_MODE - SRTOS_ADDR_TASK_MC_ID + 7];
    state_.path_remain_distance = data[SRTOS_ADDR_MC_MODE - SRTOS_ADDR_TASK_MC_ID + 8] * 100;
    state_.path_total_distance = data[SRTOS_ADDR_MC_MODE - SRTOS_ADDR_TASK_MC_ID + 9] * 100;
    state_.movement_state = (SRC_MOVEMENT_t)data[SRTOS_ADDR_MC_MODE - SRTOS_ADDR_TASK_MC_ID + 10];
    g_src_state.movement_state = state_.movement_state;
    if (g_src_state.cur_checkpoint != data[SRTOS_ADDR_MC_MODE - SRTOS_ADDR_TASK_MC_ID + 11]) {
        LOG(INFO) << "checkpoint change: " << (int)g_src_state.cur_checkpoint << " -> "
                  << data[SRTOS_ADDR_MC_MODE - SRTOS_ADDR_TASK_MC_ID + 11];
        g_src_state.cur_checkpoint = data[SRTOS_ADDR_MC_MODE - SRTOS_ADDR_TASK_MC_ID + 11];
    }
    if (g_src_state.is_waitting_checkpoint != (bool)data[SRTOS_ADDR_MC_MODE - SRTOS_ADDR_TASK_MC_ID + 12]) {
        LOG(INFO) << "is_waiting_checkpoint change: " << g_src_state.is_waitting_checkpoint << " -> "
                  << (bool)data[SRTOS_ADDR_MC_MODE - SRTOS_ADDR_TASK_MC_ID + 12];
        g_src_state.is_waitting_checkpoint = (bool)data[SRTOS_ADDR_MC_MODE - SRTOS_ADDR_TASK_MC_ID + 12];
    }
    g_src_state.cur_v = state_.v_x;
    g_src_state.cur_w = state_.w;
    g_src_state.cur_remain_distance = state_.path_remain_distance;
    g_src_state.cur_remain_time = state_.path_remain_time;
    g_src_state.cur_total_distance = state_.path_total_distance;

    if (state_callback_f_ && src_sdk->isInitProtoOk()) {
        state_callback_f_(state_);
    }
}
void SrcSdkV2::updateActionControlState() {
    auto data = src_sdk->getParameters(SRTOS_ADDR_TASK_AC_NO, 11);
    if (data.empty()) {
        return;
    }

    auto ac_task_state = (TaskState)data[4];
    state_.ac_task_no = (TaskResult)data[0];
    state_.ac_result = (TaskResult)data[5];
    state_.ac_result_code = data[6];
    TaskStateStruct task_state_struct(state_.ac_task_no, ac_task_state, state_.ac_result, state_.ac_result_code);
    state_.ac_task_state = ac_task_state;
    action_state_callback_f_(task_state_struct);
}

// 通信协议：https://standard-robots.yuque.com/docs/share/ff381c56-40f9-4f79-ba3d-450cf4e8370d?#51d8 《SRTOS通信协议v2.1.0》
void SrcSdkV2::updateActionControlCurState() {
    auto data = src_sdk->getParameters(SRTOS_ADDR_CUR_ACTION_STATE, 24);
    if (data.empty()) {
        LOG(ERROR) <<"读到数据为空！！！";
        return;
    }

    static int32_t last = 0;
    static int32_t rotate_value = 0;
    
    // 当前动作状态信息srtos下层协议已与oasis和gulf兼容
    state_.ac_type = (enum SRC_AC_TYPE)data[0]; // 当前动作控制搭载执行机构类型
    state_.reserved_0 = data[3]; // 载货状态
    g_state.rotate_value = data[10]; // 顶升货架时，货架相对于小车的旋转角度,逆时针为正;
    g_state.fork_height_encoder = data[11] / 1000.0;
    g_state.fork_updown_state = data[12];
    g_state.io_1353 = data[17];
    g_state.steering_angle = data[18];  //100倍的角度
    ManualBtnState mode = (enum ManualBtnState)data[19];
    src_sdk->handlManualSignal(mode);

    // debug
    if(last != data[3] || rotate_value != data[10]) {
         LOG(INFO) <<"load_state: "<< data[3] <<", rotate_value:"<< (int)data[10] << " ==>g_state.rotate_value: "
         << g_state.rotate_value;
         last = data[3];
         rotate_value = data[10];
    }
   
    if (g_state.action_unit ==sros::core::ACTION_UNIT_SRC) {
        if (g_state.multi_load_state != state_.reserved_0) {
            g_state.multi_load_state = state_.reserved_0;
//            LOG(INFO) << "multi_load_state: " << g_state.multi_load_state;
        }
        sros::core::LoadState new_load_state = state_.reserved_0 ==0?sros::core::LOAD_FREE :sros::core::LOAD_FULL;
        if (g_state.load_state != new_load_state) {
            g_state.load_state = new_load_state;

            LOG(INFO) <<" g_state.load_state: "<< data[3] <<", g_state.rotate_value:"<< (int)data[10] ;
            if (new_load_state ==sros::core::LOAD_FULL) {
//                LOG(INFO) << "Load state changed: LOAD_FREE -> LOAD_FULL";
                std::ostringstream oss;
                oss <<"Load state changed: LOAD_FREE -> LOAD_FULL";
                logSendMonitor(oss);
            } else {
//                LOG(INFO) << "Load state changed: LOAD_FULL -> LOAD_FREE";
                std::ostringstream oss;
                oss <<"Load state changed: LOAD_FULL -> LOAD_FREE";
                logSendMonitor(oss);
            }
        }
    }
    
    // TODO, 注释掉，by neethan
    // std::vector<uint16_t> data_16(data.size() * 2);
    // for (int i = 0; i < data.size(); ++i) {
    //     data_16[i * 2] = (data[i] > 16) & 0xFFFF;
    //     data_16[i * 2 + 1] = data[i] & 0xFFFF;
    // }
    // RegisterAdmin::getInstance()->writeRelativeInputRegisters(sros::SRTOS_ACTION_DEBUG_INFO, data_16);
}

void SrcSdkV2::updateDeviceState() {
    if (g_src_state.total_device_count > 50) {
        LOG(ERROR) << "设备个数太多！！！ device count is " << g_src_state.total_device_count;
        return;
    }
    const int DEVICE_ADDR_COUNT = 32;  // 一个设备有多少个寄存器
    auto data = src_sdk->getParameters(SRTOS_ADDR_CPU_USAGE, 0x08);
    if (data.empty()) {
        LOG(ERROR) << "读到数据为空！！！";
        return;
    } else if (data.size() != 0x08) {
        LOG(ERROR) << "SRTOS_ADDR_CPU_USAGE size != 8";
        return;
    }

    //LOG(INFO) << "data.size: " << data.size() << ", " << numberListToStr(data.cbegin(), data.cend());

    g_src_state.cpu_usage = data[0];
    g_src_state.memory_usage = data[1];
    //LOG(INFO) << "total_device_count: " << g_src_state.total_device_count;

    for (int i = 0; i < g_src_state.total_device_count; ++i) {
        data = src_sdk->getParameters(SRTOS_ADDR_MOTOR_1_ID + i * DEVICE_ADDR_COUNT, DEVICE_ADDR_COUNT);
        if (data.empty()) {
            LOG(ERROR) << "读到数据为空！！！";
            return;
        } else if (data.size() != DEVICE_ADDR_COUNT) {
            LOG(ERROR) << "addr: " <<  std::hex << "0x" <<SRTOS_ADDR_MOTOR_1_ID + i * DEVICE_ADDR_COUNT << ", count: 0x" << DEVICE_ADDR_COUNT;
            return;
        }

        auto device_id = data[0];
        auto p_device = DeviceManager::getInstance()->getDeviceById(device_id);
        if (!p_device) {
            std::string device_name;
            switch (device_id)
            {
            case DEVICE_ID_MC_MOTOR_1:
                device_name = DEVICE_MOTOR_1;
                break;
            case DEVICE_ID_MC_MOTOR_2:
                device_name = DEVICE_MOTOR_2;
                break;
            case DEVICE_ID_AC_MOTOR_1:
                device_name = DEVICE_MOTOR_3;
                break;
            case DEVICE_ID_AC_MOTOR_2:
                device_name = DEVICE_MOTOR_4;
                break;
            case DEVICE_ID_WALK_1:
                device_name = DEVICE_WALK_1;
                break;
            case DEVICE_ID_WALK_2:
                device_name = DEVICE_WALK_2;
                break;
            case DEVICE_ID_WALK_3:
                device_name = DEVICE_WALK_3;
                break;
            case DEVICE_ID_ROTATE_1:
                device_name = DEVICE_ROTATE_1;
                break;
            case DEVICE_ID_ROTATE_2:
                device_name = DEVICE_ROTATE_2;
                break;
            case DEVICE_ID_ROTATE_3:
                device_name = DEVICE_ROTATE_3;
                break;
            case DEVICE_ID_PULL_ROPE_1:
                device_name = DEVICE_PULL_ROPE_1;
                break;
            case DEVICE_ID_PULL_ROPE_2:
                device_name = DEVICE_PULL_ROPE_2;
                break;
            case DEVICE_ID_PULL_SWAY_1:
                device_name = DEVICE_PULL_SWAY_1;
                break;
            case DEVICE_ID_PULL_SWAY_2:
                device_name = DEVICE_PULL_SWAY_2;
                break;
            case DEVICE_ID_IOEXTEND_1353:
                device_name = DEVICE_IOEXTEND_1353;
                break;
            default:
                device_name = std::to_string(device_id);
                break;
            }

            p_device = createDevice<VirtualDevice>(device_name, (DeviceID)device_id, DEVICE_COMM_INTERFACE_TYPE_CAN_2,
                                                   DEVICE_MOUNT_SRC);
            auto encoder_ptr = std::dynamic_pointer_cast<sros::device::Encoder>(p_device);
            if (nullptr != encoder_ptr) {
                encoder_ptr->setPosition(data[24]);
                LOG(INFO) << device_name << " set position : " << encoder_ptr->getPosition();
            }
            
            auto device_model_no = convertInt32ListToStr(data.cbegin() + 3, data.cbegin() + 6);
            auto device_serial_no = convertInt32ListToStr(data.cbegin() + 7, data.cbegin() + 14);
            auto device_version_no = convertInt32ListToStr(data.cbegin() + 15, data.cbegin() + 22);
            p_device->setSerialNo(device_serial_no);
            p_device->setVersionNo(device_version_no);
            p_device->setModelNo(device_model_no);
        }

        auto device_status = data[1];
        auto device_error_code = data[2];
        p_device->setState((DeviceState)device_status, device_error_code);
        p_device->setInfo(numberListToStr(data.cbegin() + 27, data.cend()));
        p_device->updateAliveTime();
    }
}

void SrcSdkV2::handleVelocityMsg(const std::vector<uint8_t> &data) {
    //LOG(INFO) << "[sdk] handleVelocityMsg";
    VelocityMsg_ptr msg = make_shared<VelocityMsg>();
    msg->rawData(data);
    msg->decode();
    //LOG(INFO) << "new g_src_state.cur_v:" << msg->getVx() << ", g_src_state.cur_w:" << msg->getVtheta();

    if (velocity_callback_f_) {
        velocity_callback_f_(sros::core::Velocity(msg->getVx() / CONST_FLOAT_10, 0, msg->getVtheta() / CONST_FLOAT_10));
    }
}

void SrcSdkV2::subRecvUartDataHandle(const std::vector<uint8_t> &data) {
}

void SrcSdkV2::executeAction(uint32_t no, uint16_t id, int16_t param0, int16_t param1, int16_t param2) {
    return sendCommandMsgDo(COMMAND_EXECUTE_ACTION, (int)no, id, param0, param1, param2);
}
}  // namespace sdk
