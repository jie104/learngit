//
// Created by caoyan on 2020-11-12.
//

#include "src_sdk.h"
#include "src_sdk_v1.h"
#include "src_sdk_v2.h"

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

#include "core/hardware/SRC.h"
#include "core/logger.h"
#include "core/settings.h"
#include "core/util/utils.h"
#include "protocol/security_msg.hpp"
#include "protocol/src_protocol.h"
#include "protocol/sros_state_msg.hpp"

#include "core/error_code.h"
#include "core/msg/command_msg.hpp"
#include "core/task/task.h"
#include "core/task/task_manager.h"
#include "core/msg_bus.h"

using namespace std;
using namespace src;
using namespace sros::device;

//#define SRC_PROTO_DBUG

sdk::SrcSdk::SrcSdk() : connection_() {}

sdk::SrcSdk::~SrcSdk() { disconnect(); }

/**
 * 在调用connect前一定要先设置callback
 */
bool sdk::SrcSdk::connect(std::string port, unsigned int baud_rate) {
    connection_ = make_shared<usart::Connection<usart::FrameV1<>>>();
    connection_->setHwDevName(DEVICE_SRC);
    connection_->setRecvDataCallback(boost::bind(&SrcSdk::onRecvUartData, this, _1));

    if (!src_device_) {
        // src_device_ = sros::device::DeviceManager::getInstance()->registerDevice(
        //    DEVICE_SRC, DEVICE_ID_SRC, DEVICE_COMM_INTERFACE_TYPE_RS232_2, DEVICE_MOUNT_HARDWARE);
        src_device_ =
            createDevice<SRC>(DEVICE_SRC, DEVICE_ID_SRC, DEVICE_COMM_INTERFACE_TYPE_RS232_2, DEVICE_MOUNT_HARDWARE);
        src_device_->setStateInitialization();

        auto &s = sros::core::Settings::getInstance();
        int insepction_interval = s.getValue<int>("inspection.src_inspection_intvl", 6000);
        LOG(INFO) << "src_inspection_intvl : " << insepction_interval;
        src_device_->setTimeoutTime(insepction_interval);
        if (src_device_) {
        } else {
            LOG(WARNING) << "SRC register to DeviceManager failed";
        }
    }

    if (!connection_->connect(port, baud_rate)) {
        LOG(ERROR) << "UsartModule: serial device " << port << " open failed!";

        if (src_device_) {
            src_device_->setStateOpenFailed();
        }
        return false;
    }

    return true;
}

void sdk::SrcSdk::disconnect() {
    if (connection_) {
        connection_->disconnect();
    }
}

void sdk::SrcSdk::initState() {
    LOG(INFO) << "src init!";

    if (isInitProtoOk()) {
        return;
    }

    //先去获取版本号，再去发起连接
    getSrcProtoVersion();

    if (src_proto_version_ == SRC_PROTO_VERSION_NONE) {
        reconn_src_ = true;
        disconnect();  //通讯异常先断开
        return;
    } else {
        reconn_src_ = false;
        sendCommandMsg(COMMAND_CONNECT);
        initProtoDo();
    }
}

void sdk::SrcSdk::initProtoDo() {
    if (src_sdk_proto_do_) {
        src_sdk_proto_do_->init();
    }
}

void sdk::SrcSdk::setPaths(sros::core::NavigationPathi_vector paths) {
    if (src_sdk_proto_do_) {
        src_sdk_proto_do_->setPaths(paths);
    }
}

void sdk::SrcSdk::setVelocity(int v, int w) {
    if (checkVelocityCallback) {
        checkVelocityCallback(v,w);
    }
    if (src_sdk_proto_do_) {
        src_sdk_proto_do_->setVelocity(v, w);
    }
}

void sdk::SrcSdk::pauseMovement(int pause_level) {
    if (pause_level < 0 || pause_level > 5) {
        pause_level = 0;
    }

    std::lock_guard<std::recursive_mutex> lg(pause_mutex);
    sros_pause_ = true;
    sendCommandMsg(COMMAND_PAUSE, pause_level, 0);
    boost::this_thread::sleep_for(boost::chrono::milliseconds(2));
    sendCommandMsg(COMMAND_PAUSE, pause_level, 0);
}

void sdk::SrcSdk::continueMovement() {
    std::lock_guard<std::recursive_mutex> lg(pause_mutex);
    sros_pause_ = false;
    sendCommandMsg(COMMAND_CONTINUE);
}

void sdk::SrcSdk::stop() {
    std::lock_guard<std::recursive_mutex> lg(pause_mutex);
    sros_pause_ = false;
    sendCommandMsg(COMMAND_STOP);
}

void sdk::SrcSdk::navBegin() {
    sendCommandMsg(COMMAND_NAV_STATE, 1, g_state.station_no);
    LOG(INFO) << "Command is (NAV_RUNNING) " << g_state.station_no;
}

void sdk::SrcSdk::navFinished() {
    sendCommandMsg(COMMAND_NAV_STATE, 2, g_state.station_no);
    LOG(INFO) << "Command is (NAV_FINISH) " << g_state.station_no;
}

bool sdk::SrcSdk::upgradeRequest(const std::string &upgrade_bin_file_path) {
    if (src_sdk_proto_do_) {
        return src_sdk_proto_do_->upgradeRequest(upgrade_bin_file_path);
    }
    return false;
}

void sdk::SrcSdk::upgradSrcTest() {
    if (src_sdk_proto_do_) {
        src_sdk_proto_do_->upgradSrcTest();
    }
}

void sdk::SrcSdk::executeAction(uint32_t no, uint16_t id, int16_t param0, int16_t param1, int16_t param2) {

    auto &s = sros::core::Settings::getInstance();
    int min_height = s.getValue<int>("forklift.fork_arm_min_height", 80);
    int max_height = s.getValue<int>("forklift.fork_arm_max_height", 10000);

    //叉臂指令超出软限位报错返回
    if( (id == 24) && (param0 == 3 || param0 == 4 || param0 == 5 || param0 == 10) && (param1 < min_height || param1 > max_height) ) {
        sros::core::TaskStateStruct task_state_struct;
        task_state_struct.no_ = no;
        task_state_struct.state_ = sros::core::TaskState::TASK_FINISHED;
        task_state_struct.result_ = sros::core::TaskResult::TASK_RESULT_FAILED;
        task_state_struct.result_value_ = sros::core::ERROR_CODE_FORK_ADJUST_HEIGHT_OUT_RANGE;

        if (src_sdk_proto_do_) {
            src_sdk_proto_do_->action_state_callback_f_(task_state_struct);
        }

        return ;
    }
    // //货叉零位标定记录零位高度
    // if( (id == 24) && (param0 == 13)) {
    //     sros::core::Settings::getInstance().setValue("forklift.fork_arm_min_height", param1);
    // }

    if (src_sdk_proto_do_) {
        src_sdk_proto_do_->executeAction(no, id, param0, param1, param2);
    }
}

void sdk::SrcSdk::cancelAction(uint32_t no) {
    LOG(INFO) << "cancelAction no: " << no;
    sendCommandMsg(COMMAND_CANCEL_ACTION, no, 0);
}

void sdk::SrcSdk::setSpeedLevel(uint8_t level) {
    level = (level > 100) ? (uint8_t)100 : level;

    sendCommandMsg(COMMAND_SET_SPEED_LEVEL, level, 0);
}

void sdk::SrcSdk::sendPoseBack(sros::core::Pose pose) {
    if (src_sdk_proto_do_) {
        src_sdk_proto_do_->sendPoseBack(pose);
    }
}

void sdk::SrcSdk::setPoseAvailable(bool available) {
    if (src_sdk_proto_do_) {
        src_sdk_proto_do_->setPoseAvailable(available);
    }
}

const SRCState &sdk::SrcSdk::getSRCState() {
    if (src_sdk_proto_do_) {
        return src_sdk_proto_do_->getSRCState();
    } else {
        static SRCState src_state;
        return src_state;
    }
}

const sros::core::Pose &sdk::SrcSdk::getCurPose() {
    if (src_sdk_proto_do_) {
        return src_sdk_proto_do_->getCurPose();
    } else {
        static sros::core::Pose cur_pose;
        return cur_pose;
    }
}

bool sdk::SrcSdk::isPathFinished() {
    if (src_sdk_proto_do_) {
        return src_sdk_proto_do_->isPathFinished();
    } else {
        return false;
    }
}

void sdk::SrcSdk::resetFault() {
    if (src_sdk_proto_do_) {
        src_sdk_proto_do_->resetFault();
    }
}

uint32_t sdk::SrcSdk::getSRCVersion() const {
    if (src_sdk_proto_do_) {
        return src_sdk_proto_do_->getSRCVersion();
    } else {
        return 0;
    }
}
string sdk::SrcSdk::getSRCVersionStr() const {
    if (src_sdk_proto_do_) {
        return src_sdk_proto_do_->getSRCVersionStr();
    } else {
        return "";
    }
}

string sdk::SrcSdk::getUpPGVScanCode() {
    const uint16_t ADDR_PGV_UP_UNUSED = 0x1200;
    const uint16_t ADDR_PGV_UP_SCAN_CODE = 0x1208;

    auto scan_code = getParametersString(ADDR_PGV_UP_SCAN_CODE, 32);

    LOG(INFO) << "PGV_UP_SCAN_CODE: " << scan_code;

    return scan_code;
}

string sdk::SrcSdk::getDownPGVScanCode() {
    const uint16_t ADDR_PGV_DOWN_UNUSED = 0x1210;
    const uint16_t ADDR_PGV_DOWN_SCAN_CODE = 0x1218;

    auto scan_code = getParametersString(ADDR_PGV_DOWN_SCAN_CODE, 32);

    LOG(INFO) << "PGV_DOWN_SCAN_CODE: " << scan_code;

    return scan_code;
}

void sdk::SrcSdk::setCheckpoint(uint8_t checkpoint_no) {
    sendCommandMsg(COMMAND_SET_CHECKPOINT, checkpoint_no, 0);
//    LOG(INFO) << "Set checkpoint " << (int)checkpoint_no;
}

bool sdk::SrcSdk::sendIAPdata(const std::vector<uint8_t> &data) {
    if (data.size() && data[0] == MSG_IAP) {
        return connection_->sendData(data);
    }
    return false;
}

std::vector<int> sdk::SrcSdk::getCurPgvInfo() {
    const uint16_t ADDR_PGV_CUR_INFO = 0x1220;

    auto results = getParameters(ADDR_PGV_CUR_INFO, 4);
    if (results.size() != 4) {
        if (!results.empty()) {
            LOG(ERROR) << "getParameters 返回的数据有问题！！！ size: " << results.size();
            for (auto ret : results) {
                LOG(ERROR) << ret;
            }
        }
        LOG(WARNING) << "获取PGV状态失败！";
        return std::vector<int>{0, 0, 0, 0};
    }

    // 当pgv扫描到蓝色地面时，会误识别为色带，导致上报一些奇怪的数据，所以我们认为id为0数据帧都有问题
    if (results.at(0) == 0) {
        results.at(1) = 0;
        results.at(2) = 0;
        results.at(3) = 0;
    }

    return results;
}

bool sdk::SrcSdk::setTargetDMCodeOffset(int pgv_id, double x, double y, double yaw) {
    if (src_sdk_proto_do_) {
        return src_sdk_proto_do_->setTargetDMCodeOffset(pgv_id, x, y, yaw);
    } else {
        return false;
    }
}

bool sdk::SrcSdk::setPgvExecInfo(double x, double y, double yaw, int enable) {
    if (src_sdk_proto_do_) {
        return src_sdk_proto_do_->setPgvExecInfo(x, y, yaw, enable);
    } else {
        return false;
    }
}

std::vector<int> sdk::SrcSdk::getSrcDebugInfo() {
    if (src_sdk_proto_do_) {
        return src_sdk_proto_do_->getSrcDebugInfo();
    } else {
        return std::vector<int>();
    }
}

void sdk::SrcSdk::sendVelocityBack(sros::core::Velocity velocity) {
    if (src_sdk_proto_do_) {
        src_sdk_proto_do_->sendVelocityBack(velocity);
    }
}

bool sdk::SrcSdk::setUpCameraDetectInfo(int valid, int x, int y, int yaw, int dm_code_info) {
    if (src_sdk_proto_do_) {
        return src_sdk_proto_do_->setUpCameraDetectInfo(valid, x, y, yaw, dm_code_info);
    } else {
        return false;
    }
}

bool sdk::SrcSdk::setDownCameraDetectInfo(int valid, int x, int y, int yaw, int dm_code_info) {
    if (src_sdk_proto_do_) {
        return src_sdk_proto_do_->setDownCameraDetectInfo(valid, x, y, yaw, dm_code_info);
    } else {
        return false;
    }
}

bool sdk::SrcSdk::setTractorDockingDMCodeOffset(double x, double y, double yaw, double deviate) {
    if (src_sdk_proto_do_) {
        return src_sdk_proto_do_->setTractorDockingDMCodeOffset(x, y, yaw, deviate);
    } else {
        return false;
    }
}

bool sdk::SrcSdk::isSrtosRollBackSrc() {
    //判断当前是否是从srtos回退到src

    SRC_PROTO_VERSION cur_src_proto_version;

    for (int count = 0 ; count < 5; count ++) {
        auto data = getParameters(SRTOS_ADDR_HW_VERSION, 18);
        LOG(INFO) << "cur_src_proto_version: " << numberListToStr(data.cbegin(), data.cend());
        if (data.empty()) {
            cur_src_proto_version = SRC_PROTO_VERSION_V1;
        } else if (data[0] == 0 || data[9] == 0 || data[10] == 0) {
            // V1 协议版本全部返回的0
            cur_src_proto_version = SRC_PROTO_VERSION_V1;
        } else {
            cur_src_proto_version = SRC_PROTO_VERSION_V2;
            break;
        }

        boost::this_thread::sleep_for(boost::chrono::milliseconds(1000)); 
    }

    LOG(INFO) << "src_proto_version_: V" << src_proto_version_ << ", cur_src_proto_version: V" << cur_src_proto_version;

    if (src_proto_version_ == SRC_PROTO_VERSION_V2 && cur_src_proto_version == SRC_PROTO_VERSION_V1 ) {
        LOG(INFO) << "isSrtosRollBackSrc YES";
        return true;

    } else {
        LOG(INFO) << "isSrtosRollBackSrc NO";
        return false;
    }

}

void sdk::SrcSdk::getSrcProtoVersion() {
    //以src V2协议去读取版本号，为了兼容V1版本协议

    auto data = getParameters(SRTOS_ADDR_HW_VERSION, 18);
    LOG(INFO) << "getSrcProtoVersion: " << numberListToStr(data.cbegin(), data.cend());
    if (data.empty()) {
        if (isConnSrcFailFlag()) {
            // src升级崩溃了，直接设置成V1版本
            LOG(ERROR) << "conn src fail, just set src v1";
            src_proto_version_ = SRC_PROTO_VERSION_V1;
            is_set_version_manual_ = true;
            setConnSrcFailFlag(false);
        } else {
            LOG(ERROR) << "cannot getSrcProtoVersion data, src_proto_version_: none";
            src_proto_version_ = SRC_PROTO_VERSION_NONE;
            return;  //返回
        }
    } else if (data[0] == 0 || data[9] == 0 || data[10] == 0) {
        // V1 协议版本全部返回的0
        src_proto_version_ = SRC_PROTO_VERSION_V1;
        is_set_version_manual_ = false;
    } else {
        src_proto_version_ = SRC_PROTO_VERSION_V2;
        is_set_version_manual_ = false;
    }

    //根据不同的版本来加载对应的处理类
    LOG(INFO) << "src_proto_version_: V" << src_proto_version_;
    if (src_proto_version_ == SRC_PROTO_VERSION_V2) {
        src_sdk_proto_do_ = std::make_shared<sdk::SrcSdkV2>();

        // v2超时时间
        set_single_param_timeout_ms_ = 100;
        set_multiple_param_timeout_ms_ = 100;

        // 更新srtos版本号
        updateSrtosVersion(data);

    } else {
        src_sdk_proto_do_ = std::make_shared<sdk::SrcSdkV1>();

        // v1超时时间
        set_single_param_timeout_ms_ = 200;
        set_multiple_param_timeout_ms_ = 200;
    }
}

void sdk::SrcSdk::onRecvUartData(const std::vector<uint8_t> &data) {
    uint8_t type = data[0];
    assert(src_device_ != nullptr);

    if (type != MSG_WWDG) {
        src_device_->keepAlive();
    }

    switch (type) {
        case MSG_IAP: {
            if (src_sdk_proto_do_) {
                src_sdk_proto_do_->handleIAPRequest(data);
            }
            break;
        }
        case MSG_VELOCITY: {
            if (src_sdk_proto_do_) {
                src_sdk_proto_do_->handleVelocityMsg(data);
            }
            break;
        }
        case MSG_POSE: {
            if (src_sdk_proto_do_) {
                src_sdk_proto_do_->handlePoseMsg(data);
            }
            break;
        }
        case MSG_PARAMETER: {
            handleParameterMsg(data);
            break;
        }
        case MSG_SIGNAL: {
            if (src_sdk_proto_do_) {
                src_sdk_proto_do_->handleSignalMsg(data);
            }
            break;
        }
        case MSG_OPT_POSE: {
            if (src_sdk_proto_do_) {
                src_sdk_proto_do_->handleOptPoseMsg(data);
            }
            break;
        }
        case MSG_SECURITY: {
            if (src_sdk_proto_do_) {
                src_sdk_proto_do_->handleSecurityMsg(data);
            }
            break;
        }
        default:
            if (src_sdk_proto_do_) {
                src_sdk_proto_do_->subRecvUartDataHandle(data);
            }
            break;
    }
}

void sdk::SrcSdk::sendMsg(src::BaseMsg_ptr msg) {
    if (connection_ && msg) {
        msg->encode();
        connection_->sendData(msg->rawData());
    }
}

void sdk::SrcSdk::sendMsg(const std::vector<uint8_t> &data) {
    if (connection_) {
        connection_->sendData(data);
    }
}

void sdk::SrcSdk::sendCommandMsg(COMMAND_t command, int32_t param0, int32_t param1) {
    if (src_sdk_proto_do_) {
        src_sdk_proto_do_->sendCommandMsgDo(command, param0, param1, 0, 0);
    }
}

void sdk::SrcSdk::sendCommandMsg(COMMAND_t command) { sendCommandMsg(command, 0, 0); }

bool sdk::SrcSdk::getParameterInTime(uint16_t addr, int &value) { return getParameter(addr, value, 100); }

std::string sdk::SrcSdk::getParametersString(uint16_t addr, int str_len, bool try_get) {
    const auto parameter_len = (uint16_t)ceil(str_len / 4.0);

    auto int32_list = getParameters(addr, parameter_len, try_get);

    auto parameter_str = convertInt32ListToStr(int32_list.cbegin(), int32_list.cend());

    // LOG(INFO) << "getParametersString() " << fhex(4) << addr << " -> " << parameter_str;

    return parameter_str;
}

bool sdk::SrcSdk::getParameter(uint16_t addr, int &value, uint32_t timeout_ms, bool try_get) {
    auto rs = getParameters(addr, 1, timeout_ms, try_get);
    if (rs.empty()) {
        return false;
    }

    value = rs[0];
    return true;
}

std::vector<int> sdk::SrcSdk::getParameters(uint16_t start_addr, uint16_t count, uint32_t timeout_ms, bool try_get) {
    if (!connection_ || !connection_->isConnected()) {
        return vector<int>();
    }

    std::unique_lock<std::mutex> ul(com_mutex_, std::defer_lock);
    if (try_get) {
        if (!ul.try_lock()) {
            return std::vector<int>();
        }
    } else {
        ul.lock();
    }

    const uint16_t MAX_FRAME_VALUE_COUNT = 32;

    auto values = vector<int>();

    auto getParametersFunc = [&](uint16_t start_addr, uint16_t count, uint32_t timeout_ms) {
        ParameterMsg_ptr msg_result = std::make_shared<ParameterMsg>();
        reply_msg.reset();

        ParameterMsg_ptr m = std::make_shared<ParameterMsg>();
        m->setAddr(start_addr, count);
        sendMsg(m);

        auto cur_timestamp = sros::core::util::get_time_in_ms();
        bool ret = reply_msg.waitForResult(timeout_ms, msg_result);
        if (!ret) {  // 阻塞等待回复
            // 超时
            LOG(ERROR) << "SRC::getParameters(0x" << std::hex << start_addr << ", " << count
                       << ") -> wait timeout:" << std::dec << timeout_ms;
            return std::vector<int>();
        }

        //增加对请求个数和起始地址的精确匹配
        if (msg_result->count_ != count || msg_result->start_addr_ != start_addr)
        {
            LOG(ERROR) << "SRC::getParameters(0x" << std::hex << start_addr << ", " << count << ") -> match error:(0x" << std::hex << msg_result->start_addr_ << ", " << msg_result->count_ << ")";
            return std::vector<int>();
        }

        get_param_time_plot_.shooting(sros::core::util::get_time_in_ms() - cur_timestamp);

        if (msg_result->isError()) {
            LOG(ERROR) << "SRC::getParametersFunc(0x" << std::hex << start_addr << ", count: " << count
                       << " ) -> Failed: error code is " << msg_result->values_.front();
        }

        if (get_param_time_plot_.shoot_count() % (6000 * 2) == 0) {
            LOG(INFO) << get_param_time_plot_;
            LOG(INFO) << set_param_time_plot_;
        }

        return msg_result->values_;
    };

    if (count > MAX_FRAME_VALUE_COUNT) {
        auto end_addr = start_addr + count;
        for (int i = start_addr; i < end_addr; i += MAX_FRAME_VALUE_COUNT) {
            auto c = (end_addr - i > MAX_FRAME_VALUE_COUNT) ? MAX_FRAME_VALUE_COUNT : end_addr - i;

            auto vs = getParametersFunc(i, c, timeout_ms);  // 分块读取

            if (vs.empty()) {
                // 出错情况，返回空列表
                return vector<int>();
            }

            // LOG(INFO) << "append " << vs.size() << " value to values ";
            values.insert(values.end(), vs.begin(), vs.end());  // 将vs插入values后面
        }

        return values;
    }

    return getParametersFunc(start_addr, count, timeout_ms);
}

bool sdk::SrcSdk::setParameter(uint16_t addr, int value, uint32_t timeout_ms, bool try_set) {
    if (!connection_ || !connection_->isConnected()) {
        return false;
    }
    std::unique_lock<std::mutex> ul(com_mutex_, std::defer_lock);
    if (try_set) {
        if (!ul.try_lock()) {
            return false;
        }
    } else {
        ul.lock();
    }

    ParameterMsg_ptr msg_result = std::make_shared<ParameterMsg>();
    reply_msg.reset();

    ParameterMsg_ptr m = std::make_shared<ParameterMsg>();
    m->setValue(addr, value);
    sendMsg(m);

    auto cur_timestamp = sros::core::util::get_time_in_ms();
    if (timeout_ms == 0) {
        timeout_ms = set_single_param_timeout_ms_;
    }

    bool ret = reply_msg.waitForResult(timeout_ms, msg_result);
    if (!ret) {  // 阻塞等待回复
        // 超时
        LOG(ERROR) << "SRC::setParameter() : start_addr " << fhex(4) << addr << ", value " << value
                   << "-> wait timeout: " << timeout_ms;
        return false;
    }
    set_param_time_plot_.shooting(sros::core::util::get_time_in_ms() - cur_timestamp);

    if (msg_result->isError()) {
        LOG(ERROR) << "SRC::setParameter(0x" << std::hex << addr << ", value: " << value
                   << " ) -> Failed: error code is " << msg_result->values_.front();
        return false;
    }

    return true;
}

bool sdk::SrcSdk::setParameters(uint16_t start_addr, const std::vector<int> &values, uint32_t timeout_ms,
                                bool try_set) {
    if (!connection_ || !connection_->isConnected()) {
        return false;
    }

    std::unique_lock<std::mutex> ul(com_mutex_, std::defer_lock);
    if (try_set) {
        if (!ul.try_lock()) {
            LOG(ERROR) << "try_lock failed.";
            return false;
        }
    } else {
        ul.lock();
    }

    uint16_t MAX_FRAME_VALUE_COUNT = 32;
    // v2版本
    if (src_proto_version_ == SRC_PROTO_VERSION_V2) {
        MAX_FRAME_VALUE_COUNT = 60;
    }
    auto total_count = values.size();

    // 每帧数据最多仅能搭载 MAX_FRAME_VALUE_COUNT 个数据，所以此处需要进行分割
    for (uint16_t i = 0; i < total_count; i += MAX_FRAME_VALUE_COUNT) {
        std::vector<int> vs;

        for (uint16_t j = 0; j < MAX_FRAME_VALUE_COUNT && (i + j) < values.size(); j++) {
            vs.push_back(values[i + j]);
        }

        ParameterMsg_ptr msg_result = std::make_shared<ParameterMsg>();
        reply_msg.reset();

        ParameterMsg_ptr m = std::make_shared<ParameterMsg>();
        m->setValues(start_addr + i, vs);
        sendMsg(m);
        
        auto cur_timestamp = sros::core::util::get_time_in_ms();
        if (timeout_ms == 0) {
            timeout_ms = set_multiple_param_timeout_ms_;
        }

        bool ret = reply_msg.waitForResult(timeout_ms, msg_result);
        if (!ret) {  // 阻塞等待回复
            // 超时
            LOG(ERROR) << "SRC::setParameters(0x" << std::hex << start_addr << ", " << vs.size()
                       << " count) -> wait timeout: " << timeout_ms;
            return false;
        }

        set_param_time_plot_.shooting(sros::core::util::get_time_in_ms() - cur_timestamp);
        if (!msg_result->isError()) {
            if (msg_result->values_.size() != vs.size()) {
                LOG(ERROR) << "SRC::setParameters(0x" << std::hex << start_addr << ", " << vs.size()
                           << " count) -> Failed: value count mismatch : " << msg_result->values_.size()
                           << " != " << vs.size();
                return false;
            }

            for (int k = 0; k < msg_result->values_.size(); k++) {
                if (msg_result->values_[k] != vs[k]) {
                    LOG(ERROR) << "SRC::setParameters(0x" << std::hex << start_addr << ", " << vs.size()
                               << " count) -> Failed: value mismatch at " << k << " : " << msg_result->values_[k]
                               << " != " << vs[k];
                    LOG(ERROR) << "vs: " << numberListToStr(vs.cbegin(), vs.cend());
                    LOG(ERROR) << "result: " << numberListToStr(msg_result->values_.cbegin(), msg_result->values_.cend());
                    return false;
                }
            }
        } else {
            LOG(ERROR) << "SRC::setParameters(0x" << std::hex << start_addr << ", " << vs.size()
                       << " count) -> Failed: error code is " << msg_result->values_.front();
            return false;
        }
    }

    return true;
}

bool sdk::SrcSdk::setForkTruckParams(int fork_up_speed_rate, int fork_down_speed_rate, 
                                     int proportional_critical_value, int fork_feed_speed,
                                     int forklift_type, int fork_encoder_resolution, int fork_coupling_speed_threshold) {
    if (src_proto_version_ == SRC_PROTO_VERSION_V2) {   //V2不需要传
        return true;
    }
    const uint16_t ADDR_SET_FORK_TRUCK_PARAMS = 0x1560;
    std::vector<int> values{fork_up_speed_rate, fork_down_speed_rate, proportional_critical_value, 
                            fork_feed_speed,forklift_type,fork_encoder_resolution,fork_coupling_speed_threshold};
                            // fork_feed_speed,forklift_type,fork_encoder_resolution};
    LOG(INFO) << "setForkTruckParameters  up:"<< fork_up_speed_rate 
              << " down:" << fork_down_speed_rate 
              << " proportional:" << proportional_critical_value
              << " fork_feed_speed:" << fork_feed_speed
              << " forklift_type:" << forklift_type
              << " fork_encoder_resolution:" << fork_encoder_resolution
              << " fork_coupling_speed_threshold:" << fork_coupling_speed_threshold;
    return setParameters(ADDR_SET_FORK_TRUCK_PARAMS, values);
}


void sdk::SrcSdk::handleParameterMsg(const std::vector<uint8_t> &data) {
#ifdef SRC_PROTO_DBUG
    LOG(INFO) << "data: " << numberListToStr(data.cbegin(), data.cend());
#endif

    auto msg = make_shared<ParameterMsg>();
    msg->rawData(data);
    msg->decode();
    reply_msg.setResult(std::move(msg));

#ifdef SRC_PROTO_DBUG
    LOG(INFO) << "%%%%%%% Parameter: "
              << "start_addr: " << fhex(4) << msg->start_addr_ << ", "
              << "count: " << msg->count_ << ", "
              << "size: " << msg->values_.size() << ", ";
#endif
}

sdk::EM_BASE_TYPE sdk::SrcSdk::getBaseType() const {
    if(src_proto_version_ == SRC_PROTO_VERSION_V1) {
        return (sdk::EM_BASE_TYPE)sros::core::Settings::getInstance().getValue<int>("src.base_type", 0);
    } else {
        return (sdk::EM_BASE_TYPE)sros::core::Settings::getInstance().getValue<int>("mc.base_type", 0);
    }
}

void sdk::SrcSdk::executeActionInt(uint32_t no, uint16_t id, uint16_t param0, uint16_t param1, int32_t param2) {
    if (src_sdk_proto_do_) {
        uint32_t cmd_param0 = no;
        uint32_t cmd_param1 = id;
        uint32_t cmd_param2 = (((uint32_t)param0) << 16) + param1;
        src_sdk_proto_do_->sendCommandMsgDo(COMMAND_EXECUTE_ACTION, cmd_param0, cmd_param1, cmd_param2, param2);
    }
}

// 更新功能参数
bool sdk::SrcSdk::setFunctionParam(SrcFunctionCode _code, const std::vector<int> &values, uint32_t timeout_ms)
{
    bool bRet = false;
    uint16_t uAddr = getFunctionParamAddr(_code);
    bRet = src_sdk->setParameters(uAddr, values,timeout_ms);
    return bRet;
}

// 获取功能参数
bool sdk::SrcSdk::getFunctionParam(SrcFunctionCode _code, int &value, uint32_t timeout_ms)
{
    bool bRet = false;
    uint16_t uAddr = getFunctionParamAddr(_code);

    if(uAddr == 0x289B + 12) {
        value =  g_state.fork_updown_state;
        return true;
    }

    bRet = src_sdk->getParameter(uAddr, value,timeout_ms);
    return bRet;
}

// 获取功能码地址
uint16_t sdk::SrcSdk::getFunctionParamAddr(SrcFunctionCode _code)
{
    uint16_t uAddr = 0;
    switch (_code)
    {
    case EM_REMOVEPALLET:{
        if (src_proto_version_ == SRC_PROTO_VERSION_V1) {
            uAddr = 0x1245;
        } else if (src_proto_version_ == SRC_PROTO_VERSION_V2) {
            uAddr = 0x2828;
        }
        break;
    }
    case EM_FORKLIFTSTATE:{
        if (src_proto_version_ == SRC_PROTO_VERSION_V1) {
            uAddr = 0x1247;
        } else if (src_proto_version_ == SRC_PROTO_VERSION_V2) {
            uAddr = 0x289B + 12;
        }
        break;
    }
    // case EM_SENSORINSTATE:{
    //     if (src_proto_version_ == SRC_PROTO_VERSION_V1) {
    //         uAddr = 0x1451;
    //     } else if (src_proto_version_ == SRC_PROTO_VERSION_V2) {
    //         uAddr = 0x289B + 17;
    //     }
    //     break;
    // }
    case EM_STEERINGANGLE:{
        if (src_proto_version_ == SRC_PROTO_VERSION_V1) {
            uAddr = 0x1414;
        } else if (src_proto_version_ == SRC_PROTO_VERSION_V2) {
            uAddr = 0x289B + 18;
        }
        break;
    }
    default:
        break;
    }
    return uAddr;
}

// 更新SRTOS版本
void sdk::SrcSdk::updateSrtosVersion(const std::vector<int> &data) 
{
    if (src_device_) 
    {
        src_device_->setModelNo(convertInt32ListToStr(data.cbegin(), data.cbegin() + 2) + "(kernel:" + versionUint2Str(data[7]) + ")");
        src_device_->setVersionNo(versionUint2Str(data.at(2)));
        src_device_->setSerialNo(convertInt32ListToStr(data.cbegin() + 3, data.cbegin() + 7));

        std::time_t build_time = data[15];
        uint32_t fm_boot_version = data[8];
        std::string str_version = "SRTOS_" + versionUint2Str(data[9]) + "(" + int_to_hex(data[10]) + ")[" +
                        convertInt32ListToStr(data.begin() + 11, data.begin() + 15) + "]" +
                        std::asctime(std::localtime(&build_time)) + "(" + versionUint2Str(fm_boot_version) + ")";
        src_device_->setVersionNo(str_version);
        src_device_->setStateOK();

        if (src_sdk_proto_do_) {
            src_sdk_proto_do_->setSRCVersion(data[9]);
            src_sdk_proto_do_->setSRCVersionStr(str_version);
        }
    }    
}

//处理叉车手动按钮信号
void sdk::SrcSdk::handlManualSignal(sros::core::ManualBtnState mode)   //0->自动 1=>手动
{
    if(mode !=  g_state.manual_btn_state){
        if(mode == sros::core::ManualBtnState::BTN_MANUAL){  //manual control
            LOGGER(INFO, DEVICE) << "manual_btn_state: ON.";
            cancleTask("manual_btn");
        }else{  //auto control
            LOGGER(INFO, DEVICE) << "manual_btn_state: OFF.";
            src_sdk->stop();
        }
         g_state.manual_btn_state = mode;
    }
}

//取消当前任务
void sdk::SrcSdk::cancleTask(std::string reason)   //reason: "manual_btn"/""    
{
    auto isMovementRunningFun = []() {
        auto cur_task = sros::core::TaskManager::getInstance()->getMovementTask();
        return cur_task && cur_task->isRunning();
    };

    auto isActionRunningFun = []() {
        auto cur_task = sros::core::TaskManager::getInstance()->getActionTask();
        return cur_task && cur_task->isRunning();
        
    };

    if(isActionRunningFun()){
        auto mm = make_shared<sros::core::CommandMsg>("DEBUG_CMD");
        mm->command = sros::core::CommandType::CMD_CANCEL_ACTION_TASK;
        mm->user_name = reason;
        sros::core::MsgBus::sendMsg(mm);
    }
    if(isMovementRunningFun()){
        auto mm = make_shared<sros::core::CommandMsg>("DEBUG_CMD");
        mm->command = sros::core::CommandType::CMD_CANCEL_MOVEMENT_TASK;
        mm->user_name = reason;
        sros::core::MsgBus::sendMsg(mm);
    }

}

bool sdk::SrcSdk::checkSrcParamters()
{
    auto version = this->getVersion();
    /**
     * 校验流程
    */
    //get data from db.
    bool result = true;
    //等待SRC的参数初始化完成。
    boost::this_thread::sleep_for(boost::chrono::milliseconds(50));

    //checking the data;
    auto compareSrcParameters = [&](std::string class_name, int config_start_id, int src_start_register_addr) -> bool {
        //get data from db.
        auto configs = sros::core::Settings::getInstance().getItemInfoListOfClass(class_name);
        int total_item_counts = configs.back().id - config_start_id + 1;

        // get data from src
        auto src_values = this->getParameters(src_start_register_addr,total_item_counts,5000);
        if (src_values.size() != total_item_counts) {
            LOG(INFO) << "compare " << class_name << " parameters failed : expected size = " << total_item_counts << ",actual size = " << src_values.size();
            return false;
        }

        for (auto i = 0; i < configs.size(); ++i) {
            int value = 0;
            try {
                value = std::stoi(configs[i].value);
            } catch (std::exception &e) {
                LOG(ERROR) << e.what();
                return false;
            }
            
            int offset = configs[i].id - config_start_id;
            if (offset < src_values.size()) {
                if (value != src_values[offset]){
                    LOG(INFO) << "compare " << class_name << " parameters failed by No:"<< configs[i].id << "---db_value:"<< value <<"--src_values:"<< src_values[offset];
                    return false;
                }
            } else {
                LOG(INFO) << "compare " << class_name << " parameters failed by offset = " << offset;
            }
        }
        return true;
    };

    if (version == sdk::SRC_PROTO_VERSION_V1){
        /**
         * check src parameters
        */
        result = compareSrcParameters("ac", 601, SRTOS_ADDR_MC_CONFIG);
        if (!result){
            LOG(INFO) << "src parameters: src. it is failed";
            return result;
        }
    
        LOG(INFO) << "Check SRC Parameters successfully!";
        return result;  
    } 

    /**
    * check srtos parameters
    */
    //get result
    result = compareSrcParameters("ac", SRTOS_CONFIG_AC_ID, SRTOS_ADDR_AC_CONFIG);
    if (!result){
        LOG(INFO) << "srtos parameters :AC. it is failed";
        return result;
    } 

    result = compareSrcParameters("mc", SRTOS_CONFIG_MC_ID, SRTOS_ADDR_MC_CONFIG);
    if (!result){
        LOG(INFO) << "srtos parameters :MC. it is failed";
        return result;
    } 

    result = compareSrcParameters("srtos", SRTOS_CONFIG_START_ID, SRTOS_ADDR_SYSTEM_CONFIG);
    if (!result){
        LOG(INFO) << "srtos parameters :srtos. it is failed";
        return result;
    }

    LOG(INFO) << "Check SRTOS Parameters successfully!";
    return result;
}

/*
兼容之前的setGPIOOuput接口，并提供保护机制
@example1 在当前io输出的基础上,将第地址为i的io输出置为1,地址为j的io输出置为0(需要设置多位时同理):
    uint8_t value_enable = 1 << i;
    uint8_t value_clear = 1 << j;
    setGPIOOuputBits(vaule_clear, value_enable);

@example2 不依赖当前io输出值，将io输出设置为value
    uint8_t value_clear = 0xff;
    uint8_t value_eanble = value;
    setGPIOOuputBits(vaule_clear, value_enable);
*/
void sdk::SrcSdk::setGPIOOuputBits(uint8_t value_clear, uint8_t value_enable)
{
    std::unique_lock<std::mutex> lk(gpio_output_mutex);

    if(last_gpio_output_set == -1) {
        const auto &src_state = src_sdk->getSRCState();
        last_gpio_output_set = (int)src_state.gpio_output; 
    }

    uint8_t new_output_value = (uint8_t)last_gpio_output_set;
    new_output_value &= ~value_clear;
    new_output_value |= value_enable;
    if (new_output_value != (uint8_t)last_gpio_output_set) {
        LOG(INFO) << "ori_output_value = " << (int)last_gpio_output_set;
        LOG(INFO) << "new_output_value = " << (int)new_output_value;
        last_gpio_output_set = new_output_value;
        setGPIOOuput(new_output_value);
    }
}

