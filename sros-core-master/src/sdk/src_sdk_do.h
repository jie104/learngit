//
// Created by caoyan on 2020-11-12
//

#ifndef SRC_SDK_DO_H
#define SRC_SDK_DO_H

#include <boost/function.hpp>
#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>
#include <list>
#include <mutex>
#include <future>
#include <sstream>
#include "core/navigation_path.h"
#include "core/pose.h"
#include "core/msg/str_msg.hpp"
#include "core/device/device.h"

#include "core/hardware/motor.h"
#include "core/task/task.h"
#include "core/usart/connection.hpp"
#include "core/usart/frame_v1.h"
#include "core/util/async_condition_variable.hpp"
#include "protocol/all_msg.h"
#include "core/util/distribution_plot.h"
#include "src_upgrade.h"
#include "core/hardware/motor.h"
#include "modules/security/security_state_handle.h"
#include "protocol/security_msg.hpp"
#include "core/hardware/Encoder.h"
#include "core/hardware/IOExtend.h"

namespace sdk {

#define fhex(_v) "0x" << std::setw(_v) << std::hex << std::setfill('0')

#define CONST_INT_10 (10)
#define CONST_INT_1000 (1000)
#define CONST_INT_10000 (10000)

#define CONST_FLOAT_10 (10.0)
#define CONST_FLOAT_1000 (1000.0)
#define CONST_FLOAT_10000 (10000.0)

/**
 * 内部src协议版本
 */
enum SRC_PROTO_VERSION {
    SRC_PROTO_VERSION_NONE = 0x00,
    SRC_PROTO_VERSION_V1 = 0x01,
    SRC_PROTO_VERSION_V2 = 0x02,
};

/**
 * 安全单元类型
 */
enum SECURITY_UNIT_TYPE {
    SECURITY_UNIT_TYPE_NONE = 0x00,
    SECURITY_UNIT_TYPE_VSC = 0x01,
    SECURITY_UNIT_TYPE_SRC = 0x02,
    SECURITY_UNIT_TYPE_CE = 0x03,
};
 

/**
 * 上层应用模块回调函数定义
 */
typedef boost::function<void(sros::core::Pose &)> PoseCallbackFunc_t;
typedef boost::function<void(sros::core::Pose)> OptPoseCallbackFunc_t;
typedef boost::function<void(sros::core::Velocity)> VelocityCallbackFunc_t;
typedef boost::function<void(const SRCState &)> StateCallbackFunc_t;
typedef boost::function<void(sros::core::TaskStateStruct)> ActionStateCallbackFunc_t;
typedef boost::function<void(SRCMonitorState)> MonitorStateCallbackFunc_t;

typedef boost::function<void(void)> PathFinishCallbackFunc_t;
typedef boost::function<void(void)> PathAbortedCallbackFunc_t;
typedef boost::function<void(void)> PathPausedCallbackFunc_t;
typedef boost::function<void(std::vector<uint8_t>)> USARTDataCallbackFunc_t;
typedef boost::function<void(int)> UpgradeCallbackFunc_t;
typedef boost::function<void(void)> PoseTimeoutCallbackFunc_t;

using namespace std;



class NotifyQueue {
public:
    NotifyQueue() {}
    virtual ~NotifyQueue() {}
    //static NotifyQueue* getInstance();

    bool put(src::BaseMsg_ptr item);
    src::BaseMsg_ptr get();
   //  src::BaseMsg_ptr try_get();
   //  size_t size() const ;

   //  bool clear();

private:
    mutable std::mutex mutex_;
    std::condition_variable cond_;

    std::list<src::BaseMsg_ptr> queue_;
};


class SrcSdkDo {
 public:
    SrcSdkDo();
    virtual ~SrcSdkDo();

    uint32_t getSRCVersion() const { return src_version_; }
    void setSRCVersion(const uint32_t& _iVersion) {
        src_version_ = _iVersion;
    }
    
    string getSRCVersionStr() const { return src_version_str_;}
    void setSRCVersionStr(const std::string& _str_version) {
        src_version_str_ = _str_version;
    }

    virtual void init() = 0;

    virtual void setPaths(sros::core::NavigationPathi_vector paths) = 0;
    virtual void setVelocity(int v, int w) = 0;
    virtual void sendVelocityBack(sros::core::Velocity velocity) = 0;       //发送目标速度给src.
    virtual void sendPoseBack(sros::core::Pose pose) = 0;                   // 发回融合后的pose

    bool upgradeRequest(const std::string &upgrade_bin_file_path);
    void upgradSrcTest();

    virtual void executeAction(uint32_t no, uint16_t id, int16_t param0, int16_t param1, int16_t param2) = 0;
    virtual void sendCommandMsgDo(COMMAND_t command, int32_t param0, int32_t param1, int32_t param2, int32_t param3) = 0;
    virtual void subRecvUartDataHandle(const std::vector<uint8_t> &data) = 0;  // 没处理完的消息丢给onRecvUartData()来处理

    void setPoseAvailable(bool available);                 // 设置融合的pose是否可用
    const SRCState &getSRCState();
    const sros::core::Pose &getCurPose();
    bool isPathFinished();
    void resetFault();

    virtual bool setTargetDMCodeOffset(int pgv_id, double x, double y, double yaw) = 0; // 设置需要执行的pgv信息
    virtual bool setUpCameraDetectInfo(int valid, int x, int y, int yaw, int dm_code_info) = 0;
    virtual bool setDownCameraDetectInfo(int valid, int x, int y, int yaw, int dm_code_info) = 0;

    virtual bool setTractorDockingDMCodeOffset(double x, double y, double yaw, double deviate) = 0;  // 牵引车相对二维码的坐标

    virtual bool setPgvExecInfo(double x, double y, double yaw, int enable) = 0;

    virtual std::vector<int> getSrcDebugInfo() = 0;  // 获取src调试信息

    //事件类型处理
    virtual void handlePoseMsg(const std::vector<uint8_t>& data) = 0;
    virtual void handleSignalMsg(const std::vector<uint8_t>& data) = 0;
    virtual void handleOptPoseMsg(const std::vector<uint8_t> &data) = 0;
    virtual void handleVelocityMsg(const std::vector<uint8_t> &data) = 0;
    void handleSecurityMsg(const std::vector<uint8_t> &data);
    void handleIAPRequest(const std::vector<uint8_t> &data);
    void setNewStateByNotify();
    
 public:
    static PoseCallbackFunc_t pose_callback_f_;
    static OptPoseCallbackFunc_t opt_pose_callback_f_;
    static StateCallbackFunc_t state_callback_f_;
    static VelocityCallbackFunc_t velocity_callback_f_;
    static ActionStateCallbackFunc_t action_state_callback_f_;
    static MonitorStateCallbackFunc_t monitor_state_callback_f_;
    static PathFinishCallbackFunc_t path_finish_callback_f_;
    static PathAbortedCallbackFunc_t path_aborted_callback_f_;
    static PathPausedCallbackFunc_t path_paused_callback_f_;
    static USARTDataCallbackFunc_t usart_data_callback_f_;
    static UpgradeCallbackFunc_t src_upgrade_callback_f_;
    static PoseTimeoutCallbackFunc_t pose_timeout_callback_f_;

 protected:

    sros::device::Device_ptr getDevice(const string &device_name, bool create_if_not_exist = false) const;
    void updateDeviceState(const string &device_name, uint8_t state, uint32_t error_no = 0) const;

    void updateDeviceSerialNo(const string &name, const string &serial_no) const;

    void dealSrcPoseTimeout();

    void logSendMonitor(std::ostringstream& oss);

    // 处理Src心跳机制
    void dealSrcHeartBeat();

 protected:
    typedef sros::core::Pose Pose;
    typedef sros::core::Rotation Rotation;
    security::SecurityStateHandle security_state_handle_;
    NotifyQueue notify_queue_;

    typedef sros::core::Location Location;

    SRCState state_;

    SRCActionState action_state_;
    uint32_t src_version_;
    string src_version_str_;

    uint32_t odo_version_;

    bool is_path_finished_;  // 是否执行完全部路径
    sros::core::Pose cur_pose_;  // 当前融合后的准确位姿

    bool is_pose_available_;     // 融合位姿是否可用

    uint32_t command_msg_seq_no_;  // CommandMsg的序列号

    uint64_t last_pose_timestamp_;  // 上一次上传pose的时间戳

    uint64_t last_src_pose_serial_; // src上一次上传pose的序号

    DistributionPlot pose_send_back_time_plot_{"将pose回传耗时ms", 5};

    SrcUpgrade src_upgrade_;  // src升级流程

    bool is_reset_fault_ = false; //发送清除电机故障

};

}  // namespace sdk

#endif  // SRC_SDK_DO_H
