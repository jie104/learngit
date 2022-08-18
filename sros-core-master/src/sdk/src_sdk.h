//
// Created by caoyan on 2020-11-12
//

#ifndef SRC_SDK_DEMO_SRC_H
#define SRC_SDK_DEMO_SRC_H

#include "src_sdk_do.h"

#include <boost/function.hpp>
#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>
#include <future>
#include <list>
#include <mutex>
#include "core/navigation_path.h"
#include "core/pose.h"
#include "core/state.h"
#include "core/device/device.h"

#include "core/hardware/motor.h"
#include "core/task/task.h"
#include "core/usart/connection.hpp"
#include "core/usart/frame_v1.h"
#include "core/util/async_condition_variable.hpp"
#include "core/util/distribution_plot.h"
#include "modules/security/security_state_handle.h"
#include "protocol/all_msg.h"
#include "src_upgrade.h"
#include "core/msg/parameter_msg.hpp"

namespace sdk {

using namespace std;

// 底盘形式枚举
enum EM_BASE_TYPE {
    DIFF = 0x01,  //差速底盘
    MEC,          //麦克纳姆轮底盘
    DIFF_STR,     //舵轮差速底盘
    Slide_Steer,  //四轮滑移转向底盘
    Gulf,
    Jungle,
    Gulf_14,
    Gulf_1400,  //瑞博特叉车底盘模块
    DIFF_3,
    DIFF_4,
    DUAL_STR,
    DIFF_TWIN_BASE,
};
typedef boost::function<void(int&,int&)> CheckVelocityCallbackFunc_t;

class SrcSdk {
 public:
    SrcSdk();
    virtual ~SrcSdk();

    bool connect(std::string port, unsigned int baud_rate);

    void disconnect();

    void reset() { sendCommandMsg(COMMAND_CPU_RESET); }

    void initState();
    void initProtoDo();

    //上层程序回调函数处理
    void setPoseCallback(PoseCallbackFunc_t callback) { SrcSdkDo::pose_callback_f_ = callback; }
    void setOptPoseCallback(OptPoseCallbackFunc_t callback) { SrcSdkDo::opt_pose_callback_f_ = callback; }
    void setStateCallback(StateCallbackFunc_t callback) { SrcSdkDo::state_callback_f_ = callback; }
    void setVelocityCallback(VelocityCallbackFunc_t callback) { SrcSdkDo::velocity_callback_f_ = callback; }
    void setActionStateCallback(ActionStateCallbackFunc_t callback) { SrcSdkDo::action_state_callback_f_ = callback; }
    void setMonitorStateCallback(MonitorStateCallbackFunc_t callback) {
        SrcSdkDo::monitor_state_callback_f_ = callback;
    }
    void setPathFinishCallback(PathFinishCallbackFunc_t callback) { SrcSdkDo::path_finish_callback_f_ = callback; }
    void setPathAbortedCallback(PathAbortedCallbackFunc_t callback) { SrcSdkDo::path_aborted_callback_f_ = callback; }
    void setPathPausedCallback(PathPausedCallbackFunc_t callback) { SrcSdkDo::path_paused_callback_f_ = callback; }
    void setUSARTDataCallback(USARTDataCallbackFunc_t callback) { SrcSdkDo::usart_data_callback_f_ = callback; }
    void setUpgradeCallback(UpgradeCallbackFunc_t callback) { SrcSdkDo::src_upgrade_callback_f_ = callback; }
    void setPoseTimeoutCallback(PoseTimeoutCallbackFunc_t callback) { SrcSdkDo::pose_timeout_callback_f_ = callback; }
    void setCheckManualVelocityCallback(CheckVelocityCallbackFunc_t callback){ checkVelocityCallback = callback; }

    void setPaths(sros::core::NavigationPathi_vector paths);
    void setVelocity(int v, int w);

    void run() {
        sendCommandMsg(COMMAND_RUN);
        run_sport_control_flag_ = true;
    }

    void pauseMovement(int pause_level = 0);
    void continueMovement();
    void stop();

    void navBegin();
    void navFinished();
    // 急停时下发送的暂停命令
    void emergencyPause(SRCEmergencyPauseReason reason,
                        sros::core::EmergencySource source = sros::core::EMERGENCY_SRC_NONE) {
        sendCommandMsg(COMMAND_EMERGENCY_PAUSE, (int32_t)reason, (int32_t)source);
    }
    // 急停取消时发送的继续命令
    void emergencyContinue(SRCEmergencyPauseReason reason) {
        sendCommandMsg(COMMAND_EMERGENCY_CANCEL, (int32_t)reason, 0);
    }

    // 设置抱闸状态
    void setBrakeState(int32_t _state) {
        sendCommandMsg(COMMAND_SET_BRAKE, _state, 0);
    }

    void enterPathMode() { sendCommandMsg(COMMAND_PATH_MODE); }
    void enterVelocityMode() { sendCommandMsg(COMMAND_VELOCITY_MODE); }

    bool upgradeRequest(const std::string &upgrade_bin_file_path);
    void upgradeTest() { sendCommandMsg(COMMAND_UPGRADE_TEST); }
    void upgradSrcTest(); //为了兼容srtos->src 版本回退

    void executeAction(uint32_t no, uint16_t id, int16_t param0, int16_t param1, int16_t param2 = 0);

    void executeActionInt(uint32_t no, uint16_t id, uint16_t param0, uint16_t param1, int32_t param2 = 0);

    void cancelAction(uint32_t no);  // 取消动作任务的执行

    // level范围为1~100(表示最大速度的1%~100%)
    void setSpeedLevel(uint8_t level);

    // 清除当前的所有路径,准备接受新路径
    void clearPath() { sendCommandMsg(COMMAND_CLEAR_PATH); }

    void sendPoseBack(sros::core::Pose pose);  // 发回融合后的pose
    void setPoseAvailable(bool available);     // 设置融合的pose是否可用

    bool getParameter(uint16_t addr, int &value, uint32_t timeout_ms = 50, bool try_get = false);  // 可能会出现获取值失败的情况

    bool getParameterInTime(uint16_t addr, int &value);  //该接口用于管理频繁获取下位机参数的情况

    std::vector<int> getParameters(uint16_t start_addr, uint16_t count, uint32_t timeout_ms = 50, bool try_get = false);
    std::string getParametersString(uint16_t addr, int str_len, bool try_get = false);

    bool setParameter(uint16_t addr, int value, uint32_t timeout_ms = 0, bool try_set = false);
    bool setParameters(uint16_t start_addr, const std::vector<int> &values, uint32_t timeout_ms = 0,
                       bool try_set = false);

    bool setForkTruckParams(int fork_up_speed_rate, int fork_down_speed_rate, int proportional_critical_value, 
							int fork_feed_speed, int forklift_type, int fork_encoder_resolution, int fork_coupling_speed_threshold);

    void resetFault();

    const SRCState &getSRCState();

    const sros::core::Pose &getCurPose();

    uint32_t getSRCVersion() const;
    string getSRCVersionStr() const;

    bool isPathFinished();

    void setGPIOOuputBits(uint8_t value_clear, uint8_t value_enable);

    string getUpPGVScanCode();

    string getDownPGVScanCode();

    void sendCommandMsg(COMMAND_t command, int32_t param0, int32_t param1);

    void sendCommandMsg(COMMAND_t command);

    void setCheckpoint(uint8_t checkpoint_no);

    void sendMsg(src::BaseMsg_ptr msg);
    void sendMsg(const std::vector<uint8_t> &data);

    bool sendIAPdata(const std::vector<uint8_t> &data);

    std::vector<int> getCurPgvInfo();  // 获取实时的pvg信息(id, x, y, yaw)

    /**
     * 设置动作（7,1,0）需要矫正到的目标角度
     * @param pgv_id
     * @param x （m）
     * @param y (m)
     * @param yaw (rad)
     * @return
     */
    bool setTargetDMCodeOffset(int pgv_id, double x, double y, double yaw);  // 设置需要执行的pgv信息

    std::vector<int> getSrcDebugInfo();  // 获取src调试信息

    void sendVelocityBack(sros::core::Velocity velocity);  //发送目标速度给src.

    //以上是原来对外提供的接口，下面是新增的接口
    // new add
    SRC_PROTO_VERSION getVersion() { return src_proto_version_; };
    bool isConnected() const { return connection_ && connection_->isConnected(); }
    /**
     * 设置上视摄像头偏差
     * @param valid 0x00：无效，0x01：有效
     * @param x 	单位0.1mm
     * @param y 	单位0.1mm
     * @param yaw 单位0.0001 rad
     * @param dm_code_info  	上视SR PGV扫码内容 内容字符串强制转为数字
     * @return
     */
    bool setUpCameraDetectInfo(int valid, int x, int y, int yaw, int dm_code_info);
    bool setDownCameraDetectInfo(int valid, int x, int y, int yaw, int dm_code_info);

    bool setPgvExecInfo(double x, double y, double yaw, int enable);

    const sros::device::VirtualDevice_ptr &getSrcVirtualDevicePtr() { return src_device_; }

    bool setTractorDockingDMCodeOffset(double x, double y, double yaw, double deviate);  // 牵引车相对二维码的实时坐标下发

    /**
     * 获取底盘类型
     * @return
     */
    EM_BASE_TYPE getBaseType() const;

    //获取src协议版本号V1/V2
    void getSrcProtoVersion();

    bool isSrtosRollBackSrc();

    bool isInitProtoOk() { return (src_proto_version_ != SRC_PROTO_VERSION_NONE); }

    bool isReconnSrc() { return reconn_src_; }

    bool getRunSportControlFlag() { return run_sport_control_flag_; }
    void offRunSportControlFlag() { run_sport_control_flag_ = false; }

    bool isConnSrcFailFlag() { return is_conn_src_fail_flag_; }
    void setConnSrcFailFlag(bool flag) { is_conn_src_fail_flag_ = flag; }

    bool isSetSrcVersionManual() { return is_set_version_manual_; }

    typedef enum {
        EM_NONE,
        EM_REMOVEPALLET,    //拆栈板
        EM_FORKLIFTSTATE,   //叉臂升降状态
        // EM_SENSORINSTATE,   //传感器输入状态
        EM_STEERINGANGLE,   //轮子转向角度
    }SrcFunctionCode;

    // 设置功能参数
    bool setFunctionParam(SrcFunctionCode _code, const std::vector<int> &values, uint32_t timeout_ms = 0);

    // 获取功能参数
    bool getFunctionParam(SrcFunctionCode _code, int &value, uint32_t timeout_ms = 0);

    // 获取功能码地址
    uint16_t getFunctionParamAddr(SrcFunctionCode _code);

    // 更新SRTOS版本
    void updateSrtosVersion(const std::vector<int> &data);

    //
    void handlManualSignal(sros::core::ManualBtnState mode);   //0->自动 1=>手动
    /**
     * @brief 取消正在执行的任务
     * @param reason 用于在接收指令时判断取消原因，返回对应的任务错误码
     */
    void cancleTask(std::string reason = "");

    bool checkSrcParamters();

    uint8_t setGPIOOuput(uint16_t value, uint16_t mask = 0xFFFF) {
        sendCommandMsg(COMMAND_SET_IO_OUPUT, value, mask);
        return 0;
    }

 protected:
    //事件类型处理
    void handleParameterMsg(const std::vector<uint8_t> &data);

    SRC_PROTO_VERSION src_proto_version_ = SRC_PROTO_VERSION_NONE;

    void onRecvUartData(const std::vector<uint8_t> &data);

    std::shared_ptr<usart::Connection<usart::FrameV1<>>> connection_;  // 串口通信抽象
    
    // 用于同步获取ReplyW（暂废弃）
    AsyncConditionVariable<std::pair<bool, std::vector<int>>> reply_cond_;

    // 将数据以消息结构直接返回
    AsyncConditionVariable<src::ParameterMsg_ptr> reply_msg;

    // sros::device::Device_ptr src_device_;
    sros::device::VirtualDevice_ptr src_device_;

    //发送数据同步
    std::mutex com_mutex_;

    bool sros_pause_ = false;
    mutable std::recursive_mutex pause_mutex;  // 用于暂停继续的锁

    //兼容V1/V2设置参数超时时间不一致
    uint32_t set_single_param_timeout_ms_;
    uint32_t set_multiple_param_timeout_ms_;

    //协议处理
    std::shared_ptr<sdk::SrcSdkDo> src_sdk_proto_do_;

    CheckVelocityCallbackFunc_t checkVelocityCallback;

    DistributionPlot get_param_time_plot_{"获取参数数据回应耗时ms", 1};
    DistributionPlot set_param_time_plot_{"设置参数数据回应耗时ms", 1};

    bool reconn_src_ = false;

    bool run_sport_control_flag_ = false;

    bool is_conn_src_fail_flag_ = false;  //长时间连接不上，是src升级崩溃了

    bool is_set_version_manual_ = false;   //是否强制设置协议版本

    std::mutex gpio_output_mutex; //设置io输出的数据保护锁
    int last_gpio_output_set = -1; //最近一次的io输出值
};

}  // namespace sdk

#endif  // SRC_SDK_DEMO_SRC_H
