//
// Created by caoyan on 2020-11-12
//

#ifndef SROS_SRC_SDK_V2_H
#define SROS_SRC_SDK_V2_H

#include "src_sdk_do.h"
#include "src_spi_log.hpp"

namespace sdk {

// 配置参数中存存储SRC配置的位置
const int SRTOS_CONFIG_START_ID = 5001;
const int SRTOS_CONFIG_MC_ID = 5301;
const int SRTOS_CONFIG_AC_ID = 5501;

// int单位到STROS的单位，如NavigationPathi_vector为mm，SROT为01.mm
constexpr int INT_UNIT_TO_SRTOS_UNIT(int x) { return x * 10; };
constexpr int SRTOS_UNIT_TO_INT_UNIT(int x) { return x / 10.0; };
constexpr int FLOAT_UNIT_TO_SRTOS_UNIT(double x) { return x * 10000; };
constexpr double SROTS_UNIT_TO_FLOAT_UNIT(int x) { return x / 10000.0; };

class SrcSdkV2 : public SrcSdkDo {
 public:
    explicit SrcSdkV2();
    virtual ~SrcSdkV2();

    void init() override;

    void setPaths(sros::core::NavigationPathi_vector paths) override;
    void setVelocity(int v, int w) override;
    void sendVelocityBack(sros::core::Velocity velocity) override;
    void sendPoseBack(sros::core::Pose pose) override;

    void sendCommandMsgDo(COMMAND_t command, int32_t param0, int32_t param1, int32_t param2, int32_t param3) override;
    void executeAction(uint32_t no, uint16_t id, int16_t param0, int16_t param1, int16_t param2) override;
    void subRecvUartDataHandle(const std::vector<uint8_t> &data) override;

    bool setUpCameraDetectInfo(int valid, int x, int y, int yaw, int dm_code_info) override ;
    bool setDownCameraDetectInfo(int valid, int x, int y, int yaw, int dm_code_info) override;
    bool setTargetDMCodeOffset(int pgv_id, double x, double y, double yaw) override ; // 设置需要执行的pgv信息

    bool setTractorDockingDMCodeOffset(double x, double y, double yaw, double deviate) override;

    bool setPgvExecInfo(double x, double y, double yaw, int enable) override;

    std::vector<int> getSrcDebugInfo() override;

    //事件类型处理
    void handleOptPoseMsg(const std::vector<uint8_t> &data) override;
    void handleSignalMsg(const std::vector<uint8_t>& data) override;
    void handleVelocityMsg(const std::vector<uint8_t> &data) override;
    void handlePoseMsg(const std::vector<uint8_t>& data) override;
    void handler_src_log();

 protected:
    void sendCommandMsgDo(COMMAND_t command, int32_t param0, int32_t param1, int32_t param2, int32_t param3, int32_t param4);
    
    /**
     * 是否为急停相关命令
     */
    bool isEmergencyCmd(COMMAND_t command);

    void syncState();
    bool setSrcConfig();

    void updateStaticSystemState();         // 更新静态的系统状态
    void updateSystemState();
    void updateSportControlState();         // 更新执行移动任务时的状态
    void updateActionControlState();        // 更新执行动作控制任务时的状态
    void updateActionControlCurState();      // 更新当前动作控制状态信息
    void updateDeviceState();               // 更新设备状态
   std::shared_ptr<sdkSrtos::SrcSdkSpiLog> spi_log_;
};

}  // namespace sdk

#endif  // SROS_SRC_SDK_V2_H
