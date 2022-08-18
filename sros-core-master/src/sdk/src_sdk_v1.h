//
// Created by caoyan on 2020-11-12
//

#ifndef SROS_SRC_SDK_V1_H
#define SROS_SRC_SDK_V1_H

#include "src_sdk_do.h"

namespace sdk {
class SrcSdkV1 : public SrcSdkDo {
 public:
    SrcSdkV1();
    virtual ~SrcSdkV1(){}

    void init() override;

    void setPaths(sros::core::NavigationPathi_vector paths) override;
    void setVelocity(int v, int w) override;
    void sendVelocityBack(sros::core::Velocity velocity) override;
    void sendPoseBack(sros::core::Pose pose) override;

    void sendCommandMsgDo(COMMAND_t command, int32_t param0, int32_t param1, int32_t param2, int32_t param3) override;
    void executeAction(uint32_t no, uint16_t id, int16_t param0, int16_t param1, int16_t param2) override ;
    void subRecvUartDataHandle(const std::vector<uint8_t> &data) override;

    bool setUpCameraDetectInfo(int valid, int x, int y, int yaw, int dm_code_info) override;
    bool setDownCameraDetectInfo(int valid, int x, int y, int yaw, int dm_code_info) override;
    bool setTargetDMCodeOffset(int pgv_id, double x, double y, double yaw) override;

    bool setTractorDockingDMCodeOffset(double x, double y, double yaw, double deviate) override;

    bool setPgvExecInfo(double x, double y, double yaw, int enable) override;

    std::vector<int> getSrcDebugInfo() override;

    //事件类型处理
    void handleOptPoseMsg(const std::vector<uint8_t> &data) override;
    void handleVelocityMsg(const std::vector<uint8_t> &data) override;
    void handleSignalMsg(const std::vector<uint8_t> &data) override ;
    void handlePoseMsg(const std::vector<uint8_t>& data) override;

 protected:

    void checker();
    void initMotorSerialNo();
    void handleStateMsg(const std::vector<uint8_t> &data);
    void handleInfoMsg(const std::vector<uint8_t> &data);
    void handleActionStateMsg(const std::vector<uint8_t> &data);
    void handleMonitorStateMsg(const std::vector<uint8_t> &data);
    void handleCommandMsg(const std::vector<uint8_t> &data);
    void handleWWDGMsg(const std::vector<uint8_t> &data);

    sros::core::TaskStateStruct srcActionStateToTaskStateStruct(const SRCActionState &action_state) const;

    void updateActionControlDebugInfo();
    void update1353IO();
    void updateForkHeight();
    void updateControlMode();
};
}  // namespace sdk

#endif  // SROS_SRC_SDK_V1_H
