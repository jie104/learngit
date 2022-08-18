/**
 * @file network_module.h
 *
 * @author lhx
 * @date 15-12-25.
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef MODULES_NETWORK_NETWORK_MODULE_H_
#define MODULES_NETWORK_NETWORK_MODULE_H_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "core/core.h"
#include "core/mission/mission_manager.h"
#include "core/msg/notification_msg.hpp"
#include "core/navigation_path.h"
#include "core/task/action_task.h"
#include "core/task/movement_task.h"

#include "core/session_manager.h"

#include "../../../sros-file-transmit/Filetransmitter.hpp"
#include "client/client.h"
#include "core/util/async_condition_variable.hpp"
#include "real_time_detect_info.h"
#include "server/server.h"

namespace network {

class NetworkModule : public sros::core::Module {
 public:
    NetworkModule();

    virtual ~NetworkModule() = default;

    virtual void run();

 private:
    void msgCallback(proto::Message_ptr msg, const std::string &peer_ip_str, unsigned short port);

    void msgSendCallback(proto::Message_ptr msg, const std::string &peer_ip_str, unsigned short port);

    void onConnectCallback(std::string peer_ip, unsigned short port);

    void onDisconnectCallback(std::string peer_ip, unsigned short port);

    void notifyConnectedClientCount();

    void runServer();

    void onTimer_100ms(sros::core::base_msg_ptr m);

    void onTimer_1s(sros::core::base_msg_ptr m);

    void handleRequestMonitorData(const proto::Message_ptr &m);

    void onMonitorData(sros::core::base_msg_ptr m);

    bool needUploadLaserPoint();

    bool needUploadFeatureInfo();
    void uploadExtLaserPoint(proto::Message_ptr &msg);
    void uploadObaLaserPoint(proto::Message_ptr &msg);

    void sendLaserPointMsgToNetwork(proto::Message_ptr msg);
    void sendSystemStateToNetwork(proto::Message_ptr msg);
    void sendHardwareStateToNetwork(proto::Message_ptr msg);
    void sendFeatureInfoToNetwork(proto::Message_ptr msg);

    void onLaserMsg(sros::core::base_msg_ptr m);

    void onAvoidObstacleMsg(sros::core::base_msg_ptr m);
    void onObstacleMsg(sros::core::base_msg_ptr m);
    void onLmkMatchMsg(sros::core::base_msg_ptr m);

    void onFeatureInfoMsg(sros::core::base_msg_ptr m);

    void onNewCommonPosesInfoMsg(sros::core::base_msg_ptr m);

    void onSetTimerSendCcommonPosesInfo(sros::core::base_msg_ptr m);

    void onTimer_200ms(sros::core::base_msg_ptr m);

    void onNavPathMsg(sros::core::base_msg_ptr m);

    void onCommandResponseMsg(sros::core::base_msg_ptr m);

    void onNotifyMsg(sros::core::base_msg_ptr m);

    void onCommonNotifyMsg(sros::core::base_msg_ptr m);

    void sendFileThread(sros::ft::Filetransmitter_ptr sender, uint32_t seq, uint64_t session_id);

    bool recvFileThread(sros::ft::Filetransmitter_ptr recver);

    void recvMapFileThread(sros::ft::Filetransmitter_ptr recver, std::string save_path, uint32_t seq,
                           uint64_t session_id);                                                       /* map saving */
    void recvImportMapThread(sros::ft::Filetransmitter_ptr recver, uint32_t seq, uint64_t session_id); /* map import */
    void recvImportUpdateThread(sros::ft::Filetransmitter_ptr recver, uint32_t seq, uint64_t session_id);

    void sendExportMapThread(std::string map_names_str, std::string peer_ip_str, uint32_t seq, uint64_t session_id);

    void FileServerThread(sros::ft::Filetransmitter_ptr server, uint32_t seq, uint64_t session_id);

    void onUpgradeResult(sros::core::base_msg_ptr m);

    network::server::server server_;
    int connected_client_count_;  // 已连接的客户端数目

    bool is_in_file_transmit_;

    void handleRequestCurMap(const proto::Message_ptr &m);

    void handleRequestTimeStamp(const proto::Message_ptr &m);

    void handleRequestMapList(uint32_t req_seq, uint64_t session_id);

    void handleRequestMsg(const proto::Message_ptr &msg, const std::string &peer_ip, unsigned short port);

    void handleCommandMsg(const proto::Message_ptr &msg, const std::string &peer_ip);

    void handleFileOperate(const proto::Message_ptr &msg, const std::string &peer_ip_str);

    void handleMissionTask(const proto::Message_ptr &msg);

    void handleReorderMissionList(const proto::Message_ptr &msg);

    void handleMovementTask(const proto::Message_ptr &msg);

    void handleActionTask(const proto::Message_ptr &msg);

    void convertPath(const sros::core::NavigationPathi &nav_path, proto::Path *pp) const;

    void convertPathReverse(const proto::Path &p, sros::core::NavigationPathi *path) const;

    void handleRequestAllState(uint32_t req_seq, uint64_t session_id);

    void handleRequestHardwareState(uint32_t seq, uint64_t session_id);

    void handleRequestSystemState(uint32_t seq, uint64_t session_id);

    void handleRequestBagList(uint32_t seq, uint64_t session_id);

    void handleRequestFileList(const proto::Message_ptr &msg);

    // 请求任务队列中的任务
    void handleRequestMissionQueue(const proto::Message_ptr &msg);

    void handleRequestInfo(uint32_t req_seq, uint64_t session_id);

    void handleRequestTaskState(uint32_t seq, uint64_t session_id);

    void handleRequestConnectionInfo(uint32_t seq, uint64_t session_id);

    void handleRequestReadInputRegisters(const proto::Message_ptr &msg, uint32_t seq, uint64_t session_id);

    void handleRequestReadAllRegisters(const proto::Message_ptr &msg, uint32_t seq, uint64_t session_id);

    void responseCommandResult(sros::core::CommandType command_type, sros::core::ResultState result_state,
                               uint32_t result_code, uint32_t req_seq, uint64_t session_id);

    void handleSyncTime(const proto::Message_ptr &msg, const uint64_t& receive_cmd_time);

    void handleRequestLoadConfig(uint32_t seq, uint64_t session_id);

    void handleRequestLoadTmpConfig(const proto::Message_ptr &m);
    void handleRequestSaveTmpConfig(const proto::Message_ptr &m);

    void handleRequestSaveConfig(const proto::Message_ptr &msg);

    void handleRequestLogin(const proto::Message_ptr &msg, const std::string &peer_ip, unsigned short port);

    void handleRequestLogout(const proto::Message_ptr &msg);

    void handleChangePassword(const proto::Message_ptr &msg);

    void handleRequestHeatbeat(const proto::Message_ptr &msg);

    void handleNotificationAck(const proto::Message_ptr &msg, const std::string &peer_ip_str);

    void convertMovementTask(const std::shared_ptr<sros::core::MovementTask> &move_task,
                             proto::MovementTask *p_task) const;

    void convertActionTask(const sros::core::ActionTask_ptr &action_task, proto::ActionTask *p_task) const;

    void buildMissionState(sros::core::MissionManager *mission_manager, proto::Mission *running_mission) const;

    bool checklogin(const std::string &username, const std::string &password);

    void sendNotifyMessage(proto::Message_ptr message, sros::core::NotificationMsg::NotifyType type);

    void ResponseFileOperate(proto::ResponseResult::ResultState ret, uint32_t seq, uint64_t session_id,
                             int result_code = sros::core::ERROR_CODE_NONE);

    bool sendMsgToNetwork(proto::Message_ptr msg);

    sros::core::SessionManager &session_manager_;

    std::string monitor_data_;

    bool restoreFactorySettings(uint64_t session_id, uint32_t seq);

    bool backupFactorySettings(uint64_t session_id, uint32_t seq);

    void generateConfigFile();

    void multiLogExport(uint32_t seq, uint64_t session_id);

    void classicLogExport(uint32_t seq, uint64_t session_id);

    bool updateSRCParameters();

    std::vector<std::string> getMapFilesPathByName(const std::string &map_name,
                                                   const proto::Request::FileOperateType &file_op);

    void buildProtoSystemState(proto::SystemState *sys_state) const;

    void buildProtoHardwareState(proto::HardwareState *hw_state);

    void buildTaskState(proto::Response *response_msg) const;

    void buildProtoMission(proto::Mission *proto_mission, sros::core::MissionInstancePtr mission_run_info);

    void updateIfNeedAvoidObstaclePrediction();

    void updateIfNeedDebugInfo();

    void notifyUpdateResult(int32_t ret);

    void onFeatureExtractorDebug(std::string sensor_name);
    void offFeatureExtractorDebug(std::string sensor_name);

    void onUpdateSrcParaMsg(sros::core::base_msg_ptr m);

    AsyncConditionVariable<bool> upgrade_promise_;

    RealTimeDetectInfo real_time_detect_info_;

    sros::core::base_msg_ptr common_poses_info_msg_ptr_ = nullptr;

    int basis_hold_register_num;
    int basis_input_register_num;
    int extend_hold_register_num;
    int extend_input_register_num;
    bool is_seacp;
    bool seacp_enable_extend;

    int manaul_control_time_record = 0;
};

}  // namespace network

#endif  // MODULES_NETWORK_NETWORK_MODULE_H_
