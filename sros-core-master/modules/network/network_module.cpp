/**
 * @file network_module.cpp
 *
 * @author lhx
 * @date 15-12-25.
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#include "modules/network/network_module.h"

#include <netdb.h>
#include <time.h>
#include <sys/time.h>
#include <deque>
#include <utility>

#include <core/state.h>
#include <boost/filesystem.hpp>
#include <boost/functional/hash.hpp>

#include "core/msg/ObstacleMsg.hpp"
#include "core/msg/command_msg.hpp"
#include "core/msg/common_command_msg.hpp"
#include "core/msg/common_msg.hpp"
#include "core/msg/common_poses_info_msg.hpp"
#include "core/msg/feature_info_msg.hpp"
#include "core/msg/laser_scan_msg.hpp"
#include "core/msg/path_msg.hpp"
#include "core/msg/str_msg.hpp"

#include "core/map_manager.h"

#include "core/version.h"

#include "core/settings.h"
#include "core/src.h"
#include "core/user_manager.h"

#include "core/device/device_manager.h"
#include "core/log/run_logger.h"
#include "core/monitor/monitor.h"
#include "core/task/task_manager.h"

#include "core/util/md5.h"

#include "core/mission/mission_manager.h"
#include "core/modbus/register_admin.h"
#include "core/util/utils.h"

#include "modules/upgrade/upgrade_module.h"

#include "core/exec_error.hpp"
#include "core/fault_center.h"
#include "protobuf_session_item.hpp"
#include "core/dump.h"

namespace network {

using namespace std;
using namespace sros::core;

#define MAX_TIME_GAP 172800  // two days secs
const char SAVE_MAP_FILE_TEMP_PATH[] = "/tmp/save-map-temp.map";

const char IMPORT_MAP_FILE_PATH[] = "/tmp/import-map.tar.bz";

const char SROS_UPDATE_LOG[] = "/sros/update.log";

const char EXPORT_LOG_FILE_PATH[] = "/tmp/log.tar.bz";
const char EXPORT_MULTI_LOG_FILE_PATH[] = "/tmp/log.tar.bz";
const char EXPORT_MAP_FILE_PATH[] = "/tmp/map.export";

const char FACTORY_SETTINGS_PATH[] = "/sros/factory_settings.tar.bz";

const char MAP_FILE_DIR[] = "/sros/map/";
const char CFG_FILE_DIR[] = "/sros/cfg/";
const char LOG_FILE_DIR[] = "/sros/log/";
const char MONITOR_FILE_DIR[] = "/sros/monitor/";
const char LOG_TEMP_DIR[] = "/tmp/";
const char CONFIG_FILE[] = "/sros/log/cfg.ini";

const char LOG_KEY[] = "SROS2016-log";

NetworkModule::NetworkModule()
    : sros::core::Module("Network"),
      connected_client_count_(0),
      is_in_file_transmit_(false),
      server_("0.0.0.0", "5001"),
      session_manager_(g_state.network_session_manager) {}

void NetworkModule::run() {
    LOG(INFO) << "Module Network is running";

    boost::this_thread::sleep_for(boost::chrono::milliseconds(500));

    subscribeTopic("MONITOR_DATA", CALLBACK(&NetworkModule::onMonitorData));
    // subscribeTopic("TOPIC_OBSTACLE", CALLBACK(&NetworkModule::onObstacleMsg));
    subscribeTopic("OBSTACLES", CALLBACK(&NetworkModule::onObstacleMsg));
    subscribeTopic("AVOID_OBSTACLES", CALLBACK(&NetworkModule::onAvoidObstacleMsg));
    subscribeTopic("TOPIC_LASER", CALLBACK(&NetworkModule::onLaserMsg));
    subscribeTopic("LMK_MATCH_INFO", CALLBACK(&NetworkModule::onLmkMatchMsg));
    subscribeTopic("TOPIC_COMMON_POSES_INFO", CALLBACK(&NetworkModule::onNewCommonPosesInfoMsg));
    subscribeTopic("TOPIC_TIMER_SEND_COMMON_POSES_INFO", CALLBACK(&NetworkModule::onSetTimerSendCcommonPosesInfo));
    subscribeTopic("TOPIC_NOTIFY", CALLBACK(&NetworkModule::onNotifyMsg));
    subscribeTopic("COMMON_NOTIFY", CALLBACK(&NetworkModule::onCommonNotifyMsg));
    subscribeTopic("SCAN_FEATURE_INFO", CALLBACK(&NetworkModule::onFeatureInfoMsg));
    subscribeTopic("CAMERA_FEATURE_INFO", CALLBACK(&NetworkModule::onFeatureInfoMsg));
    

    subscribeTopic("TOPIC_CMD_RESPONSE", CALLBACK(&NetworkModule::onCommandResponseMsg));
    subscribeTopic("TIMER_200MS", CALLBACK(&NetworkModule::onTimer_200ms));
    subscribeTopic("TIMER_100MS", CALLBACK(&NetworkModule::onTimer_100ms));
    subscribeTopic("TIMER_1S", CALLBACK(&NetworkModule::onTimer_1s));
    // subscribeTopic("NAV_PATH", CALLBACK(&NetworkModule::onNavPathMsg));

    subscribeTopic("TOPIC_UPDATA_SROS_REPLY", CALLBACK(&NetworkModule::onUpgradeResult));
    subscribeTopic("TOPIC_UPDATA_SRC_REPLY", CALLBACK(&NetworkModule::onUpgradeResult));
    subscribeTopic("TOPIC_UPDATA_IAP_REPLY", CALLBACK(&NetworkModule::onUpgradeResult));
    subscribeTopic("TOPIC_UPDATE_SRC_PARA", CALLBACK(&NetworkModule::onUpdateSrcParaMsg));

    Dump::getInstance()->init();

    auto &s = sros::core::Settings::getInstance();
    basis_hold_register_num = s.getValue<int>("device.eac_basis_input_registers_num", 50);
    basis_input_register_num = s.getValue<int>("device.eac_basis_hold_registers_num", 50);
    extend_hold_register_num = s.getValue<int>("device.seacp_extend_input_registers_num", 50);
    extend_input_register_num = s.getValue<int>("device.seacp_extend_hold_registers_num", 50);
    is_seacp = (s.getValue<std::string>("device.eac_comm_protocol", "EACP") == "SEACP");
    seacp_enable_extend = (s.getValue<std::string>("device.seacp_enable_extend", "False") == "True");

    boost::thread t(boost::bind(&NetworkModule::runServer, this));

    g_state.updateFleetState();

    dispatch();
}

void NetworkModule::FileServerThread(sros::ft::Filetransmitter_ptr server, uint32_t seq, uint64_t session_id) {
    if (server->start()) {
        ResponseFileOperate(proto::ResponseResult::RESPONSE_OK, seq, session_id);
        LOG(INFO) << "FileServerThread finished";
    } else {
        ResponseFileOperate(proto::ResponseResult::RESPONSE_FAILED, seq, session_id);
        LOG(WARNING) << "FileServerThread failed";
    }
    is_in_file_transmit_ = false;
}

void NetworkModule::sendFileThread(sros::ft::Filetransmitter_ptr sender, uint32_t seq, uint64_t session_id) {
    bool is_sent = sender->start();
    if (is_sent) {
        ResponseFileOperate(proto::ResponseResult::RESPONSE_OK, seq, session_id);
        LOG(INFO) << "sendFileThread finished";
    } else {
        ResponseFileOperate(proto::ResponseResult::RESPONSE_FAILED, seq, session_id);
        LOG(WARNING) << "sendFileThread failed";
    }
    is_in_file_transmit_ = false;
}

bool NetworkModule::recvFileThread(sros::ft::Filetransmitter_ptr recver) {
    bool is_finished = recver->start();
    if (is_finished) {
        DLOG(INFO) << "recvFileThread finished";
        syncDisk();
    }
    return is_finished;
}

void NetworkModule::recvMapFileThread(sros::ft::Filetransmitter_ptr recver, std::string save_path, uint32_t seq,
                                      uint64_t session_id) {
    bool is_finished = recvFileThread(recver);

    if (is_finished) {
        ResponseFileOperate(proto::ResponseResult::RESPONSE_OK, seq, session_id);

        std::string cmd_str = std::string("mv ") + SAVE_MAP_FILE_TEMP_PATH + " " + save_path;

        systemWrapper(cmd_str);

        syncDisk();
    } else {
        ResponseFileOperate(proto::ResponseResult::RESPONSE_FAILED, seq, session_id);
        LOG(WARNING) << "Some errors happened during receiving the file!";
    }

    is_in_file_transmit_ = false;
}

/// 使用新线程执行耗时的压缩打包过程，避免阻塞NetworkModule
void NetworkModule::sendExportMapThread(std::string map_names_str, std::string peer_ip_str, uint32_t seq,
                                        uint64_t session_id) {
    // compress the tar file in Filetransmitter
    std::string command_str = string("tar cvzf ") + EXPORT_MAP_FILE_PATH + " " + map_names_str;

    LOG(INFO) << "\"" << command_str << "\"";

    systemWrapper(command_str);
    // if (!systemWrapper(command_str)) {
    //    LOG(WARNING) <<"Unable to compress the files ! export mapfile failed!";
    //    ResponseFileOperate(proto::ResponseResult::RESPONSE_FAILED, seq, session_id);
    //    return;
    //}

    using namespace sros::ft;
    Filetransmitter_ptr sender(new sros::ft::Filetransmitter(EXPORT_MAP_FILE_PATH, sros::ft::COMPRESS_GZIP,
                                                             sros::ft::SPEED_5Mbps));  // sending
    sendFileThread(sender, seq, session_id);
}

void NetworkModule::recvImportMapThread(sros::ft::Filetransmitter_ptr recver, uint32_t seq, uint64_t session_id) {
    bool is_finished = recvFileThread(recver);
    if (!is_finished) {
        ResponseFileOperate(proto::ResponseResult::RESPONSE_FAILED, seq, session_id);
        LOG(WARNING) << "Some errors happened during receiving the file!";
        return;
    }

    is_in_file_transmit_ = false;

    std::string cmd_str = string("tar xvf ") + IMPORT_MAP_FILE_PATH + " -C / ";

    if (!systemWrapper(cmd_str)) {
        ResponseFileOperate(proto::ResponseResult::RESPONSE_FAILED, seq, session_id);
        return;
    }

    ResponseFileOperate(proto::ResponseResult::RESPONSE_OK, seq, session_id);

    syncDisk();
}

void NetworkModule::recvImportUpdateThread(sros::ft::Filetransmitter_ptr recver, uint32_t seq, uint64_t session_id) {
    namespace fs = boost::filesystem;
    std::string cmd_str = "";
    if (fs::exists(fs::path(sros::IMPORT_UPDATE_DIR))) {
        LOG(INFO) << "Remove " << sros::IMPORT_UPDATE_DIR << " !";
        cmd_str = "rm " + sros::IMPORT_UPDATE_DIR + " -rf";
        systemWrapper(cmd_str);
    }
    LOG(INFO) << "Make new " << sros::IMPORT_UPDATE_DIR << " !";
    cmd_str = "";
    cmd_str = "mkdir " + sros::IMPORT_UPDATE_DIR;
    systemWrapper(cmd_str);

    bool is_finished = recvFileThread(recver);
    if (!is_finished) {
        ResponseFileOperate(proto::ResponseResult::RESPONSE_FAILED, seq, session_id);
        LOG(WARNING) << "Some errors happened during receiving the file!";
        return;
    }

    is_in_file_transmit_ = false;

    upgrade_promise_.reset();

    fs::path update_file_path(sros::IMPORT_UPDATE_FILE_PATH);
    if (fs::file_size(update_file_path) <
        1 * 1024 * 1024) {  // 小于一兆的情况默认为是直接升级src, TODO:以后用HTTP传输文件后将此方法废弃掉
        const std::string SRC_BIN_FILE_PATH =
            sros::IMPORT_UPDATE_DIR +
            "srs-v0.0.0-0000000.bin";  // src_bin的位置，由于protobuf不没有将src升级包的名字传过来的原因
        cmd_str = "mv " + sros::IMPORT_UPDATE_FILE_PATH + " " + SRC_BIN_FILE_PATH + "";
        systemWrapper(cmd_str);

        // 模拟生成md5校验
        cmd_str = "md5sum " + SRC_BIN_FILE_PATH + " > " + SRC_BIN_FILE_PATH + "_md5sum.txt";
        systemWrapper(cmd_str);

        // 通知UpgradeModule对SRC进行升级
        auto upgrade_msg = std::make_shared<sros::core::CommonMsg>("TOPIC_UPDATA_SRC");
        upgrade_msg->str_0_ = SRC_BIN_FILE_PATH;
        sendMsg(upgrade_msg);
    } else {
        auto upgrade_msg = std::make_shared<sros::core::CommonMsg>("TOPIC_UPDATA_SROS");
        upgrade_msg->str_0_ = sros::IMPORT_UPDATE_FILE_PATH;
        sendMsg(upgrade_msg);
    }

    // 等待升级模块回复
    bool exec_succeed = upgrade_promise_.waitResult();
    if (exec_succeed) {
        ResponseFileOperate(proto::ResponseResult::RESPONSE_OK, seq, session_id);
    } else {
        ResponseFileOperate(proto::ResponseResult::RESPONSE_FAILED, seq, session_id);
    }
}

void NetworkModule::onUpgradeResult(sros::core::base_msg_ptr m) {
    auto msg = std::dynamic_pointer_cast<sros::core::CommonMsg>(m);
    upgrade_promise_.setResult(msg->int_0_ == -1 ? false : true);
}

void convertPose(const proto::Pose &proto_pose, sros::core::Pose *p) {
    p->x() = static_cast<double>(proto_pose.x()) / 1000.0;
    p->y() = static_cast<double>(proto_pose.y()) / 1000.0;
    p->z() = static_cast<double>(proto_pose.z()) / 1000.0;

    p->roll() = proto_pose.roll() / 1000.0;
    p->pitch() = proto_pose.pitch() / 1000.0;
    p->yaw() = proto_pose.yaw() / 1000.0;
}

void convertPose(const sros::core::Pose &p, proto::Pose *proto_pose) {
    proto_pose->set_x(static_cast<int>(p.x() * 1000));
    proto_pose->set_y(static_cast<int>(p.y() * 1000));
    proto_pose->set_z(static_cast<int>(p.z() * 1000));

    proto_pose->set_roll(static_cast<int>(p.roll() * 1000));
    proto_pose->set_pitch(static_cast<int>(p.pitch() * 1000));
    proto_pose->set_yaw(static_cast<int>(p.yaw() * 1000));

    proto_pose->set_confidence(static_cast<int>(p.confidence() * 100));  // confidence = 0% ~ 100%
}

void NetworkModule::msgSendCallback(proto::Message_ptr msg, const std::string &peer_ip, unsigned short port) {
    // update live session list according to ip and port

    if (msg->session_id() == 0 && msg->type() == proto::MSG_RESPONSE) {
        auto response = msg->mutable_response();
        if (response->response_type() != proto::Response::RESPONSE_SYSTEM_STATE &&
            response->response_type() != proto::Response::RESPONSE_LASER_POINTS &&
            response->response_type() != proto::Response::RESPONSE_HARDWARE_STATE &&
            response->response_type() != proto::Response::RESPONSE_MONITOR_DATA) {
            LOGGER(WARNING, PROTOBUF) << "msg with invalid session id msg_type = " << msg->type()
                                      << " response type = " << response->response_type();
        }

        // if (response->response_type() == proto::Response::RESPONSE_COMMAND) {
        //    //LOG(WARNING)<<" COMMAND RESPONSE "<<"msg_seq = "<<msg->seq()<<" session_id = "<<msg->session_id();
        //}

        // if (response->response_type() == proto::Response::RESPONSE_SYSTEM_STATE) {
        //    LOG(WARNING)<<" SYSTEM STATE UPDATE sent to "<<peer_ip<<","<<port;
        //}
    }

    if (msg->type() == proto::MSG_NOTIFICATION) {
        auto notify = msg->mutable_notification();
        LOGGER(INFO, PROTOBUF) << "Notification " << notify->notify_type() << " is sent! "
                               << "seq=" << msg->seq() << " session_id=" << msg->session_id();
    } else if (msg->type() == proto::MSG_RESPONSE) {
        auto response = msg->mutable_response();
        if (response->response_type() == proto::Response::RESPONSE_MONITOR_DATA) {
            LOGGER(INFO, PROTOBUF) << "Response(monitor data) " << response->response_type() << " is sent!";
        }
    }
}

void NetworkModule::msgCallback(proto::Message_ptr msg, const std::string &peer_ip_str, unsigned short port) {
    auto session_id = msg->session_id();
    auto m = msg->mutable_request();
    auto session = session_manager_.getItem(session_id);
    if (!session) {
        // 只处理LOGIN请求
        if (msg->type() == proto::MSG_REQUEST && m->request_type() == proto::Request::REQUEST_LOGIN) {
            LOGGER(INFO, PROTOBUF) << "Client " << peer_ip_str << " request login!";
        } else {
            // shutdown the connection if session id is null
            server_.close(peer_ip_str, port);
            LOGGER(ERROR, PROTOBUF) << "session id is null, illegal request or command!";
            return;
        }
    } else {
        if (session->username == USERNAME_FMS) {
            session->updateAliveTime();
        }
    }

    //    LOG(INFO) <<"recv msg seq("<<msg->seq()<<") session_id("<<msg->session_id()<<") msg type("<<msg->type()<<")";

    switch (msg->type()) {
        case ::proto::MSG_REQUEST: {
            handleRequestMsg(msg, peer_ip_str, port);
            break;
        }
        case ::proto::MSG_COMMAND: {
            handleCommandMsg(msg, peer_ip_str);
            break;
        }
        case ::proto::MSG_RESPONSE: {
            // only for ack notification
            handleNotificationAck(msg, peer_ip_str);
            break;
        }
        default:
            break;
    }
}

void NetworkModule::handleCommandMsg(const proto::Message_ptr &msg, const std::string &peer_ip) {
    auto seq = msg->seq();  // 请求序列号
    auto session_id = msg->session_id();

    if (!msg->has_command()) {
        LOGGER(ERROR, PROTOBUF) << "Handle command msg: command is none!";
        return;
    }

    auto m = msg->mutable_command();

    switch (m->command()) {
        case proto::CMD_START_MISSION: {
            LOGGER(INFO, PROTOBUF) << "CMD_START_MISSION";
            handleMissionTask(msg);
            g_state.fresh_state = sros::core::FRESH_NO;
            break;
        }
        case proto::CMD_CANCEL_MISSION: {
            LOGGER(INFO, PROTOBUF) << "CMD_CANCEL_MISSION: mission_id=" << m->param_str();
            auto mm = make_shared<sros::core::CommandMsg>(getName());
            mm->req_seq = seq;
            mm->session_id = session_id;
            mm->command = sros::core::CMD_CANCEL_MISSION;
            mm->mission_no = std::stoll(m->param_str());
            sendMsg(mm);
            break;
        }
        case proto::CMD_CONTINUE_MISSION: {
            LOGGER(INFO, PROTOBUF) << "CMD_CONTINUE_MISSION";
            auto mm = make_shared<sros::core::CommandMsg>(getName());
            mm->req_seq = seq;
            mm->session_id = session_id;
            mm->command = sros::core::CMD_CONTINUE_MISSION;
            sendMsg(mm);
            break;
        }
        case proto::CMD_REORDER_MISSION: {
            LOGGER(INFO, PROTOBUF) << "CMD_RECORDER_MISSION";
            handleReorderMissionList(msg);
            break;
        }
        case proto::CMD_NEW_MOVEMENT_TASK: {
            LOGGER(INFO, PROTOBUF) << "CMD_NEW_MOVEMENT_TASK";
            g_state.enable_back_main_laser_oba = true; //防止动作中关闭避障没有恢复，（后续优化：动作后需要恢复的东西需要一个统一的地方处理）
            handleMovementTask(msg);
            g_state.fresh_state = sros::core::FRESH_NO;
            break;
        }
        case proto::CMD_PATH_REPLACE: {
            LOGGER(INFO, PROTOBUF) << "CMD_PATH_REPLACE";
            auto mm = make_shared<sros::core::CommandMsg>(getName());
            mm->req_seq = seq;
            mm->session_id = session_id;
            mm->command = sros::core::CMD_PATH_REPLACE;
            sros::core::NavigationPathi_vector dst_paths;

            if (m->paths().empty()) {
                LOGGER(ERROR, PROTOBUF) << "Handling CMD_PATH_REPLACE command. the paths is empty. please check the protbuf data";
                break;
            }

            for (auto p : m->paths()) {
                sros::core::NavigationPathi path;

                convertPathReverse(p, &path);

                dst_paths.push_back(path);
            }
            mm->paths = dst_paths;
            sendMsg(mm);
            break;
        }
        case proto::CMD_NEW_ACTION_TASK: {
            LOGGER(INFO, PROTOBUF) << "CMD_NEW_ACTION_TASK";
            handleActionTask(msg);
            g_state.fresh_state = sros::core::FRESH_NO;
            break;
        }
        case proto::CMD_SYNC_TIME: {
            struct timeval curTime;
            gettimeofday(&curTime, NULL);
            uint64_t receive_time= (uint64_t)curTime.tv_sec * 1000 + (uint64_t)curTime.tv_usec / 1000;
            LOGGER(INFO, CMD_HANDER) << "Handling CMD_SYNC_TIME command";
            if (!g_state.control_mutex.get(session_id)) {
                auto locker = g_state.control_mutex.getLocker();
                SET_ERROR(ERROR_CODE_CONTROL_MUTEX_IS_LOCKED, "locker:", locker.session_id, "current:", session_id);
                responseCommandResult(sros::core::CMD_SYNC_TIME, sros::core::RESPONSE_FAILED,
                                      sros::core::ERROR_CODE_CONTROL_MUTEX_IS_LOCKED, msg->seq(), msg->session_id());
                return;
            }

            if (MissionManager::getInstance()->isMissionRunning()) {
                LOG(WARNING) << "Mission is running, can not sync time.";

                SET_ERROR(ERROR_CODE_SYNC_TIME_MISSION_RUNNING, "Mission is running, can not sync time");
                responseCommandResult(sros::core::CMD_SYNC_TIME, sros::core::RESPONSE_FAILED,
                                      sros::core::ERROR_CODE_SYNC_TIME_MISSION_RUNNING, msg->seq(), msg->session_id());
                return;
            }

            if (Settings::getInstance().getValue<string>("time.enable_ntp_sync", "False") == "True") {
                SET_ERROR(ERROR_CODE_SYNC_TIME_NTP_ENABLE, "NTP is enabled!");
                responseCommandResult(sros::core::CMD_SYNC_TIME, sros::core::RESPONSE_FAILED,
                                      sros::core::ERROR_CODE_SYNC_TIME_NTP_ENABLE, msg->seq(), msg->session_id());
                return;
            }

            if (g_state.main_state == sros::core::STATE_IDLE) {
                handleSyncTime(msg,receive_time);
            } else {
                SET_ERROR(ERROR_CODE_SYNC_TIME_SYSTEM_NOT_IDLE,
                          "cannt handle SYNC_TIME when system state is not IDLE:");
                responseCommandResult(sros::core::CMD_SYNC_TIME, sros::core::RESPONSE_FAILED,
                                      sros::core::ERROR_CODE_SYNC_TIME_SYSTEM_NOT_IDLE, msg->seq(), msg->session_id());
            }
            break;
        }
        case proto::CMD_ENABLE_AUTO_UPLOAD_LASER_POINT: {
            LOGGER(INFO, CMD_HANDER) << "Handling CMD_ENABLE_AUTO_UPLOAD_LASER_POINT command";

            auto session = dynamic_pointer_cast<ProtobufSessionItem>(session_manager_.getItem(msg->session_id()));
            if (session) {
                session->is_upload_laser_point_ = true;
            }

            responseCommandResult(sros::core::CMD_CANCEL_ACTION_TASK, sros::core::RESPONSE_OK,
                                  sros::core::ERROR_CODE_NONE, msg->seq(), msg->session_id());
            break;
        }
        case proto::CMD_DISABLE_AUTO_UPLOAD_LASER_POINT: {
            LOGGER(INFO, CMD_HANDER) << "Handling CMD_DISABLE_AUTO_UPLOAD_LASER_POINT command";

            auto session = dynamic_pointer_cast<ProtobufSessionItem>(session_manager_.getItem(msg->session_id()));
            if (session) {
                session->is_upload_laser_point_ = false;
            }

            responseCommandResult(sros::core::CMD_CANCEL_ACTION_TASK, sros::core::RESPONSE_OK,
                                  sros::core::ERROR_CODE_NONE, msg->seq(), msg->session_id());
            break;
        }
        case proto::CMD_ENABLE_UPLOAD_AVOID_OBSTACLE_PREDICTION: {
            LOGGER(INFO, CMD_HANDER) << "Handling CMD_ENABLE_UPLOAD_AVOID_OBSTACLE_PREDICTION command";

            auto session = dynamic_pointer_cast<ProtobufSessionItem>(session_manager_.getItem(msg->session_id()));
            if (session) {
                session->is_upload_avoid_obstacle_prediction_ = true;
                g_state.need_avoid_obstacle_prediction = true;
            }

            responseCommandResult(sros::core::CMD_CANCEL_ACTION_TASK, sros::core::RESPONSE_OK,
                                  sros::core::ERROR_CODE_NONE, msg->seq(), msg->session_id());
            break;
        }
        case proto::CMD_DISABLE_UPLOAD_AVOID_OBSTACLE_PREDICTION: {
            LOGGER(INFO, CMD_HANDER) << "Handling CMD_DISABLE_UPLOAD_AVOID_OBSTACLE_PREDICTION command";

            auto session = dynamic_pointer_cast<ProtobufSessionItem>(session_manager_.getItem(msg->session_id()));
            if (session) {
                session->is_upload_avoid_obstacle_prediction_ = false;
                updateIfNeedAvoidObstaclePrediction();
            }

            responseCommandResult(sros::core::CMD_CANCEL_ACTION_TASK, sros::core::RESPONSE_OK,
                                  sros::core::ERROR_CODE_NONE, msg->seq(), msg->session_id());
            break;
        }
        case proto::CMD_ENABLE_DEBUG_INFO: {
            LOGGER(INFO, CMD_HANDER) << "Handling CMD_ENABLE_DEBUG_INFO command";

            auto session = dynamic_pointer_cast<ProtobufSessionItem>(session_manager_.getItem(msg->session_id()));
            if (session) {
                session->need_debug_info_ = true;
                g_state.setNeedDebugInfo(true);
            }

            responseCommandResult(sros::core::CMD_CANCEL_ACTION_TASK, sros::core::RESPONSE_OK,
                                  sros::core::ERROR_CODE_NONE, msg->seq(), msg->session_id());
            break;
        }
        case proto::CMD_DISABLE_DEBUG_INFO: {
            LOGGER(INFO, CMD_HANDER) << "Handling CMD_DISABLE_DEBUG_INFO command";

            auto session = dynamic_pointer_cast<ProtobufSessionItem>(session_manager_.getItem(msg->session_id()));
            if (session) {
                session->need_debug_info_ = false;
                updateIfNeedDebugInfo();
            }

            responseCommandResult(sros::core::CMD_CANCEL_ACTION_TASK, sros::core::RESPONSE_OK,
                                  sros::core::ERROR_CODE_NONE, msg->seq(), msg->session_id());
            break;
        }
        case proto::CMD_ENABLE_DEBUG_DATA: {
            LOGGER(INFO, CMD_HANDER) << "Handing CMD_ENABLE_DEBUG_DATA command";
            auto session = dynamic_pointer_cast<ProtobufSessionItem>(session_manager_.getItem(msg->session_id()));

            std::string device_name = m->param_str();
            LOGGER(INFO, CMD_HANDER) << "debug_data device_name: " << device_name;

            if (device_name == sros::device::DEVICE_ETH0) {
                Dump::getInstance()->dumpTcpEth0();
            } else if (device_name == sros::device::DEVICE_ENP3S0) {
                Dump::getInstance()->dumpTcpEnp3s0();
            } else if (device_name == sros::device::DEVICE_CAN) {
                Dump::getInstance()->dumpCan();
            } else if (device_name == sros::device::DEVICE_SRC) {
                Dump::getInstance()->dumpSrc();
            } else if (device_name == sros::device::DEVICE_VSC) {
                Dump::getInstance()->dumpVsc();
            }

            responseCommandResult(sros::core::CMD_CANCEL_ACTION_TASK, sros::core::RESPONSE_OK,
                                  sros::core::ERROR_CODE_NONE, msg->seq(), msg->session_id());
            break;
        }
        case proto::CMD_DISABLE_DEBUG_DATA: {
            LOGGER(INFO, CMD_HANDER) << "Handing CMD_DISABLE_DEBUG_DATA command";
            auto session = dynamic_pointer_cast<ProtobufSessionItem>(session_manager_.getItem(msg->session_id()));

            std::string device_name = m->param_str();
            LOGGER(INFO, CMD_HANDER) << "debug_data device_name: " << device_name;

            if (device_name == sros::device::DEVICE_ETH0) {
                Dump::getInstance()->stopDumpTcpEth0();
            } else if (device_name == sros::device::DEVICE_ENP3S0) {
                Dump::getInstance()->stopDumpTcpEnp3s0();
            } else if (device_name == sros::device::DEVICE_CAN) {
                Dump::getInstance()->stopDumpCan();
            } else if (device_name == sros::device::DEVICE_SRC) {
                Dump::getInstance()->stopDumpSrc();
            } else if (device_name == sros::device::DEVICE_VSC) {
                Dump::getInstance()->stopDumpVsc();
            }

            responseCommandResult(sros::core::CMD_CANCEL_ACTION_TASK, sros::core::RESPONSE_OK,
                                  sros::core::ERROR_CODE_NONE, msg->seq(), msg->session_id());
            break;
        }

	// feature
        case proto::CMD_ENABLE_DEBUG_FEATURE: {
            LOGGER(INFO, CMD_HANDER) << "Handing CMD_ENABLE_DEBUG_FEATURE command";
            auto session = dynamic_pointer_cast<ProtobufSessionItem>(session_manager_.getItem(msg->session_id()));

            std::string sensor_name = m->param_str();
            LOGGER(INFO, CMD_HANDER) << "feature sensor_name: " << sensor_name;

            if (sensor_name == sros::device::DEVICE_LIDAR) {
                session->need_debug_lidar_fr_ = true;
            } else if (sensor_name == sros::device::DEVICE_CAMERA_FORWARD) {
                session->need_debug_front_camera_fr_ = true;
            } else if (sensor_name == sros::device::DEVICE_CAMERA_BACKWARD) {
                session->need_debug_back_camera_fr_ = true;
            } else if (sensor_name == sros::device::DEVICE_SVC100_UP) {
                session->need_debug_up_camera_fr_ = true;
            } else if (sensor_name == sros::device::DEVICE_SVC100_DOWN) {
                session->need_debug_down_camera_fr_ = true;
            } else if (sensor_name == sros::device::DEVICE_CAMERA_LEFT) {
                session->need_debug_left_camera_fr_ = true;
            } else if (sensor_name == sros::device::DEVICE_CAMERA_RIGHT) {
                session->need_debug_right_camera_fr_ = true;
            }

            onFeatureExtractorDebug(sensor_name);
            responseCommandResult(sros::core::CMD_CANCEL_ACTION_TASK, sros::core::RESPONSE_OK,
                                  sros::core::ERROR_CODE_NONE, msg->seq(), msg->session_id());
            break;
        }

        case proto::CMD_DISABLE_DEBUG_FEATURE: {
            LOGGER(INFO, CMD_HANDER) << "Handing CMD_DISABLE_DEBUG_FEATURE command";
            auto session = dynamic_pointer_cast<ProtobufSessionItem>(session_manager_.getItem(msg->session_id()));

            std::string sensor_name = m->param_str();
            LOGGER(INFO, CMD_HANDER) << "feature sensor_name: " << sensor_name;

            if (sensor_name == sros::device::DEVICE_LIDAR) {
                session->need_debug_lidar_fr_ = false;
            } else if (sensor_name == sros::device::DEVICE_CAMERA_FORWARD) {
                session->need_debug_front_camera_fr_ = false;
            } else if (sensor_name == sros::device::DEVICE_CAMERA_BACKWARD) {
                session->need_debug_back_camera_fr_ = false;
            } else if (sensor_name == sros::device::DEVICE_SVC100_UP) {
                session->need_debug_up_camera_fr_ = false;
            } else if (sensor_name == sros::device::DEVICE_SVC100_DOWN) {
                session->need_debug_down_camera_fr_ = false;
            } else if (sensor_name == sros::device::DEVICE_CAMERA_LEFT) {
                session->need_debug_left_camera_fr_ = false;
            } else if (sensor_name == sros::device::DEVICE_CAMERA_RIGHT) {
                session->need_debug_right_camera_fr_ = false;
            }

            offFeatureExtractorDebug(sensor_name);
            responseCommandResult(sros::core::CMD_CANCEL_ACTION_TASK, sros::core::RESPONSE_OK,
                                  sros::core::ERROR_CODE_NONE, msg->seq(), msg->session_id());
            break;
        }

        default: {
            // proto::CommandType::core::DebugCmd定义相同
            auto cmd = (sros::core::CommandType)m->command();
            LOGGER(INFO, PROTOBUF) << "cmd:" << cmd;

            auto mm = make_shared<sros::core::CommandMsg>(getName());
            mm->req_seq = seq;
            mm->session_id = session_id;
            mm->command = cmd;
            mm->map_name = m->param_str();
            mm->param0 = m->param_int();
            mm->param1 = m->param_int1();
            mm->locker_ip_address = m->locker_ip_address();
            mm->locker_nickname = m->locker_nickname();
            mm->param_boolean = m->param_boolean();

            sros::core::Pose initial_pose;
            convertPose(m->pose(), &initial_pose);
            mm->pose = initial_pose;

            sendMsg(mm);

            break;
        }
    }
}

void NetworkModule::handleSyncTime(const proto::Message_ptr &msg, const uint64_t& receive_cmd_time) {
    auto mm = msg->mutable_command();

    // new_time是UTC下的时间戳
    //int64_t new_time = stoll(mm->param_str()) / 1000;  // long long int atoll()
    int64_t new_time = stoll(mm->param_str());  // long long int atoll()

    // stop timer module before configure system time
    auto msgs = make_shared<sros::core::StrMsg>("STOP_TIMER");
    sendMsg(msgs);
    boost::this_thread::sleep_for(
        boost::chrono::milliseconds(100));  // 等待定时任务处理玩后再同步时间，比如：等待记录总运行

    //time_t tick = (time_t)new_time;
    //char s[100];
    //strftime(s, sizeof(s), "%Y-%m-%d %H:%M:%S", std::localtime(&tick));

    // sync RTC，保证写入rtc的是UTC时间
    //boost::this_thread::sleep_for(boost::chrono::milliseconds(1000));
    // std::string cmd = "timedatectl set-time '" + std::string(s) + "'";
    //bool ret = systemWrapper(cmd);
    struct timeval curTime;
    gettimeofday(&curTime, NULL);
    uint64_t nowtime= (uint64_t)curTime.tv_sec * 1000 + (uint64_t)curTime.tv_usec / 1000;
    new_time += nowtime - receive_cmd_time;
    curTime.tv_sec = new_time / 1000;
    curTime.tv_usec = (new_time % 1000) * 1000;
    bool ret = (settimeofday(&curTime, NULL) == 0);
    LOG(INFO) << "spend time: " << nowtime - receive_cmd_time;
    if (ret) {
        // restart sros, time associate too many things, sros must reset when the time changed!
        auto reset_msg = std::make_shared<sros::core::CommandMsg>(getName());
        reset_msg->command = sros::core::CMD_RESET_SROS;
        sendMsg(reset_msg);

        LOGGER(INFO, CMD_HANDER) << "time sync success!";
        responseCommandResult(sros::core::CMD_SYNC_TIME, sros::core::RESPONSE_OK, sros::core::ERROR_CODE_NONE,
                              msg->seq(), msg->session_id());
    } else {
        // 不管时间同步成功还是失败都得重启，应为timer已经不对了
        auto reset_msg = std::make_shared<sros::core::CommandMsg>(getName());
        reset_msg->command = sros::core::CMD_RESET_SROS;
        sendMsg(reset_msg);

        SET_ERROR(ERROR_CODE_UNDEFINED, "time sync failed!");
        responseCommandResult(sros::core::CMD_SYNC_TIME, sros::core::RESPONSE_FAILED, sros::core::ERROR_CODE_UNDEFINED,
                              msg->seq(), msg->session_id());
    }
}

void NetworkModule::handleRequestMsg(const proto::Message_ptr &msg, const std::string &peer_ip, unsigned short port) {
    auto seq = msg->seq();  // 请求序列号
    auto session_id = msg->session_id();

    auto m = msg->mutable_request();

    switch (m->request_type()) {
        case proto::Request::REQUEST_LOGIN: {
            LOGGER(INFO, PROTOBUF) << "REQUEST_LOGIN, session id is " << session_id;
            handleRequestLogin(msg, peer_ip, port);
            break;
        }
        case proto::Request::REQUEST_HEARTBEAT: {
            //            LOGGER(INFO, PROTOBUF) << "REQUEST_HEARTBEAT"; // 心跳会一直收到，所以不打印
            handleRequestHeatbeat(msg);
            break;
        }
        case proto::Request::REQUEST_CHANGE_PW: {
            LOGGER(INFO, PROTOBUF) << "REQUEST_CHANGE_PASSWORD, session id is " << session_id;
            handleChangePassword(msg);
            break;
        }
        case proto::Request::REQUEST_INFO: {
            LOGGER(INFO, PROTOBUF) << "REQUEST_INFO, session id is " << session_id;
            handleRequestInfo(seq, session_id);
            break;
        }
        case proto::Request::REQUEST_ALL_STATE: {
            LOGGER(INFO, PROTOBUF) << "REQUEST_ALL_STATE, session id is " << session_id;
            handleRequestAllState(seq, session_id);
            break;
        }
        case proto::Request::REQUEST_HARDWARE_STATE: {
            //            LOGGER(INFO, PROTOBUF) << "REQUEST_HARDWARE_STATE, session id is " << session_id;
            handleRequestHardwareState(seq, session_id);
            break;
        }
        case proto::Request::REQUEST_SYSTEM_STATE: {
            //                        LOGGER(INFO, PROTOBUF) << "REQUEST_SYSTEM_STATE, session id is " << session_id;
            handleRequestSystemState(seq, session_id);
            break;
        }
        case proto::Request::REQUEST_MAP_LIST: {
            LOGGER(INFO, PROTOBUF) << "REQUEST_MAP_LIST, session id is " << session_id;
            handleRequestMapList(seq, session_id);
            LOG(INFO) << "REQUEST_MAP_LIST end";
            break;
        }
        case proto::Request::REQUEST_BAG_LIST: {
            LOGGER(INFO, PROTOBUF) << "REQUEST_BAG_LIST, session id is " << session_id;
            handleRequestBagList(seq, session_id);
            break;
        }
        case proto::Request::REQUEST_LASER_POINTS: {
            LOGGER(INFO, PROTOBUF) << "REQUEST_LASER_POINTS, this request is abandoned, session id is " << session_id;
            break;
        }
        case proto::Request::REQUEST_TASK_STATE: {
            //            LOGGER(INFO, PROTOBUF) << "TASK_STATE_REQUEST, session id is " << session_id;
            handleRequestTaskState(seq, session_id);
            break;
        }
        case proto::Request::REQUEST_FILE_OPERATE: {
            LOGGER(INFO, PROTOBUF) << "REQUEST_FILE_OPERATE, session id is " << session_id;
            handleFileOperate(msg, peer_ip);
            break;
        }
        case proto::Request::REQUEST_LOGOUT: {
            LOGGER(INFO, PROTOBUF) << "REQUEST_LOGOUT, session id is " << session_id;
            handleRequestLogout(msg);
            break;
        }
        case proto::Request::REQUEST_LOAD_CONFIG: {
            LOGGER(INFO, PROTOBUF) << "REQUEST_LOAD_CONFIG, session id is " << session_id;
            handleRequestLoadConfig(seq, session_id);
            break;
        }
        case proto::Request::REQUEST_SAVE_CONFIG: {
            LOGGER(INFO, PROTOBUF) << "REQUEST_SAVE_CONFIG, session id is " << session_id;
            // 操作耗时较长(~5s)，在独立线程中执行
            boost::thread t(boost::bind(&NetworkModule::handleRequestSaveConfig, this, msg));
            break;
        }
        case proto::Request::REQUEST_LOAD_TMP_CONFIG: {
            LOGGER(INFO, PROTOBUF) << "REQUEST_LOAD_TMP_CONFIG, session id is " << session_id;
            handleRequestLoadTmpConfig(msg);
            break;
        }
        case proto::Request::REQUEST_SAVE_TMP_CONFIG: {
            LOGGER(INFO, PROTOBUF) << "REQUEST_SAVE_TMP_CONFIG, session id is " << session_id;
            handleRequestSaveTmpConfig(msg);
            break;
        }
        case proto::Request::REQUEST_FILE_LIST: {
            LOGGER(INFO, PROTOBUF) << "REQUEST_FILE_LIST, session id is " << session_id;
            handleRequestFileList(msg);
            break;
        }
        case proto::Request::REQUEST_MONITOR_DATA: {
            LOGGER(INFO, PROTOBUF) << "REQUEST_MONITOR_DATA, session id is " << session_id;
            handleRequestMonitorData(msg);
            break;
        }
        case proto::Request::REQUEST_TIMESTAMP: {
            LOGGER(INFO, PROTOBUF) << "REQUEST_TIMESTAMP, session id is " << session_id;
            handleRequestTimeStamp(msg);
            break;
        }
        case proto::Request::REQUEST_GET_CUT_MAP: {
            LOGGER(INFO, PROTOBUF) << "REQUEST_GET_CUT_MAP, session id is " << session_id;
            handleRequestCurMap(msg);
            break;
        }
        case proto::Request::REQUEST_MISSION_LIST: {
            LOGGER(INFO, PROTOBUF) << "REQUEST_MISSION_LIST, session id is " << session_id;
            handleRequestMissionQueue(msg);
            break;
        }
        case proto::Request::REQUEST_CONNECT_INFO: {
            LOGGER(INFO, PROTOBUF) << "REQUEST_CONNECT_INFO, session id is " << session_id;
            handleRequestConnectionInfo(seq, session_id);
            break;
        }
        case proto::Request::REQUEST_READ_INPUT_REGISTER: {
            handleRequestReadInputRegisters(msg, seq, session_id);
            break;
        }
        case proto::Request::REQUEST_READ_ALL_REGISTERS: {
            handleRequestReadAllRegisters(msg,seq,session_id);
            break;
        }
        default:
            LOG(WARNING) << "Do not support request " << m->request_type();
            break;
    }
}

void NetworkModule::handleMovementTask(const proto::Message_ptr &msg) {
    auto seq = msg->seq();  // 请求序列号
    auto session_id = msg->session_id();

    auto move_task = msg->mutable_command()->mutable_movement_task();

    // 当任务启动或，再次启动相同session_id和task no时，直接回复成功
    auto cur_movement_task = TaskManager::getInstance()->getMovementTask();
    if (cur_movement_task && cur_movement_task->getTaskSessionId() == session_id &&
        cur_movement_task->getTaskNo() == move_task->no()) {
        LOGGER(WARNING, PROTOBUF) << "Movement task idempotent! session id: " << session_id
                                  << ", task no: " << move_task->no();
        responseCommandResult(sros::core::CMD_NEW_MOVEMENT_TASK, RESPONSE_OK, 0, seq, session_id);
        return;
    }

    // 以下代码将 proto::MovementTask 转为 sros::core::MovementTask

    sros::core::MovementTask_ptr new_task;

    auto avoid_policy = (sros::core::ObstacleAvoidPolicy)move_task->avoid_policy();

    if (move_task->type() == proto::MovementTask::MT_MOVE_TO_POSE) {
        if (move_task->poses_size() == 0) {
            SET_ERROR(ERROR_CODE_MOVEMENT_INVALID_CMD_PARAM, "poses is empty!");
            responseCommandResult(sros::core::CMD_NEW_MOVEMENT_TASK, sros::core::RESPONSE_FAILED,
                                  sros::core::ERROR_CODE_MOVEMENT_INVALID_CMD_PARAM, msg->seq(), msg->session_id());
            return;
        }

        std::deque<sros::core::Pose> dst_poses;
        sros::core::Pose pose;
        for (auto p : move_task->poses()) {
            convertPose(p, &pose);
            dst_poses.push_back(pose);
        }

        LOGGER(INFO, PROTOBUF) << "MOVE_TO_POSE: " << pose.x() << ", " << pose.y() << ", " << pose.yaw();

        new_task = std::make_shared<sros::core::MovementTask>(move_task->no(), getName(), dst_poses, avoid_policy);
        new_task->setForceNavOnMap(move_task->force_nav_on_map());

    } else if (move_task->type() == proto::MovementTask::MT_MOVE_TO_STATION) {
        if (move_task->stations_size() == 0) {
            SET_ERROR(ERROR_CODE_MOVEMENT_INVALID_CMD_PARAM, "stations is empty!");

            responseCommandResult(sros::core::CMD_NEW_MOVEMENT_TASK, sros::core::RESPONSE_FAILED,
                                  sros::core::ERROR_CODE_MOVEMENT_INVALID_CMD_PARAM, msg->seq(), msg->session_id());
            return;
        }

        LOGGER(INFO, PROTOBUF) << "MOVE_TO_STATION: station_no " << move_task->stations(0);

        std::deque<sros::core::StationNo_t> dst_stations;
        for (auto s : move_task->stations()) {
            dst_stations.push_back((sros::core::StationNo_t)s);
        }

        new_task = std::make_shared<sros::core::MovementTask>(move_task->no(), getName(), dst_stations, avoid_policy);

        if (move_task->stations_size() == 1 && move_task->dst_station_type() == proto::MovementTask::DS_NO_ROTATE) {
            new_task->setDstStationType((sros::core::DstStationType)move_task->dst_station_type());
        }
    } else if (move_task->type() == proto::MovementTask::MT_MOVE_FOLLOW_PATH) {
        if (move_task->paths_size() == 0) {
            SET_ERROR(ERROR_CODE_MOVEMENT_INVALID_CMD_PARAM, "paths is empty!");

            responseCommandResult(sros::core::CMD_NEW_MOVEMENT_TASK, sros::core::RESPONSE_FAILED,
                                  sros::core::ERROR_CODE_MOVEMENT_INVALID_CMD_PARAM, msg->seq(), msg->session_id());
            return;
        }

        sros::core::NavigationPathi_vector dst_paths;
        for (auto p : move_task->paths()) {
            sros::core::NavigationPathi path;

            convertPathReverse(p, &path);

            dst_paths.push_back(path);
        }

        std::deque<sros::core::StationNo_t> dst_stations;
        for (auto s : move_task->stations()) {
            dst_stations.push_back((sros::core::StationNo_t)s);
        }

        new_task = std::make_shared<sros::core::MovementTask>(move_task->no(), getName(), dst_paths);
        if (!dst_stations.empty()) {  // 发送路径的同时，可能会发送路径的目标站点
            new_task->setCurDstStation(dst_stations.front());
        }

        // 获取用户发送路径的最终位置
        bool is_get_dst_pose = false;
        Pose dst_pose;
        for (auto it = dst_paths.rbegin(); it < dst_paths.rend(); ++it) {
            if (it->type_ != PATH_ROTATE) {
                dst_pose.x() = it->ex_ / 1000.0;
                dst_pose.y() = it->ey_ / 1000.0;
                is_get_dst_pose = true;
                break;
            }
        }
        if (!is_get_dst_pose) {  // 若没有找到目标位置，那么只存在旋转的情况，此时只要将当前车辆的坐标当做目标坐标即可
            dst_pose = src_sdk->getCurPose();
        }
        dst_pose.yaw() = dst_paths.rbegin()->e_facing_;  // FIXME(pengjiali): 朝向现在都为0
        new_task->setCurDstPose(dst_pose);

    } else if (move_task->type() == proto::MovementTask::MT_MOVE_MIX2) {
        if (move_task->paths_size() == 0 || move_task->stations_size() == 0) {
            LOGGER(WARNING, PROTOBUF) << "undefined movement task type! ";
            return;
        }

        sros::core::NavigationPathi_vector dst_paths;
        for (auto p : move_task->paths()) {
            sros::core::NavigationPathi path;

            convertPathReverse(p, &path);

            dst_paths.push_back(path);
        }

        std::deque<sros::core::StationNo_t> dst_stations;
        for (auto s : move_task->stations()) {
            dst_stations.push_back((sros::core::StationNo_t)s);
        }

        new_task = std::make_shared<sros::core::MovementTask>(move_task->no(), getName(), dst_paths, dst_stations,
                                                              avoid_policy);
    }

    new_task->setTaskSeq(seq);
    new_task->setTaskSessionId(session_id);

    auto mm = make_shared<sros::core::CommandMsg>(getName());
    mm->req_seq = seq;
    mm->session_id = session_id;
    mm->command = sros::core::CMD_NEW_MOVEMENT_TASK;
    mm->movement_task = new_task;
    mm->param0 = msg->mutable_command()->param_int();

    sendMsg(mm);
}

void NetworkModule::handleActionTask(const proto::Message_ptr &msg) {
    auto msg_seq = msg->seq();
    auto session_id = msg->session_id();

    auto action_task = msg->mutable_command()->mutable_action_task();

    // 当任务启动或，再次启动相同session_id和task no时，直接回复成功
    auto cur_action_task = TaskManager::getInstance()->getActionTask();
    if (cur_action_task && cur_action_task->getTaskSessionId() == session_id &&
        cur_action_task->getTaskNo() == action_task->no()) {
        LOGGER(WARNING, PROTOBUF) << "Action Task idempotent! session id: " << session_id
                                  << ", task no: " << action_task->no();
        responseCommandResult(sros::core::CMD_NEW_ACTION_TASK, RESPONSE_OK, 0, msg_seq, session_id);
        return;
    }

    auto new_task =
        std::make_shared<sros::core::ActionTask>(action_task->no(), getName(), action_task->id(), action_task->param0(),
                                                 action_task->param1(), action_task->param2());

    new_task->setTaskSeq(msg_seq);
    new_task->setTaskSessionId(session_id);
    new_task->setActionParamStr(action_task->param_str());

    auto mm = make_shared<sros::core::CommandMsg>(getName());
    mm->req_seq = msg_seq;
    mm->session_id = session_id;
    mm->command = sros::core::CMD_NEW_ACTION_TASK;
    mm->param0 = new_task->getTaskNo();
    mm->action_task = new_task;

    sendMsg(mm);
}

void NetworkModule::handleRequestAllState(uint32_t req_seq, uint64_t session_id) {
    auto msg = std::make_shared<proto::Message>();
    msg->set_type(proto::MSG_RESPONSE);
    msg->set_seq(req_seq);
    msg->set_session_id(session_id);

    auto response_msg = msg->mutable_response();
    response_msg->set_response_type(proto::Response::RESPONSE_ALL_STATE);

    auto response_result = response_msg->mutable_result();
    response_result->set_result_state(proto::ResponseResult::RESPONSE_OK);

    auto sys_state = response_msg->mutable_system_state();
    buildProtoSystemState(sys_state);

    auto hw_state = response_msg->mutable_hardware_state();
    buildProtoHardwareState(hw_state);

    sendMsgToNetwork(msg);
}

void NetworkModule::handleRequestHardwareState(uint32_t req_seq, uint64_t session_id) {
    auto msg = std::make_shared<proto::Message>();
    msg->set_type(proto::MSG_RESPONSE);
    msg->set_seq(req_seq);
    msg->set_session_id(session_id);

    auto response_msg = msg->mutable_response();
    response_msg->set_response_type(proto::Response::RESPONSE_HARDWARE_STATE);

    auto response_result = response_msg->mutable_result();
    response_result->set_result_state(proto::ResponseResult::RESPONSE_OK);

    auto hw_state = response_msg->mutable_hardware_state();
    buildProtoHardwareState(hw_state);

    if (session_id == 0) {
        sendHardwareStateToNetwork(msg);
    } else {
        sendMsgToNetwork(msg);
    }
}

void NetworkModule::handleRequestBagList(uint32_t req_seq, uint64_t session_id) {}

void NetworkModule::handleRequestSystemState(uint32_t req_seq, uint64_t session_id) {
    auto msg = std::make_shared<proto::Message>();
    msg->set_type(proto::MSG_RESPONSE);
    msg->set_seq(req_seq);
    msg->set_session_id(session_id);

    auto response_msg = msg->mutable_response();
    response_msg->set_response_type(proto::Response::RESPONSE_SYSTEM_STATE);

    auto response_result = response_msg->mutable_result();
    response_result->set_result_state(proto::ResponseResult::RESPONSE_OK);

    // 以下构造SystemStateMsg并发送
    auto sys_state = response_msg->mutable_system_state();

    buildProtoSystemState(sys_state);

    if (session_id == 0) {
        sendSystemStateToNetwork(msg);
    } else {
        sendMsgToNetwork(msg);
    }
}

void NetworkModule::buildProtoSystemState(proto::SystemState *sys_state) const {
    assert(sys_state != nullptr);

    auto &s = sros::core::Settings::getInstance();

    // 是否精简SystemState
    auto trim_mode = s.getValue<std::string>("network.enable_trim_system_state_upload", "False") == "True";

    sys_state->set_sys_state((proto::SystemState::SysState)g_state.sys_state);
    sys_state->set_location_state((proto::SystemState::LocationState)g_state.location_state);
    sys_state->set_emergency_state((proto::SystemState::EmergencyState)g_state.emergency_state);
    sys_state->set_emergency_source((proto::SystemState::EmergencySource)g_state.emergency_source);
    sys_state->set_new_movement_task_state(g_state.ready_for_new_movement_task
                                               ? proto::SystemState::NEW_MOVEMENT_TASK_STATE_READY
                                               : proto::SystemState::NEW_MOVEMENT_TASK_STATE_USELESS);
    sys_state->set_map_name(g_state.getCurMapName());
    sys_state->set_map_saving_progress((google::uint32)g_state.progress);
    sys_state->set_oba_enable_state((proto::SystemState::ObaEnableState)g_state.oba_state);
    sys_state->set_fresh_state((proto::SystemState::FreshState)g_state.fresh_state);
    sys_state->set_load_state((proto::SystemState::LoadState)g_state.load_state);
    sys_state->set_operation_state((proto::SystemState::OperationState)g_state.operation_state);
    sys_state->set_speed_level(g_state.speed_level);
    sys_state->set_cur_volume(g_state.cur_volume);
    sys_state->set_control_mutex_lock_state(
        g_state.control_mutex.isLock()
            ? proto::SystemState::ControlMutexLockState::SystemState_ControlMutexLockState_LOCKED
            : proto::SystemState::ControlMutexLockState::SystemState_ControlMutexLockState_UNLOCKED);
    sys_state->set_last_error_code(g_state.laster_error_code);
    auto fault_codes_proto = sys_state->mutable_fault_codes();
    auto faults_proto = sys_state->mutable_faults();
    auto fault_list = sros::core::FaultCenter::getInstance()->getFaultList();
    for (const auto &fault : *fault_list) {
        fault_codes_proto->Add((uint32_t)fault->id);
        auto fault_proto = faults_proto->Add();
        fault_proto->set_id(fault->id);
        fault_proto->set_response_behavior(fault->response_behavior);
        fault_proto->set_can_automatically_recover(fault->can_automatically_recover_func());
        fault_proto->set_raise_timestamp(fault->raise_timestamp);
    }

    auto control_mutex_info = sys_state->mutable_control_mutex_info();
    auto locker = g_state.control_mutex.getLocker();
    control_mutex_info->set_session_id(locker.session_id);
    control_mutex_info->set_ip_address(locker.ip_address);
    control_mutex_info->set_nick_name(locker.nick_name);
    control_mutex_info->set_user_name(locker.user_name);
    sys_state->set_multi_load_state(g_state.multi_load_state);

    // MovementTask状态
    auto move_task = sros::core::TaskManager::getInstance()->getMovementTask();
    if (move_task && !trim_mode) {
        auto p_task = sys_state->mutable_movement_state();
        convertMovementTask(move_task, p_task);
    }

    auto action_task = sros::core::TaskManager::getInstance()->getActionTask();
    if (action_task && !trim_mode) {
        auto p_task = sys_state->mutable_action_state();
        convertActionTask(action_task, p_task);
    }

    if (!trim_mode) {
        auto mission_manager = sros::core::MissionManager::getInstance();
        auto running_mission = sys_state->mutable_running_mission();
        buildMissionState(mission_manager, running_mission);
    }

    // 运动控制状态
    auto mc_state = sys_state->mutable_mc_state();

    mc_state->set_state(proto::MotionControlState::MC_IDLE);
    mc_state->set_v_x(g_src_state.cur_v);
    mc_state->set_w(g_src_state.cur_w);
    mc_state->set_path_no(static_cast<int>(g_src_state.cur_path_no));

    auto pose = sys_state->mutable_location_pose();
    convertPose(src_sdk->getCurPose(), pose);

    sys_state->set_station_no(g_state.station_no);

    auto p_cur_pgv_info = sys_state->mutable_cur_pgv_info();
    auto offset = g_state.cur_down_camera_offset.get();
    //    LOG(INFO) << offset;
    p_cur_pgv_info->set_pgv_no(offset.no);
    p_cur_pgv_info->set_pgv_id(offset.id);
    p_cur_pgv_info->set_x(offset.x * 1000);
    p_cur_pgv_info->set_y(offset.y * 1000);
    p_cur_pgv_info->set_yaw(offset.yaw * 1000);

    auto p_station_pgv_info = sys_state->mutable_station_pgv_info();
    auto station_pgv_info = g_state.station_camera_offset.get();
    p_station_pgv_info->set_pgv_no(station_pgv_info.no);
    p_station_pgv_info->set_pgv_id(station_pgv_info.id);
    p_station_pgv_info->set_x(station_pgv_info.x * 1000);
    p_station_pgv_info->set_y(station_pgv_info.y * 1000);
    p_station_pgv_info->set_yaw(station_pgv_info.yaw * 1000);

    //    sys_state->set_expect_fleet_state((proto::SystemState::FleetMode)g_state.expect_fleet_state);
    sys_state->set_fleet_mode((proto::SystemState::FleetMode)g_state.fleet_mode);

    //叉车状态信息
    auto forklift_state = sys_state->mutable_forklift_state();
    std::string vehicleType = s.getValue<std::string>("main.vehicle_type","");
    std::string type = vehicleType.substr(0,4);
    if(type == "gulf" || type == "Gulf"){   //叉车系列
        forklift_state->set_arm_height(g_state.fork_height_encoder);
        forklift_state->set_steering_angle(g_state.steering_angle / 100.0);
        auto pose = forklift_state->mutable_goods_pose();
        convertPose(g_state.goods_pose, pose);
        

    } else{
        forklift_state->set_arm_height(0.0);
        forklift_state->set_steering_angle(0.0);
        auto pose = forklift_state->mutable_goods_pose();
    }

}

void NetworkModule::buildProtoHardwareState(proto::HardwareState *hw_state) {
    hw_state->set_cpu_usage(g_state.cpu_usage / 10);
    hw_state->set_memory_usage(g_state.memory_usage / 10);

    size_t total_disk_size, available_disk_size;
    disk_info(total_disk_size, available_disk_size);
    auto available_disk_usage = 100 - available_disk_size * 100.0 / total_disk_size;
    hw_state->set_disk_usage(static_cast<uint32_t>(available_disk_usage));
    hw_state->set_remain_disk_space(static_cast<int>(available_disk_size / 1024));

    hw_state->set_wifi_state(proto::HardwareState::WIFI_NA);
    hw_state->set_wifi_name("");
    hw_state->set_wifi_strength(0);

    hw_state->set_battery_state((proto::HardwareState::BatteryState)g_state.battery_state);
    hw_state->set_battery_percentage(g_state.battery_percentage);
    hw_state->set_battery_current(g_state.battery_current);
    hw_state->set_battery_voltage(g_state.battery_voltage);
    hw_state->set_battery_temperature(g_state.battery_temperature);
    hw_state->set_battery_remain_capacity(g_state.battery_remain_capacity);
    hw_state->set_battery_nominal_capacity(g_state.battery_nominal_capacity);
    hw_state->set_battery_use_cycles(g_state.battery_use_cycles);
    hw_state->set_battery_remain_time(g_state.battery_remain_time);

    hw_state->set_power_state((proto::HardwareState::PowerState)g_state.power_state);
    hw_state->set_break_sw_state((proto::HardwareState::BreakSwitchState)g_state.break_sw_state);

    hw_state->set_laser_state(proto::HardwareState::LASER_NA);

    hw_state->set_cpu_temperature(g_state.cpu_temperature / 1000);
    hw_state->set_box_temperature(g_state.board_temperature / 1000);
    hw_state->set_box_temperature_max(g_state.box_temperature_max);
    hw_state->set_box_temperature_min(g_state.box_temperature_min);
    hw_state->set_box_humidity_max(g_state.box_humidity_max);
    hw_state->set_box_humidity_min(g_state.box_humidity_min);
    hw_state->set_fan_switch_state(g_state.fan_switch_state);
    //    LOG(INFO) << g_state.box_temperature_max << ", " << g_state.box_temperature_min << ", " <<
    //    g_state.box_humidity_max
    //              << ", " << g_state.box_humidity_min << ", " << std::boolalpha << g_state.fan_switch_state;

    hw_state->set_general_io_input(g_state.gpio_input);
    hw_state->set_general_io_output(g_state.gpio_output);
    hw_state->set_hardware_state((proto::HardwareState::HState)g_state.hardware_state);
    hw_state->set_hardware_error_code(g_state.hardware_error_code);
    hw_state->set_ip_address(g_state.ip_addr.get());

    auto src_hardware_state = hw_state->mutable_src_hardware_state();
    src_hardware_state->set_m1_status_code(g_src_state.m1_status_code);
    src_hardware_state->set_m2_status_code(g_src_state.m2_status_code);
    src_hardware_state->set_m3_status_code(g_src_state.m3_status_code);
    src_hardware_state->set_m4_status_code(g_src_state.m4_status_code);

    //    src_hardware_state->set_total_mileage(g_src_state.total_mileage);
    //    src_hardware_state->set_total_power_cycle(g_src_state.total_power_cycle);
    //    src_hardware_state->set_total_poweron_time(g_src_state.total_poweron_time);

    auto &run_logger = sros::core::RunLogger::getInstance();
    src_hardware_state->set_total_mileage(run_logger.getTotalMileage());
    src_hardware_state->set_total_power_cycle(run_logger.getTotalBootTimes());
    src_hardware_state->set_total_poweron_time(run_logger.getTotalRunTimeInSeconds());

    src_hardware_state->set_src_state(g_src_state.src_state);
    src_hardware_state->set_src_state_error_reason(g_src_state.src_state_error_reason);

    auto dm = sros::device::DeviceManager::getInstance();
    auto device_map = dm->getDeviceList();
    for (const auto &it : *device_map) {
        const auto &device = it.second;
        if (device->getState() != sros::device::DEVICE_NONE) {  // 存在device为DEVICE_NONE的情况
            auto d = hw_state->add_devices();
            d->set_id(device->getID());
            d->set_name(device->getName());
            // 注意，此处需要保证 main.proto 与 core/device/device.h 中的定义一致
            d->set_state((proto::Device_DeviceState)device->getState());

            d->set_error_code(device->getRawFaultCode());
            d->set_serial_no(device->getSerialNo());
            d->set_model_no(device->getModelNo());
            d->set_version_no(device->getVersionNo());
            d->set_info(device->getInfo());
            d->set_interface_name(device->getInterfaceName());
        }
    }
}

void NetworkModule::convertActionTask(const sros::core::ActionTask_ptr &action_task, proto::ActionTask *p_task) const {
    if (!action_task || !p_task) {
        LOGGER(WARNING, PROTOBUF) << "convertActionTask(): action_task or p_task is null";
        return;
    }

    p_task->set_no(action_task->getTaskNo());
    p_task->set_session_id(action_task->getTaskSessionId());
    p_task->set_state((proto::ActionTask_TaskState)action_task->getState());

    p_task->set_id(action_task->getActionID());
    p_task->set_param0(action_task->getActionParam());
    p_task->set_param1(action_task->getActionParam1());
    p_task->set_param2(action_task->getActionParam2());

    p_task->set_result((proto::TaskResult)action_task->getTaskResult());
    p_task->set_result_str(action_task->getResultValueStr());
    p_task->set_result_code(action_task->getActionResultValue());
}

void NetworkModule::convertMovementTask(const sros::core::MovementTask_ptr &move_task,
                                        proto::MovementTask *p_task) const {
    if (!move_task || !p_task) {
        LOGGER(WARNING, PROTOBUF) << "convertMovementTask(): move_task or p_task is null";
        return;
    }

    p_task->set_no(move_task->getTaskNo());
    p_task->set_session_id(move_task->getTaskSessionId());
    p_task->set_state((proto::MovementTask::TaskState)move_task->getState());

    for (auto p : move_task->getPaths()) {
        auto pp = p_task->add_paths();
        this->convertPath(p, pp);
    }
    p_task->set_paths_replacement_times(move_task->getPathsReplacementTimes());

    p_task->set_cur_path_no(g_src_state.cur_path_no);
    p_task->set_cur_checkpoint_no(g_src_state.cur_checkpoint);
    p_task->set_remain_time(move_task->getRemainTime());
    p_task->set_remain_distance(move_task->getRemainDistance());
    p_task->set_total_distance(move_task->getTotalDistance());

    p_task->set_result((proto::TaskResult)move_task->getTaskResult());

    p_task->add_stations(move_task->getCurDstStation());

    p_task->set_failed_code((proto::MovementTask_FailedCode)move_task->getFailedCode());

    // auto pose = p_task->mutable_poses();
    // proto::Pose* pose
    // convertPose(move_task->getCurDstPose(), pose);
    // p_task->add_poses(proto_pose);

    auto p_pgv_info = p_task->mutable_station_pgv_info();
    auto offset = move_task->getDownCameraOffset();
    p_pgv_info->set_pgv_no(offset.no);
    p_pgv_info->set_pgv_id(offset.id);
    p_pgv_info->set_x(offset.x * 1000);
    p_pgv_info->set_y(offset.y * 1000);
    p_pgv_info->set_yaw(offset.yaw * 1000);
    //    std::cout << "历史PGV——INFO" << offset << std::endl;
}

void NetworkModule::buildMissionState(sros::core::MissionManager *mission_manager,
                                      proto::Mission *running_mission) const {
    auto mission = mission_manager->getCurrentRunningMission();
    if (!mission) {
        return;
    }

    running_mission->set_id(mission->getId());
    running_mission->set_no(mission->no_);
    running_mission->set_state(static_cast<proto::Mission::MissionStatus>(mission->state_));
    running_mission->set_cur_step_id(mission->cur_step_id_);
    running_mission->set_result((proto::TaskResult)mission->result_);
    running_mission->set_total_cycle_time(mission->total_cycle_time_);
    running_mission->set_finish_cycle_time(mission->finish_cycle_time_);

    // 当前正在执行任务的所有父任务步骤
    auto parent_missions = mission_manager->getParentMissionList();
    for (auto parent_mission : parent_missions) {
        auto parent_step = running_mission->add_step_list();
        parent_step->set_id(parent_mission->currentStepId());
        parent_step->set_mission_id(parent_mission->getId());
    }

    // 当前正在执行中的任务步骤
    auto cur_step = running_mission->add_step_list();
    cur_step->set_id(mission->currentStepId());
    cur_step->set_mission_id(mission->getId());
}

void NetworkModule::handleRequestInfo(uint32_t req_seq, uint64_t session_id) {
    auto msg = std::make_shared<proto::Message>();
    msg->set_type(proto::MSG_RESPONSE);
    msg->set_seq(req_seq);
    msg->set_session_id(session_id);

    auto response_msg = msg->mutable_response();
    response_msg->set_response_type(proto::Response::RESPONSE_INFO);

    auto response_result = response_msg->mutable_result();
    response_result->set_result_state(proto::ResponseResult::RESPONSE_OK);

    auto info = response_msg->mutable_info();

    sros::core::Settings &settings = sros::core::Settings::getInstance();
    string nickname = settings.getValue<string>("main.nickname", "NA");

    string serial_no = settings.getValue<string>("main.serial_no", "NA");
    string vehicle_serial_no = settings.getValue<string>("main.vehicle_serial_no", "NA");
    string hardware_version = settings.getValue<string>("main.vehicle_controller_type", "NA");

    auto vehicle_type = settings.getValue<string>("main.vehicle_type", "NA");
    auto action_unit = settings.getValue<string>("main.action_unit", "NA");

    info->set_serial_no(settings.getValue<string>("main.serial_no", "NA"));
    info->set_nickname(nickname);

    info->set_hardware_version(hardware_version);
    info->set_kernel_release(g_state.kernel_release);

    info->set_sros_version(SROS_VERSION);
    info->set_sros_version_str(SROS_VERSION_STR);

    info->set_src_version(src_sdk->getSRCVersion());
    info->set_src_version_str(src_sdk->getSRCVersionStr());
    info->set_vehicle_type(vehicle_type);
    info->set_action_unit(action_unit);

    info->set_vehicle_serial_no(vehicle_serial_no);

    sendMsgToNetwork(msg);
}

void NetworkModule::responseCommandResult(sros::core::CommandType command_type, sros::core::ResultState result_state,
                                          uint32_t result_code, uint32_t req_seq, uint64_t session_id) {
    auto msg = std::make_shared<proto::Message>();
    msg->set_type(proto::MSG_RESPONSE);
    msg->set_seq(req_seq);
    msg->set_session_id(session_id);

    auto response_msg = msg->mutable_response();
    response_msg->set_response_type(proto::Response::RESPONSE_COMMAND);

    buildTaskState(response_msg);
    auto system_state = response_msg->mutable_system_state();
    buildProtoSystemState(system_state);

    auto command_result = response_msg->mutable_result();
    command_result->set_result_state((proto::ResponseResult::ResultState)result_state);
    command_result->set_result_code((proto::ResponseResult::ResultCode)result_code);

    LOGGER(INFO, PROTOBUF) << "Response command : command = " << command_type << " result state = " << result_state
                           << " result code  = " << result_code << " seq = " << req_seq
                           << " session id = " << session_id;

    sendMsgToNetwork(msg);
}

void NetworkModule::handleRequestTaskState(uint32_t req_seq, uint64_t session_id) {
    auto msg = std::make_shared<proto::Message>();
    msg->set_type(proto::MSG_RESPONSE);
    msg->set_seq(req_seq);
    msg->set_session_id(session_id);

    auto response_msg = msg->mutable_response();
    response_msg->set_response_type(proto::Response::RESPONSE_TASK_STATE);

    auto response_result = response_msg->mutable_result();
    response_result->set_result_state(proto::ResponseResult::RESPONSE_OK);

    buildTaskState(response_msg);

    sendMsgToNetwork(msg);
}

void NetworkModule::buildTaskState(proto::Response *response_msg) const {
    auto cur_move_task = sros::core::TaskManager::getInstance()->getMovementTask();
    if (cur_move_task) {
        auto movement_task = response_msg->mutable_movement_task();

        movement_task->set_no(cur_move_task->getTaskNo());
        //        movement_task->set_no(0);
        movement_task->set_state((proto::MovementTask_TaskState)cur_move_task->getState());
        //        movement_task->set_type();
        movement_task->set_avoid_policy((proto::MovementTask_AvoidPolicy)cur_move_task->getAvoidPolicy());
        movement_task->set_result((proto::TaskResult)cur_move_task->getTaskResult());
    }

    auto cur_action_task = sros::core::TaskManager::getInstance()->getActionTask();
    if (cur_action_task) {
        auto action_task = response_msg->mutable_action_task();

        action_task->set_no(cur_action_task->getTaskNo());
        //        action_task->set_no(0);
        action_task->set_state((proto::ActionTask_TaskState)cur_action_task->getState());
        action_task->set_id(cur_action_task->getActionID());
        action_task->set_result((proto::TaskResult)cur_action_task->getTaskResult());
    }
}

void NetworkModule::handleRequestMapList(uint32_t req_seq, uint64_t session_id) {
    sros::core::MapManager map_manager;
    map_manager.freshMapList();

    auto msg = make_shared<proto::Message>();
    msg->set_type(proto::MSG_RESPONSE);
    msg->set_seq(req_seq);
    msg->set_session_id(session_id);

    auto response_msg = msg->mutable_response();
    response_msg->set_response_type(proto::Response::RESPONSE_MAP_LIST);

    auto response_result = response_msg->mutable_result();
    response_result->set_result_state(proto::ResponseResult::RESPONSE_FAILED);

    auto map_list_msg = response_msg->mutable_map_list();
    map_list_msg->set_req_seq(req_seq);

    auto map_info_list = map_manager.getMapInfoList();

    using namespace boost::filesystem;

    for (auto map_info : map_info_list) {
        auto proto_map_info = map_list_msg->add_list();
        auto map_write_time = last_write_time(path(map_info.file_path_));
        auto map_file_size = file_size(path(map_info.file_path_));

        proto_map_info->set_name(map_info.name_);
        proto_map_info->set_size(map_file_size);
        proto_map_info->set_last_write_time(map_write_time);
        response_result->set_result_state(proto::ResponseResult::RESPONSE_OK);
    }

    sendMsgToNetwork(msg);
}

void NetworkModule::handleRequestLoadConfig(uint32_t seq, uint64_t session_id) {
    auto mm = make_shared<proto::Message>();
    mm->set_type(proto::MSG_RESPONSE);
    mm->set_seq(seq);
    mm->set_session_id(session_id);

    auto response_msg = mm->mutable_response();
    response_msg->set_response_type(proto::Response::RESPONSE_LOAD_CONFIG);
    auto response_result = response_msg->mutable_result();

    // get config from database
    // ItemInfoLists getItemInfoLists(std::string table_name);
    auto &s = sros::core::Settings::getInstance();
    sros::core::ItemInfoLists item_info_list = s.getItemInfoLists();
    if (item_info_list.empty()) {
        LOGGER(ERROR, PROTOBUF) << "Loading config (DB empty)";
        response_result->set_result_state(proto::ResponseResult::RESPONSE_FAILED);
        return;
    }

    auto session = session_manager_.getItem(session_id);
    if (!session) {
        LOGGER(ERROR, PROTOBUF) << "Sessionid " << session_id << " is invalid!";
        return;
    }

    auto &ss = sros::core::UserManager::getInstance();
    auto user_item = ss.getUserItem(session->username);

    LOGGER(INFO, PROTOBUF) << "Session " << session_id << "'s permission is "
                           << UserManager::getPermissionStr(stoi(user_item.permission));

    for (auto p : item_info_list) {
        // sros::core::ItemInfo item;
        auto pp = response_msg->add_config();

        if (p.permission > stoi(user_item.permission)) {
            continue;
        }
        pp->set_id(p.id);
        pp->set_key(p.key);
        pp->set_name(p.name);
        pp->set_value(p.value);
        pp->set_value_unit(p.value_units);
        pp->set_value_type(p.value_type);
        pp->set_default_value(p.default_value);
        pp->set_value_range(p.value_range);
        pp->set_description(p.description);
        pp->set_permission(p.permission);
        pp->set_changed_time(p.changed_time);
        pp->set_changed_user(p.changed_user);
    }

    response_result->set_result_state(proto::ResponseResult::RESPONSE_OK);
    sendMsgToNetwork(mm);
}

void NetworkModule::handleRequestLoadTmpConfig(const proto::Message_ptr &msg) {
    auto session_id = msg->session_id();
    auto mm = make_shared<proto::Message>();
    mm->set_type(proto::MSG_RESPONSE);
    mm->set_seq(msg->seq());
    mm->set_session_id(session_id);

    auto response_msg = mm->mutable_response();
    response_msg->set_response_type(proto::Response::RESPONSE_LOAD_TMP_CONFIG);
    auto response_result = response_msg->mutable_result();

    auto request_msg = msg->mutable_request();
    if (request_msg->tmp_configs_size() == 0) {
        response_result->set_result_state(proto::ResponseResult::RESPONSE_FAILED);
        return;
    }

    auto &s = sros::core::Settings::getInstance();
    for (const auto &tmp_config : request_msg->tmp_configs()) {
        auto response_tmp_config = response_msg->add_tmp_configs();
        response_tmp_config->set_key(tmp_config.key());
        response_tmp_config->set_value(s.getValue<std::string>(tmp_config.key(), ""));
    }

    response_result->set_result_state(proto::ResponseResult::RESPONSE_OK);
    sendMsgToNetwork(mm);
}

void NetworkModule::handleRequestSaveTmpConfig(const proto::Message_ptr &msg) {
    auto session_id = msg->session_id();
    auto mm = make_shared<proto::Message>();
    mm->set_type(proto::MSG_RESPONSE);
    mm->set_seq(msg->seq());
    mm->set_session_id(session_id);

    auto response_msg = mm->mutable_response();
    response_msg->set_response_type(proto::Response::RESPONSE_SAVE_TMP_CONFIG);
    auto response_result = response_msg->mutable_result();

    auto request_msg = msg->mutable_request();
    if (request_msg->tmp_configs_size() == 0) {
        response_result->set_result_state(proto::ResponseResult::RESPONSE_FAILED);
        return;
    }

    auto &s = sros::core::Settings::getInstance();
    for (const auto &tmp_config : request_msg->tmp_configs()) {
        s.setTmpValue(tmp_config.key(), tmp_config.value());
    }

    response_result->set_result_state(proto::ResponseResult::RESPONSE_OK);
    sendMsgToNetwork(mm);
}

void NetworkModule::handleRequestSaveConfig(const proto::Message_ptr &msg) {
    // TODO(pengjiali): 后期保证原子操作
    bool is_src_param = false;
    auto session_id = msg->session_id();
    auto mm = make_shared<proto::Message>();
    mm->set_type(proto::MSG_RESPONSE);
    mm->set_seq(msg->seq());
    mm->set_session_id(session_id);

    auto response_msg = mm->mutable_response();
    response_msg->set_response_type(proto::Response::RESPONSE_SAVE_CONFIG);
    auto response_result = response_msg->mutable_result();

    auto request_msg = msg->mutable_request();
    if (request_msg->config_size() == 0) {
        response_result->set_result_state(proto::ResponseResult::RESPONSE_FAILED);
        return;
    }

    auto session = session_manager_.getItem(session_id);
    if (!session) {
        LOGGER(ERROR, PROTOBUF) << "invalid session id ! sessionid = " << session_id;
        return;
    }

    // set config to database
    sros::core::ItemInfoLists item_info_list;
    sros::core::ItemInfoLists item_info_list_src;
    auto &s = sros::core::Settings::getInstance();
    for (auto p : request_msg->config()) {
        // sros::core::ItemInfo item;
        sros::core::ItemInfo item = s.getItemInfo(p.key());
        if (!item.key.empty() && item.key.find("src.") == 0) {
            item_info_list_src.push_back(item);
        }

        if (item.key.empty()) {
            item.id = p.id();
            item.default_value = p.default_value();
            item.key = p.key();
        }

        if (item.key.find("src.") == 0) {
            is_src_param = true;
        }

        item.name = p.name();
        item.value = p.value();
        item.value_units = p.value_unit();
        item.value_type = p.value_type();
        item.value_range = p.value_range();
        item.description = p.description();
        item.permission = p.permission();
        item.changed_time = p.changed_time();
        // item.changed_user   = p.changed_user();
        item.changed_user = session->username;

        item_info_list.push_back(item);

        LOGGER(INFO, SROS) << "Set " << p.name() << " = " << p.value();

        // 设置的一些动作，暂时放置此处
        if (item.key == "debug.vlog_v") {  // 设置vlog等级
            FLAGS_v = std::atoi(item.value.c_str());
            LOGGER(INFO, SROS) << "set vlog level： " << FLAGS_v;
        }
    }

    if (!g_state.control_mutex.get(session_id)) {
        if (item_info_list.size() == 1 && item_info_list.cbegin()->key == "main.fleet_mode" &&
            item_info_list.cbegin()->value == "FLEET_MODE_OFFLINE") {
            // 在调度模式下，当前由fms独占，但允许切换到单机模式
        } else {
            auto locker = g_state.control_mutex.getLocker();
            SET_ERROR(ERROR_CODE_CONTROL_MUTEX_IS_LOCKED, "locker:", locker.session_id, "current:", session_id);
            response_result->set_result_state(proto::ResponseResult::RESPONSE_FAILED);
            response_result->set_result_code(ERROR_CODE_CONTROL_MUTEX_IS_LOCKED);
            server_.send(mm);
            return;
        }
    }

    auto isMovementRunningFun = []() {
        auto cur_task = sros::core::TaskManager::getInstance()->getMovementTask();

        return cur_task && cur_task->isRunning();
    };
    auto isActionRunningFun = []() {
        auto cur_task = sros::core::TaskManager::getInstance()->getActionTask();

        return cur_task && cur_task->isRunning();
    };
    auto isMissionRunningFun = []() { return sros::core::MissionManager::getInstance()->isMissionRunning(); };
    if (is_src_param) {
        if (isMovementRunningFun()) {
            SET_ERROR(ERROR_CODE_SET_CONFIG_MOVEMENT_RUNNING, "");
            response_result->set_result_state(proto::ResponseResult::RESPONSE_FAILED);
            response_result->set_result_code(ERROR_CODE_SET_CONFIG_MOVEMENT_RUNNING);
            server_.send(mm);
            return;
        }
        if (isActionRunningFun()) {
            SET_ERROR(ERROR_CODE_SET_CONFIG_ACTION_RUNNING, "");
            response_result->set_result_state(proto::ResponseResult::RESPONSE_FAILED);
            response_result->set_result_code(ERROR_CODE_SET_CONFIG_ACTION_RUNNING);
            server_.send(mm);
            return;
        }
        if (isMissionRunningFun()) {
            SET_ERROR(ERROR_CODE_SET_CONFIG_MISSION_RUNNING, "");
            response_result->set_result_state(proto::ResponseResult::RESPONSE_FAILED);
            response_result->set_result_code(ERROR_CODE_SET_CONFIG_MISSION_RUNNING);
            server_.send(mm);
            return;
        }
    }

    bool ret = s.setItemInfoList(item_info_list);
    if (!ret) {
        response_result->set_result_state(proto::ResponseResult::RESPONSE_FAILED);
        server_.send(mm);
        return;
    }

    if (is_src_param) {
        auto r = updateSRCParameters();
        if (!r) {
            // TODO(nobody): 如果update失败，应该将数据库中的配置项改为从SRC读取到的值
            bool ret = s.setItemInfoList(item_info_list_src);
            if (!ret) {
                LOG(ERROR) << "database src config has be update but setSrcParamters wrong";
            }
            response_result->set_result_state(proto::ResponseResult::RESPONSE_FAILED);
            server_.send(mm);
            return;
        }
    }

    response_result->set_result_state(proto::ResponseResult::RESPONSE_OK);
    sendMsgToNetwork(mm);
}

// 从db中读取所有src相关配置项，写入src中，最后从src中读取并校验
bool NetworkModule::updateSRCParameters() {
    int total_src_item_counts = 0;
    auto settings_list = sros::core::Settings::getInstance().getItemInfoLists();
    for (auto item : settings_list) {
        auto section_name = sros::core::Settings::getSectionName(item.key);

        if (section_name == "src") {
            total_src_item_counts++;
        }
    }

    total_src_item_counts += 1;  // 加上地址0的保留项

    int start_id = 600;  // SRC的参数在SROS参数表中的起始id（地址）
    int end_id = start_id + total_src_item_counts;

    std::vector<int> values;
    for (int id = start_id; id < end_id; id++) {
        auto item = sros::core::Settings::findItemByID(id, settings_list);

        int value = 0;
        try {
            value = stoi(item.value);
        } catch (exception &e) {
            LOGGER(WARNING, SROS) << "updateSRCParameters() : std::stoi() exception: id: " << item.id << ", key "
                                  << item.key << ", value " << item.value << "(" << value << ") -> " << e.what();
        }

        values.push_back(value);

        LOGGER(INFO, SROS) << "SRC_parameter_item: id " << item.id << ", key " << item.key << ", value " << item.value
                           << "(" << value << ")";
    }

    auto r = src_sdk->setParameters(0, values, 5000);

    if (!r) {
        LOGGER(ERROR, SROS) << "src_sdk->setParameters() return false";
        return false;
    }

    // 读回所有参数做校验
    auto car_values = src_sdk->getParameters(0, total_src_item_counts, 5000);

    if (car_values.size() != values.size()) {
        LOGGER(ERROR, SROS) << "Failed: value count mismatch : " << car_values.size() << " != " << values.size();
        return false;
    }

    for (int i = 0; i < car_values.size(); i++) {
        if (car_values[i] != values[i]) {
            LOGGER(ERROR, SROS) << "Failed: value mismatch at " << i << " : " << car_values[i] << " != " << values[i];
            return false;
        }
    }

    LOGGER(INFO, SROS) << "SRC parameter verification passed";

    return true;
}

void NetworkModule::handleRequestLogout(const proto::Message_ptr &msg) {
    auto session_id = msg->session_id();

    auto session = session_manager_.getItem(session_id);
    if (!session) {
        LOGGER(ERROR, PROTOBUF) << "invalid session id ! session id = " << session_id;
        return;
    }

    auto &s = sros::core::UserManager::getInstance();
    auto user_item = s.getUserItem(session->username);
    if (!user_item.username.empty()) {
        // decrease online count
        if (user_item.item_valid > 0) {
            user_item.item_valid--;
            s.setUserItem(user_item);
        }
    }

    // 构造回复消息
    auto mm = make_shared<proto::Message>();
    mm->set_type(proto::MSG_RESPONSE);
    mm->set_seq(msg->seq());
    mm->set_session_id(session_id);

    auto response_msg = mm->mutable_response();
    response_msg->set_response_type(proto::Response::RESPONSE_LOGOUT);

    auto response_result = response_msg->mutable_result();
    response_result->set_result_state(proto::ResponseResult::RESPONSE_OK);

    sendMsgToNetwork(mm);

    if (g_state.control_mutex.isLock(session->session_id)) {
        g_state.control_mutex.unlock();
    }
    // 要发送完RESPONSE_LOGOUT消息后再清除session
    session_manager_.removeItem(session);

    updateIfNeedAvoidObstaclePrediction();

    updateIfNeedDebugInfo();

    if (session && session->username == USERNAME_FMS) {
        g_state.updateFleetState();
    }
}

void NetworkModule::handleRequestLogin(const proto::Message_ptr &msg, const std::string &peer_ip, unsigned short port) {
    auto m = msg->mutable_request();
    auto login_request = m->mutable_login_request();

    auto mm = make_shared<proto::Message>();
    mm->set_type(proto::MSG_RESPONSE);
    mm->set_seq(msg->seq());

    auto response_msg = mm->mutable_response();
    response_msg->set_response_type(proto::Response::RESPONSE_LOGIN);
    auto response_result = response_msg->mutable_result();

    const auto &username = login_request->username();
    const auto &password = login_request->password();

    if (!checklogin(username, login_request->password())) {
        // 用户名或密码错误
        response_result->set_result_state(proto::ResponseResult::RESPONSE_FAILED);
        SET_ERROR(sros::core::ERROR_CODE_USER_OR_PASSWORD_INVALID, username);
        response_result->set_result_code(sros::core::ERROR_CODE_USER_OR_PASSWORD_INVALID);

        // 这里不能用SendMsgToNetwork()
        server_.send(mm, peer_ip, port);
        std::thread t([&] {
            std::this_thread::sleep_for(std::chrono::milliseconds(80));
            server_.close(peer_ip, port);
        });
        t.detach();

        return;
    }

    // 用户设置的session_id
    auto user_set_session_id = msg->session_id();
    LOG(INFO) << "user_set_session_id " << user_set_session_id;

    auto &s = sros::core::Settings::getInstance();
    bool enable_upload_laser_point = s.getValue<std::string>("main.enable_upload_laser_point", "False") == "True";
    bool enable_upload_avoid_obstacle_prediction =
        s.getValue<std::string>("nav.enable_avoid_obstacle_debug", "False") == "True";

    ProtobufSessionItem_ptr session;
    if (user_set_session_id == 0) {
        session = dynamic_pointer_cast<ProtobufSessionItem>(session_manager_.addItem(
            std::make_shared<ProtobufSessionItem>(username, peer_ip, port, enable_upload_laser_point)));
    } else {
        // 检查session_id是否存在，如果存在可以重用
        session = dynamic_pointer_cast<ProtobufSessionItem>(session_manager_.getItem(user_set_session_id));

        // 如果用户名或IP地址不同，那么新建一个session
        if (session && session->session_id == user_set_session_id && session->username == username &&
            session->ip_addr == peer_ip) {
            // 重用session
            LOG(INFO) << "reuse the session id " << session->session_id << " new connection is " << session->ip_addr
                      << " " << port;
            // 重连的时候，可能老的socket还存在，为了防止出网络延迟导致还从老的socket中传来数据，需要将老的socket关闭掉。
            if (session->ip_port != port) {
                LOG(INFO) << "close old socket " << session->ip_addr << " " << session->ip_port;
                // 此处会触发disconnect without logout，然后再下一条指令，继续设置为connected
                server_.close(peer_ip, session->ip_port);
            }

            session_manager_.toggleItemConnected(session->session_id, true);
            session_manager_.updateItemIPPort(session->session_id, port);  // 需要更新port
            session->is_upload_laser_point_ = enable_upload_laser_point;  // 重新设置是否上传雷达点，为兼容chip
            session->is_upload_avoid_obstacle_prediction_ =
                enable_upload_avoid_obstacle_prediction;  // 重新设置是否上传避障调试模式，为兼容chip
        } else {
            session = dynamic_pointer_cast<ProtobufSessionItem>(session_manager_.addItem(
                std::make_shared<ProtobufSessionItem>(username, peer_ip, port, enable_upload_laser_point)));
        }
    }
    if (!session) {
        LOG(ERROR) << "invalid session!";
        return;
    }

    // fms 登录后，若是调度模式，fms独占
    if (session->username == USERNAME_FMS) {
        // 若当前已经有了fms登录，后面来的fm是都不允许登录，防止两个fms来回刷的问题
        auto session_list = session_manager_.getItemList();
        for (const auto &item : session_list) {
            if (item->username == USERNAME_FMS && item->is_connected && item->session_id != session->session_id) {
                LOG(INFO) << "Last connected fms session, ip is " << item->ip_addr << " and port is " << item->ip_port
                          << ", remove current fms session";

                // 用户名或密码错误
                response_result->set_result_state(proto::ResponseResult::RESPONSE_FAILED);
                SET_ERROR(sros::core::ERROR_CODE_USER_FMS_ALREADY_EXISTS, username);
                response_result->set_result_code(sros::core::ERROR_CODE_USER_FMS_ALREADY_EXISTS);

                // 这里不能用SendMsgToNetwork()
                server_.send(mm, peer_ip, port);
                std::thread t(
                    [&](std::string session_ip, unsigned short session_port, uint64_t cur_session_id) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(80));
                        server_.close(session_ip, session_port);
                        session_manager_.toggleItemConnected(cur_session_id, false);
                    },
                    peer_ip, port, session->session_id);
                t.detach();

                return;
            }
        }

        g_state.updateFleetState();
        session->is_upload_system_state = false;
        session->is_upload_laser_point_ = false;
        session->is_upload_avoid_obstacle_prediction_ = false;
        session->is_upload_hardware_state = false;
    }

    // Will remove in v7.0
    auto login_info = response_msg->mutable_login_info();
    login_info->set_session_id(session->session_id);

    response_result->set_result_state(proto::ResponseResult::RESPONSE_OK);
    response_result->set_result_code(ERROR_CODE_NONE);

    // 在 RESPONSE_LOGIN 中返回 SystemState，用于 FMS 在重连后检测任务状态
    auto system_state = response_msg->mutable_system_state();
    buildProtoSystemState(system_state);

    mm->set_session_id(session->session_id);

    LOGGER(INFO, PROTOBUF) << "Response for login succeed!, session_id is " << session->session_id;

    sendMsgToNetwork(mm);
}

void NetworkModule::handleChangePassword(const proto::Message_ptr &msg) {
    auto session_id = msg->session_id();
    auto m = msg->mutable_request();

    const auto &origin_passwd = m->param_str1();
    const auto &new_passwd = m->param_str2();

    auto mm = make_shared<proto::Message>();
    mm->set_type(proto::MSG_RESPONSE);
    mm->set_seq(msg->seq());
    mm->set_session_id(session_id);

    auto response_msg = mm->mutable_response();
    response_msg->set_response_type(proto::Response::RESPONSE_CHANGE_PW);
    auto response_result = response_msg->mutable_result();

    auto session = session_manager_.getItem(session_id);
    if (!session) {
        LOG(ERROR) << "invalid session id ! sessionid = " << session_id;
        return;
    }

    const auto &username = session->username;

    auto &s = sros::core::UserManager::getInstance();
    auto user_item = s.getUserItem(session->username);

    if (user_item.username != username || user_item.passwd != origin_passwd) {
        SET_ERROR(sros::core::ERROR_CODE_USER_OR_PASSWORD_INVALID, username,
                  "failed to change password(invalid username or password");
        response_result->set_result_code(ERROR_CODE_USER_OR_PASSWORD_INVALID);
        sendMsgToNetwork(mm);
        return;
    }

    user_item.passwd = new_passwd;
    s.setUserItem(user_item);  // 更新密码到数据库中
    LOGGER(INFO, SROS) << username << " has changed password";

    response_result->set_result_state(proto::ResponseResult::RESPONSE_OK);
    sendMsgToNetwork(mm);
}

bool NetworkModule::checklogin(const std::string &username, const std::string &password) {
    auto &s = sros::core::UserManager::getInstance();

    LOGGER(INFO, PROTOBUF) << "Input username is " << username << " and password is ***";

    if (username.empty()) {
        LOGGER(WARNING, PROTOBUF) << "Login authentication failed! input username is empty!";
        return false;
    }

    auto user_item = s.getUserItem(username);
    if (user_item.username.empty()) {
        LOGGER(WARNING, PROTOBUF) << "Login authentication failed! user not exists!";
        return false;
    }

    if (user_item.username == username && user_item.passwd == password) {
        LOGGER(INFO, PROTOBUF) << "Login authentication passed! username is " << username;

        // update login table
        user_item.login_time = sros::core::util::get_timestamp_in_ms();
        user_item.item_valid++;

        s.setUserItem(user_item);

        return true;
    } else {
        LOGGER(WARNING, PROTOBUF) << "Login authentication failed! username or password error!";
        return false;
    }
}

void NetworkModule::handleRequestFileList(const proto::Message_ptr &msg) {
    auto m = msg->mutable_request();

    // get file path
    const auto &file_path = m->param_str();
    const auto &file_type = m->param_str1();

    std::vector<std::string> file_name_list;

    // response
    auto mm = make_shared<proto::Message>();
    mm->set_type(proto::MSG_RESPONSE);
    mm->set_seq(msg->seq());
    mm->set_session_id(msg->session_id());

    auto response_msg = mm->mutable_response();
    response_msg->set_response_type(proto::Response::RESPONSE_FILE_LIST);

    auto response_result = response_msg->mutable_result();

    namespace bf = boost::filesystem;

    if (file_path.empty() || !bf::exists(bf::path(file_path))) {
        LOG(WARNING) << "Request file list error (parameter not specified!), path: " << file_path;

        response_result->set_result_state(proto::ResponseResult::RESPONSE_FAILED);
        sendMsgToNetwork(mm);
        return;
    }

    LOGGER(INFO, PROTOBUF) << "handleRequestFileList path : " << file_path << ", type: " << file_type;

    bf::directory_iterator end;
    if (file_type.empty()) {
        for (bf::directory_iterator pos(file_path); pos != end; ++pos) {
            auto filename = pos->path().filename();
            file_name_list.push_back(filename.string());
        }
    } else {
        if (file_type == "monitor") {
            for (bf::directory_iterator pos(file_path); pos != end; ++pos) {
                auto filename = pos->path().filename();
                if (filename.extension().string() == "mf") {
                    file_name_list.push_back(filename.string());
                }
            }
        } else if (file_type == "log") {
        } else if (file_type == "map") {
            for (bf::directory_iterator pos(file_path); pos != end; ++pos) {
                auto filename = pos->path().filename();
                if (filename.extension().string() == "map") {
                    file_name_list.push_back(filename.string());
                }
            }
        } else if (file_type == "record") {
        } else if (file_type == "cfg") {
        }
    }

    response_result->set_result_state(proto::ResponseResult::RESPONSE_OK);
    auto file_list_msg = response_msg->mutable_list();

    for (const auto &p : file_name_list) {
        LOG(INFO) << "find file:  " << p;
        auto proto_file_info = file_list_msg->add_list();
        proto_file_info->set_name(bf::path(p).filename().string());
    }

    sendMsgToNetwork(mm);
}

std::vector<std::string> NetworkModule::getMapFilesPathByName(const std::string &map_name,
                                                              const proto::Request::FileOperateType &file_op) {
    std::vector<std::string> file_paths;

    if (file_op == proto::Request::EXPORT_NAV_MAP_FILE || file_op == proto::Request::EXPORT_MAP_FILE) {
        file_paths = sros::core::getNavMapFilesPath(map_name);
    } else if (file_op == proto::Request::EXPORT_OPT_MAP_FILE) {
        file_paths = sros::core::getOptimizeMapFilesPath(map_name);
    } else if (file_op == proto::Request::EXPORT_ALL_MAP_FILE) {
        file_paths = sros::core::getAllMapFilesPath(map_name);
    }

    return file_paths;
}

void NetworkModule::handleFileOperate(const proto::Message_ptr &msg, const std::string &peer_ip_str) {
    auto seq = msg->seq();
    auto session_id = msg->session_id();

    sros::core::MapManager map_manager;
    map_manager.freshMapList();

    auto m = msg->mutable_request();
    auto op_type = m->file_op();

    std::vector<std::string> map_names;
    map_names.push_back(m->param_str());

    auto getControlMutexFun = [&]() {
        if (!g_state.control_mutex.get(session_id)) {
            auto locker = g_state.control_mutex.getLocker();
            SET_ERROR(ERROR_CODE_CONTROL_MUTEX_IS_LOCKED, "locker:", locker.session_id, "current:", session_id);
            ResponseFileOperate(proto::ResponseResult::RESPONSE_FAILED, seq, session_id,
                                ERROR_CODE_CONTROL_MUTEX_IS_LOCKED);
            return false;
        }
        return true;
    };

    //================== GET_NAV_MAP_FILE + GET_RAW_MAP_FILE
    if (op_type == proto::Request::GET_NAV_MAP_FILE || op_type == proto::Request::GET_RAW_MAP_FILE) {
        auto map_name = m->param_str();
        sros::core::MapInfo map_info = map_manager.getMapInfo(map_name);
        if (map_info.name_ != map_name) {
            LOG(ERROR) << "Can't find map: " << map_name;
            ResponseFileOperate(proto::ResponseResult::RESPONSE_FAILED, seq, session_id);
            return;
        }

        std::string map_path;
        if (op_type == proto::Request::GET_RAW_MAP_FILE) {  // 获取原始pgm地图
            map_path = map_info.raw_pgm_path_;
        } else {
            map_path = map_info.file_path_;
        }

        LOG(INFO) << "TASK_FILE_OPERATE: map_name: " << map_name << ", file_path: " << map_path;

        if (is_in_file_transmit_) {
            LOG(WARNING) << "already in file transmit mode!";
            ResponseFileOperate(proto::ResponseResult::RESPONSE_FAILED, seq, session_id);
            return;
        }

        using namespace sros::ft;
        Filetransmitter_ptr sender(new sros::ft::Filetransmitter(map_path, sros::ft::COMPRESS_GZIP,
                                                                 sros::ft::SPEED_5Mbps));  // sending
        boost::thread(boost::bind(&NetworkModule::sendFileThread, this, sender, seq, session_id));

        //================== EXPORT_MAP_FILE
    } else if (op_type == proto::Request::EXPORT_MAP_FILE || op_type == proto::Request::EXPORT_NAV_MAP_FILE ||
               op_type == proto::Request::EXPORT_OPT_MAP_FILE || op_type == proto::Request::EXPORT_ALL_MAP_FILE) {
        std::string map_names_str;

        for (std::string map_name : map_names) {
            auto file_paths = getMapFilesPathByName(map_name, op_type);
            for (auto file_path : file_paths) {
                map_names_str += " " + file_path;
            }
        }

        LOG(INFO) << "TASK_FILE_OPERATE: export " << map_names_str;

        if (is_in_file_transmit_) {
            LOG(WARNING) << "already in file transmit mode!";
            ResponseFileOperate(proto::ResponseResult::RESPONSE_FAILED, seq, session_id);
            return;
        }

        // 使用新线程执行耗时的压缩打包过程
        boost::thread(
            boost::bind(&NetworkModule::sendExportMapThread, this, map_names_str, peer_ip_str, seq, session_id));

        //================== SET_NAV_MAP_FILE + SET_RAW_MAP_FILE
    } else if (op_type == proto::Request::SET_NAV_MAP_FILE || op_type == proto::Request::SET_RAW_MAP_FILE) {
        if (!getControlMutexFun()) {
            return;
        }

        std::string map_save_to;

        auto map_name = m->param_str();
        sros::core::MapInfo map_info = map_manager.getMapInfo(map_name);
        if (map_info.name_ != map_name) {
            map_save_to = MAP_FILE_DIR + map_name;
            LOG(ERROR) << "Can't find map: " << map_name << " save this map file as " << map_save_to;
            // ResponseFileOperate(proto::ResponseResult::RESPONSE_FAILED, seq, session_id);
            // return;
        } else {
            map_save_to = map_info.file_path_;
        }

        LOG(INFO) << "TASK_FILE_OPERATE: SET_NAV_MAP_FILE map_name: " << map_name;
        if (is_in_file_transmit_) {
            LOG(WARNING) << "already in file transmit mode!";
            ResponseFileOperate(proto::ResponseResult::RESPONSE_FAILED, seq, session_id);
            return;
        }

        // 接收地图文件并将其保存到临时目录下, 全部接收结束后再覆盖原地图文件
        using namespace sros::ft;
        Filetransmitter_ptr recver(new sros::ft::Filetransmitter(SAVE_MAP_FILE_TEMP_PATH, sros::ft::COMPRESS_GZIP,
                                                                 sros::ft::SPEED_5Mbps));  // sending
        boost::thread(boost::bind(&NetworkModule::recvMapFileThread, this, recver, map_save_to, seq, session_id));

        //================== IMPORT_MAP_FILE
    } else if (op_type == proto::Request::IMPORT_MAP_FILE) {
        LOG(INFO) << "TASK_FILE_OPERATE: IMPORT_MAP_FILE";
        if (!getControlMutexFun()) {
            return;
        }

        if (is_in_file_transmit_) {
            LOG(WARNING) << "already in file transmit mode!";
            ResponseFileOperate(proto::ResponseResult::RESPONSE_FAILED, seq, session_id);
            return;
        }

        using namespace sros::ft;
        Filetransmitter_ptr recver(new sros::ft::Filetransmitter(IMPORT_MAP_FILE_PATH, sros::ft::COMPRESS_GZIP,
                                                                 sros::ft::SPEED_5Mbps));  // sending
        boost::thread(boost::bind(&NetworkModule::recvImportMapThread, this, recver, seq, session_id));

        //================== DELETE_MAP_FILE
    } else if (op_type == proto::Request::DELETE_MAP_FILE) {
        if (!getControlMutexFun()) {
            return;
        }

        std::string map_names_str;

        for (std::string map_name : map_names) {
            auto file_paths = sros::core::getAllMapFilesPath(map_name);
            for (auto file_path : file_paths) {
                map_names_str += " " + file_path;
            }
        }

        LOG(INFO) << "TASK_FILE_OPERATE: DELETE_MAP_FILE " << map_names_str;

        std::string command_str = "rm -f " + map_names_str;

        systemWrapper(command_str);  // 删除地图文件

        // 若删除的当前地图，将地图名设置为NO_MAP
        if (map_names_str == g_state.getCurMapName()) {
            auto mm = make_shared<sros::core::CommandMsg>(getName());
            mm->command = sros::core::CMD_SET_CUR_MAP;
            mm->map_name = sros::core::NO_MAP;

            sendMsg(mm);
        }

        ResponseFileOperate(proto::ResponseResult::RESPONSE_OK, seq, session_id);

        //================== RENAME_MAP_FILE
    } else if (op_type == proto::Request::RENAME_MAP_FILE) {
        if (!getControlMutexFun()) {
            return;
        }

        const std::string &map_name_from = m->param_str();  // 原地图名
        const std::string &map_name_to = m->param_str1();   // 要修改为的名称

        if (map_name_from.empty()) {
            ResponseFileOperate(proto::ResponseResult::RESPONSE_FAILED, seq, session_id,
                                ERROR_CODE_MAP_RENAME_ORIGINAL_NAME_IS_EMPTY);
            return;
        }
        if (map_name_to.empty()) {
            ResponseFileOperate(proto::ResponseResult::RESPONSE_FAILED, seq, session_id,
                                ERROR_CODE_MAP_RENAME_DESTINATION_NAME_IS_EMPTY);
            return;
        }

        auto renameFileFun = [&]() {
            auto from_names = sros::core::getAllMapFilesPath(map_name_from);
            auto to_names = sros::core::getAllMapFilesPath(map_name_to);

            std::string command_str;
            for (int i = 0; i < from_names.size(); i++) {
                command_str = "mv " + from_names[i] + " " + to_names[i];
                systemWrapper(command_str);  // 由于有些地图名不一定存在，所以要分开运行
            }

            LOG(INFO) << "TASK_FILE_OPERATE: RENAME_MAP_FILE " << map_name_from << " -> " << map_name_to;
        };

        // 最好不要修改当前地图的地图名
        if (map_name_from == g_state.getCurMapName()) {
            auto isMovementRunningFun = []() {
                auto cur_task = sros::core::TaskManager::getInstance()->getMovementTask();
                return cur_task && cur_task->isRunning();
            };
            auto isMissionRunningFun = []() { return MissionManager::getInstance()->isMissionRunning(); };

            if(isMovementRunningFun() || isMissionRunningFun() || g_state.main_state != STATE_IDLE) {
                ResponseFileOperate(proto::ResponseResult::RESPONSE_FAILED, seq, session_id,
                                            ERROR_CODE_MAP_RENAME_CURRENT_MAP_IN_USE);
                return;
            }

            renameFileFun();
            LOGGER(INFO, CMD_HANDER)<< "rename current  map, so set current map to " << map_name_to;
            if (!MapManager::getInstance()->setCurrentMap(map_name_to)) {
                ResponseFileOperate(proto::ResponseResult::RESPONSE_FAILED, seq, session_id,
                                            ERROR_CODE_SET_MAP_MAP_LOAD_ERROR);
                return;
            }
            g_state.setCurMapName(map_name_to);
            ResponseFileOperate(proto::ResponseResult::RESPONSE_OK, seq, session_id);
            return;
        }

        renameFileFun();
        ResponseFileOperate(proto::ResponseResult::RESPONSE_OK, seq, session_id);

        //================== EXPORT_CONFIG_FILE
    } else if (op_type == proto::Request::EXPORT_CONFIG_FILE) {
        auto config_name = m->param_str();

        if (config_name != "main") {  // TODO(nobody): 支持多配置文件
            LOG(ERROR) << "Can't find config: " << config_name;
            ResponseFileOperate(proto::ResponseResult::RESPONSE_FAILED, seq, session_id);
            return;
        }

        LOG(INFO) << "TASK_GET_CONFIG_FILE: " << config_name;
        //================== IMPORT_CONFIG_FILE
    } else if (op_type == proto::Request::IMPORT_CONFIG_FILE) {
        if (!getControlMutexFun()) {
            return;
        }

        auto config_name = m->param_str();

        if (config_name != "main") {
            LOG(ERROR) << "Can't find config: " << config_name;
            ResponseFileOperate(proto::ResponseResult::RESPONSE_FAILED, seq, session_id);
            return;
        }

        LOG(INFO) << "TASK_SET_CONFIG_FILE: " << config_name;
        //================== EXPORT_LOG_FILE
    } else if (op_type == proto::Request::EXPORT_LOG_FILE) {
        std::string export_type = m->param_str();

        if (is_in_file_transmit_) {
            LOG(WARNING) << "already in file transmit mode!";
            ResponseFileOperate(proto::ResponseResult::RESPONSE_FAILED, seq, session_id);
            return;
        }

        using namespace sros::ft;
        namespace fs = boost::filesystem;
        if (export_type == "single") {
            // common file send
            std::string log_files;
            if (fs::exists(fs::path(LOG_FILE_DIR + m->param_str()))) {
                log_files = LOG_FILE_DIR + m->param_str();
            } else if (fs::exists(fs::path(LOG_TEMP_DIR + m->param_str()))) {
                log_files = LOG_TEMP_DIR + m->param_str();
            } else {
                LOG(WARNING) << "Specified log file does not exists!";
                ResponseFileOperate(proto::ResponseResult::RESPONSE_FAILED, seq, session_id);
                return;
            }
            Filetransmitter_ptr sender(new sros::ft::Filetransmitter(log_files, sros::ft::COMPRESS_NONE,
                                                                     sros::ft::SPEED_5Mbps));  // sending
            boost::thread(boost::bind(&NetworkModule::sendFileThread, this, sender, seq, session_id));
        } else if (export_type == "multi") {
            multiLogExport(seq, session_id);
        } else if (export_type == "classic") {
            classicLogExport(seq, session_id);
        }

        //================== IMPORT_SYSTEM_UPDATE_FILE
    } else if (op_type == proto::Request::IMPORT_SYSTEM_UPDATE_FILE) {
        if (!getControlMutexFun()) {
            return;
        }

        LOG(INFO) << "TASK_FILE_OPERATE: IMPORT_SYSTEM_UPDATE_FILE";

        if (is_in_file_transmit_) {
            LOG(WARNING) << "already in file transmit mode!";
            ResponseFileOperate(proto::ResponseResult::RESPONSE_FAILED, seq, session_id);
            return;
        }

        using namespace sros::ft;
        Filetransmitter_ptr recver(new sros::ft::Filetransmitter(sros::IMPORT_UPDATE_FILE_PATH, sros::ft::COMPRESS_NONE,
                                                                 sros::ft::SPEED_5Mbps));  // sending
        boost::thread(boost::bind(&NetworkModule::recvImportUpdateThread, this, recver, seq, session_id));
    } else if (op_type == proto::Request::RESTORE_FACTORY_SETTINGS) {
        if (!getControlMutexFun()) {
            return;
        }

        LOG(INFO) << "TASK_FILE_OPERATE: RESTORE_FACTORY_FILE";
        boost::thread(boost::bind(&NetworkModule::restoreFactorySettings, this, session_id, seq));

    } else if (op_type == proto::Request::BACKUP_AS_FACTORY_SETTINGS) {
        if (!getControlMutexFun()) {
            return;
        }

        LOG(INFO) << "TASK_FILE_OPERATE: RESTORE_FACTORY_FILE";
        // TODO(nobody): 功能待实现
        boost::thread(boost::bind(&NetworkModule::backupFactorySettings, this, session_id, seq));
        // TODO(nobody): 恢复stm32中的程序为出厂状态

    } else if (op_type == proto::Request::GET_COMMON_FILES || op_type == proto::Request::SET_COMMON_FILES) {
        if (!getControlMutexFun()) {
            return;
        }

        // use this file server to send or receive common files
        LOG(INFO) << "TASK_FILE_OPERATE: GET_COMMON_FILES( file transmit server started!)";
        if (is_in_file_transmit_) {
            LOG(WARNING) << "already in file transmit mode!";
            ResponseFileOperate(proto::ResponseResult::RESPONSE_FAILED, seq, session_id);
            return;
        }
        using namespace sros::ft;
        auto path = m->param_str();
        if (path.empty() && op_type == proto::Request::SET_COMMON_FILES) {
            path = "/tmp/";
        } else if (op_type == proto::Request::GET_COMMON_FILES) {
            path = "";
        }

        Filetransmitter_ptr file_server(
            new sros::ft::Filetransmitter(path.c_str(), sros::ft::COMPRESS_GZIP, sros::ft::SPEED_5Mbps));
        boost::thread(boost::bind(&NetworkModule::FileServerThread, this, file_server, seq, session_id));
    }
}

void NetworkModule::multiLogExport(uint32_t seq, uint64_t session_id) {
    using namespace boost::filesystem;

    std::string file_names_str;

    directory_iterator end;
    for (directory_iterator pos(LOG_FILE_DIR); pos != end; ++pos) {
        path p = pos->path();
        file_names_str += " " + p.string();
        LOG(INFO) << "--> [*] " << p.string();
    }

    generateConfigFile();

    if (exists(path(STM32_UPDATE_LOG))) {
        file_names_str += string(" ") + STM32_UPDATE_LOG;
    }

    if (exists(path(CONFIG_FILE))) {
        file_names_str += string(" ") + CONFIG_FILE;
    }

    if (exists(path(SROS_UPDATE_LOG))) {
        file_names_str += string(" ") + SROS_UPDATE_LOG;
    }

    LOG(INFO) << "TASK_FILE_OPERATE: EXPORT_LOG_FILE " << file_names_str;

    std::string command_str = "tar cvzf - " + file_names_str + " > " + EXPORT_MULTI_LOG_FILE_PATH;

    LOG(INFO) << "\"" << command_str << "\"";

    if (!systemWrapper(command_str)) {
        ResponseFileOperate(proto::ResponseResult::RESPONSE_FAILED, seq, session_id);
        return;
    }

    using namespace sros::ft;
    Filetransmitter_ptr sender(new sros::ft::Filetransmitter(EXPORT_MULTI_LOG_FILE_PATH, sros::ft::COMPRESS_GZIP,
                                                             sros::ft::SPEED_5Mbps));  // sending
    boost::thread(boost::bind(&NetworkModule::sendFileThread, this, sender, seq, session_id));
}

void NetworkModule::classicLogExport(uint32_t seq, uint64_t session_id) {
    using namespace boost::filesystem;
    auto cur_timestamp = sros::core::util::get_timestamp_in_ms() / 1000;

    std::string file_names_str;
    std::string file_to_remove_str;
    std::string command_str;

    directory_iterator end;
    for (directory_iterator pos(LOG_FILE_DIR); pos != end; ++pos) {
        path p = pos->path();
        std::string filename = p.filename().string();

        uint64_t last_time_w = last_write_time(path(p.string()));
        if (filename.find("sros.") != std::string::npos) {
            if ((cur_timestamp > last_time_w) && (cur_timestamp - last_time_w) > MAX_TIME_GAP) {
                file_to_remove_str += " " + p.string();
            } else {
                file_names_str += " " + p.string();
                LOG(INFO) << "--> [*] " << p.string();
            }
        }
    }

    if (file_names_str.size() == 0 && file_to_remove_str.size()) {
        // remove cancelled
        file_names_str = file_to_remove_str;
        file_to_remove_str = "";
    } else if (file_to_remove_str.size()) {
        // remove log file which was not generated in last two days
        command_str = "rm " + file_to_remove_str;
        systemWrapper(command_str);
    }

    generateConfigFile();

    if (exists(path(STM32_UPDATE_LOG))) {
        file_names_str += string(" ") + STM32_UPDATE_LOG;
    }

    if (exists(path(CONFIG_FILE))) {
        file_names_str += string(" ") + CONFIG_FILE;
    }

    if (exists(path(SROS_UPDATE_LOG))) {
        file_names_str += string(" ") + SROS_UPDATE_LOG;
    }
    // LOG(INFO) << "TASK_FILE_OPERATE: EXPORT_LOG_FILE " << file_names_str;

    command_str = "tar cvzf - " + file_names_str + " > " + EXPORT_LOG_FILE_PATH;

    LOG(INFO) << "\"" << command_str << "\"";

    if (!systemWrapper(command_str)) {
        ResponseFileOperate(proto::ResponseResult::RESPONSE_FAILED, seq, session_id);
        return;
    }

    using namespace sros;
    ft::Filetransmitter_ptr sender(new ft::Filetransmitter(EXPORT_LOG_FILE_PATH, ft::COMPRESS_GZIP,
                                                           ft::SPEED_5Mbps));  // sending
    boost::thread(boost::bind(&NetworkModule::sendFileThread, this, sender, seq, session_id));
}

void NetworkModule::generateConfigFile() {
    auto &s = sros::core::Settings::getInstance();
    sros::core::ItemInfoLists item_info_list = s.getItemInfoLists();
    if (item_info_list.empty()) {
        LOGGER(WARNING, PROTOBUF) << "Loading config (DB empty)";
        return;
    }

    std::ofstream out(CONFIG_FILE, std::ofstream::out | std::ofstream::binary);
    if (!out) {
        LOGGER(ERROR, PROTOBUF) << "Failed to create config file";
        return;
    }

    for (auto p : item_info_list) {
        std::string tmp = p.key + "," + p.name + "," + p.value + "\n";
        out.write(tmp.c_str(), tmp.size());
    }

    out.close();
}

bool NetworkModule::backupFactorySettings(uint64_t session_id, uint32_t seq) {
    auto &s = sros::core::Settings::getInstance();
    sros::core::ItemInfoLists item_info_list = s.getItemInfoLists();
    if (item_info_list.empty()) {
        LOGGER(WARNING, PROTOBUF) << "Loading config (DB empty)";
        return false;
    }

    sros::core::ItemInfoLists backup_list;
    for (auto p : item_info_list) {
        if (p.value != p.default_value) {
            p.default_value = p.value;
            backup_list.push_back(p);
        }
    }

    if (!backup_list.empty()) {
        if (s.setItemInfoList(backup_list)) {
            ResponseFileOperate(proto::ResponseResult::RESPONSE_OK, seq, session_id);
            return true;
        } else {
            ResponseFileOperate(proto::ResponseResult::RESPONSE_FAILED, seq, session_id);
            return false;
        }
    } else {
        LOGGER(INFO, PROTOBUF) << "Nothing to backup !";
        ResponseFileOperate(proto::ResponseResult::RESPONSE_OK, seq, session_id);
        return true;
    }
}

bool NetworkModule::restoreFactorySettings(uint64_t session_id, uint32_t seq) {
    auto &s = sros::core::Settings::getInstance();
    sros::core::ItemInfoLists item_info_list = s.getItemInfoLists();
    if (item_info_list.empty()) {
        LOGGER(WARNING, PROTOBUF) << "Loading config (DB empty)";
        return false;
    }

    sros::core::ItemInfoLists restore_list;
    for (auto p : item_info_list) {
        if (p.value != p.default_value) {
            p.value = p.default_value;
            restore_list.push_back(p);
        }
    }

    if (!restore_list.empty()) {
        if (s.setItemInfoList(restore_list)) {
            ResponseFileOperate(proto::ResponseResult::RESPONSE_OK, seq, session_id);
            return true;
        } else {
            ResponseFileOperate(proto::ResponseResult::RESPONSE_FAILED, seq, session_id);
            return true;
        }
    } else {
        LOGGER(INFO, PROTOBUF) << "Nothing to restore!";
        ResponseFileOperate(proto::ResponseResult::RESPONSE_OK, seq, session_id);
        return true;
    }
}

void NetworkModule::runServer() {
    server_.set_connected_callback_func(boost::bind(&NetworkModule::onConnectCallback, this, _1, _2));
    server_.set_disconnected_callback_func(boost::bind(&NetworkModule::onDisconnectCallback, this, _1, _2));
    server_.set_msg_callback_func(boost::bind(&NetworkModule::msgCallback, this, _1, _2, _3));
    server_.set_msg_sent_callback_func(boost::bind(&NetworkModule::msgSendCallback, this, _1, _2, _3));

    server_.run();
}

bool NetworkModule::needUploadLaserPoint() {
    bool enable = false;
    auto session_list = session_manager_.getItemList();
    for (int i = 0; i < session_list.size(); ++i) {
        auto protobuf_session = dynamic_pointer_cast<ProtobufSessionItem>(session_list.at(i));
        if (protobuf_session->is_connected && protobuf_session->is_upload_laser_point_) {
            enable = true;
            break;
        }
    }
    return enable;
}

bool NetworkModule::needUploadFeatureInfo() {
    bool enable = false;
    auto session_list = session_manager_.getItemList();
    for (int i = 0; i < session_list.size(); ++i) {
        auto protobuf_session = dynamic_pointer_cast<ProtobufSessionItem>(session_list.at(i));
        if (protobuf_session->is_connected &&
            (protobuf_session->need_debug_lidar_fr_ || protobuf_session->need_debug_front_camera_fr_ ||
             protobuf_session->need_debug_back_camera_fr_ || protobuf_session->need_debug_up_camera_fr_ ||
             protobuf_session->need_debug_down_camera_fr_ || protobuf_session->need_debug_left_camera_fr_ ||
             protobuf_session->need_debug_right_camera_fr_)) {
            enable = true;
            break;
        }
    }
    return enable;
}

void NetworkModule::sendFeatureInfoToNetwork(proto::Message_ptr msg) {
    auto session_list = session_manager_.getItemList();
    for (int i = 0; i < session_list.size(); ++i) {
        auto protobuf_session = dynamic_pointer_cast<ProtobufSessionItem>(session_list.at(i));
        if (protobuf_session->is_connected &&
            (protobuf_session->need_debug_lidar_fr_ || protobuf_session->need_debug_front_camera_fr_ ||
             protobuf_session->need_debug_back_camera_fr_ || protobuf_session->need_debug_up_camera_fr_ ||
             protobuf_session->need_debug_down_camera_fr_ || protobuf_session->need_debug_left_camera_fr_ ||
             protobuf_session->need_debug_right_camera_fr_)) {
            msg->set_session_id(protobuf_session->session_id);
            sendMsgToNetwork(msg);
        }
    }
}

void NetworkModule::sendLaserPointMsgToNetwork(proto::Message_ptr msg) {
    auto session_list = session_manager_.getItemList();
    for (int i = 0; i < session_list.size(); ++i) {
        auto protobuf_session = dynamic_pointer_cast<ProtobufSessionItem>(session_list.at(i));
        if (protobuf_session->is_connected && protobuf_session->is_upload_laser_point_) {
            msg->set_session_id(protobuf_session->session_id);
            sendMsgToNetwork(msg);
        }
    }
}

void NetworkModule::sendSystemStateToNetwork(proto::Message_ptr msg) {
    auto session_list = session_manager_.getItemList();
    for (int i = 0; i < session_list.size(); ++i) {
        auto protobuf_session = dynamic_pointer_cast<ProtobufSessionItem>(session_list.at(i));
        if (protobuf_session->is_connected && protobuf_session->is_upload_system_state && protobuf_session->ip_addr == "127.0.0.1") {
            msg->set_session_id(protobuf_session->session_id);
            sendMsgToNetwork(msg);
        }
    }
}

void NetworkModule::sendHardwareStateToNetwork(proto::Message_ptr msg) {
    auto session_list = session_manager_.getItemList();
    for (int i = 0; i < session_list.size(); ++i) {
        auto protobuf_session = dynamic_pointer_cast<ProtobufSessionItem>(session_list.at(i));
        if (protobuf_session->is_connected && protobuf_session->is_upload_hardware_state && protobuf_session->ip_addr == "127.0.0.1") {
            msg->set_session_id(protobuf_session->session_id);
            sendMsgToNetwork(msg);
        }
    }
}

void NetworkModule::onFeatureInfoMsg(sros::core::base_msg_ptr m) {
    LOG(INFO) << "onFeatureInfoMsg";
    if (!needUploadFeatureInfo()) {
        return;
    }
    auto feature_infos = std::dynamic_pointer_cast<sros::core::FeatureInfoMsg>(m);

    auto msg = make_shared<proto::Message>();
    msg->set_type(proto::MSG_RESPONSE);
    msg->set_seq(0);

    auto response_msg = msg->mutable_response();
    response_msg->set_response_type(proto::Response::RESPONSE_FEATURE_INFO);

    int feature_pose_unit = 1e6;
    int feature_point_uint = 1000;

    for (const auto &feature_info : feature_infos->feature_infos) {
        //每一个特征信息
        auto feature_msg = response_msg->add_feature_infos();
        auto pose = feature_msg->mutable_pose();
        pose->set_x(feature_info.pose_in_world_.x() * feature_pose_unit);
        pose->set_y(feature_info.pose_in_world_.y() * feature_pose_unit);
        pose->set_z(feature_info.pose_in_world_.z() * feature_pose_unit);
        pose->set_yaw(feature_info.pose_in_world_.yaw() * feature_pose_unit);
        pose->set_pitch(feature_info.pose_in_world_.pitch() * feature_pose_unit);
        pose->set_roll(feature_info.pose_in_world_.roll() * feature_pose_unit);

        feature_msg->set_feature_type(proto::FeatureInfos_FeatureType(feature_info.code_type_));
        feature_msg->set_feature_name(feature_info.feature_name_);
        feature_msg->set_sensor_name(feature_info.sensor_name_);
        LOG(INFO) << "feature_name:" << feature_info.feature_name_ << ", sensor_name:" << feature_info.sensor_name_;

        for (const auto &point : feature_info.points_) {
            auto feature_point = feature_msg->add_points();
            feature_point->set_x(point.x() * feature_point_uint);
            feature_point->set_y(point.y() * feature_point_uint);
        }
    }

    sendFeatureInfoToNetwork(msg);
}

void NetworkModule::onLaserMsg(sros::core::base_msg_ptr m) {
    auto laser_scan = std::dynamic_pointer_cast<sros::core::LaserScanMsg>(m);

    real_time_detect_info_.setLaserScanInfo(laser_scan);
}

void NetworkModule::onLmkMatchMsg(sros::core::base_msg_ptr m) {
    auto lmk_info = std::dynamic_pointer_cast<sros::core::LmkMatchinfoMsg>(m);
    real_time_detect_info_.setLmkMatchInfo(lmk_info);
}

void NetworkModule::onSetTimerSendCcommonPosesInfo(sros::core::base_msg_ptr m) {
    LOG(INFO) << "onSetTimerSendCcommonPosesInfo";
    common_poses_info_msg_ptr_ = m;
}

void NetworkModule::onNewCommonPosesInfoMsg(sros::core::base_msg_ptr m) {
    auto msg = make_shared<proto::Message>();
    msg->set_type(proto::MSG_RESPONSE);
    msg->set_seq(0);

    auto response_msg = msg->mutable_response();
    response_msg->set_response_type(proto::Response::RESPONSE_COMMON_POSE_INFO);

    auto common_poses_info_proto = response_msg->mutable_common_poses_info();
    auto common_poses_info_msg = std::dynamic_pointer_cast<sros::core::CommonPosesInfoMsg>(m);

    common_poses_info_proto->set_type((proto::CommonPosesInfo::Type)common_poses_info_msg->type);

    for (const auto &pose : common_poses_info_msg->car_simulate_poses) {  // 每个位姿
        auto new_pose_proto = common_poses_info_proto->add_car_simulate_poses();
        for (const auto &vertex : pose) {  // 每个多边形
            auto new_vertex = new_pose_proto->add_vertexs();
            new_vertex->set_x(vertex.x);
            new_vertex->set_y(vertex.y);

//            LOG(INFO) << "x: " << vertex.x << ", y: " << vertex.y
//                      << ", sensor_name:" << common_poses_info_msg->sensor_name
//                      << ", type: " << (int)common_poses_info_msg->type;
        }
    }

    common_poses_info_proto->set_sensor_name(common_poses_info_msg->sensor_name);

    // 上传需要避障预测信息的客户端
    auto session_list = session_manager_.getItemList();
    for (int i = 0; i < session_list.size(); ++i) {
        auto protobuf_session = dynamic_pointer_cast<ProtobufSessionItem>(session_list.at(i));
        if (protobuf_session->is_connected && protobuf_session->is_upload_avoid_obstacle_prediction_) {
            msg->set_session_id(protobuf_session->session_id);
            sendMsgToNetwork(msg);
        }
    }
}

void NetworkModule::onConnectCallback(std::string peer_ip, unsigned short port) {
    connected_client_count_++;
    notifyConnectedClientCount();
    LOGGER(INFO, PROTOBUF) << "Client " << peer_ip << ":" << port << " connected! current client count is "
                           << connected_client_count_;

    //    handleRequestInfo(0, 0); // TODO ?
}

void NetworkModule::onDisconnectCallback(std::string peer_ip, unsigned short port) {
    connected_client_count_--;
    notifyConnectedClientCount();
    LOGGER(INFO, PROTOBUF) << "Client " << peer_ip << ":" << port << " disconnected! current client count is "
                           << connected_client_count_;

    auto session = session_manager_.getItem(peer_ip, port);
    if (session && session->session_id != 0) {
        LOGGER(WARNING, PROTOBUF) << peer_ip << " disconnect without logout";
        //        session_manager_.removeItem(session);
        if (g_state.control_mutex.isLock(session->session_id)) {
            g_state.control_mutex.unlock();
        }
        session_manager_.toggleItemConnected(session->session_id, false);
        updateIfNeedAvoidObstaclePrediction();

        updateIfNeedDebugInfo();

        if (session && session->username == USERNAME_FMS) {
            g_state.updateFleetState();
        }
    }
}

void NetworkModule::notifyConnectedClientCount() {
    auto new_msg = std::make_shared<sros::core::CommonMsg>("TOPIC_NOTIFY_CONNECTED_CLIENT_COUNT");
    new_msg->int_0_ = connected_client_count_;

    sendMsg(new_msg);
}

void NetworkModule::onTimer_100ms(sros::core::base_msg_ptr m) {
    auto mm = std::dynamic_pointer_cast<sros::core::StrMsg>(m);

    auto &s = sros::core::Settings::getInstance();
    int system_state_update_freq = s.getValue<int>("network.system_state_update_freq", 10);
    int hardware_state_update_freq = s.getValue<int>("network.hardware_state_update_freq", 50);

    if (system_state_update_freq > 0 && mm->counter % system_state_update_freq == 0) {
        // 主动上传SystemState
        handleRequestSystemState(0, 0);
    }

    if (hardware_state_update_freq > 0 && mm->counter % hardware_state_update_freq == 0) {
        // 主动上传HardwareState
        handleRequestHardwareState(0, 0);
    }
}

void NetworkModule::onTimer_1s(sros::core::base_msg_ptr m) {
    auto session_list = session_manager_.getItemList();
    for (const auto &item : session_list) {
        if (item->username == USERNAME_FMS && item->is_connected && !item->checkIsAlive()) {
            LOG(INFO) << "remove not alive connected fms session, ip is " << item->ip_addr << " and port is "
                      << item->ip_port;
            server_.close(item->ip_addr, item->ip_port);
            session_manager_.toggleItemConnected(item->session_id, false);
            break;
        }
    }

    Dump::getInstance()->judgeTimeout();
}

void NetworkModule::onTimer_200ms(sros::core::base_msg_ptr m) {

//    LOG(INFO) << "need_avoid_obstacle_prediction: " << g_state.need_avoid_obstacle_prediction
//              << ", is_send_forklift_common_poses: " << g_state.is_send_forklift_common_poses;

    if(g_state.need_avoid_obstacle_prediction 
        && g_state.is_send_forklift_common_poses) {
        
        if(common_poses_info_msg_ptr_) {
            onNewCommonPosesInfoMsg(common_poses_info_msg_ptr_);
        }
    }

    if (!needUploadLaserPoint()) {
        return;
    }

    auto msg = make_shared<proto::Message>();
    msg->set_type(proto::MSG_RESPONSE);
    msg->set_seq(0);

    auto response_msg = msg->mutable_response();
    response_msg->set_response_type(proto::Response::RESPONSE_LASER_POINTS);

    auto laser_msg = response_msg->mutable_laser_points();
    laser_msg->set_req_seq(0);

    int max_laser_point_num = 1080;                           // 最多允许发送的雷达点数
    if (g_state.isLocateSucceed() && !g_state.isDrawMap()) {  // 由于绘图也是定位状态，但是绘图只能传雷达点
        // 雷达0 的数据
        auto obstacle_msg = real_time_detect_info_.getObstacleInfo(real_time_detect_info_.getLaserName());
        if (obstacle_msg) {
            // 计算需要去除的雷达点比例
            int factor = static_cast<int>(obstacle_msg->point_cloud.size() / max_laser_point_num);
            int mod = static_cast<int>(obstacle_msg->point_cloud.size() % max_laser_point_num);

            if (mod > 0) {
                factor += 1;
            }

            for (int i = 0; i < obstacle_msg->point_cloud.size(); i++) {
                if (i % factor == 0) {  // 均匀去除多余的雷达点
                    laser_msg->add_xs(static_cast<int>((obstacle_msg->point_cloud[i].x() * 1000)));
                    laser_msg->add_ys(static_cast<int>((obstacle_msg->point_cloud[i].y() * 1000)));
                }
            }
        } else {
            return;  // 雷达1没有数据必须退出，雷达而没有数据不必要退出
        }
        // 雷达1的数据
        obstacle_msg = real_time_detect_info_.getObstacleInfo(sros::device::DEVICE_R2100);
        if (obstacle_msg) {
            // 计算需要去除的雷达点比例
            int factor = static_cast<int>(obstacle_msg->point_cloud.size() / max_laser_point_num);
            int mod = static_cast<int>(obstacle_msg->point_cloud.size() % max_laser_point_num);

            if (mod > 0) {
                factor += 1;
            }

            for (int i = 0; i < obstacle_msg->point_cloud.size(); i++) {
                if (i % factor == 0) {  // 均匀去除多余的雷达点
                    laser_msg->add_xs1(static_cast<int>((obstacle_msg->point_cloud[i].x() * 1000)));
                    laser_msg->add_ys1(static_cast<int>((obstacle_msg->point_cloud[i].y() * 1000)));
                }
            }
        }

        // tof/eu100/d435雷达点云数据
        uploadExtLaserPoint(msg);

        //避障点数据
        uploadObaLaserPoint(msg);

    } else {
        auto laser_scan = real_time_detect_info_.getLaserScanInfo();
        if (laser_scan) {
            // 计算需要去除的雷达点比例
            int factor = static_cast<int>(laser_scan->ranges.size() / max_laser_point_num);
            int mod = static_cast<int>(laser_scan->ranges.size() % max_laser_point_num);

            if (mod > 0) {
                factor += 1;
            }

            double angle = laser_scan->angle_min;
            for (int i = 0; i < laser_scan->ranges.size(); i++) {
                double range = laser_scan->ranges[i];

                if (i % factor == 0) {  // 均匀去除多余的雷达点
                    laser_msg->add_xs(static_cast<int>(range * cos(angle) * 1000));
                    laser_msg->add_ys(static_cast<int>(range * sin(angle) * 1000));
                }

                angle += laser_scan->angle_increment;
            }
        } else {
            return;
        }
    }

    // lmk match info
    auto lmk_match_info = real_time_detect_info_.getLmkMatchInfo();
    if (lmk_match_info) {
        for (const auto &lmk : lmk_match_info->lmk_infos) {
            auto new_lmk_proto = laser_msg->add_lmks();
            new_lmk_proto->set_x(lmk.pose_x * 1000);
            new_lmk_proto->set_y(lmk.pose_y * 1000);
            new_lmk_proto->set_yaw(lmk.pose_yaw * 1000);
            new_lmk_proto->set_lmk_type(::proto::LmkMatchInfo_LmkMatchType(lmk.lmk_type));
            new_lmk_proto->set_is_matched(lmk.is_matched);
        }
    }

    sendLaserPointMsgToNetwork(msg);
}

void NetworkModule::uploadExtLaserPoint(proto::Message_ptr &msg) {
    auto laser_msg = msg->mutable_response()->mutable_laser_points();

    auto ext_obstacle_msg = real_time_detect_info_.getExtObstacleInfo();

    for (auto &iter : ext_obstacle_msg) {
        // LOG(INFO) << "ext_sensor_name: " << iter.first << ", point_num: " << iter.second->point_cloud.size();
        auto ext_laser_msg = laser_msg->add_ext_laser_points();
        ext_laser_msg->set_sensor_name(iter.first);

        for (int i = 0; i < iter.second->point_cloud.size(); i++) {
            // LOG(INFO) << "laser_xs: " << (iter.second->point_cloud[i].x() * 1000)
            //           << ", laser_ys: " << (iter.second->point_cloud[i].y() * 1000);
            ext_laser_msg->add_xs(static_cast<int>((iter.second->point_cloud[i].x() * 1000)));
            ext_laser_msg->add_ys(static_cast<int>((iter.second->point_cloud[i].y() * 1000)));
        }
    }
}

void NetworkModule::uploadObaLaserPoint(proto::Message_ptr &msg) {
    auto laser_msg = msg->mutable_response()->mutable_laser_points();

    auto avoid_obstacle_msg = real_time_detect_info_.getAvoidObstacleInfo();

    for (auto &iter : avoid_obstacle_msg) {
        // LOG(INFO) << "avoid_oba_name: " << iter.first;
        auto oba_laser_msg = laser_msg->add_oba_laser_points();
        oba_laser_msg->set_sensor_name(iter.first);

        for (int i = 0; i < iter.second->point_cloud.size(); i++) {
            // LOG(INFO) << "oba_xs: " << (iter.second->point_cloud[i].x() * 1000)
            //           << ", oba_ys: " << (iter.second->point_cloud[i].y() * 1000);
            oba_laser_msg->add_xs(static_cast<int>((iter.second->point_cloud[i].x() * 1000)));
            oba_laser_msg->add_ys(static_cast<int>((iter.second->point_cloud[i].y() * 1000)));
        }
    }
}

void NetworkModule::onNavPathMsg(sros::core::base_msg_ptr m) {
    auto msg = dynamic_pointer_cast<sros::core::PathMsg>(m);
    if (msg->paths.empty()) {
        return;
    }

    auto move_task = sros::core::TaskManager::getInstance()->getMovementTask();

    sros::core::NavigationPathi_vector ps;
    for (auto p : msg->paths) {
        sros::core::NavigationPathi pp;

        pp.type_ = p.type_;

        pp.sx_ = static_cast<int>(p.sx_ * 1000);
        pp.sy_ = static_cast<int>(p.sy_ * 1000);
        pp.ex_ = static_cast<int>(p.ex_ * 1000);
        pp.ey_ = static_cast<int>(p.ey_ * 1000);
        pp.cx_ = static_cast<int>(p.cx_ * 1000);
        pp.cy_ = static_cast<int>(p.cy_ * 1000);
        pp.dx_ = static_cast<int>(p.dx_ * 1000);
        pp.dy_ = static_cast<int>(p.dy_ * 1000);

        pp.radius_ = static_cast<int>(p.radius_ * 1000);
        pp.rotate_angle_ = static_cast<int>(p.rotate_angle_ * 1000);
        pp.limit_v_ = static_cast<int>(p.limit_v_ * 1000);
        pp.limit_w_ = static_cast<int>(p.limit_w_ * 1000);

        ps.push_back(pp);
    }
    move_task->setPaths(ps);
}

// NavigationPathi => proto::Path
void NetworkModule::convertPath(const sros::core::NavigationPathi &nav_path, proto::Path *pp) const {
    pp->set_type((proto::Path_PathType)nav_path.type_);

    pp->set_sx(static_cast<int>(nav_path.sx_));
    pp->set_sy(static_cast<int>(nav_path.sy_));
    pp->set_ex(static_cast<int>(nav_path.ex_));
    pp->set_ey(static_cast<int>(nav_path.ey_));
    pp->set_cx(static_cast<int>(nav_path.cx_));
    pp->set_cy(static_cast<int>(nav_path.cy_));
    pp->set_dx(static_cast<int>(nav_path.dx_));
    pp->set_dy(static_cast<int>(nav_path.dy_));

    pp->set_radius(static_cast<int>(nav_path.radius_));
    pp->set_rotate_angle(static_cast<int>(nav_path.rotate_angle_));
    pp->set_limit_v(static_cast<int>(nav_path.limit_v_));
    pp->set_limit_w(static_cast<int>(nav_path.limit_w_));
}

// proto::Path => NavigationPathi
void NetworkModule::convertPathReverse(const proto::Path &p, sros::core::NavigationPathi *path) const {
    path->type_ = (sros::core::PathType)p.type();

    path->sx_ = p.sx();
    path->sy_ = p.sy();
    path->ex_ = p.ex();
    path->ey_ = p.ey();

    path->cx_ = p.cx();
    path->cy_ = p.cy();
    path->dx_ = p.dx();
    path->dy_ = p.dy();

    path->radius_ = p.radius();
    path->rotate_angle_ = p.rotate_angle();
    path->limit_v_ = p.limit_v();
    path->limit_w_ = p.limit_w();
    path->avoid_policy_ = sros::core::OBSTACLE_AVOID_WAIT;
    path->direction_ = p.direction();

    if (p.direction() == 0) {
        path->direction_ = 0x01;  // 默认为前进
    }
}

void NetworkModule::onMonitorData(sros::core::base_msg_ptr m) {
    auto common_msg = dynamic_pointer_cast<sros::core::CommonMsg>(m);
    monitor_data_ = common_msg->str_0_;
}

void NetworkModule::handleRequestCurMap(const proto::Message_ptr &m) {
    auto msg = make_shared<proto::Message>();
    msg->set_seq(m->seq());
    msg->set_session_id(m->session_id());
    msg->set_type(proto::MSG_RESPONSE);
    auto response_msg = msg->mutable_response();

    auto response_result = response_msg->mutable_result();
    response_result->set_result_state(proto::ResponseResult::RESPONSE_OK);
    response_msg->set_response_type(proto::Response::RESPONSE_GET_CUR_MAP);
    response_msg->set_return_str(g_state.getCurMapName());

    sendMsgToNetwork(msg);
}

void NetworkModule::handleRequestTimeStamp(const proto::Message_ptr &m) {
    auto msg = make_shared<proto::Message>();
    msg->set_seq(m->seq());
    msg->set_session_id(m->session_id());
    msg->set_type(proto::MSG_RESPONSE);
    auto response_msg = msg->mutable_response();

    auto response_result = response_msg->mutable_result();
    response_result->set_result_state(proto::ResponseResult::RESPONSE_OK);

    // 返回的是UTC时间戳
    int64_t timestamp = sros::core::util::get_timestamp_in_ms();

    response_msg->set_response_type(proto::Response::RESPONSE_TIMESTAMP);
    response_msg->set_timestamp(timestamp);
    sendMsgToNetwork(msg);
}

void NetworkModule::handleRequestMonitorData(const proto::Message_ptr &m) {
    auto msg = make_shared<proto::Message>();
    msg->set_seq(m->seq());
    msg->set_session_id(m->session_id());
    msg->set_type(proto::MSG_RESPONSE);

    auto response_msg = msg->mutable_response();
    response_msg->set_response_type(proto::Response::RESPONSE_MONITOR_DATA);

    auto record = response_msg->mutable_record();

    auto response_result = response_msg->mutable_result();

    if (monitor_data_.empty()) {
        response_result->set_result_state(proto::ResponseResult::RESPONSE_FAILED);
    } else {
        response_result->set_result_state(proto::ResponseResult::RESPONSE_OK);
        record->ParseFromString(monitor_data_);
    }

    sendMsgToNetwork(msg);
}

void NetworkModule::onObstacleMsg(sros::core::base_msg_ptr m) {
    auto obstacle_msg = dynamic_pointer_cast<sros::core::ObstacleMsg>(m);
    real_time_detect_info_.setObstacleInfo(obstacle_msg);
}

void NetworkModule::onAvoidObstacleMsg(sros::core::base_msg_ptr m) {
    auto avoid_obstacle_msg = dynamic_pointer_cast<sros::core::ObstacleMsg>(m);
    real_time_detect_info_.setAvoidObstacleInfo(avoid_obstacle_msg);
}

void NetworkModule::handleNotificationAck(const proto::Message_ptr &msg, const std::string &peer_ip_str) {
    uint32_t seq = msg->seq();
    uint64_t session_id = msg->session_id();
    auto m = msg->mutable_response();
    if (m->response_type() == proto::Response::RESPONSE_NOTIFY) {
        auto notify_msg = m->mutable_notify_response();
        if (!notify_msg->ack()) {
            return;
        }
        LOGGER(INFO, PROTOBUF) << "Notification ACK session_id = " << session_id << " seq = " << seq << " is recved";
    }
}

void NetworkModule::onCommonNotifyMsg(sros::core::base_msg_ptr m) {
    auto notify_msg = dynamic_pointer_cast<sros::core::NotificationMsg>(m);

    if (notify_msg->notify_type != sros::core::NotificationMsg::NOTIFY_CALIBRATION_FINISHED) {
        return;
    }

    LOGGER(INFO, PROTOBUF) << "====> proto::Notification::NOTIFY_CALIBRATION_FINISHED";

    auto msg = make_shared<proto::Message>();
    msg->set_type(proto::MSG_NOTIFICATION);
    msg->set_seq(notify_msg->seq);
    msg->set_session_id(notify_msg->session_id);

    auto notify = msg->mutable_notification();
    notify->set_notify_type(proto::Notification::NOTIFY_CALIBRATION_FINISHED);

    auto cal_result = notify->mutable_cal_result();
    cal_result->set_x(notify_msg->calibration.x());
    cal_result->set_y(notify_msg->calibration.y());
    cal_result->set_theta(notify_msg->calibration.yaw());
    cal_result->set_status(notify_msg->calibration.z());

    sendMsgToNetwork(msg);
}

void NetworkModule::onNotifyMsg(sros::core::base_msg_ptr m) {
    auto notify_msg = dynamic_pointer_cast<sros::core::NotificationMsg>(m);
    auto msg = make_shared<proto::Message>();
    msg->set_type(proto::MSG_NOTIFICATION);

    auto notify = msg->mutable_notification();

    notify->set_cur_station_no(g_state.station_no);
    auto pose = notify->mutable_cur_pose();
    convertPose(src_sdk->getCurPose(), pose);

    if (notify_msg->notify_type == sros::core::NotificationMsg::NOTIFY_MOVE_TASK_FINISHED) {
        notify->set_notify_type(proto::Notification::NOTIFY_MOVE_TASK_FINISHED);

        // MovementTask状态
        auto move_task = notify_msg->movement_task;
        msg->set_seq(move_task->getTaskSeq());
        msg->set_session_id(move_task->getTaskSessionId());
        if (move_task->getTaskSourceModule() != getName()) {
            LOG(INFO) << "不是网络发的通知，忽略！： "
                      << "  seq: " << move_task->getTaskSeq() << " sessionId: " << move_task->getTaskSessionId();
            return;
        }

        LOGGER(INFO, PROTOBUF) << "====> proto::Notification::NOTIFY_MOVE_TASK_FINISHED";

        if (move_task) {
            auto p_task = notify->mutable_movement_task();
            // convertMovementTask(move_task, p_task);
            notify->set_reserved_field(move_task->getDstCheckResult());
            // task No must be set!
            p_task->set_no(move_task->getTaskNo());
            p_task->set_state((proto::MovementTask_TaskState)move_task->getState());
            p_task->set_result((proto::TaskResult)move_task->getTaskResult());
            p_task->set_failed_code((proto::MovementTask_FailedCode)move_task->getFailedCode());
        }

        sendNotifyMessage(msg, sros::core::NotificationMsg::NOTIFY_MOVE_TASK_FINISHED);

    } else if (notify_msg->notify_type == sros::core::NotificationMsg::NOTIFY_ACTION_TASK_FINISHED) {
        notify->set_notify_type(proto::Notification::NOTIFY_ACTION_TASK_FINISHED);
        notify->set_cur_station_no(g_state.station_no);

        // ActionTask状态
        auto action_task = notify_msg->action_task;
        msg->set_seq(action_task->getTaskSeq());
        msg->set_session_id(action_task->getTaskSessionId());
        if (action_task->getTaskSourceModule() != getName()) {
            LOG(INFO) << "不是网络发的通知，忽略！： "
                      << "  seq: " << action_task->getTaskSeq() << " sessionId: " << action_task->getTaskSessionId();
            return;
        }

        LOGGER(INFO, PROTOBUF) << "====> proto::Notification::NOTIFY_ACTION_TASK_FINISHED";

        if (action_task) {
            auto notify_action_task = notify->mutable_action_task();
            notify_action_task->set_no(action_task->getTaskNo());
            notify_action_task->set_id(action_task->getActionID());
            notify_action_task->set_param0(action_task->getActionParam());
            notify_action_task->set_param1(action_task->getActionParam1());
            notify_action_task->set_param2(action_task->getActionParam2());

            notify_action_task->set_state((proto::ActionTask::TaskState)action_task->getState());
            notify_action_task->set_result((proto::TaskResult)action_task->getTaskResult());

            // 对于es4600，result_str即为扫描得到的结果
            notify_action_task->set_result_str(action_task->getResultValueStr());
            notify_action_task->set_result_code(action_task->getActionResultValue());
        }

        sendNotifyMessage(msg, sros::core::NotificationMsg::NOTIFY_ACTION_TASK_FINISHED);

    } else if (notify_msg->notify_type == sros::core::NotificationMsg::NOTIFY_MISSION_LIST_CHANGED) {
        notify->set_notify_type(proto::Notification::NOTIFY_MISSION_LIST_CHANGED);
        LOGGER(INFO, PROTOBUF) << "NetworkModule::onNotifyMsg(): mission list changed";
        for (auto it = notify_msg->mission_list.begin(); it != notify_msg->mission_list.end(); it++) {
            auto mission = notify->add_mission_list();
            buildProtoMission(mission, (*it));
        }
        sendMsgToNetwork(msg);
    } else if (notify_msg->notify_type == sros::core::NotificationMsg::NOTIFY_UPDATE_FINISHED) {
        int32_t update_result = notify_msg->param_int;
        notifyUpdateResult(update_result);
    }
}

void NetworkModule::sendNotifyMessage(proto::Message_ptr message, sros::core::NotificationMsg::NotifyType type) {
    auto session_id = message->session_id();
    auto seq = message->seq();

    std::string task_str;

    if (type == sros::core::NotificationMsg::NOTIFY_MOVE_TASK_FINISHED) {
        task_str = "MoveTask";
    } else if (type == sros::core::NotificationMsg::NOTIFY_ACTION_TASK_FINISHED) {
        task_str = "ActionTask";
    }

    LOGGER(INFO, PROTOBUF) << "Sending " << task_str << " notification for msg seq = " << seq
                           << ", session_id = " << session_id;

    bool ret = sendMsgToNetwork(message);

    if (!ret) {
        LOGGER(WARNING, PROTOBUF) << "user " << session_id << " is offline, failed to send notification!";
    }
}

void NetworkModule::ResponseFileOperate(proto::ResponseResult::ResultState ret, uint32_t seq, uint64_t session_id,
                                        int result_code) {
    auto msg = make_shared<proto::Message>();
    msg->set_type(proto::MSG_RESPONSE);
    msg->set_seq(seq);
    msg->set_session_id(session_id);

    auto response_msg = msg->mutable_response();
    response_msg->set_response_type(proto::Response::RESPONSE_FILE_OPERATE);

    auto result_msg = response_msg->mutable_result();
    result_msg->set_result_state(ret);
    result_msg->set_result_code(result_code);
    SET_ERROR(result_code, "");
    LOGGER(INFO, PROTOBUF) << "Response FileOperate result : " << ret << " result code: " << result_code;
    sendMsgToNetwork(msg);
}

void NetworkModule::onCommandResponseMsg(sros::core::base_msg_ptr m) {
    auto cmd_response = dynamic_pointer_cast<sros::core::CommandMsg>(m);
    if (cmd_response->source != getName()) {  // 这条命令不是由网络模块发出的，直接拒收
        return;
    }

    responseCommandResult(cmd_response->command, cmd_response->result_state, cmd_response->result_code,
                          cmd_response->req_seq, cmd_response->session_id);
}

bool NetworkModule::sendMsgToNetwork(proto::Message_ptr msg) {
    auto session_id = msg->session_id();

    //LOG(INFO) << "session_id: " << session_id << ", type: " << msg->type() << ", rsp_type: " << msg->mutable_response()->response_type();

    if (session_id != 0) {
        auto session = session_manager_.getItem(session_id);

        if (!session) {
            LOGGER(WARNING, PROTOBUF) << "session id " << session_id << " not exist!";
            return false;
        }

        if (session->is_connected) {
            server_.send(msg, session->ip_addr, session->ip_port);
//            if (session->ip_addr == "172.20.248.187" || session->ip_addr == "172.20.245.131") {
//
//                size_t frame_body_size = msg->ByteSizeLong() + 4;
//
//                LOG(INFO) << "ip: " << session->ip_addr
//                          << ", type: " << msg->type()
//                          << ", rsp_type: " << msg->mutable_response()->response_type()
//                          << ", size: " << frame_body_size;
//
//            }

            return true;
        } else {
            LOGGER(WARNING, PROTOBUF) << "user is disconnected, message not sent! session_id = " << session_id
                                      << "seq = " << msg->seq() << " type = " << msg->type();
            return false;
        }
    } else {
        int session_connected = 0;
        // message broadcast
        for (auto session : session_manager_.getItemList()) {
            if (session->is_connected) {
                msg->set_session_id(session->session_id);
                server_.send(msg, session->ip_addr, session->ip_port);
                session_connected++;
            }
        }

        if (connected_client_count_ != session_connected) {
            LOG_EVERY_N(WARNING, 20) << "connected_client_count_ = " << connected_client_count_
                                     << " but session_connected = " << session_connected;
        }

        return true;
    }
}

void NetworkModule::handleMissionTask(const proto::Message_ptr &msg) {
    if (msg->mutable_command()->mission_list_size() <= 0) {
        LOGGER(ERROR, PROTOBUF) << "Mission list is empty!";
        return;
    }

    auto seq = msg->seq();  // 请求序列号
    auto session_id = msg->session_id();
    auto mission = msg->mutable_command()->mutable_mission_list()->Get(0);
    auto no = mission.no();
    auto mission_id = mission.id();
    auto start_step_id = mission.cur_step_id();
    auto avoid_policy = static_cast<sros::core::ObstacleAvoidPolicy>(mission.avoid_policy());

    auto mm = make_shared<sros::core::CommandMsg>(getName());
    mm->req_seq = seq;
    mm->session_id = session_id;
    mm->command = sros::core::CMD_START_MISSION;
    mm->mission_no = no;
    mm->mission_id = mission_id;
    mm->mission_cur_step_id = start_step_id;
    mm->mission_avoid_policy = avoid_policy;
    auto session = session_manager_.getItem(session_id);
    if (session) {
        mm->user_name = session->username;
    }
    sendMsg(mm);
}

void NetworkModule::handleRequestMissionQueue(const proto::Message_ptr &msg) {
    // response
    LOGGER(INFO, PROTOBUF) << "handleRequestMissionQueue";
    auto mm = make_shared<proto::Message>();
    mm->set_type(proto::MSG_RESPONSE);
    mm->set_seq(msg->seq());
    mm->set_session_id(msg->session_id());

    auto response_msg = mm->mutable_response();
    response_msg->set_response_type(proto::Response::RESPONSE_MISSION_LIST);

    auto response_result = response_msg->mutable_result();
    response_result->set_result_state(proto::ResponseResult::RESPONSE_OK);

    auto missions = sros::core::MissionManager::getInstance()->allMissions();
    for (const auto &run_mission : missions) {
        auto mission = response_msg->add_mission_list();
        buildProtoMission(mission, run_mission);
    }

    sendMsgToNetwork(mm);
}

void NetworkModule::buildProtoMission(proto::Mission *proto_mission, sros::core::MissionInstancePtr mission_run_info) {
    if (!proto_mission || !mission_run_info) {
        return;
    }

    proto_mission->set_id(mission_run_info->getId());
    proto_mission->set_name(mission_run_info->mission_info_.name_);
    proto_mission->set_map_name(mission_run_info->mission_info_.map_name_);
    proto_mission->set_no(mission_run_info->no_);
    proto_mission->set_cur_step_id(mission_run_info->cur_step_id_);
    proto_mission->set_state((proto::Mission::MissionStatus)mission_run_info->state_);
    proto_mission->set_result((proto::TaskResult)mission_run_info->result_);
    proto_mission->set_error_code(mission_run_info->err_code_);
    proto_mission->set_start_timestamp(mission_run_info->start_timestamp_);
    proto_mission->set_finish_timestamp(mission_run_info->finish_timestamp_);
}

void NetworkModule::handleReorderMissionList(const proto::Message_ptr &msg) {
    auto mission_list = msg->mutable_command()->mission_list();

    if (mission_list.empty()) {
        return;
    }

    auto seq = msg->seq();  // 请求序列号
    auto session_id = msg->session_id();

    auto mm = make_shared<sros::core::CommandMsg>(getName());
    mm->req_seq = seq;
    mm->session_id = session_id;
    mm->command = sros::core::CMD_REORDER_MISSION;

    for (auto mission : mission_list) {
        mm->mission_no_lst.push_back(mission.no());
    }

    sendMsg(mm);
}

void NetworkModule::handleRequestHeatbeat(const proto::Message_ptr &msg) {
    auto mm = make_shared<proto::Message>();
    mm->set_seq(msg->seq());
    mm->set_session_id(msg->session_id());
    mm->set_type(proto::MSG_RESPONSE);
    auto response_msg = mm->mutable_response();
    response_msg->set_response_type(proto::Response::RESPONSE_HEARTBEAT);

    auto response_result = response_msg->mutable_result();
    response_result->set_result_state(proto::ResponseResult::RESPONSE_OK);
    sendMsgToNetwork(mm);
}

void NetworkModule::handleRequestConnectionInfo(uint32_t seq, uint64_t session_id) {
    auto msg = make_shared<proto::Message>();
    msg->set_type(proto::MSG_RESPONSE);
    msg->set_seq(seq);
    msg->set_session_id(session_id);

    auto response_msg = msg->mutable_response();
    response_msg->set_response_type(proto::Response::RESPONSE_CONNECT_INFO);

    auto response_result = response_msg->mutable_result();
    response_result->set_result_state(proto::ResponseResult::RESPONSE_OK);

    auto connection_info = response_msg->mutable_connect_info();

    LOG(INFO) << session_manager_.connectCount();
    for (auto session_info : session_manager_.getItemList()) {
        if (session_info->is_connected) {
            auto proto_session_info = connection_info->add_protobuf();

            proto_session_info->set_session_id(session_info->session_id);
            proto_session_info->set_username(session_info->username);
            proto_session_info->set_ip_addr(session_info->ip_addr);
            proto_session_info->set_ip_port(session_info->ip_port);
        }
    }

    for (auto session_info : g_state.modbus_session_manager.getItemList()) {
        if (session_info->is_connected) {
            auto proto_session_info = connection_info->add_modbus();

            proto_session_info->set_session_id(session_info->session_id);
            proto_session_info->set_username(session_info->username);
            proto_session_info->set_ip_addr(session_info->ip_addr);
            proto_session_info->set_ip_port(session_info->ip_port);
        }
    }

    sendMsgToNetwork(msg);
}

void NetworkModule::handleRequestReadInputRegisters(const proto::Message_ptr &msg, uint32_t seq, uint64_t session_id) {
    auto m = msg->mutable_request();
    auto start_addr = m->param_int();
    auto read_count = m->param_int1();

    LOG(INFO) << "REQUEST_READ_INPUT_REGISTER start addr: " << start_addr << " read count: " << read_count;
    auto mm = make_shared<proto::Message>();
    mm->set_type(proto::MSG_RESPONSE);
    mm->set_seq(msg->seq());

    auto response_msg = mm->mutable_response();
    response_msg->set_response_type(proto::Response::RESPONSE_READ_INPUT_REGISTER);
    auto response_result = response_msg->mutable_result();

    if (read_count != 1) {  // FIXME(pengjiali): v4.8.0 此处暂时只能读取一个寄存器
        response_result->set_result_state(proto::ResponseResult::RESPONSE_FAILED);
        response_result->set_result_code(proto::ResponseResult::RESULT_CODE_UNDEFINED);

        LOG(INFO) << "read count not one regester!";

        sendMsgToNetwork(mm);
        return;
    }

    uint16_t value16 = sros::core::RegisterAdmin::getInstance()->readInputRegisterUint16(start_addr);
    response_msg->set_registers(&value16, 2);
    response_result->set_result_state(proto::ResponseResult::RESPONSE_OK);
    response_result->set_result_code(proto::ResponseResult::RESULT_CODE_NONE);
    sendMsgToNetwork(mm);
}


void NetworkModule::handleRequestReadAllRegisters(const proto::Message_ptr &msg, uint32_t seq, uint64_t session_id) {
    LOG(INFO) << "read all registers";
    auto mm = make_shared<proto::Message>();
    mm->set_type(proto::MSG_RESPONSE);
    mm->set_seq(msg->seq());
    auto response_msg = mm->mutable_response();
    response_msg->set_response_type(proto::Response::RESPONSE_READ_ALL_REGISTERS);
    auto response_result = response_msg->mutable_result();
    auto registers_values = response_msg->mutable_registers_values();

    auto driver = sros::core::RegisterAdmin::getInstance();
    std::vector<uint16_t> inputRegisterValues = driver->readInputRegisters(30300, basis_input_register_num);
    std::vector<uint16_t> holdRegisterValues = driver->readHoldRegisters(40300, basis_hold_register_num);

    for(int i = 0; i < basis_input_register_num; i++) {
        if(i >= registers_values->input_registers_size()) {
            registers_values->add_input_registers(inputRegisterValues[i]);
        } else {
            registers_values->set_input_registers(i, inputRegisterValues[i]);
        }
    }

    for(int i = 0; i < basis_hold_register_num; i++) {
        if(i >= registers_values->hold_registers_size()) {
            registers_values->add_hold_registers(holdRegisterValues[i]);
        } else {
            registers_values->set_hold_registers(i, holdRegisterValues[i]);
        }
    }

    if(is_seacp && seacp_enable_extend) {
        std::vector<uint16_t> extendInputRegisterValues = driver->readInputRegisters(30350, extend_input_register_num);
        std::vector<uint16_t> extendHoldRegisterValues = driver->readHoldRegisters(40350, extend_hold_register_num);

        for(int i = 0; i < extend_input_register_num; i++) {
            if(i >= registers_values->extend_input_registers_size()) {
                registers_values->add_extend_input_registers(extendInputRegisterValues[i]);
            } else {
                registers_values->set_extend_input_registers(i, extendInputRegisterValues[i]);
            }
        }

        for(int i = 0; i < extend_hold_register_num; i++) {
            if(i >= registers_values->extend_hold_registers_size()) {
                registers_values->add_extend_hold_registers(extendHoldRegisterValues[i]);
            } else {
                registers_values->set_extend_hold_registers(i, extendHoldRegisterValues[i]);
            }
        }

    }

    response_result->set_result_state(proto::ResponseResult::RESPONSE_OK);
    response_result->set_result_code(proto::ResponseResult::RESULT_CODE_NONE);
    sendMsgToNetwork(mm);
}



void NetworkModule::updateIfNeedAvoidObstaclePrediction() {
    bool enable = false;
    auto session_list = session_manager_.getItemList();
    for (int i = 0; i < session_list.size(); ++i) {
        auto protobuf_session = dynamic_pointer_cast<ProtobufSessionItem>(session_list.at(i));
        if (protobuf_session->is_connected && protobuf_session->is_upload_avoid_obstacle_prediction_) {
            enable = true;
            break;
        }
    }
    g_state.need_avoid_obstacle_prediction = enable;
}

void NetworkModule::updateIfNeedDebugInfo() {
    bool enable = false;
    auto session_list = session_manager_.getItemList();
    for (int i = 0; i < session_list.size(); ++i) {
        auto protobuf_session = dynamic_pointer_cast<ProtobufSessionItem>(session_list.at(i));
        if (protobuf_session->is_connected && protobuf_session->need_debug_info_) {
            enable = true;
            break;
        }
    }
    g_state.setNeedDebugInfo(enable);
}

void NetworkModule::notifyUpdateResult(int32_t ret) {
    auto msg = make_shared<proto::Message>();
    msg->set_type(proto::MSG_NOTIFICATION);
    auto notify = msg->mutable_notification();
    notify->set_notify_type(proto::Notification::NOTIFY_UPDATE_FINISHED);
    auto result = notify->mutable_update_result();

    switch (ret) {
        case -1:
        case 2:
            LOGGER(INFO, PROTOBUF) << "Update result notification sent ! (UPDATE_FAILED)";
            result->set_result_code(proto::UpdateResult::UPDATE_FAILED);
            break;
        case 1:
            LOGGER(INFO, PROTOBUF) << "Update result notification sent ! (UPDATE_SUCCESS)";
            result->set_result_code(proto::UpdateResult::UPDATE_SUCCESS);
            break;
        default:
            LOGGER(INFO, PROTOBUF) << "Update result notification sent ! (UPDATE_FAILED)";
            result->set_result_code(proto::UpdateResult::UPDATE_FAILED);
            break;
    }
    sendMsgToNetwork(msg);
}

void NetworkModule::onFeatureExtractorDebug(std::string sensor_name) {
    auto msg = std::make_shared<sros::core::CommonCommandMsg<std::string>>("TOPIC_EXTRACT_COMMAND");
    msg->command = "DEBUG_START_EXTRACTOR";
    msg->str1 = sensor_name;
    sros::core::MsgBus::sendMsg(msg);
}

void NetworkModule::offFeatureExtractorDebug(std::string sensor_name) {
    auto msg = std::make_shared<sros::core::CommonCommandMsg<std::string>>("TOPIC_EXTRACT_COMMAND");
    msg->command = "DEBUG_STOP_EXTRACTOR";
    msg->str1 = sensor_name;
    sros::core::MsgBus::sendMsg(msg);
}

void NetworkModule::onUpdateSrcParaMsg(sros::core::base_msg_ptr m) {
    LOG(INFO) << "update src paramters";
    auto r = updateSRCParameters();
    if(!r) {
        LOG(INFO) <<"update src paramters failed!";
    }
}

}  // namespace network
