/*
 * usart_module.cpp
 *
 *  Created on: 2016/12/01
 *      Author: lhx
 */

#include "usart_module.h"

#include "core/msg/command_msg.hpp"
#include "core/msg/usart_data_msg.hpp"
#include "core/msg/sonar_data_msg.hpp"
#include "core/msg/common_command_msg.hpp"
#include "core/msg/PoseStampedMsg.h"
#include "core/msg/laser_scan_msg.hpp"

#include "core/src.h"
#include "core/settings.h"
#include "core/state.h"

using namespace std;

namespace usart {

UsartModule::UsartModule() :
        Module("UsartModule"),
        enable_lmns_usart_(false),
        connection_ptr_(),
        timer_cnt_(0) {
}

UsartModule::~UsartModule() {

}

void UsartModule::run() {
//    waitForStartCommand();

    LOG(INFO) << "UsartModule module start running";

    auto &s = sros::core::Settings::getInstance();
    auto device_name1 = s.getValue<std::string>("lmns.serial_device", "/dev/ttyTHS1");
    auto device_name2 = s.getValue<std::string>("hmi.touch_screen_device_name", "/dev/ttyTHS1");
    auto enable_touch_screen = (s.getValue<std::string>("hmi.enable_touch_screen", "False") == "True");

    enable_lmns_usart_ = (s.getValue<std::string>("main.enable_lmns_usart", "True") == "True");

    if (device_name1 == device_name2 && enable_lmns_usart_ && enable_touch_screen) {
        LOG(WARNING) <<"Settings of UsartModule conflict with touchScreen, UsartModule will not start!";
        return;
    }

    if (!enable_lmns_usart_) {
        // 如果没有启用串口通信，则停止Module执行
        LOG(INFO) << "UsartModule module stop running(disable)";
        stop();
        return;
    }

    subscribeTopic("USART_DATA_RECV", CALLBACK(&UsartModule::onRecvUsartDataMsg));
    subscribeTopic("TOPIC_LASER", CALLBACK(&UsartModule::onLaserScanMsg));

    initUsartConnection();

    dispatch();
}

void UsartModule::initUsartConnection() {
    // SRC串口数据回调只能在一处绑定，所以Usart不绑定SRC串口，转为接收VSC模块发来的串口数据

    // 初始化VC300的LMNS通信串口
    auto &s = sros::core::Settings::getInstance();
    string device_name = s.getValue<std::string>("lmns.serial_device", "/dev/ttyTHS1");
    auto baud_rate = s.getValue<unsigned int>("lmns.serial_baud_rate", 115200);

    connection_ptr_ = make_shared<Connection<FrameV1<>>>();
    connection_ptr_->setRecvDataCallback(boost::bind(&UsartModule::onRecvUsartData, this, _1));

    if (!connection_ptr_->connect(device_name, baud_rate)) {
        LOG(ERROR) << "UsartModule: serial device " << device_name << " open failed!";
    }
}

void UsartModule::onRecvUsartData(const vector<uint8_t> &data) {
    handleUsartLMNSData(data);
}

void UsartModule::onRecvUsartDataMsg(sros::core::base_msg_ptr msg) {
    auto m = dynamic_pointer_cast<sros::core::UsartDataMsg>(msg);

    vector<uint8_t> payload;

    // 截取raw_data中payload部分
    for (int i = 2; i < m->raw_data.size() - 2; i++) {
        payload.push_back(m->raw_data[i]);
    }

    handleUsartLMNSData(payload);
}

void UsartModule::handleUsartLMNSData(const vector<uint8_t> &data) {
    const uint8_t PROTOCOL_VERSION = 0x05; // 串口v1.4版本协议
    uint8_t type = data[0];

    if (type != 0xFF) {
//        LOG(WARNING) << "UsartModule: recv invalid type data " << type;
        return;
    }

    uint8_t cmd = data[1];

    std::vector<uint8_t> r; // 返回的数据
    r.push_back(0xFF); // r[0] = type

    if (cmd == LMNS_CMD_QUERY_VERSION) {
        r.push_back((uint8_t) LMNS_CMD_RETURN_VERSION); // r[1] = cmd

        r.push_back(PROTOCOL_VERSION);
    } else if (cmd == LMNS_CMD_QUERY_STATE) {

        DLOG(INFO) << "onUsartMsg: LMNS_CMD_QUERY_STATE ";

        r.push_back((uint8_t) LMNS_CMD_RETURN_STATE);

        r.push_back((uint8_t) (g_state.sys_state));
        r.push_back((uint8_t) (g_state.location_state));
        r.push_back((uint8_t) (g_state.station_no >> 8)); // 当前站点编号高8位
        r.push_back((uint8_t) (g_state.station_no)); // 当前站点编号低8位

        r.push_back((uint8_t) (g_state.battery_percentage));
        r.push_back((uint8_t) (g_state.battery_state));

        r.push_back((uint8_t) (g_state.operation_state));

        r.push_back(0x00); // 保留位
    } else if (cmd == LMNS_CMD_QUERY_OBSTACLE_POINT) {
        r.push_back((uint8_t) LMNS_CMD_RETURN_OBSTACLE_POINT); // r[1] = cmd

        r.push_back((uint8_t) ((obstacle_p1_.first & 0xFF00) >> 8));
        r.push_back((uint8_t) ((obstacle_p1_.first & 0x00FF)));

        r.push_back((uint8_t) ((obstacle_p1_.second & 0xFF00) >> 8));
        r.push_back((uint8_t) ((obstacle_p1_.second & 0x00FF)));

        r.push_back((uint8_t) ((obstacle_p2_.first & 0xFF00) >> 8));
        r.push_back((uint8_t) ((obstacle_p2_.first & 0x00FF)));

        r.push_back((uint8_t) ((obstacle_p2_.second & 0xFF00) >> 8));
        r.push_back((uint8_t) ((obstacle_p2_.second & 0x00FF)));

        r.push_back((uint8_t) ((obstacle_p3_.first & 0xFF00) >> 8));
        r.push_back((uint8_t) ((obstacle_p3_.first & 0x00FF)));

        r.push_back((uint8_t) ((obstacle_p3_.second & 0xFF00) >> 8));
        r.push_back((uint8_t) ((obstacle_p3_.second & 0x00FF)));

    } else if (cmd == LMNS_CMD_START_LOCATION) {
        // 设置初始位姿
        auto d_msg = make_shared<sros::core::CommandMsg>(getName());
        d_msg->command = sros::core::CMD_SET_LOCATION_INITIAL_POSE;
        d_msg->pose = getPoseFromUsartData(data);
        sendMsg(d_msg);

        DLOG(INFO) << "onUsartMsg: LMNS_CMD_START_LOCATION ";

        if (g_state.slam_state == sros::core::STATE_SLAM_IDLE) {
            // 构造DebugCmdMsg处理
            auto d_msg = std::make_shared<sros::core::CommandMsg>(getName());
            d_msg->command = sros::core::CMD_START_LOCATION;
            d_msg->param0 = 0;

            sendMsg(d_msg);
        }

        // 返回定位状态
        setReturnLocationUsartData(r);
    } else if (cmd == LMNS_CMD_START_LOCATION_STATION) {
        uint16_t station_no = 0;

        station_no = ((uint16_t) data[2]) << 8;
        station_no += data[3];

        LOG(INFO) << "onUsartMsg: LMNS_CMD_START_LOCATION_STATION " << station_no;

        // 构造DebugCmdMsg处理
        auto d_msg = std::make_shared<sros::core::CommandMsg>(getName());
        d_msg->command = sros::core::CMD_START_LOCATION;
        d_msg->param0 = station_no;
        sendMsg(d_msg);

        // 返回定位状态
        setReturnLocationUsartData(r);
    } else if (cmd == LMNS_CMD_STOP_LOCATION) {
        DLOG(INFO) << "onUsartMsg: LMNS_CMD_STOP_LOCATION ";

        auto d_msg = std::make_shared<sros::core::CommandMsg>(getName());
        d_msg->command = sros::core::CMD_STOP_LOCATION;
        sendMsg(d_msg);

        // 返回定位状态
        setReturnLocationUsartData(r);
    } else if (cmd == LMNS_CMD_QUERY_LOCATION) {

        DLOG(INFO) << "onUsartMsg: LMNS_CMD_QUERY_LOCATION ";

        setReturnLocationUsartData(r);
    } else if (cmd == LMNS_CMD_SET_CUR_MAP) {
        char buf[32] = {'\0'}; // 暂存地图名字符串
        for (int i = 2; i < data.size() && i < (2 + 32); i++) {
            buf[i - 2] = data[i];
        }

        DLOG(INFO) << "onUsartMsg: LMNS_CMD_SET_CUR_MAP " << buf;

        auto mm = make_shared<sros::core::CommandMsg>(getName());
        mm->command = sros::core::CMD_SET_CUR_MAP;
        mm->map_name = std::string(buf);
        sendMsg(mm);

        r.push_back((uint8_t) LMNS_CMD_SET_CUR_MAP_ACK);
    } else if (cmd == LMNS_CMD_MOVE_TO_STATION) {

        sros::core::StationNo_t station_no = (data[2] << 8) + data[3];
        auto avoid_policy = (sros::core::ObstacleAvoidPolicy) data[4];

        LOG(INFO) << "onUsartMsg: LMNS_CMD_MOVE_TO_STATION " << station_no;

        std::deque<sros::core::StationNo_t> dst_stations;
        dst_stations.push_back((sros::core::StationNo_t) station_no);

        auto new_task = std::make_shared<sros::core::MovementTask>(0, getName(), dst_stations, avoid_policy);

        auto mm = make_shared<sros::core::CommandMsg>(getName());
        mm->command = sros::core::CMD_NEW_MOVEMENT_TASK;
        mm->movement_task = new_task;

        sendMsg(mm);

        r.push_back((uint8_t) LMNS_CMD_MOVE_TO_STATION_ACK);

    } else if (cmd == LMNS_CMD_MOVE_TO_POSE) {
        sros::core::Pose dst_pose = getPoseFromUsartData(data);
        auto avoid_policy = (sros::core::ObstacleAvoidPolicy) data[14];

        LOG(INFO) << "onUsartMsg: LMNS_CMD_MOVE_TO_POSE";

        std::deque<sros::core::Pose> dst_poses;
        dst_poses.push_back(dst_pose);

        auto new_task = std::make_shared<sros::core::MovementTask>(0, getName(), dst_poses, avoid_policy);

        auto mm = make_shared<sros::core::CommandMsg>(getName());
        mm->command = sros::core::CMD_NEW_MOVEMENT_TASK;
        mm->movement_task = new_task;

        sendMsg(mm);

        r.push_back((uint8_t) LMNS_CMD_MOVE_TO_POSE_ACK);

    } else if (cmd == LMNS_CMD_MOVE_CONTROL) {
        DLOG(INFO) << "onUsartMsg: LMNS_CMD_MOVE_CONTROL";

        auto debug_cmd = (sros::core::CommandType) data[2];
        int param = (data[3] << 8) + data[4];

        // 构造DebugCmdMsg处理
        auto d_msg = std::make_shared<sros::core::CommandMsg>(getName());
        d_msg->command = debug_cmd;
        d_msg->param0 = param;

        sendMsg(d_msg);

        r.push_back((uint8_t) LMNS_CMD_MOVE_CONTROL_ACK);
    } else if (cmd == LMNS_CMD_MOVE_CANCEL) {
        DLOG(INFO) << "onUsartMsg: LMNS_CMD_MOVE_CANCEL";

        auto mm = make_shared<sros::core::CommandMsg>(getName());
        mm->command = sros::core::CMD_COMMON_CANCEL;

        sendMsg(mm);

        r.push_back((uint8_t) LMNS_CMD_MOVE_CANCEL_ACK);
    } else if (cmd == LMNS_CMD_ACTION_NEW) {
        DLOG(INFO) << "onUsartMsg: LMNS_CMD_ACTION_NEW";

        uint8_t action_id = data[2];
        uint16_t action_param_0 = (data[3] << 8) + data[4];

        // FIXME: 需要新建ActionTask
//        auto d_msg = std::make_shared<sros::core::CommandMsg>(getName());
//        d_msg->command = sros::core::CMD_TASK_ACTION;
//        d_msg->param0 = 0;
//
//        sendMsg(d_msg);
//
//        r.push_back((uint8_t) LMNS_CMD_ACTION_NEW_ACK);
    }

    // 通过VC300串口发送数据
    connection_ptr_->sendData(r);

    // 通过SRC发送数据
    auto m = make_shared<sros::core::UsartDataMsg>("USART_DATA_SEND");
    m->raw_data = r;
    m->decodeRawData();

    sendMsg(m);
}

void UsartModule::setReturnLocationUsartData(vector<uint8_t> &r) const {
    r.push_back((uint8_t) LMNS_CMD_RETURN_LOCATION);

    r.push_back((uint8_t) g_state.location_state);

    setPoseToUsartData(src_sdk->getCurPose(), r);
}

void UsartModule::setPoseToUsartData(sros::core::Pose pose, vector<uint8_t> &r) const {
    uint8_t *tmp;

    int pose_x = (int) (pose.x() * 1000);
    tmp = (uint8_t *) (&pose_x);
    r.push_back(tmp[3]);
    r.push_back(tmp[2]);
    r.push_back(tmp[1]);
    r.push_back(tmp[0]);

    int pose_y = (int) (pose.y() * 1000);
    tmp = (uint8_t *) (&pose_y);
    r.push_back(tmp[3]);
    r.push_back(tmp[2]);
    r.push_back(tmp[1]);
    r.push_back(tmp[0]);

    int pose_yaw = (int) (pose.yaw() * 1000);
    tmp = (uint8_t *) (&pose_yaw);
    r.push_back(tmp[3]);
    r.push_back(tmp[2]);
    r.push_back(tmp[1]);
    r.push_back(tmp[0]);
}

sros::core::Pose UsartModule::getPoseFromUsartData(const vector<uint8_t> &data) const {
    char tmp[4];

    tmp[0] = data[2 + 3];
    tmp[1] = data[2 + 2];
    tmp[2] = data[2 + 1];
    tmp[3] = data[2 + 0];
    int pose_x_i = *((int *) tmp);
    double pose_x = pose_x_i / 1000.0;

    tmp[0] = data[2 + 4 + 3];
    tmp[1] = data[2 + 4 + 2];
    tmp[2] = data[2 + 4 + 1];
    tmp[3] = data[2 + 4 + 0];
    int pose_y_i = *((int *) tmp);
    double pose_y = pose_y_i / 1000.0;

    tmp[0] = data[2 + 8 + 3];
    tmp[1] = data[2 + 8 + 2];
    tmp[2] = data[2 + 8 + 1];
    tmp[3] = data[2 + 8 + 0];
    int pose_yaw_i = *((int *) tmp);
    double pose_yaw = pose_yaw_i / 1000.0;

    return sros::core::Pose(sros::core::Location(pose_x, pose_y),
                            sros::core::Rotation(pose_yaw));
}

void UsartModule::onLaserScanMsg(sros::core::base_msg_ptr msg) {
    auto laser_scan = std::dynamic_pointer_cast<sros::core::LaserScanMsg>(msg);

    sros::core::Location_Vector ps;

    const double BIG_VALUE = 999999999;

    pair<double, double> p1(BIG_VALUE, 0); // 最小值
    pair<double, double> p2(BIG_VALUE, 0);
    pair<double, double> p3(BIG_VALUE, 0);

    auto &s = sros::core::Settings::getInstance();
    double user_laser_min = s.getValue<double>("slam.laser_angle_min", -M_PI);
    double user_laser_max = s.getValue<double>("slam.laser_angle_max", M_PI);

    // 强制设置雷达区域为前方180°范围内
    user_laser_min = -M_PI_2;
    user_laser_max = M_PI_2;

    // 获取最小的三个点
    double angle = laser_scan->angle_min;
    for (int i = 0; i < laser_scan->ranges.size(); i++) {
        double range = laser_scan->ranges[i];
        angle += laser_scan->angle_increment;

        if (angle < user_laser_min || angle > user_laser_max) {
            // 不在用户设置的雷达扫描角度范围内
            continue;
        }

        pair<double, double> p(range, angle);

        if (p.first < p3.first) {
            if (p.first < p2.first) {
                if (p.first < p1.first) {
                    p3 = p2;
                    p2 = p1;
                    p1 = p;
                } else {
                    p3 = p2;
                    p2 = p;
                }
            } else {
                p3 = p;
            }
        }
    }

    obstacle_p1_.first = (u_int16_t) (p1.first * 1000);
    obstacle_p1_.second = (int16_t) (p1.second * 1000 * -1); // 取反是为了与串口通信协议角度正负规则保持一致

    obstacle_p2_.first = (u_int16_t) (p2.first * 1000);
    obstacle_p2_.second = (int16_t) (p2.second * 1000 * -1);

    obstacle_p3_.first = (u_int16_t) (p3.first * 1000);
    obstacle_p3_.second = (int16_t) (p3.second * 1000 * -1);

}

} /* namespace usart */
