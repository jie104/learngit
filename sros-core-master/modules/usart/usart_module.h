/*
 * usart_module.h
 *
 *  Created on: 2016/12/01
 *      Author: lhx
 */

#ifndef SROS_USART_MODULE_H_
#define SROS_USART_MODULE_H_

#include "core/core.h"
#include "core/pose.h"
#include "core/module.h"
#include "core/msg/str_msg.hpp"

#include "core/usart/frame_v1.h"
#include "core/usart/connection.hpp"

namespace usart {

class UsartModule: public sros::core::Module {
public:
    UsartModule();
    virtual ~UsartModule();

    virtual void run();

private:

    enum LMNS_CMD {
        LMNS_CMD_QUERY_VERSION = 0x11,
        LMNS_CMD_RETURN_VERSION = 0x12,
        LMNS_CMD_QUERY_STATE = 0x13,
        LMNS_CMD_RETURN_STATE = 0x14,
        LMNS_CMD_QUERY_OBSTACLE_POINT = 0x15,
        LMNS_CMD_RETURN_OBSTACLE_POINT = 0x16,

        LMNS_CMD_START_LOCATION = 0x21,
        LMNS_CMD_STOP_LOCATION = 0x22,
        LMNS_CMD_QUERY_LOCATION = 0x23,
        LMNS_CMD_RETURN_LOCATION = 0x24,
        LMNS_CMD_SET_CUR_MAP = 0x25,
        LMNS_CMD_SET_CUR_MAP_ACK = 0x26,
        LMNS_CMD_START_LOCATION_STATION = 0x27,

        LMNS_CMD_MOVE_TO_STATION = 0x31,
        LMNS_CMD_MOVE_TO_STATION_ACK = 0x32,
        LMNS_CMD_MOVE_TO_POSE = 0x33,
        LMNS_CMD_MOVE_TO_POSE_ACK = 0x34,
        LMNS_CMD_MOVE_CONTROL = 0x35,
        LMNS_CMD_MOVE_CONTROL_ACK = 0x36,
        LMNS_CMD_MOVE_CANCEL = 0x37,
        LMNS_CMD_MOVE_CANCEL_ACK = 0x38,

        LMNS_CMD_ACTION_NEW = 0x41,
        LMNS_CMD_ACTION_NEW_ACK = 0x42,
    };


    bool enable_lmns_usart_; // 是否启用LMNS串口通信

    uint64_t timer_cnt_; // 100ms定时器计数

    typedef pair<u_int16_t, int16_t> ObstaclePoint_t;
    ObstaclePoint_t obstacle_p1_;
    ObstaclePoint_t obstacle_p2_;
    ObstaclePoint_t obstacle_p3_;

    shared_ptr<Connection<FrameV1<>>> connection_ptr_; // 串口通信抽象

    void initUsartConnection();

    void onRecvUsartData(const vector<uint8_t>& data);

    void onRecvUsartDataMsg(sros::core::base_msg_ptr msg);

    void onLaserScanMsg(sros::core::base_msg_ptr msg);

    void handleUsartLMNSData(const std::vector<uint8_t> &data);

    sros::core::Pose getPoseFromUsartData(const std::vector<uint8_t> &data) const;

    void setPoseToUsartData(sros::core::Pose pose, std::vector<uint8_t> &r) const;

    void setReturnLocationUsartData(std::vector<uint8_t> &r) const;

};

} /* namespace usart */

#endif /* SROS_USART_MODULE_H_ */
