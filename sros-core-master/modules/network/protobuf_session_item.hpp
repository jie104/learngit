/**
 * @file protobuf_session_item.hpp
 *
 * @author pengjiali
 * @date 2019/06/04
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef SROS_PROTOBUF_SESSION_ITEM_HPP
#define SROS_PROTOBUF_SESSION_ITEM_HPP

#include "core/session_manager.h"

namespace network {

class ProtobufSessionItem : public sros::core::SessionItem {
 public:
    ProtobufSessionItem(const std::string &username, const std::string &ip, unsigned short port,
                        bool is_upload_laser_point)
        : SessionItem(username, ip, port), is_upload_laser_point_(is_upload_laser_point) {}

    bool is_upload_laser_point_ = false;  // 是否上传雷达点, 初始值根据main.enable_upload_laser_point来
    bool is_upload_avoid_obstacle_prediction_ = false;  // 是否避障预测信息

    bool need_debug_info_ = false; // 是否需要调试信息

    bool need_debug_lidar_fr_ = false;          //是否需要雷达特征信息
    bool need_debug_front_camera_fr_ = false;   //是否需要前置摄像头特征信息
    bool need_debug_back_camera_fr_ = false;    //是否需要后置摄像头特征信息
    bool need_debug_up_camera_fr_ = false;      //是否需要上视摄像头特征信息
    bool need_debug_down_camera_fr_ = false;    //是否需要下视摄像头特征信息
    bool need_debug_left_camera_fr_ = false;    //是否需要左侧摄像头特征信息
    bool need_debug_right_camera_fr_ = false;   //是否需要右侧摄像头特征信息

    bool is_upload_system_state = true; // 是否主动上传系统个状态
    bool is_upload_hardware_state = true; // 是否主动上传个状态
};

using ProtobufSessionItem_ptr = std::shared_ptr<ProtobufSessionItem>;

}  // namespace network

#endif  // SROS_PROTOBUF_SESSION_ITEM_HPP
