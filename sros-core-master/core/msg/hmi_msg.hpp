/**
 * @file hmi_msg
 *
 * @author pengjiali
 * @date 2020/7/8.
 *
 * @describe
 *
 * @copyright Copyright (c) 2020 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef SROS_HMI_MSG_H
#define SROS_HMI_MSG_H

#include "base_msg.h"

enum HmiCommand {
    HMI_COMMAND_NONE = 0,
    HMI_COMMAND_STOP = 1, // 停止喇叭播放，灯光边红。重启车辆前用到
    HMI_COMMAND_MUTE,
    HMI_COMMAND_MUTE_CANCEL,
    HMI_COMMAND_SET_ERROR_MUSIC, // 设置执行出错的播放音乐，播放音乐id见参数int_0_
    HMI_COMMAND_SET_SUCCEED_MUSIC, // 设置执行成功的播放音乐，播放音乐id见参数int_0_
};

namespace sros {
namespace core {
class HmiMsg : public BaseMsg {
 public:
    HmiMsg() : BaseMsg("TOPIC_MUSIC", TYPE_HMI) {}

    virtual ~HmiMsg(){};

    virtual void getTime(){};

    HmiCommand command = HMI_COMMAND_NONE;
    int int_0 = 0;
};

}  // namespace core
}  // namespace sros
#endif  // SROS_HMI_MSG_H
