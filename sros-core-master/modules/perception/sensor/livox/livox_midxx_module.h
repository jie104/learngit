/**
 * @file livox_midxx_module.h
 * @brief 简述文件内容
 *
 * 此处写文件的详细内容，注意需要与上下内容用空格分开。
 *
 * @author zhangxu@standard-robots.com
 * @date create date：2021/6/4
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef SROS_LIVOX_MIDXX_MODULE_H
#define SROS_LIVOX_MIDXX_MODULE_H

// INCLUDE
#include "modules/laser/base_laser_module.h"
#include "lds_lidar.h"
#include "core/core.h"
#include "core/module.h"

#include "core/msg/livox_points_msg.hpp"


// CODE
/**
 * @description : TODO
 * @author      : zhangxu
 * @date        : 2021/6/4 下午9:15
 */
namespace laser {
class LivoxMidXXModule : public sros::core::Module {
 public:
    explicit LivoxMidXXModule();

    ~LivoxMidXXModule() override;

    bool init();

 private:
    void doOpen();

    void onTimer_1s(const sros::core::base_msg_ptr &msg);

    void onEnablePublisherMsg(const sros::core::base_msg_ptr &msg);

    virtual void run() override;

    void updateFrame();

private:
    void initLivoxMsgHead();


    sros::device::Device_ptr device_;

    std::atomic_bool is_update_frame_{};
    bool is_enable_publisher_{};
    std::thread update_frame_thread_;

    std::shared_ptr<sros::core::LivoxPointsMsg> livox_msg_ptr_ ;

    /** @brief The number of accumulated laser frames. */
    const int ACCUMULATE_MID70_FRAMES_COUNT = 3000 ;
};
}
#endif  // SROS_LIVOX_MIDXX_MODULE_H