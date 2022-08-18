//
// Created by caoyan on 4/10/21.
//

#ifndef SROS_ACTION_171_H
#define SROS_ACTION_171_H

#include "base_action.h"

using namespace std;
using namespace sros::core;

namespace ac {

//ACTION_ID_UNLOAD_DETECT     = 171,   //卸货检测动作
class Action171 : public BaseAction {
 public:
    Action171(): BaseAction(ACTION_ID_UNLOAD_DETECT) {}
    virtual ~Action171() {}

    void doStart() override;
    void doCancel() override;

    void onSrcAcFinishSucceed(int result_value) override;
    void onSrcAcFinishFailed(int result_value) override;
    void onSrcAcFinishCanceled(int result_value) override;

    void onAlgoResultCallback(const sros::core::base_msg_ptr& msg) override;

    void onSrcMcFinishSucceed(int result_value) override;
    void onSrcMcFinishFailed(int result_value) override;

    bool onSubSrcActionCheck(uint32_t& sub_src_action_no) override;

 private:
    void detectGoodsRacks();

    void handleUnloadDetect();

 private:
    //站点位姿
    double dst_pose_x_;    // m
    double dst_pose_y_;    // m
    double dst_pose_yaw_;  //弧度

    bool is_forklift_running_ = false;
    bool is_detect_goods_racks_running_ = true;
    bool is_keep_recv_src_ret_ = true;

    bool enable_action_check_pallet_signal_ = false;
};


}


#endif  // SROS_ACTION_171_H
