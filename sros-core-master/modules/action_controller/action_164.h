//
// Created by caoyan on 4/9/21.
//

#ifndef SROS_ACTION_164_H
#define SROS_ACTION_164_H

#include "base_action.h"

using namespace std;
using namespace sros::core;

namespace ac {

// ACTION_ID_LOAD_MOVE_BACK    = 164,   //载货退出动作
class Action164 : public BaseAction {
 public:
    Action164() : BaseAction(ACTION_ID_LOAD_MOVE_BACK) {}
    virtual ~Action164() {}

    void doStart() override;

    void onSrcAcFinishSucceed(int result_value) override;
    void onSrcAcFinishFailed(int result_value) override;
    void onSrcAcFinishCanceled(int result_value) override;
    void onSrcMcFinishSucceed(int result_value) override;
    void onSrcMcFinishFailed(int result_value) override;

    bool doCancelToInCancel() override { return true;}
    void doInCancel() override;
    void doTimerEvent (uint64_t cur_time) override;

 private:
    void genOneSegmentMovePath(sros::core::NavigationPath_vector& dst_paths);
    void genTwoSegmentMovePath(sros::core::NavigationPath_vector& dst_paths);
    void doForkLift();

 private:
    //站点位姿
    Pose dst_pose_;

    bool is_move_task_running_ = false;

    bool is_forklift_running_ = false;

    double back_move_test_len_;

    bool enable_action_check_pallet_signal_ = false;
};

}




#endif  // SROS_ACTION_164_H
