//
// Created by caoyan on 7/22/21.
//

#ifndef SROS_ACTION_136_H
#define SROS_ACTION_136_H

#include "base_action.h"

using namespace std;
using namespace sros::core;

namespace ac {

// ACTION_ID_POSTURE_CORRECT = 136,    //位姿矫正动作
class Action136 : public BaseAction {
public:
    Action136() : BaseAction(ACTION_ID_POSTURE_CORRECT) {}
    virtual ~Action136() {}

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
    void genMovePath(const Pose &mid_pose, const Pose &dst_pose, NavigationPath_vector& dst_paths);
    uint32_t CalcSrcActionNo();
    void sendDetectCmd(bool first_send_cmd = false);
    void toggleSendSrcCodeInfoState(const std::string &camera_name,bool state);
private:
    PostureCorrectCommandMsg::EnumCorrectDir correct_dir_;
    uint32_t src_action_no_;
    uint32_t last_src_action_no_;
    std::string camera_device_name_;

    bool is_move_task_running_;
    bool is_src_action_running_;

    uint32_t replan_paths_count_;
    uint32_t pose_correct_count_;

};

}

#endif //SROS_ACTION_136_H
