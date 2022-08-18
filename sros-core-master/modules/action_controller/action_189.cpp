//
// Created by lbx on 2022/1/8.
//

#include "action_189.h"
#include "core/logger.h"
#include "core/settings.h"
#include "core/msg_bus.h"

using namespace std;
using namespace sros::core;

namespace ac {

void Action189::doStart() {
    detectDst3DPose();
}

void Action189::detectDst3DPose() {

    Pose dst_pose_;

    PerceptionCommandMsg::Command cmd;
    cmd.detect_stage = PerceptionCommandMsg::DetectStage::DETECT_STAGE_LOAD;
    cmd.object_type = PerceptionCommandMsg::ObjectType::OBJECT_TYPE_QRCODE;
    if(action_param0_ == 0x01){
        cmd.object_type = PerceptionCommandMsg::ObjectType::OBJECT_TYPE_QRCODE_GLOBAL;
    }else{
        cmd.object_type = PerceptionCommandMsg::ObjectType::OBJECT_TYPE_QRCODE;
    }
    cmd.is_enable = true;

    sendPerceptionCmd(action_no_, cmd, g_state.cur_goal_id, dst_pose_);

}

void Action189::onAlgoResultCallback(const sros::core::base_msg_ptr& msg) {
 

    auto mm = dynamic_pointer_cast<PerceptionStateMsg>(msg);

    int detect_result = mm->detect_result;

    bool result = (detect_result == PerceptionStateMsg::DetectResult::DETECT_RESULT_SUCCESS);
    LOG(INFO) << "result: " << result;
    //拿到结果之后
    if (!result) {
        //未检测到货物
        doCancel();
        doActionFinishFailed(PerceptionStateMsg::errCodeConvert(mm->error_code));
        return;
    }
    Pose dst_pose_;
    double x = mm->goal_in_global_pose.x();
    double y = mm->goal_in_global_pose.y();
    double yaw = mm->goal_in_global_pose.yaw();
    int goal_id = mm->goal_id;

    NavigationPath_vector paths; //生成待优化
    
    // 下发更改避障模型指令
    sendAvoidObstacleCmd(AvdObaCommandMsg::AvoidObstacleFunctionType::CHANGE_MDOEL_AVD, goal_id, dst_pose_, paths); 


}
void Action189::sendAvoidObstacleCmd(AvdObaCommandMsg::AvoidObstacleFunctionType avdoba_type, const int& goal_id,
                                     Pose pose, NavigationPath_vector paths) {
    AvdObaCommandMsg::Command command;
    command.oba_model.is_change_oba_model = true;
    command.oba_model.QR_Code_id = goal_id;

    sendAvdObaCmd(action_no_, command);
}

}
