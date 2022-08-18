//
// Created by caoyan on 7/22/21.
//

#include "action_136.h"
#include "core/logger.h"
#include "core/msg_bus.h"
#include "core/state.h"
#include "path_utils.h"

using namespace std;
using namespace sros::core;

namespace ac {

#define REPLAN_PATHS_MAX_COUNT (3)
#define POSE_CORRECT_MAX_COUNT (5)

void Action136::doStart() {

    is_move_task_running_ = false;
    is_src_action_running_ = false;

    replan_paths_count_ = 0;
    pose_correct_count_ = 0;

    camera_device_name_ = sros::device::DEVICE_SVC100_UP;
    if(action_param0_ == 0){
        correct_dir_ = PostureCorrectCommandMsg::EnumCorrectDir::CORRECT_DIR_FORWARD;
    }else if(action_param0_ == 1){
        correct_dir_ = PostureCorrectCommandMsg::EnumCorrectDir::CORRECT_DIR_BACK;
    }else if(action_param0_ == 2){
        correct_dir_ = PostureCorrectCommandMsg::EnumCorrectDir::CORRECT_DOWN_FORWARD;
    }else if(action_param0_ == 3){
        correct_dir_ = PostureCorrectCommandMsg::EnumCorrectDir::CORRECT_DOWN_BACK;
    }else if(action_param0_ == 4){
        correct_dir_ = PostureCorrectCommandMsg::EnumCorrectDir::CORRECT_DOWN_CHARGE_FORWARD;
    }else if(action_param0_ == 5){
        correct_dir_ = PostureCorrectCommandMsg::EnumCorrectDir::CORRECT_DOWN_CHARGE_BACK;
    }else{
        LOG(INFO) << "cannot detect action param!";
        doActionFinishFailed(ERROR_CODE_ACTION_PARAM_0_NOT_SUPPORT);
        return;
    }
    if (action_param0_ == 2 || action_param0_ == 3 || action_param0_ == 4 || action_param0_ == 5) {
        camera_device_name_ = sros::device::DEVICE_SVC100_DOWN;
    }

    //打开上视扫码
    enableSVC100Camera(true, camera_device_name_);
    toggleSendSrcCodeInfoState(camera_device_name_, true);
    sendDetectCmd(true);

    src_action_no_ = -1;
    last_src_action_no_ = -1;
}

void Action136::doCancel() {
    if (is_move_task_running_) {
        cancelMoveTask();
        is_move_task_running_ = false;
    }

    if (is_src_action_running_) {
        src_sdk->cancelAction(src_action_no_);
        is_src_action_running_ = false;
    }

    enableSVC100Camera(false, camera_device_name_);
    toggleSendSrcCodeInfoState(camera_device_name_, false);
}

void Action136::onSrcAcFinishSucceed(int result_value) {
    src_action_no_ = -1;
    sendDetectCmd();
}

void Action136::onSrcAcFinishFailed(int result_value) {
    enableSVC100Camera(false, camera_device_name_);
    toggleSendSrcCodeInfoState(camera_device_name_, false);
    doActionFinishFailed(result_value);
}

void Action136::onSrcAcFinishCanceled(int result_value) {
    enableSVC100Camera(false, camera_device_name_);
    toggleSendSrcCodeInfoState(camera_device_name_, false);
    doActionFinishFailed(result_value);
}

void Action136::onSrcMcFinishSucceed(int result_value) {
    sendDetectCmd();
}

void Action136::onSrcMcFinishFailed(int result_value) {
    enableSVC100Camera(false, camera_device_name_);
    toggleSendSrcCodeInfoState(camera_device_name_, false);
    doActionFinishFailed(result_value);
}

bool Action136::onSubSrcActionCheck(uint32_t& sub_src_action_no) {
    sub_src_action_no = src_action_no_;
    return true;
}

void Action136::onAlgoResultCallback(const sros::core::base_msg_ptr& msg) {
    auto mm = dynamic_pointer_cast<PostureCorrectCommandMsg>(msg);

    if (mm == nullptr) return;

    auto correct_result = mm->result.correct_result;
    LOG(INFO) << "correct_result: " << correct_result;

    if (correct_result == PostureCorrectCommandMsg::CORRECT_RESULT_SUCCESS) {
        enableSVC100Camera(false, camera_device_name_);
        toggleSendSrcCodeInfoState(camera_device_name_, false);
        doActionFinishSucceed();

    } else if (correct_result == PostureCorrectCommandMsg::CORRECT_RESULT_FAIL) {
        enableSVC100Camera(false, camera_device_name_);
        toggleSendSrcCodeInfoState(camera_device_name_, false);

        auto error_code = mm->result.error_code;
        doActionFinishFailed(error_code);

    } else if (correct_result == PostureCorrectCommandMsg::CORRECT_RESULT_REPLAN_PATHS) {
        replan_paths_count_ = replan_paths_count_ + 1;
        LOG(INFO) << "CORRECT_RESULT_REPLAN_PATHS: " << replan_paths_count_;
        if (replan_paths_count_ > REPLAN_PATHS_MAX_COUNT) {
            enableSVC100Camera(false, camera_device_name_);
            toggleSendSrcCodeInfoState(camera_device_name_, false);
            doActionFinishFailed(ERROR_CODE_AGV_NOT_ARRIVED_DEST);
            return;
        }

        auto mid_pose = mm->result.pose.mid_pose;
        auto dst_pose = mm->result.pose.dst_pose;

        //生成调整路径
        sros::core::NavigationPath_vector dst_paths;
        genMovePath(mid_pose, dst_pose, dst_paths);

        //发送路径
        debugOuputPaths(dst_paths);
        sendMoveTask(dst_paths);

    } else if (correct_result == PostureCorrectCommandMsg::CORRECT_RESULT_POSE_CORRECT) {
        pose_correct_count_ = pose_correct_count_ + 1;
        LOG(INFO) << "CORRECT_RESULT_POSE_CORRECT: " << pose_correct_count_;
        if (pose_correct_count_ > POSE_CORRECT_MAX_COUNT) {
            enableSVC100Camera(false, camera_device_name_);
            toggleSendSrcCodeInfoState(camera_device_name_, false);
            doActionFinishFailed(ERROR_CODE_AGV_NOT_ARRIVED_DEST);
            return;
        }

        int16_t offset_x = mm->result.offset.offset_x;
        int16_t offset_y = mm->result.offset.offset_y;
        int32_t offset_angle = mm->result.offset.offset_angle;

        src_action_no_ = CalcSrcActionNo();
        src_sdk->executeActionInt(src_action_no_, 40, (uint16_t)offset_x, (uint16_t)offset_y, offset_angle);
        LOG(INFO) << "######## SRC::executeAction(): "
                  << "no " << src_action_no_ << ", "
                  << "id " << 40 << ", "
                  << "p0 " << offset_x << ", "
                  << "p1 " << offset_y << ", "
                  << "p2 " << offset_angle;
    }

}

void Action136::genMovePath(const Pose &mid_pose, const Pose &dst_pose, NavigationPath_vector& dst_paths) {
    //当前点位姿
    auto curr_pose = src_sdk->getCurPose();
    LOG(INFO) << "curr_pose, [x: " << curr_pose.x() << "][y: " << curr_pose.y() << "][yaw: " << curr_pose.yaw() << "]";
    LOG(INFO) << "mid_pose, [x: " << mid_pose.x() << "][y: " << mid_pose.y() << "][yaw: " << mid_pose.yaw() << "]";
    LOG(INFO) << "dst_pose, [x: " << dst_pose.x() << "][y: " << dst_pose.y() << "][yaw: " << dst_pose.yaw() << "]";

    //第一阶段贝塞尔回退的路径方向
    PathDirection path_dir_first;
    if (correct_dir_ == PostureCorrectCommandMsg::EnumCorrectDir::CORRECT_DIR_FORWARD ||
        correct_dir_ == PostureCorrectCommandMsg::EnumCorrectDir::CORRECT_DOWN_FORWARD||
        correct_dir_ == PostureCorrectCommandMsg::EnumCorrectDir::CORRECT_DOWN_CHARGE_FORWARD) {
        path_dir_first = PathDirection::PATH_BACKWARD;
    } else {
        path_dir_first = PathDirection::PATH_FORWARD;
    }

    BezierPath bez = genBezierPath(curr_pose, mid_pose, path_dir_first);
    bez.updateFacing();
//    bez.limit_v_
    dst_paths.push_back(bez);

    //第二阶段直线前进的路径方向
    PathDirection path_dir_second;
    if (correct_dir_ == PostureCorrectCommandMsg::EnumCorrectDir::CORRECT_DIR_FORWARD ||
        correct_dir_ == PostureCorrectCommandMsg::EnumCorrectDir::CORRECT_DOWN_FORWARD ||
        correct_dir_ == PostureCorrectCommandMsg::EnumCorrectDir::CORRECT_DOWN_CHARGE_FORWARD) {
        path_dir_second = PathDirection::PATH_FORWARD;
    } else {
        path_dir_second = PathDirection::PATH_BACKWARD;
    }
    NavigationPath<double> rotate_path_e(dst_pose.yaw());
    dst_paths.push_back(rotate_path_e);
    LinePath p;
    p = makeLine(mid_pose.x(), mid_pose.y(), dst_pose.x(), dst_pose.y(), path_dir_second);
    dst_paths.push_back(p);
    dst_paths.push_back(rotate_path_e);
    for (auto& path : dst_paths) {
        path.limit_v_ = 0.15;
    }
    //    PathDirection path_dir_first;
//    double fisrt_rotate_direction = 0;
//    if (correct_dir_ == PostureCorrectCommandMsg::EnumCorrectDir::CORRECT_DIR_FORWARD) {
//        path_dir_first = PathDirection::PATH_BACKWARD;
//        fisrt_rotate_direction = atan2(curr_pose.y() - mid_pose.y(), curr_pose.x() - mid_pose.x());
//    } else {
//        path_dir_first = PathDirection::PATH_FORWARD;
//        fisrt_rotate_direction = atan2(-curr_pose.y() + mid_pose.y(), -curr_pose.x() + mid_pose.x());
//    }
//    NavigationPath<double> rotate_path(fisrt_rotate_direction);
//    dst_paths.push_back(rotate_path);
//    LinePath first_p;
//    first_p = makeLine(curr_pose.x(), curr_pose.y(), mid_pose.x(), mid_pose.y(), path_dir_first);
//    dst_paths.push_back(first_p);
////    BezierPath bez = genBezierPath(curr_pose, mid_pose, path_dir_first);
////    bez.updateFacing();
//
//    //第二阶段直线前进的路径方向
//    PathDirection path_dir_second;
//    if (correct_dir_ == PostureCorrectCommandMsg::EnumCorrectDir::CORRECT_DIR_FORWARD) {
//        path_dir_second = PathDirection::PATH_FORWARD;
//    } else {
//        path_dir_second = PathDirection::PATH_BACKWARD;
//    }
//    NavigationPath<double> rotate_path_m(dst_pose.yaw());
//    dst_paths.push_back(rotate_path_m);
//    LinePath p;
//    p = makeLine(mid_pose.x(), mid_pose.y(), dst_pose.x(), dst_pose.y(), 0.1, 0, path_dir_second);
//    dst_paths.push_back(p);
//    NavigationPath<double> rotate_path_e(dst_pose.yaw());
//    dst_paths.push_back(rotate_path_e);
//    for (auto& path : dst_paths) {
//        path.limit_v_ = 0.1;
////        path.limit_w_ = 0.2;
//    }
    //genRotateBetweenPaths(dst_paths);
    //genRotateBetweenPaths(dst_paths);
}

uint32_t Action136::CalcSrcActionNo() {
    if (last_src_action_no_ == action_no_) {
        last_src_action_no_ = action_no_ - 1;
        return (action_no_ - 1);
    } else {
        last_src_action_no_ = action_no_;
        return action_no_;
    }
}

void Action136::sendDetectCmd(bool first_send_cmd) {

    sendPostureCorrectCmd(action_no_,
                          PostureCorrectCommandMsg::EnumCorrectCmd::CORRECT_CMD_START,
                          camera_device_name_,
                          correct_dir_,first_send_cmd);
}
void Action136::toggleSendSrcCodeInfoState(const std::string& camera_name, bool state) {
    if(camera_name == sros::device::DEVICE_SVC100_UP){
        g_state.is_forbid_send_up_svc100_to_src = state;
    }else if(camera_name == sros::device::DEVICE_SVC100_DOWN){
        g_state.is_forbid_send_down_svc100_to_src = state;
    }
}

}

