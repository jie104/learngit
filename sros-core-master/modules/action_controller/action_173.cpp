//
// Created by caoyan on 4/10/21.
//
#include "action_173.h"
#include "core/logger.h"
#include "core/msg_bus.h"
#include "core/state.h"
#include "path_utils.h"

using namespace std;
using namespace sros::core;

namespace ac {

void Action173::doStart() {
    
    is_move_task_running_ = false;
    is_forklift_running_ = false;

    if((!isForkControlTypeSrc()) && (!isEnableEac())) {
        doActionFinishFailed(ERROR_CODE_ACTION_EAC_DISABLED);
        return;
    }

    auto& s = sros::core::Settings::getInstance();
    enable_action_check_pallet_signal_ = (s.getValue<string>("forklift.enable_action_check_pallet_signal", "True") == "True");

//    if(!checkPalletInPlaceSignal()) {
//        LOG(ERROR) << "checkPalletInPlaceSignal: false";
//        sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_LOAD_PALLET_BOUNCE);
//        doActionFinishFailed(sros::core::ERROR_CODE_ACTION_LOAD_PALLET_BOUNCE);
//        return;
//    }

    // 1.通过目标站点的id去获取目标点的位姿
    auto dst_station_id = action_param0_;
    auto dst_station = MapManager::getInstance()->getStation(dst_station_id);

    if (dst_station_id != 0 && dst_station.id == 0) {
        LOG(ERROR) << "not exist station id: " << dst_station_id;
        doActionFinishFailed(sros::core::ERROR_CODE_NAV_FIND_PATH_NO_STATION_IN_MAP);
        return;
    }

    enableBackLidar(false);

    dst_pose_.x() = dst_station.pos.x / 100.0;
    dst_pose_.y() = dst_station.pos.y / 100.0;
    //dst_pose_.yaw() = normalizeYaw(dst_station.pos.yaw);

    dst_pose_.yaw() = g_state.dst_pose_yaw;

    //防止支路中心的站点方向搞错
    double deviate_max_yaw = s.getValue<double>("forklift.deviate_max_yaw", 10.0);
    deviate_max_yaw = deviate_max_yaw / 180 * M_PI;
    auto curr_pose = src_sdk->getCurPose();

    double offset_yaw = std::fabs(normalizeYaw((dst_pose_.yaw() - curr_pose.yaw())));
    LOG(INFO) << "[offset_yaw: " << offset_yaw << "][deviate_max_yaw: " << deviate_max_yaw << "]"; 
    if(offset_yaw > deviate_max_yaw) {
        LOG(INFO) << "net station yaw deviate too large: offset_yaw(" 
                  << offset_yaw << ") > deviate_max_yaw(" << deviate_max_yaw << ")";
        dst_pose_.yaw() = curr_pose.yaw();
    }

    LOG(INFO) << "dst_pose_, [x: " << dst_pose_.x() << "][y: " << dst_pose_.y() << "][yaw: " << dst_pose_.yaw() << "]";

    back_move_test_len_ = s.getValue<double>("forklift.back_move_test_len", 0.1);//m
    back_move_test_len_ = back_move_test_len_ * 100; //cm

    // 2.判断当前位姿是否已经在站点
    if(!checkCurPoseArriveDestStation(dst_station)) {
        sros::core::NavigationPath_vector dst_paths;
        genMovePath(dst_paths);

        //调试
        debugOuputPaths(dst_paths);

        //向SRC发送移动对接路径
        sendMoveTask(dst_paths);

        is_move_task_running_ = true;

    } else {
        doForkLift();
        is_forklift_running_ = true;
    }
}

void Action173::onSrcAcFinishSucceed(int result_value) {

    auto& s = sros::core::Settings::getInstance();
    float fork_move_height = s.getValue<float>("forklift.fork_move_height", 0.5);
    if (checkForkHeightFautl(fork_move_height)) {
        return;
    }

    is_forklift_running_ = false;
    //成功返回
    doActionFinishSucceed();

    g_state.is_send_forklift_common_poses = false;
}

void Action173::onSrcAcFinishFailed(int result_value) {
    is_forklift_running_ = false;
    LOG(ERROR) << "货叉降叉至行走高度失败, result_value:" << result_value;
    sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_FORK_LIFT_FAULT);
    doActionFinishFailed(sros::core::ERROR_CODE_ACTION_FORK_LIFT_NOT_REACH);
}

void Action173::onSrcAcFinishCanceled(int result_value) {
    is_forklift_running_ = false;
    LOG(ERROR) << "货叉降叉至行走高度失败";
    sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_FORK_LIFT_FAULT);
    doActionFinishFailed(sros::core::ERROR_CODE_ACTION_FORK_LIFT_NOT_REACH);
}

void Action173::onSrcMcFinishSucceed(int result_value) {
    is_move_task_running_ = false;
    doForkLift();
    is_forklift_running_ = true;
    
}

void Action173::onSrcMcFinishFailed(int result_value) {
    is_move_task_running_ = false;
    LOG(ERROR) << "卸货退出路径导航失败";
    sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_BACKFORK_MOVE_NOT_REACH);
    doActionFinishFailed(sros::core::ERROR_CODE_ACTION_UNLOAD_MOVE_NOT_REACH);
    enableBackLidar(true);
}

void Action173::genMovePath(sros::core::NavigationPath_vector& dst_paths) {
    LOG(INFO) << "genMovePath";

    auto curr_pose = src_sdk->getCurPose();
    LOG(INFO) << "curr_pose: [x: " <<  curr_pose.x() << "][y: " << curr_pose.y() << "][yaw: " << curr_pose.yaw() << "]";
    LOG(INFO) << "dst_pose: [x: " <<  dst_pose_.x() << "][y: " << dst_pose_.y() << "][yaw: " << dst_pose_.yaw() << "]";

    LinePath p;
    p = makeLine(curr_pose.x(), curr_pose.y(), dst_pose_.x(), dst_pose_.y(), PATH_FORWARD);
    dst_paths.push_back(p);
}


void Action173::doInCancel() {

    if(is_move_task_running_) {
        //这里发起结束移动
        cancelMoveTask();
        is_move_task_running_ = false;
    }

    if(is_forklift_running_) {
        if(isForkControlTypeSrc()) {
            src_sdk->cancelAction(action_no_);
        } else {
            cancelEacActionTask(action_no_);
        }

        is_forklift_running_ = false;
    }

    enableBackLidar(true);
}

void Action173::doTimerEvent (uint64_t cur_time) {
    if(is_move_task_running_) {
        //LOG(INFO) << "g_src_state.src_state: " << (int)g_src_state.src_state;
        //src 运行状态
        if(g_src_state.src_state != SRC_STATE_t::STATE_PATH_RUNNING) {
            return;
        }

        double move_length = g_src_state.cur_total_distance - g_src_state.cur_remain_distance;

        // LOG(INFO) << "total_distance: " << g_src_state.cur_total_distance
        //           << ", g_src_state.cur_remain_distance: " << g_src_state.cur_remain_distance
        //           << ", move_length: " << move_length << ", back_move_test_len_: " << back_move_test_len_;

        if(move_length >= back_move_test_len_) {
            if(enable_action_check_pallet_signal_ && checkPalletInPlaceSignal()) {
                //这里发起结束移动
                cancelMoveTask();

                LOG(ERROR) << "退叉测试，栈板到位信号仍然触发";
                sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_BACKFORK_MOVE_NOT_REACH);
                doActionFinishFailed(sros::core::ERROR_CODE_ACTION_BACKFORK_MOVE_NOT_REACH);
                enableBackLidar(true);

                is_move_task_running_ = false;
            } else {
                //LOG(INFO) << "退叉测试成功";
            }
        }
    }
}

void Action173::doForkLift() {
    //打开后侧避障雷达，关闭R2000滤波
    enableBackLidar(true);

    // 向SRC发送动作指令，降叉到行走时货叉的高度H
    auto& s = sros::core::Settings::getInstance();
    uint16_t fork_move_height = s.getValue<double>("forklift.fork_move_height", 0.5) * 1000;

    if(isForkControlTypeSrc()) {
        src_sdk->executeAction(action_no_, 24, 10, fork_move_height);
        LOG(INFO) << "src_ac: no " << action_no_ << ", "
                  << "id 24, p0 10, p1 " << fork_move_height;
    } else {
        sendEacActionTask(action_no_, 207, 1, fork_move_height);
        LOG(INFO) << "eac_ac: no " << action_no_ << ", "
                  << "id 207, p0 1, p1 " << fork_move_height;
    }

}






}