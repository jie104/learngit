//
// Created by caoyan on 4/10/21.
//

#include "action_171.h"
#include "core/fault_center.h"
#include "core/logger.h"
#include "core/msg_bus.h"
#include "core/settings.h"
#include "core/state.h"
#include "rfid_manager.h"
#include "path_utils.h"

using namespace std;
using namespace sros::core;

namespace ac {

#define STR_UNLOAD_LASER_NONE "UNLOAD_LASER_NONE"
#define STR_UNLOAD_LASER_HAVE "UNLOAD_LASER_HAVE"

void Action171::doStart() {
    is_forklift_running_ = false;
    is_detect_goods_racks_running_ = true;
    is_keep_recv_src_ret_ = true;

    auto& s = sros::core::Settings::getInstance();
    enable_action_check_pallet_signal_ =
        (s.getValue<string>("forklift.enable_action_check_pallet_signal", "True") == "True");

    // 检测到位信号是否触发
    if (enable_action_check_pallet_signal_ && (!checkPalletInPlaceSignal())) {
        LOG(ERROR) << "checkPalletInPlaceSignal: false";
        sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_LOAD_PALLET_BOUNCE);
        doActionFinishFailed(sros::core::ERROR_CODE_ACTION_LOAD_PALLET_BOUNCE);
        return;
    }

    // 1.通过目标站点的id去获取目标点的位姿
    auto dst_station_id = action_param0_;
    auto dst_station = MapManager::getInstance()->getStation(dst_station_id);

    if (dst_station_id != 0 && dst_station.id == 0) {
        LOG(ERROR) << "not exist station id: " << dst_station_id;
        doActionFinishFailed(sros::core::ERROR_CODE_NAV_FIND_PATH_NO_STATION_IN_MAP);
        return;
    }

    dst_pose_x_ = dst_station.pos.x / 100.0;
    dst_pose_y_ = dst_station.pos.y / 100.0;
    dst_pose_yaw_ = normalizeYaw(dst_station.pos.yaw);

    LOG(INFO) << "dst_pose, [x: " << dst_pose_x_ << "][y: " << dst_pose_y_ << "][yaw: " << dst_pose_yaw_ << "]";

    if (isForkControlTypeSrc()) {
        // 向SRC发送动作指令
        src_sdk->executeAction(action_no_, 24, 10, action_param1_);
        LOG(INFO) << "src_ac: no " << action_no_ << ", "
                  << "id 24, p0 10, p1 " << action_param1_;
    } else {
        if (!isEnableEac()) {
            doActionFinishFailed(ERROR_CODE_ACTION_EAC_DISABLED);
            return;
        }

        sendEacActionTask(action_no_, 207, 1, action_param1_);
        LOG(INFO) << "eac_ac: no " << action_no_ << ", "
                  << "id 207, p0 1, p1 " << action_param1_;
    }

    is_forklift_running_ = true;
}

void Action171::doCancel() {
    if (is_forklift_running_) {
        if (isForkControlTypeSrc()) {
            src_sdk->cancelAction(action_no_);
        } else {
            cancelEacActionTask(action_no_);
        }
    }
}

void Action171::onSrcAcFinishSucceed(int result_value) {
    if (!is_keep_recv_src_ret_) {
        return;
    }

     if (checkForkHeightFautl(action_param1_ / 1000.0)) {
        return;
    }

    is_forklift_running_ = false;
    is_keep_recv_src_ret_ = false;

    //先判断角度是否有旋转
    //防止支路中心的站点方向搞错
    auto &s = sros::core::Settings::getInstance();
    double deviate_max_yaw = s.getValue<double>("forklift.deviate_max_yaw", 10.0);
    deviate_max_yaw = deviate_max_yaw / 180 * M_PI;
    auto curr_pose = src_sdk->getCurPose();
    LOG(INFO) << "curr_pose, [x: " << curr_pose.x() << "][y: " << curr_pose.y() << "][yaw: " << curr_pose.yaw() << "]";

    double offset_yaw = std::fabs(normalizeYaw((dst_pose_yaw_ - curr_pose.yaw())));
    LOG(INFO) << "[offset_yaw: " << offset_yaw << "][deviate_max_yaw: " << deviate_max_yaw << "]";
    if (offset_yaw > deviate_max_yaw) {
        LOG(INFO) << "net station yaw deviate too large: offset_yaw(" << offset_yaw << ") > deviate_max_yaw("
                  << deviate_max_yaw << ")";

        //生成旋转路径
        sros::core::NavigationPath_vector dst_paths;
        dst_paths.push_back(makeRotate(dst_pose_yaw_));

        //调试
        debugOuputPaths(dst_paths);
        sendMoveTask(dst_paths);

    } else {
        handleUnloadDetect();
    }
}

void Action171::handleUnloadDetect() {
    //开启货叉下方雷达检测货物架是否有货
    auto& s = sros::core::Settings::getInstance();
    bool enable_unload_detect = (s.getValue<string>("main.enable_unload_detect", "True") == "True");
    if (enable_unload_detect) {
        //发起货物架检测
        detectGoodsRacks();
        is_detect_goods_racks_running_ = true;
    } else {
        doActionFinishSucceed();
    }
}

void Action171::onSrcAcFinishFailed(int result_value) {
    LOG(ERROR) << "action failed, result_value:"<<result_value;
    is_forklift_running_ = false;
    sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_FORK_FAIL_PICKUP_GOODS);
    doActionFinishFailed(sros::core::ERROR_CODE_ACTION_FORK_FAIL_PICKUP_GOODS);
}

void Action171::onSrcAcFinishCanceled(int result_value) {
    is_forklift_running_ = false;
    sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_FORK_FAIL_PICKUP_GOODS);
    doActionFinishFailed(sros::core::ERROR_CODE_ACTION_FORK_FAIL_PICKUP_GOODS);
}

void Action171::onSrcMcFinishSucceed(int result_value) {
    LOG(INFO) << "success rotate to dst yaw";
    handleUnloadDetect();
}

void Action171::onSrcMcFinishFailed(int result_value) {
    LOG(INFO) << "fail rotate to dst yaw";
    doActionFinishFailed(sros::core::ERROR_CODE_ACTION_UNLOAD_MOVE_NOT_REACH);
}

bool Action171::onSubSrcActionCheck(uint32_t& sub_src_action_no) {
    if (is_keep_recv_src_ret_) {
        return false;
    } else {
        sub_src_action_no = -1;
        return true;
    }
}

void Action171::detectGoodsRacks() {
    Pose pose;
    pose.x() = dst_pose_x_;
    pose.y() = dst_pose_y_;
    pose.yaw() = dst_pose_yaw_;

    auto& s = sros::core::Settings::getInstance();
    auto fork_goods_type = s.getValue<string>("perception.detect_goods_type", "CIRCLE");

    PerceptionCommandMsg::Command cmd;
    cmd.detect_stage = PerceptionCommandMsg::DetectStage::DETECT_STAGE_UNLOAD;
    if (fork_goods_type == "CARD") {
        cmd.object_type = PerceptionCommandMsg::ObjectType::OBJECT_TYPE_CARD;
    } else if (fork_goods_type == "CIRCLE") {
        cmd.object_type = PerceptionCommandMsg::ObjectType::OBJECT_TYPE_CIRCLE;
    } else {
        cmd.object_type = PerceptionCommandMsg::ObjectType::OBJECT_TYPE_HANDCART;
    }

    sendPerceptionCmd(action_no_, cmd, g_state.cur_goal_id, pose);
}

void Action171::onAlgoResultCallback(const sros::core::base_msg_ptr& msg) {
    is_detect_goods_racks_running_ = false;

    auto mm = dynamic_pointer_cast<PerceptionStateMsg>(msg);

    int detect_result = mm->detect_result;

    LOG(INFO) << "unload detect_result :" << detect_result;

    bool result = (detect_result == PerceptionStateMsg::DetectResult::DETECT_RESULT_SUCCESS);

    //上传卸货检测的调试区域
    sendCommonPosesInfo(mm->put_detect_region, (result ? STR_UNLOAD_LASER_NONE : STR_UNLOAD_LASER_HAVE));

    if (!result) {
        // sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_GOODS_DETECTED);
        doActionFinishFailed(sros::core::ERROR_CODE_ACTION_GOODS_DETECTED);
        return;
    }

    doActionFinishSucceed();
}

}  // namespace ac