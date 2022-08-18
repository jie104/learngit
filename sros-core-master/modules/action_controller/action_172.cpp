//
// Created by caoyan on 4/10/21.
//


#include "action_172.h"

#include <memory>
#include "core/logger.h"
#include "core/msg_bus.h"
#include "core/state.h"
#include "path_utils.h"


using namespace std;
using namespace sros::core;

namespace ac {

void Action172::doStart() {

    is_forklift_running_ = false;

    g_state.is_loading_check_pallet_signal_ = false;

    g_state.dst_pose_yaw = 0.0;

    if((!isForkControlTypeSrc()) && (!isEnableEac())) {
        doActionFinishFailed(ERROR_CODE_ACTION_EAC_DISABLED);
        return;
    }

    auto& s = sros::core::Settings::getInstance();
    enable_action_check_pallet_signal_ = (s.getValue<string>("forklift.enable_action_check_pallet_signal", "True") == "True");
        // 取放货避障使能
    enable_fetch_release_goods_avd = (s.getValue<std::string>("obstacle.enable_fetch_release_goods_avd", "false") == "True");

    // 检测到位信号是否触发
    if(enable_action_check_pallet_signal_ && (!checkPalletInPlaceSignal())) {
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
        enableBackLidar(true);
        return;
    }

    dst_pose_.x() = dst_station.pos.x / 100.0;
    dst_pose_.y() = dst_station.pos.y / 100.0;
    dst_pose_.yaw() = normalizeYaw(dst_station.pos.yaw);

    g_state.dst_pose_yaw = dst_pose_.yaw();

    LOG(INFO) << "dst_pose, [x: " << dst_pose_.x() << "][y: " << dst_pose_.y() << "][yaw: " << dst_pose_.yaw() << "]";

    //防止货物中心的站点方向搞错
    double deviate_max_yaw = s.getValue<double>("forklift.deviate_max_yaw", 10.0);
    auto curr_pose = src_sdk->getCurPose();
    
    double offset_yaw = std::fabs(normalizeYaw((dst_pose_.yaw() - curr_pose.yaw())));
    if(offset_yaw > deviate_max_yaw) {
        LOG(ERROR) << "goods station yaw deviate too large: " << offset_yaw;
        doActionFinishFailed(sros::core::ERROR_CODE_NAV_FIND_PATH_NO_STATION_IN_MAP);
        return;
    }
    g_state.photoelectric_switch_on_action = false;
    //关闭后侧避障雷达，打开R2000滤波
    enableBackLidar(true);

    //3.生成移动下货路径
    sros::core::NavigationPath_vector dst_paths;
    genLinePath(dst_paths);

    //速度限制
    double limit_v = s.getValue<double>("forklift.unload_max_linear_speed", -1);
    if(limit_v > 0){    //-1时使用默认路网最大速度
        for (auto& path : dst_paths) {
            path.limit_v_ = limit_v;
        }
    }

    //调试
    debugOuputPaths(dst_paths);
    if(enable_fetch_release_goods_avd) {
        sendAvoidObstacleCmd(AvdObaCommandMsg::AvoidObstacleFunctionType::FETCH_RELEASE_AVD,
                            AvdObaCommandMsg::DetectAndAvdObaState::AVDOBA_STAGE_RELEASE_ON, 
                            dst_pose_, dst_paths);
    }

    sendMoveTask(dst_paths);

}

void Action172::doCancel() {
    if(is_forklift_running_) {
        if(isForkControlTypeSrc()) {
            src_sdk->cancelAction(action_no_);
        } else {
            cancelEacActionTask(action_no_);
        }

        is_forklift_running_ = false;
    }
    enableBackLidar(true);
    g_state.photoelectric_switch_on_action = false;
}

void Action172::onSrcAcFinishSucceed(int result_value) {
    g_state.photoelectric_switch_on_action = false;

    if (checkForkHeightFautl(action_param1_ / 1000.0)) {
        return;
    }

    is_forklift_running_ = false;

    // 检测到位信号是否触发
    if(enable_action_check_pallet_signal_ && (!checkPalletInPlaceSignal())) {
        LOG(ERROR) << "checkPalletInPlaceSignal: false";
        sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_LOAD_PALLET_BOUNCE);
        doActionFinishFailed(sros::core::ERROR_CODE_ACTION_LOAD_PALLET_BOUNCE);
        return;
    }


    //恢复避障模型
    
    //sendAvoidObstacleCmd(AvdObaCommandMsg::AvoidObstacleFunctionType::CHANGE_MDOEL_AVD); 

    //成功返回
    doActionFinishSucceed();
}

void Action172::sendAvoidObstacleCmd(AvdObaCommandMsg::AvoidObstacleFunctionType avdoba_type) {
    AvdObaCommandMsg::Command command;
    command.avdoba_type = avdoba_type;
    command.oba_model.is_change_oba_model = true;
    command.oba_model.restore_oba_model = true;
    
    sendAvdObaCmd(action_no_, command);
}

void Action172::onSrcAcFinishFailed(int result_value) {
    is_forklift_running_ = false;
    g_state.photoelectric_switch_on_action = false;
    LOG(ERROR) << "卸货叉臂下降失败";
    sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_FORK_FAIL_PICKDOWN_GOODS);
    doActionFinishFailed(sros::core::ERROR_CODE_ACTION_FORK_FAIL_PICKDOWN_GOODS);
    
}

void Action172::onSrcAcFinishCanceled(int result_value) {
    is_forklift_running_ = false;
    g_state.photoelectric_switch_on_action = false;
    LOG(ERROR) << "卸货叉臂下降失败";
    sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_FORK_FAIL_PICKDOWN_GOODS);
    doActionFinishFailed(sros::core::ERROR_CODE_ACTION_FORK_FAIL_PICKDOWN_GOODS);
}

void Action172::onSrcMcFinishSucceed(int result_value) {

    if(isForkControlTypeSrc()) {
        // 向SRC发送动作指令，上货路径监控动作
        src_sdk->executeAction(action_no_, 24, 5, action_param1_);
        LOG(INFO) << "src_ac: no " << action_no_ << ", "
                  << "id 24, p0 5, p1 " << action_param1_;
    } else {
        sendEacActionTask(action_no_, 207, 1, action_param1_);
        LOG(INFO) << "eac_ac: no " << action_no_ << ", "
                  << "id 207, p0 1, p1 " << action_param1_;
    }

    is_forklift_running_ = true;
    enableBackLidar(true);

}

void Action172::onSrcMcFinishFailed(int result_value) {
    LOG(ERROR) << "卸货路径导航失败, result_value:" << result_value;
    sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_UNLOAD_MOVE_NOT_REACH);
    doActionFinishFailed(sros::core::ERROR_CODE_ACTION_UNLOAD_MOVE_NOT_REACH);
    enableBackLidar(true);
    g_state.photoelectric_switch_on_action = false;
}

void Action172::genLinePath(sros::core::NavigationPath_vector& dst_paths) {
    LOG(INFO) << "genLinePath";

    double goods_len = calcGoodsLength(g_state.cur_goal_id);
    double fork_end_coordinate = calcForkEndCoordinateLength();

    //计算最终移动点偏移
    double deviation_len = (goods_len / 2) - fork_end_coordinate;

    //计算最终移动点位姿
    Pose fin_pose;
    calcDstPose(dst_pose_, fin_pose, deviation_len);
    LOG(INFO) << "fin_pose, [x: " << fin_pose.x() << "][y: " << fin_pose.y() << "][yaw: " << fin_pose.yaw() << "]";

    auto curr_pose = src_sdk->getCurPose();
    LinePath p;
    p = makeLine(curr_pose.x(), curr_pose.y(), fin_pose.x(), fin_pose.y(), PATH_BACKWARD);
    dst_paths.push_back(p);
    genStartRotatePath(dst_paths, curr_pose.yaw());
    dst_paths.push_back(RotatePath(fin_pose.x(), fin_pose.y(), fin_pose.yaw()));
}

void Action172::sendAvoidObstacleCmd(AvdObaCommandMsg::AvoidObstacleFunctionType avdoba_type,
                                     AvdObaCommandMsg::DetectAndAvdObaState state, 
                                     Pose pose, NavigationPath_vector paths) {
    AvdObaCommandMsg::Command command;
    command.avdoba_type = avdoba_type;
    command.avdoba_state = state;
    //Eigen::Vector3f goal_position = Eigen::Vector3f(pose.x(), pose.y(), pose.yaw());
    command.goal_position = pose;
    command.paths = paths;
    command.is_enable = true;

    sendAvdObaCmd(action_no_, command);
}


}
