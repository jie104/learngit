//
// Created by caoyan on 4/10/21.
//
#include "action_172_redbull.h"

#include <memory>
#include "core/logger.h"
#include "core/msg_bus.h"
#include "core/state.h"
#include "path_utils.h"


using namespace std;
using namespace sros::core;

namespace ac {

void Action172RedBull::doStart() {

    state_ = NONE;

    // camera_height_ = 300;

    g_state.is_loading_check_pallet_signal_ = false;
    height_ = action_param1_;

    is_groud_unload_ = (height_ < 500);
     //记录当前的货物id
    goal_id_ = g_state.cur_goal_id;

    if((!isForkControlTypeSrc()) && (!isEnableEac())) {
        doActionFinishFailed(ERROR_CODE_ACTION_EAC_DISABLED);
        return;
    }

    auto& s = sros::core::Settings::getInstance();
    enable_action_check_pallet_signal_ = (s.getValue<string>("forklift.enable_action_check_pallet_signal", "True") == "True");

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
        return;
    }

    dst_pose_.x() = dst_station.pos.x / 100.0;
    dst_pose_.y() = dst_station.pos.y / 100.0;
    dst_pose_.yaw() = normalizeYaw(dst_station.pos.yaw);

    //防止货物中心的站点方向搞错
    double deviate_max_yaw = s.getValue<double>("forklift.deviate_max_yaw", 10.0);
    auto curr_pose = src_sdk->getCurPose();
    double offset_yaw = std::fabs(normalizeYaw((dst_pose_.yaw() - curr_pose.yaw())));
    if(offset_yaw > deviate_max_yaw) {
        LOG(ERROR) << "goods station yaw deviate too large: " << offset_yaw;
        doActionFinishFailed(sros::core::ERROR_CODE_NAV_FIND_PATH_NO_STATION_IN_MAP);
        return;
    }

    // g_state.dst_pose_yaw = dst_pose_.yaw();

    LOG(INFO) << "dst_pose, [x: " << dst_pose_.x() << "][y: " << dst_pose_.y() << "][yaw: " << dst_pose_.yaw() << "]";

    bool enable_unload_detect = (s.getValue<string>("main.enable_unload_detect", "True") == "True");
    if (enable_unload_detect) {
        sendUnloadDetect();
    } else {
        //发起直线路径
        launchMoveTask(true);
    }

}

void Action172RedBull::sendUnloadDetect() {

    PerceptionCommandMsg::Command cmd;
    cmd.detect_stage = PerceptionCommandMsg::DetectStage::DETECT_STAGE_UNLOAD;

    if(is_groud_unload_){
        cmd.object_type = PerceptionCommandMsg::ObjectType::OBJECT_TYPE_PUT_SPACE;  //地面
            // launchMoveTask(true);
            // return ;
    }else{
        cmd.object_type = PerceptionCommandMsg::ObjectType::OBJECT_TYPE_HEAPUP_CUBE;    //堆叠
    }
        
    sendPerceptionCmd(action_no_, cmd, -1, dst_pose_);
}

void Action172RedBull::onAlgoResultCallback(const sros::core::base_msg_ptr& msg) {
    

    auto mm = dynamic_pointer_cast<PerceptionStateMsg>(msg);
    int detect_result = mm->detect_result;
    LOG(INFO) << "unload detect_result :" << detect_result;

    bool result = (detect_result == PerceptionStateMsg::DetectResult::DETECT_RESULT_SUCCESS);

    //上传卸货检测的调试区域
    sendCommonPosesInfo(mm->put_detect_region, (result ? STR_UNLOAD_LASER_NONE : STR_UNLOAD_LASER_HAVE));

    if (result) {
        if(is_groud_unload_){//地面直接放
            launchMoveTask(true);   //直线进叉
            
        }else {
            dst_3d_pose_.x() = mm->goal_in_global_pose.x();
            dst_3d_pose_.y() = mm->goal_in_global_pose.y();
            dst_3d_pose_.yaw() = mm->goal_in_global_pose.yaw();
            detect_height_ = mm->goal_in_global_pose.z()*1000;

            dst_3d_agv_pose_.x() = mm->goal_in_agv_pose.x();
            dst_3d_agv_pose_.y() = mm->goal_in_agv_pose.y();
            dst_3d_agv_pose_.yaw() = mm->goal_in_agv_pose.yaw();
            goal_id_ = mm->goal_id;
            LOG(INFO) << "vision [dst_3d__global_pose x: " << dst_3d_pose_.x() << " y: " << dst_3d_pose_.y() << " yaw: " << dst_3d_pose_.yaw()
              << "] [goal_in_agv_pose x: " << dst_3d_agv_pose_.x() << " y: " << dst_3d_agv_pose_.y() << " yaw: " << dst_3d_agv_pose_.yaw()
              << "] [detect_result: " << detect_result << " goal_id_: " << goal_id_ << "]"
              << "] [detect_height: " << detect_height_ << "]";


            //高度校验
            int offset_height = std::fabs(detect_height_- height_);
            LOG(INFO) << "offset_height: " << offset_height;
            if (offset_height > 70) {
                LOG(ERROR) << "offset_height(" << offset_height << ") > 50";
                doActionFinishFailed(sros::core::ERROR_CODE_ACTION_GOODS_POS_Z_DEVIATE);
                return;
            }

            
            // 推算得出的视觉检测到位姿
            double goods_len = calcGoodsLength(goal_id_);
            Pose calc_dst_3d_pose;
            calcDstPose(dst_pose_, calc_dst_3d_pose, goods_len/2);
            LOG(INFO) << "calc_dst_3d_pose, [x: " << calc_dst_3d_pose.x()
                    << "][y: " << calc_dst_3d_pose.y() << "][yaw: " << calc_dst_3d_pose.yaw() << "]";
            

            //计算全局坐标系下X的误差
            auto distance_two_pose = [](const Pose &p1,const Pose &p2) {
                return sqrt((p1.x()- p2.x()) * (p1.x()- p2.x()) + (p1.y()- p2.y()) * (p1.y()- p2.y()));
            };

            auto curr_pose = src_sdk->getCurPose();
            double len_agv_to_real = distance_two_pose(dst_3d_pose_, curr_pose);
            double len_agv_to_theory = distance_two_pose(calc_dst_3d_pose, curr_pose);

            double offset_x_real_sub_theory_ = len_agv_to_real - len_agv_to_theory;


            //计算相机方向下Y和YAW的误差
            double offset_y = std::fabs(mm->goal_in_agv_pose.y());
            double offset_yaw = std::fabs(mm->goal_in_agv_pose.yaw());

            LOG(INFO) << "offset, [x: " << offset_x_real_sub_theory_ << "][y: " << offset_y << "][yaw: " << offset_yaw << "]";

            auto& s = sros::core::Settings::getInstance();
            double auto_adjust_need_distance = s.getValue<double>("forklift.auto_adjust_need_distance", 0.5);
            double bezier_adjust_distance = s.getValue<double>("forklift.bezier_adjust_distance", 0.3);
            //没算上叉臂，重写
            bool enough_adjust = (std::fabs(dst_3d_agv_pose_.x()) > (auto_adjust_need_distance + bezier_adjust_distance));

            //对检测后的位姿进行校验
            checkDst3DPose(offset_x_real_sub_theory_, offset_y, offset_yaw, enough_adjust);

                    
        }

    }else {
        // sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_GOODS_DETECTED);
        doActionFinishFailed(sros::core::ERROR_CODE_ACTION_GOODS_DETECTED);
        return;
    }

}

void Action172RedBull::doCancel() {
    if(state_ == MC_RUNING || state_ == AC_RUNING){
        src_sdk->cancelAction(action_no_);
    }
    state_ = CANCLING;
    enableBackLidar(true);
}

void Action172RedBull::onSrcAcFinishSucceed(int result_value) {
    
    LOG(INFO) << "fork_height: " << g_state.fork_height_encoder; 
    //成功返回
    doActionFinishSucceed();
}

void Action172RedBull::onSrcAcFinishFailed(int result_value) {
    state_ = FAILED;
    sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_FORK_FAIL_PICKDOWN_GOODS);
    doActionFinishFailed(sros::core::ERROR_CODE_ACTION_FORK_FAIL_PICKDOWN_GOODS);
    
}

void Action172RedBull::onSrcAcFinishCanceled(int result_value) {
    is_forklift_running_ = false;
    sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_FORK_FAIL_PICKDOWN_GOODS);
    doActionFinishFailed(sros::core::ERROR_CODE_ACTION_FORK_FAIL_PICKDOWN_GOODS);
}

void Action172RedBull::onSrcMcFinishSucceed(int result_value) {
    LOG(INFO) << "onSrcMcFinishSucceed";
    // 向SRC发送动作指令
    state_ = AC_RUNING;
    src_sdk->executeAction(action_no_, 24, 34, action_param1_);
    LOG(INFO) << "src_ac: no " << action_no_ << ", "
                << "id 24, p0 34, p1 " << action_param1_;

    enableBackLidar(true);

}

void Action172RedBull::onSrcMcFinishFailed(int result_value) {
    LOG(ERROR) << "卸货路径导航失败, result_value:" << result_value;
    sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_UNLOAD_MOVE_NOT_REACH);
    doActionFinishFailed(sros::core::ERROR_CODE_ACTION_UNLOAD_MOVE_NOT_REACH);
    enableBackLidar(true);
}

void Action172RedBull::checkDst3DPose(double offset_x, double offset_y, double offset_yaw, bool enough_adjust) {

    auto& s = sros::core::Settings::getInstance();

    double deviate_distance_x_forward = s.getValue<double>("forklift.deviate_distance_x_forward", -0.1);
    double deviate_distance_x_back = s.getValue<double>("forklift.deviate_distance_x_back", 0.1);

    double deviate_max_y = s.getValue<double>("forklift.deviate_max_y", 0.3);
    double deviate_max_yaw = s.getValue<double>("forklift.deviate_max_yaw", 10.0);
    deviate_max_yaw = deviate_max_yaw / 180 * M_PI;
    LOG(INFO) << "distance_x_forward: " << deviate_distance_x_forward
              << ", distance_x_back: "  <<  deviate_distance_x_back
              << ", max_y: " << deviate_max_y 
              << ", max_yaw: " << deviate_max_yaw;

    //判断是否超过最大误差
    if (offset_yaw > deviate_max_yaw) {
        LOG(ERROR) << "offset_yaw(" << offset_yaw <<  ") > max_yaw(" << deviate_max_yaw << ")";
        doActionFinishFailed(sros::core::ERROR_CODE_ACTION_GOODS_POS_YAW_DEVIATE);
        return;
    }
    if (offset_y > deviate_max_y) {
        LOG(ERROR) << "offset_y(" << offset_y << ") > max_y(" << deviate_max_y << ")";
        doActionFinishFailed(sros::core::ERROR_CODE_ACTION_GOODS_POS_Y_DEVIATE);
        return;
    }
    if (offset_x < deviate_distance_x_forward) {
        LOG(ERROR) << "offset_x(" << offset_x << ") < forward(" << deviate_distance_x_forward << ")";
        doActionFinishFailed(sros::core::ERROR_CODE_ACTION_GOODS_POS_X_DEVIATE_FORWARD);
        return;
    }
    if (offset_x > deviate_distance_x_back) {
        LOG(ERROR) << "offset_x(" << offset_x << ") > back(" << deviate_distance_x_back << ")";
        doActionFinishFailed(sros::core::ERROR_CODE_ACTION_GOODS_POS_X_DEVIATE_BACK);
        return;
    }

    double deviate_min_y = s.getValue<double>("forklift.deviate_min_y", 0.01);
    double deviate_min_yaw = s.getValue<double>("forklift.deviate_min_yaw", 1.0);
    deviate_min_yaw = deviate_min_yaw / 180 * M_PI;

    LOG(INFO) << "min_y: " << deviate_min_y << ", min_yaw: " << deviate_min_yaw;

    bool is_one_line = (offset_y < deviate_min_y && offset_yaw < deviate_min_yaw);

    //角度需要调整，但是调整距离不够直接报错
    if ( !is_one_line && !enough_adjust) {
        LOG(ERROR) << "需要自适应调整，但是调整距离不够";
        doActionFinishFailed(sros::core::ERROR_CODE_ACTION_FORK_ADJUST_LEN_NOT_ENOUGH);
        return;
    }

    launchMoveTask(is_one_line);

}

void Action172RedBull::launchMoveTask(bool is_one_line) {

    //关闭后侧避障雷达，打开R200滤波
    enableBackLidar(false);

    //生成移动对接路径
    sros::core::NavigationPath_vector dst_paths;

    if(is_one_line) {
        genLinePath(dst_paths);
    } else {
        genMovePath(dst_paths);
    }

    //调试
    debugOuputPaths(dst_paths);

    state_ = MC_RUNING;
    //向SRC发送移动对接路径
    sendMoveTask(dst_paths);

}

void Action172RedBull::genLinePath(sros::core::NavigationPath_vector& dst_paths) {
    LOG(INFO) << "genLinePath";

    double goods_len = calcGoodsLength(goal_id_);
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
}

void Action172RedBull::genMovePath(sros::core::NavigationPath_vector& dst_paths) {
    LOG(INFO) << "genMovePath";

    double adjust_pose_optimal_distance = calcAdjustPoseOptimalDistance();

    //计算中间点位姿（第一次调整的最佳位姿）
    Pose mid_pose;
    calcDstPose(dst_3d_pose_, mid_pose, adjust_pose_optimal_distance);
    LOG(INFO) << "mid_pose, [x: " << mid_pose.x() << "][y: " << mid_pose.y() << "][yaw: " << mid_pose.yaw() << "]";

    //计算最终移动点位姿
    double fork_end_coordinate = calcForkEndCoordinateLength();
    //计算最终移动点偏移
    double deviation_len = 0 - fork_end_coordinate;

    Pose fin_pose;
    calcDstPose(dst_3d_pose_, fin_pose, deviation_len);
    LOG(INFO) << "fin_pose, [x: " << fin_pose.x() << "][y: " << fin_pose.y() << "][yaw: " << fin_pose.yaw() << "]";
    
    //根据检测返回的目标点位姿生成路径
    auto curr_pose = src_sdk->getCurPose();
    LOG(INFO) << "curr_pose, [x: " << curr_pose.x() << "][y: " << curr_pose.y() << "][yaw: " << curr_pose.yaw() << "]";
    
    BezierPath bez = genBezierPath(curr_pose, mid_pose, PATH_BACKWARD);
    bez.updateFacing();
    dst_paths.push_back(bez);

    LinePath p;
    p = makeLine(mid_pose.x(), mid_pose.y(), fin_pose.x(), fin_pose.y(), PATH_BACKWARD);
    dst_paths.push_back(p);
    genRotateBetweenPaths(dst_paths);
    genStartRotatePath(dst_paths, curr_pose.yaw());
}

}
