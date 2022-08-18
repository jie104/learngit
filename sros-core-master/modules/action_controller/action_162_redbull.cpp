//
// Created by caoyan on 4/2/21.
//

#include "action_162_redbull.h"
#include "core/logger.h"
#include "core/msg_bus.h"
#include "core/state.h"
#include "path_utils.h"
#include "rfid_manager.h"


using namespace std;
using namespace sros::core;

namespace ac {

void Action162Redbull::doStart() {

    count_ = 0;
    goal_id_ = -1;
    // is_paths_monitor_running_ = false;
    // g_state.is_fork_pose_adjust = false;
    g_state.dst_pose_yaw = 0.0;
    src_action_no_ = CalcSrcActionNo();
    state_ = NONE;

    if((!isForkControlTypeSrc()) && (!isEnableEac())) {
        doActionFinishFailed(ERROR_CODE_ACTION_EAC_DISABLED);
        return;
    }

    // 1.通过目标站点的id去获取目标点的位姿
    auto dst_station_id = action_param0_;   //无id为0的站点，为0则报错
    auto dst_station = MapManager::getInstance()->getStation(dst_station_id);

    if (dst_station.id == 0) {
        LOG(ERROR) << "not exist station id: " << dst_station_id;
        doActionFinishFailed(sros::core::ERROR_CODE_NAV_FIND_PATH_NO_STATION_IN_MAP);
        return;
    }

    dst_pose_.x() = dst_station.pos.x / 100.0;
    dst_pose_.y() = dst_station.pos.y / 100.0;
    dst_pose_.yaw() = normalizeYaw(dst_station.pos.yaw);

    g_state.dst_pose_yaw = dst_pose_.yaw();

    LOG(INFO) << "dst_pose, [x: " << dst_pose_.x() << "][y: " << dst_pose_.y() << "][yaw: " << dst_pose_.yaw() << "]";

    auto& s = sros::core::Settings::getInstance();

//    //防止货物中心的站点方向搞错
//    double deviate_max_yaw = s.getValue<double>("forklift.deviate_max_yaw", 10.0);
//    deviate_max_yaw = deviate_max_yaw / 180 * M_PI;
//    auto curr_pose = src_sdk->getCurPose();
//
//    double offset_yaw = std::fabs(normalizeYaw((dst_pose_.yaw() - curr_pose.yaw())));
//    if(offset_yaw > deviate_max_yaw) {
//        LOG(ERROR) << "goods station yaw deviate too large: " << offset_yaw;
//        doActionFinishFailed(sros::core::ERROR_CODE_NAV_FIND_PATH_NO_STATION_IN_MAP);
//        return;
//    }

    //读取rfid
    bool enable_rfid = (s.getValue<string>("device.enable_rfid", "False") == "False");
    get_rfid_data_running_ = (!action_param_str_.empty() && enable_rfid);

    //错误发生后人为重试
    if(checkPalletInPlaceSignal()) {
        LOG(ERROR) << "栈板到位信号已触发，货物已装载";
        retryJudeg();
        return;
    }


    bool enable_load_detect = (s.getValue<string>("main.enable_load_detect", "True") == "True");
    if(enable_load_detect) {
        // 2.取货目标点位姿检测
        detectDst3DPose();
    } else {
        //发起直线路径
        launchMoveTask(true);
    }

    LOG(INFO) << "is_get_rfid_: " << get_rfid_data_running_
              << ", action_param_str_: " << action_param_str_;

    if (get_rfid_data_running_) {

        //Matrix配置增加“rfid请求epc模式“项（SS_MODE 同步读 ES_MODE 异步读）
        auto &s = sros::core::Settings::getInstance();
        std::string rfid_reqepc_mode = s.getValue<std::string>("device.rfid_reqepc_mode", "SS_MODE");
        LOG(INFO) << "rfid_reqepc_mode:" << rfid_reqepc_mode;

        if (rfid_reqepc_mode == "ES_MODE") {

            // int rfid_reqepc_timeout = s.getValue<unsigned int>("device.rfid_reqepc_timeout", 2000);
            // LOG(INFO) << "rfid_reqepc_timeout:" << rfid_reqepc_timeout;
            get_rfid_data_ = "";
            RfidManager::getInstance()->asyncGetRfidCmd();

        } else if (rfid_reqepc_mode == "SS_MODE") {
            auto getRFIDDataFun = [&]() {
                // 100毫秒尝试一次
                get_rfid_data_ = "";
                while (get_rfid_data_running_) {
                    auto data = RfidManager::getInstance()->syncGetRfid();
                    if (data.empty()) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    } else {
                        LOG(INFO) << "getRFIDDataFun: " << get_rfid_data_ << ", exit thread.";
                        get_rfid_data_ = data;
                        return;
                    }
                }

                LOG(ERROR) << "getRFIDDataFun fail, exit thread";
            };

            std::thread thread1(getRFIDDataFun);
            thread1.detach();
        }    
        
    }

}

void Action162Redbull::retryJudeg() {

    if(!get_rfid_data_running_) {
        //成功返回
        doActionFinishSucceed();
        return;
    }

    //Matrix配置增加“rfid请求epc模式“项（SS_MODE 同步读 ES_MODE 异步读）
    auto &s = sros::core::Settings::getInstance();
    std::string rfid_reqepc_mode = s.getValue<std::string>("device.rfid_reqepc_mode", "SS_MODE");

    if (rfid_reqepc_mode == "ES_MODE") {

        int rfid_reqepc_timeout = s.getValue<unsigned int>("device.rfid_reqepc_timeout", 2000);
        LOG(INFO) << "rfid_reqepc_timeout:" << rfid_reqepc_timeout;

        RfidManager::getInstance()->asyncGetRfidCmd();

        std::this_thread::sleep_for(std::chrono::milliseconds(rfid_reqepc_timeout));

        auto data = RfidManager::getInstance()->asyncGetRfidData();

        if (data.empty()) {
            LOGGER(ERROR, ACTION_TASK) << "RFID get none!";
            doActionFinishFailed(sros::core::ERROR_CODE_RFID_GET_NONE);
            return;
        }

        //下发匹配串进行比较,同时兼容
        if(!action_param_str_.empty()) {
            if(data != action_param_str_) {
                LOGGER(ERROR, ACTION_TASK) << "match fail, rfid(" << data << ") != param_str(" << action_param_str_ << ")";
                action_task_->setResultValueStr(data);
                doActionFinishFailed(sros::core::ERROR_CODE_RESULT_NOT_MATCH_TASK_PARAM_STR);
                return;
            }    
        }

        action_task_->setResultValueStr(data);
        doActionFinishSucceed();      

    } else if(rfid_reqepc_mode == "SS_MODE") {
        // 100毫秒尝试一次尝试 10次
        for (int i = 0; i < 10; ++i) {

            auto data = RfidManager::getInstance()->syncGetRfid();
            if (data.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            } else {
                //下发匹配串进行比较
                if(data != action_param_str_) {
                    LOGGER(ERROR, ACTION_TASK) << "match fail, rfid(" << data << ") != param_str(" << action_param_str_ << ")";
                    action_task_->setResultValueStr(data);
                    doActionFinishFailed(sros::core::ERROR_CODE_RESULT_NOT_MATCH_TASK_PARAM_STR);
                } else {
                    action_task_->setResultValueStr(data);
                    doActionFinishSucceed();
                }
                return;
            }
        }//

        //没有读到rfid
        LOGGER(ERROR, ACTION_TASK) << "RFID get none!";
        doActionFinishFailed(sros::core::ERROR_CODE_RFID_GET_NONE);
    }

}

void Action162Redbull::doCancel() {
    if(state_ == AC_RUNING || state_ == MC_RUNING) {
        src_sdk->cancelAction(src_action_no_);
    }
    //取消时恢复
    enableBackLidar(true);

    get_rfid_data_running_ = false;
}

void Action162Redbull::onSrcAcFinishSucceed(int result_value) {

    LOG(INFO) << "onSrcAcFinishSucceed,state_:"<<state_;
    src_action_no_ = CalcSrcActionNo();
    if(state_ == AC_RUNING){
        state_ = MC_RUNING;
        launchMoveTask(is_one_line_);
        return ;
    }

    // 检测到位信号是否触发，判断进叉是否到位
    if(!checkPalletInPlaceSignal()) {
        //防止状态未及时更新到retry
        boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
        if(!checkPalletInPlaceSignal()) {
            LOG(ERROR) << "checkPalletInPlaceSignal: false";
            sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_LOAD_PALLET_BOUNCE);
            doActionFinishFailed(sros::core::ERROR_CODE_ACTION_LOAD_PALLET_BOUNCE);
            enableBackLidar(true);
            get_rfid_data_running_ = false;
            return;
        }
    }

    //rfid比较
    if (get_rfid_data_running_) {
        get_rfid_data_running_ = false;

        //Matrix配置增加“rfid请求epc模式“项（SS_MODE 同步读 ES_MODE 异步读）
        auto &s = sros::core::Settings::getInstance();
        std::string rfid_reqepc_mode = s.getValue<std::string>("device.rfid_reqepc_mode", "SS_MODE");

        if (rfid_reqepc_mode == "ES_MODE") {
            get_rfid_data_ = RfidManager::getInstance()->asyncGetRfidData();
        }

        if(get_rfid_data_.empty()) {
            doActionFinishFailed(sros::core::ERROR_CODE_RFID_GET_NONE);
            return;
        } else if (action_param_str_ != get_rfid_data_) {
            LOGGER(ERROR, ACTION_TASK) << "match fail, rfid(" << get_rfid_data_ << ") != param_str(" << action_param_str_ << ")";
            action_task_->setResultValueStr(get_rfid_data_);
            doActionFinishFailed(sros::core::ERROR_CODE_RESULT_NOT_MATCH_TASK_PARAM_STR);
            return;
        }
    }

    //成功返回
    doActionFinishSucceed();
}

void Action162Redbull::onSrcAcFinishFailed(int result_value) {
    if(state_ == AC_RUNING){   
        
        sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_FORK_LIFT_FAULT);
        doActionFinishFailed(sros::core::ERROR_CODE_ACTION_FORK_LIFT_NOT_REACH);
    }else if (state_ == MC_RUNING) {
        if(result_value == 2498){
            sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_LOAD_PALLET_BOUNCE);
            doActionFinishFailed(sros::core::ERROR_CODE_ACTION_LOAD_PALLET_BOUNCE);
        }else{
            sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_LOAD_NO_DOCKING);
            doActionFinishFailed(sros::core::ERROR_CODE_ACTION_LOAD_NO_DOCKING);
        }
        enableBackLidar(true);
    }
    LOG(ERROR) << "onSrcAcFinishFailed, result_value:"<<result_value;

    get_rfid_data_running_ = false;
}

void Action162Redbull::onSrcAcFinishCanceled(int result_value) {
    if(state_ == AC_RUNING){   
        
        sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_FORK_LIFT_FAULT);
        doActionFinishFailed(sros::core::ERROR_CODE_ACTION_FORK_LIFT_NOT_REACH);
    }else if (state_ == MC_RUNING) {
        if(result_value == 2498){
            sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_LOAD_PALLET_BOUNCE);
            doActionFinishFailed(sros::core::ERROR_CODE_ACTION_LOAD_PALLET_BOUNCE);
        }else{
            sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_LOAD_NO_DOCKING);
            doActionFinishFailed(sros::core::ERROR_CODE_ACTION_LOAD_NO_DOCKING);
        }
        enableBackLidar(true);
    }
    LOG(ERROR) << "onSrcAcFinishFailed, result_value:"<<result_value;

    get_rfid_data_running_ = false;
}

void Action162Redbull::onSrcMcFinishSucceed(int result_value) {
    //
}

void Action162Redbull::onSrcMcFinishFailed(int result_value) {
    //
}

void Action162Redbull::detectDst3DPose() {

    // auto& s = sros::core::Settings::getInstance();
    // auto fork_goods_type = s.getValue<string>("perception.detect_goods_type", "CIRCLE");

    // PerceptionCommandMsg::Command cmd;
    // cmd.detect_stage = PerceptionCommandMsg::DetectStage::DETECT_STAGE_LOAD;
    // if(fork_goods_type == "CARD") {
    //     //卡板检测
    //     cmd.object_type = PerceptionCommandMsg::ObjectType::OBJECT_TYPE_CARD;
    // } else if(fork_goods_type == "CIRCLE"){
    //     //圆盘检测
    //     cmd.object_type = PerceptionCommandMsg::ObjectType::OBJECT_TYPE_CIRCLE;
    // } else {
    //     //手推车
    //     cmd.object_type = PerceptionCommandMsg::ObjectType::OBJECT_TYPE_HANDCART;
    // }

    // sendPerceptionCmd(action_no_, cmd, -1, dst_pose_);



    auto& s = sros::core::Settings::getInstance();
    auto fork_goods_type = s.getValue<string>("perception.detect_goods_type", "CIRCLE");

    float livox_install_fork_z_offsets = s.getValue<>("perception.livox_install_fork_z_offset", -195); //unit mm
    float current_fork_height = g_state.fork_height_encoder*1000.0f; // unit mm
    float current_camera_height = current_fork_height + livox_install_fork_z_offsets; // unit mm

    PerceptionCommandMsg::Command cmd;
    cmd.detect_stage = PerceptionCommandMsg::DetectStage::DETECT_STAGE_LOAD;
    if(fork_goods_type == "CARD") {
        //卡板检测
        cmd.object_type = PerceptionCommandMsg::ObjectType::OBJECT_TYPE_CARD;
    } else if(fork_goods_type == "CIRCLE"){
        //圆盘检测
        cmd.object_type = PerceptionCommandMsg::ObjectType::OBJECT_TYPE_CIRCLE;
    } else {
        //手推车
        cmd.object_type = PerceptionCommandMsg::ObjectType::OBJECT_TYPE_HANDCART;
    }
    Pose dst_pose_;
    // sendPerceptionCmd(action_no_, cmd, -1, dst_pose_);
    sendPerceptionCmd(action_no_, cmd, -1, dst_pose_, 150, action_param1_-20, current_camera_height);
}

void Action162Redbull::onAlgoResultCallback(const sros::core::base_msg_ptr& msg) {
    auto mm = dynamic_pointer_cast<PerceptionStateMsg>(msg);
    dst_3d_pose_.x() = mm->goal_in_global_pose.x();
    dst_3d_pose_.y() = mm->goal_in_global_pose.y();
    dst_3d_pose_.yaw() = mm->goal_in_global_pose.yaw();
    detect_height_ = mm->goal_in_global_pose.z()*1000;

    dst_3d_agv_pose_.x() = mm->goal_in_agv_pose.x();
    dst_3d_agv_pose_.y() = mm->goal_in_agv_pose.y();
    dst_3d_agv_pose_.yaw() = mm->goal_in_agv_pose.yaw();

    int detect_result = mm->detect_result;
    goal_id_ = mm->goal_id;

    LOG(INFO) << "vision [dst_3d__global_pose x: " << dst_3d_pose_.x() << " y: " << dst_3d_pose_.y() << " yaw: " << dst_3d_pose_.yaw()
              << "] [goal_in_agv_pose x: " << dst_3d_agv_pose_.x() << " y: " << dst_3d_agv_pose_.y() << " yaw: " << dst_3d_agv_pose_.yaw()
              << "] [detect_result: " << detect_result << " [detect_height_: " << detect_height_ << "]"<< " [goal_id_: " << goal_id_ << "]";

    bool result = (detect_result == PerceptionStateMsg::DetectResult::DETECT_RESULT_SUCCESS);
    //拿到结果之后
    if (!result) {
        //未检测到货物
        //sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_GOODS_NOT_DETECTED);
        doActionFinishFailed(sros::core::ERROR_CODE_ACTION_GOODS_NOT_DETECTED);

        get_rfid_data_running_ = false;
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

    offset_x_real_sub_theory_ = len_agv_to_real - len_agv_to_theory;


    //计算相机方向下Y和YAW的误差
    double offset_y = std::fabs(mm->goal_in_agv_pose.y());
    double offset_yaw = std::fabs(mm->goal_in_agv_pose.yaw());

    LOG(INFO) << "offset, [x: " << offset_x_real_sub_theory_ << "][y: " << offset_y << "][yaw: " << offset_yaw << "]";

    auto& s = sros::core::Settings::getInstance();
    double auto_adjust_need_distance = s.getValue<double>("forklift.auto_adjust_need_distance", 0.5);
    double bezier_adjust_distance = s.getValue<double>("forklift.bezier_adjust_distance", 0.3);
    bool enough_adjust = (std::fabs(dst_3d_agv_pose_.x()) > (auto_adjust_need_distance + bezier_adjust_distance));

    //对检测后的位姿进行校验
    checkDst3DPose(offset_x_real_sub_theory_, offset_y, offset_yaw, enough_adjust);
}

void Action162Redbull::checkDst3DPose(double offset_x, double offset_y, double offset_yaw, bool enough_adjust) {

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
        //sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_GOODS_POS_YAW_DEVIATE);
        doActionFinishFailed(sros::core::ERROR_CODE_ACTION_GOODS_POS_YAW_DEVIATE);
        get_rfid_data_running_ = false;
        return;
    }
    if (offset_y > deviate_max_y) {
        LOG(ERROR) << "offset_y(" << offset_y << ") > max_y(" << deviate_max_y << ")";
        //sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_GOODS_POS_Y_DEVIATE);
        doActionFinishFailed(sros::core::ERROR_CODE_ACTION_GOODS_POS_Y_DEVIATE);
        get_rfid_data_running_ = false;
        return;
    }
    if (offset_x < deviate_distance_x_forward) {
        LOG(ERROR) << "offset_x(" << offset_x << ") < forward(" << deviate_distance_x_forward << ")";
        //sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_GOODS_POS_X_DEVIATE_FORWARD);
        doActionFinishFailed(sros::core::ERROR_CODE_ACTION_GOODS_POS_X_DEVIATE_FORWARD);
        get_rfid_data_running_ = false;
        return;
    }
    if (offset_x > deviate_distance_x_back) {
        LOG(ERROR) << "offset_x(" << offset_x << ") > back(" << deviate_distance_x_back << ")";
        //sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_GOODS_POS_X_DEVIATE_BACK);
        doActionFinishFailed(sros::core::ERROR_CODE_ACTION_GOODS_POS_X_DEVIATE_BACK);
        get_rfid_data_running_ = false;
        return;
    }

    double deviate_min_y = s.getValue<double>("forklift.deviate_min_y", 0.01);
    double deviate_min_yaw = s.getValue<double>("forklift.deviate_min_yaw", 1.0);
    deviate_min_yaw = deviate_min_yaw / 180 * M_PI;

    LOG(INFO) << "min_y: " << deviate_min_y << ", min_yaw: " << deviate_min_yaw;

    is_one_line_ = (offset_y < deviate_min_y && offset_yaw < deviate_min_yaw);

    //角度需要调整，但是调整距离不够直接报错
    if ( !is_one_line_ && !enough_adjust) {
        LOG(ERROR) << "需要自适应调整，但是调整距离不够";
        //sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_GOODS_POS_X_DEVIATE_BACK);
        doActionFinishFailed(sros::core::ERROR_CODE_ACTION_FORK_ADJUST_LEN_NOT_ENOUGH);
        get_rfid_data_running_ = false;
        return;
    }

    //高度校验
    int offset_height = std::fabs((detect_height_ + 15)- action_param1_);
    LOG(INFO) << "offset_height: " << offset_height;
    if (offset_height > 70) {
        LOG(ERROR) << "offset_height(" << offset_height << ") > 70";
        doActionFinishFailed(sros::core::ERROR_CODE_ACTION_GOODS_POS_Z_DEVIATE);
        return;
    }

    int height = detect_height_+15 ;
    src_sdk->executeAction(src_action_no_, 24, 3,action_param1_ );
    LOG(INFO) << "src_ac: no " << src_action_no_ << ", "
                << "id 24, p0 3, p1 " << action_param1_;
    state_ = AC_RUNING;

    // launchMoveTask(is_one_line);

}

void Action162Redbull::launchMoveTask(bool is_one_line) {

    //关闭后侧避障雷达，打开R200滤波
    enableBackLidar(false);

    //生成移动对接路径
    sros::core::NavigationPath_vector dst_paths;

    if(is_one_line_) {
        genLinePath(dst_paths);
    } else {
        g_state.is_fork_pose_adjust = true;
        genMovePath(dst_paths);
    }

    //调试
    debugOuputPaths(dst_paths);

    // 向SRC发送动作指令，上货路径监控动作
    src_sdk->executeAction(src_action_no_, 24, 30, 0);
    LOG(INFO) << "src_ac: no " << src_action_no_ << ", "
              << "id 24, p0 30, p1 0";

   state_ = MC_RUNING;

    //向SRC发送移动对接路径
    sendMoveTask(dst_paths);

    //记录当前的货物id
    g_state.cur_goal_id = goal_id_;
}

void Action162Redbull::genLinePath(sros::core::NavigationPath_vector& dst_paths) {
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

void Action162Redbull::genMovePath(sros::core::NavigationPath_vector& dst_paths) {
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
    //p = makeLine(curr_pose.x(), curr_pose.y(), mid_pose.x(), mid_pose.y(), PATH_BACKWARD);
    //dst_paths.push_back(p);
    p = makeLine(mid_pose.x(), mid_pose.y(), fin_pose.x(), fin_pose.y(), PATH_BACKWARD);
    dst_paths.push_back(p);
    genRotateBetweenPaths(dst_paths);
    genStartRotatePath(dst_paths, curr_pose.yaw());
}

//src会一直上传上一个ac_no的结果，检查只处理当前src_action_no_的结果
bool Action162Redbull::onSubSrcActionCheck(uint32_t& sub_src_action_no) {
    sub_src_action_no = src_action_no_;
    return true;
}

uint32_t Action162Redbull::CalcSrcActionNo() {
    int no = action_no_ - count_;
    count_++;
    return no ;
}

}
