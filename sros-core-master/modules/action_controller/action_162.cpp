//
// Created by caoyan on 4/2/21.
//

#include "action_162.h"
#include "core/logger.h"
#include "core/msg_bus.h"
#include "core/state.h"
#include "path_utils.h"
#include "rfid_manager.h"
#include "core/msg/command_msg.hpp"
#include "core/task/task_manager.h"

using namespace std;
using namespace sros::core;

namespace ac {

void Action162::doStart() {

    goal_id_ = -1;
    is_paths_monitor_running_ = false;
    // g_state.is_fork_pose_adjust = false;
    g_state.dst_pose_yaw = 0.0;
    offset_x_real_sub_theory_ = 0;

    //清空货物的识别坐标
    Pose pose;
    g_state.goods_pose = pose;

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
    bool enable_rfid = (s.getValue<string>("device.enable_rfid", "False") == "True");
    get_rfid_data_running_ = (!action_param_str_.empty() && enable_rfid);

    // 取放货避障使能
    enable_fetch_release_goods_avd = (s.getValue<std::string>("obstacle.enable_fetch_release_goods_avd", "false") == "True");
    
    // 载货模型变更使能
    enable_change_avd_model = (s.getValue<std::string>("obstacle.enable_change_avd_model", "false") == "True");

    //错误发生后人为重试
    if(checkPalletInPlaceSignal()) {
        LOG(ERROR) << "栈板到位信号已触发，货物已装载";
        retryJudeg();
        return;
    }

    g_state.photoelectric_switch_on_action = false;  //特殊定制，用于只有162才开启光电的场景

    bool enable_load_detect = (s.getValue<string>("main.enable_load_detect", "True") == "True");
    if(enable_load_detect) {
        // 2.取货目标点位姿检测
        detectDst3DPose();
    } else {
        //发起直线路径
        dst_3d_pose_ = dst_pose_;
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

void Action162::retryJudeg() {

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

void Action162::doCancel() {
    if(is_paths_monitor_running_) {
        src_sdk->cancelAction(action_no_);
    }
    //取消时恢复
    enableBackLidar(true);
    if(enable_fetch_release_goods_avd){
        sendAvoidObstacleCmd(AvdObaCommandMsg::AvoidObstacleFunctionType::FETCH_RELEASE_AVD, 
                             AvdObaCommandMsg::DetectAndAvdObaState::COMMAND_INVALID);
    }

    get_rfid_data_running_ = false;
    g_state.photoelectric_switch_on_action = false;
}

void Action162::onSrcAcFinishSucceed(int result_value) {

    enableBackLidar(true);
    g_state.photoelectric_switch_on_action = false;
    // 检测到位信号是否触发，判断进叉是否到位
    if(!checkPalletInPlaceSignal()) {
        //防止状态未及时更新到retry
        boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
        if(!checkPalletInPlaceSignal()) {
            LOG(ERROR) << "checkPalletInPlaceSignal: false";
            sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_LOAD_PALLET_BOUNCE);
            doActionFinishFailed(sros::core::ERROR_CODE_ACTION_LOAD_PALLET_BOUNCE);
            // enableBackLidar(true);
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

    // 对接成功后修改避障模型
    // if(enable_change_avd_model){
    //     sendAvoidObstacleCmd(AvdObaCommandMsg::AvoidObstacleFunctionType::CHANGE_MDOEL_AVD, goal_id_); 
    // }
    
}

void Action162::onSrcAcFinishFailed(int result_value) {
    if(result_value == 2498){
        //到位开关未触发
        sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_LOAD_PALLET_BOUNCE);
        doActionFinishFailed(sros::core::ERROR_CODE_ACTION_LOAD_PALLET_BOUNCE);
    }else{
        //路径执行失败
        sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_LOAD_NO_DOCKING);
        doActionFinishFailed(sros::core::ERROR_CODE_ACTION_LOAD_NO_DOCKING);
        if(enable_fetch_release_goods_avd){
            sendAvoidObstacleCmd(AvdObaCommandMsg::AvoidObstacleFunctionType::FETCH_RELEASE_AVD, 
                                 AvdObaCommandMsg::DetectAndAvdObaState::COMMAND_INVALID);
        }
    }

    enableBackLidar(true);

    get_rfid_data_running_ = false;
    g_state.photoelectric_switch_on_action = false;
}

void Action162::onSrcAcFinishCanceled(int result_value) {
    LOG(ERROR) << "急停导致动作失败, result_value:"<<result_value;
    sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_LOAD_NO_DOCKING);
    doActionFinishFailed(sros::core::ERROR_CODE_ACTION_LOAD_NO_DOCKING);
    enableBackLidar(true);
    
    get_rfid_data_running_ = false;
    g_state.photoelectric_switch_on_action = false;

    auto isMovementRunningFun = []() {
        auto cur_task = sros::core::TaskManager::getInstance()->getMovementTask();
        return cur_task && cur_task->isRunning();
    };
    if(isMovementRunningFun()){
        auto mm = make_shared<sros::core::CommandMsg>("DEBUG_CMD");
        mm->command = sros::core::CommandType::CMD_CANCEL_MOVEMENT_TASK;
        // mm->user_name = "manual_btn";
        sros::core::MsgBus::sendMsg(mm);
    }
}


void Action162::onSrcMcFinishSucceed(int result_value) {
    //
}

void Action162::onSrcMcFinishFailed(int result_value) {
    //
}

void Action162::detectDst3DPose() {

    auto& s = sros::core::Settings::getInstance();
    auto fork_goods_type = s.getValue<string>("perception.detect_goods_type", "CIRCLE");

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
    cmd.detection_state = PerceptionCommandMsg::DetectionState::DETECTION_ON;

    sendPerceptionCmd(action_no_, cmd, -1, dst_pose_);

}

void Action162::sendAvoidObstacleCmd(AvdObaCommandMsg::AvoidObstacleFunctionType avdoba_type,
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

void Action162::sendAvoidObstacleCmd(AvdObaCommandMsg::AvoidObstacleFunctionType avdoba_type,
                                     AvdObaCommandMsg::DetectAndAvdObaState state) {
    AvdObaCommandMsg::Command command;
    command.avdoba_type = avdoba_type;
    command.avdoba_state = state;
    command.is_enable = false;
    
    sendAvdObaCmd(action_no_, command);
}

void Action162::sendAvoidObstacleCmd(AvdObaCommandMsg::AvoidObstacleFunctionType avdoba_type, const int& id) {
    AvdObaCommandMsg::Command command;
    command.oba_model.is_change_oba_model = true;
    command.oba_model.restore_oba_model = false;
    command.oba_model.forklif_type = AvdObaCommandMsg::FORKLIFT_TYPE::FORKLIFT_1400;
    command.oba_model.pallet_id = id;
    command.avdoba_type = avdoba_type;

    sendAvdObaCmd(action_no_, command);
}


void Action162::onAlgoResultCallback(const sros::core::base_msg_ptr& msg) {

    auto mm = dynamic_pointer_cast<PerceptionStateMsg>(msg);
    dst_3d_pose_.x() = mm->goal_in_global_pose.x();
    dst_3d_pose_.y() = mm->goal_in_global_pose.y();
    dst_3d_pose_.yaw() = mm->goal_in_global_pose.yaw();

    dst_3d_agv_pose_.x() = mm->goal_in_agv_pose.x();
    dst_3d_agv_pose_.y() = mm->goal_in_agv_pose.y();
    dst_3d_agv_pose_.yaw() = mm->goal_in_agv_pose.yaw();

    int detect_result = mm->detect_result;
    goal_id_ = mm->goal_id;

    LOG(INFO) << "vision [dst_3d__global_pose x: " << dst_3d_pose_.x() << " y: " << dst_3d_pose_.y() << " yaw: " << dst_3d_pose_.yaw()
              << "] [goal_in_agv_pose x: " << dst_3d_agv_pose_.x() << " y: " << dst_3d_agv_pose_.y() << " yaw: " << dst_3d_agv_pose_.yaw()
              << "] [detect_result: " << detect_result << " goal_id_: " << goal_id_ << "]";

    bool result = (detect_result ==  PerceptionStateMsg::DetectResult::DETECT_RESULT_SUCCESS);
    //拿到结果之后
    if (!result) {
        //未检测到货物
        doActionFinishFailed(sros::core::ERROR_CODE_ACTION_GOODS_NOT_DETECTED);

        get_rfid_data_running_ = false;
        g_state.photoelectric_switch_on_action = false;
        return;
    }

    g_state.goods_pose = mm->goal_in_agv_pose;
    
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

void Action162::checkDst3DPose(double offset_x, double offset_y, double offset_yaw, bool enough_adjust) {

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
        g_state.photoelectric_switch_on_action = false;
        return;
    }
    if (offset_y > deviate_max_y) {
        LOG(ERROR) << "offset_y(" << offset_y << ") > max_y(" << deviate_max_y << ")";
        //sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_GOODS_POS_Y_DEVIATE);
        doActionFinishFailed(sros::core::ERROR_CODE_ACTION_GOODS_POS_Y_DEVIATE);
        get_rfid_data_running_ = false;
        g_state.photoelectric_switch_on_action = false;
        return;
    }
    if (offset_x < deviate_distance_x_forward) {
        LOG(ERROR) << "offset_x(" << offset_x << ") < forward(" << deviate_distance_x_forward << ")";
        //sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_GOODS_POS_X_DEVIATE_FORWARD);
        doActionFinishFailed(sros::core::ERROR_CODE_ACTION_GOODS_POS_X_DEVIATE_FORWARD);
        get_rfid_data_running_ = false;
        g_state.photoelectric_switch_on_action = false;
        return;
    }
    if (offset_x > deviate_distance_x_back) {
        LOG(ERROR) << "offset_x(" << offset_x << ") > back(" << deviate_distance_x_back << ")";
        //sros::core::FaultCenter::getInstance()->addFault(sros::core::FAULT_CODE_GOODS_POS_X_DEVIATE_BACK);
        doActionFinishFailed(sros::core::ERROR_CODE_ACTION_GOODS_POS_X_DEVIATE_BACK);
        get_rfid_data_running_ = false;
        g_state.photoelectric_switch_on_action = false;
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
        get_rfid_data_running_ = false;
        g_state.photoelectric_switch_on_action = false;
        return;
    }

    launchMoveTask(is_one_line);

}

void Action162::launchMoveTask(bool is_one_line) {

    //关闭后侧避障雷达，打开R200滤波
    enableBackLidar(false);

    //生成移动对接路径
    sros::core::NavigationPath_vector dst_paths;

    if(is_one_line) {
        genLinePath(dst_paths);
    } else {
        g_state.is_fork_pose_adjust = true;
        genMovePath(dst_paths);
    }

    //速度限制
    auto& s = sros::core::Settings::getInstance();
    double limit_v = s.getValue<double>("forklift.load_max_linear_speed", -1);
    double tail_speed = s.getValue<double>("forklift.load_tail_speed", 0.02);
    if(limit_v > 0){    //-1时使用默认路网最大速度
        for (auto& path : dst_paths) {
            if(path.limit_v_ != tail_speed){
                path.limit_v_ = limit_v;
            }
        }
    }

    //调试
    debugOuputPaths(dst_paths);
    
    
    if(enable_fetch_release_goods_avd ){
        sendAvoidObstacleCmd(AvdObaCommandMsg::AvoidObstacleFunctionType::FETCH_RELEASE_AVD,
                         AvdObaCommandMsg::DetectAndAvdObaState::AVDOBA_STAGE_FETCH_ON, 
                         dst_pose_, dst_paths);
    }
    
    

    // 向SRC发送动作指令，上货路径监控动作
    src_sdk->executeAction(action_no_, 24, 30, 0);
    LOG(INFO) << "src_ac: no " << action_no_ << ", "
              << "id 24, p0 30, p1 0";

    is_paths_monitor_running_ = true;

    //向SRC发送移动对接路径
    sendMoveTask(dst_paths);

    //记录当前的货物id
    g_state.cur_goal_id = goal_id_;
}

void Action162::genLinePath(sros::core::NavigationPath_vector& dst_paths) {
    LOG(INFO) << "genLinePath";

    double goods_len = calcGoodsLength(goal_id_);
    double fork_end_coordinate = calcForkEndCoordinateLength();

    //计算最终移动点偏移
    double deviation_len = (goods_len / 2) - fork_end_coordinate;  //导航到货物前表面5cm位置

    //计算最终移动点位姿
    Pose fin_pose;

    auto& s = sros::core::Settings::getInstance();
    bool enable_load_detect = (s.getValue<string>("main.enable_load_detect", "True") == "True");
    if(enable_load_detect) {
        calcDstPose(dst_3d_pose_, fin_pose, -fork_end_coordinate);
    }else {
        calcDstPose(dst_pose_, fin_pose, deviation_len);
    }

    LOG(INFO) << "fin_pose, [x: " << fin_pose.x() << "][y: " << fin_pose.y() << "][yaw: " << fin_pose.yaw() << "]";

    double forward_slow_distance = s.getValue<double>("forklift.forward_slow_distance", 0.05);
    double back_slow_distance = s.getValue<double>("forklift.back_slow_distance", -0.05);
    double tail_speed = s.getValue<double>("forklift.load_tail_speed", 0.02);

    Pose fin_pose_forward;  
    calcDstPose(fin_pose, fin_pose_forward, forward_slow_distance);   //提前10公分降速
    LOG(INFO) << "fin_pose_forward, [x: " << fin_pose_forward.x() << "][y: " << fin_pose_forward.y() << "][yaw: " << fin_pose_forward.yaw() << "]";

    Pose fin_pose_back;
    calcDstPose(fin_pose, fin_pose_back, back_slow_distance);
    LOG(INFO) << "fin_pose_back, [x: " << fin_pose_back.x() << "][y: " << fin_pose_back.y() << "][yaw: " << fin_pose_back.yaw() << "]";

   auto curr_pose = src_sdk->getCurPose();
    LinePath p;
    p = makeLine(curr_pose.x(), curr_pose.y(), fin_pose_forward.x(), fin_pose_forward.y(), PATH_BACKWARD);
    dst_paths.push_back(p);

    if(forward_slow_distance != back_slow_distance){
        LinePath low_p;
        low_p = makeLine(fin_pose_forward.x(), fin_pose_forward.y(), fin_pose_back.x(), fin_pose_back.y(), PATH_BACKWARD);
        low_p.limit_v_ = tail_speed;
        dst_paths.push_back(low_p);
    }

    genRotateBetweenPaths(dst_paths);
    genStartRotatePath(dst_paths, curr_pose.yaw());
}


void Action162::genMovePath(sros::core::NavigationPath_vector& dst_paths) {
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

    auto& s = sros::core::Settings::getInstance();
    double forward_slow_distance = s.getValue<double>("forklift.forward_slow_distance", 0.05);
    double back_slow_distance = s.getValue<double>("forklift.back_slow_distance", -0.05);
    double tail_speed = s.getValue<double>("forklift.load_tail_speed", 0.02);

    Pose fin_pose;
    calcDstPose(dst_3d_pose_, fin_pose, deviation_len);
    LOG(INFO) << "fin_pose, [x: " << fin_pose.x() << "][y: " << fin_pose.y() << "][yaw: " << fin_pose.yaw() << "]";


    Pose fin_pose_forward;  
    calcDstPose(fin_pose, fin_pose_forward, forward_slow_distance);   //提前10公分降速
    LOG(INFO) << "fin_pose_forward, [x: " << fin_pose_forward.x() << "][y: " << fin_pose_forward.y() << "][yaw: " << fin_pose_forward.yaw() << "]";
    
    Pose fin_pose_back;
    calcDstPose(fin_pose, fin_pose_back, back_slow_distance);
    LOG(INFO) << "fin_pose_back, [x: " << fin_pose_back.x() << "][y: " << fin_pose_back.y() << "][yaw: " << fin_pose_back.yaw() << "]";

    //根据检测返回的目标点位姿生成路径
    auto curr_pose = src_sdk->getCurPose();
    LOG(INFO) << "curr_pose, [x: " << curr_pose.x() << "][y: " << curr_pose.y() << "][yaw: " << curr_pose.yaw() << "]";
    
    BezierPath bez = genBezierPath(curr_pose, mid_pose, PATH_BACKWARD);
    bez.updateFacing();
    dst_paths.push_back(bez);

    LinePath p;
    p = makeLine(mid_pose.x(), mid_pose.y(), fin_pose_forward.x(), fin_pose_forward.y(), PATH_BACKWARD);
    dst_paths.push_back(p);

    if(forward_slow_distance != back_slow_distance){
        LinePath low_p;
        low_p = makeLine(fin_pose_forward.x(), fin_pose_forward.y(), fin_pose_back.x(), fin_pose_back.y(), PATH_BACKWARD);
        low_p.limit_v_ = tail_speed;
        dst_paths.push_back(low_p);
    }
    genRotateBetweenPaths(dst_paths);
    genStartRotatePath(dst_paths, curr_pose.yaw());
}


}