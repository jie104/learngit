/**
 * @file navigation_module.cpp
 *
 * @author lhx
 * @date 16-1-28.
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */
#include "navigation_module.h"

#include "avoid_oba/avdoba_processor.hpp"
#include "avoid_oba/manual_avdoba_processor.hpp"
#include "core/linear_algebra_func.h"
#include "core/mission/mission_manager.h"
#include "core/msg/ObstacleMsg.hpp"
#include "core/msg/PoseStampedMsg.h"
#include "core/msg/SlamCommandMsg.h"
#include "core/msg/command_msg.hpp"
#include "core/msg/common_msg.hpp"
#include "core/msg/common_command_msg.hpp"
#include "core/msg/common_state_msg.hpp"
#include "core/msg/parameter_msg.hpp"
#include "core/msg/path_msg.hpp"
#include "core/msg/str_msg.hpp"
#include "core/pose.h"
#include "core/rack/rack_operator_instance.hpp"
#include "core/rack_para.h"
#include "core/settings.h"
#include "core/src.h"
#include "core/task/task_manager.h"
#include "core/map/mark/FeatureMark.hpp"

#include "avoid_oba/avdoba_processor.hpp"
#include "core/util/utils.h"
#include "hybrid_astar/hybrid_a_star.h"
#include "hybrid_astar/visualization.h"
#include "navigation_on_net.h"
#include "core/tf/TFOperator.h"
#include "ChipVersion.h"

using namespace std;
using namespace sros::core;


int malloc_count = 0;

namespace nav {

NavigationModule::NavigationModule()
    : Module("Navigation"),
      enter_avoid_policy_(sros::core::OBSTACLE_AVOID_NONE),
      exit_avoid_policy_(sros::core::OBSTACLE_AVOID_NONE),
      obstacle_avoid_enable_(true),
      enable_path_oba_(true),
      enable_free_nav_(true),
      enable_net_nav_(true),
      station_check_state_(STATION_CHECK_NONE),
      pre_collision_(SEGMENT_FREE),
      nav_path_type_(BEZIER_TYPE) {
    local_real_time_slam_state_ = LocalRealTimeSlamState::REAL_TIME_SLAM_STATE_IDLE;
    navigate_.setNewCarSimulateMovePosesCallback(
        std::bind(&NavigationModule::newCarSimulateMovePoses, this, std::placeholders::_1));
}

NavigationModule::~NavigationModule() {
    //sr_local_Planner.deleteLocalMap();  // 释放局部地图的内存。
}

void NavigationModule::run() {
    boost::this_thread::sleep_for(boost::chrono::milliseconds(500));

    LOG(INFO) << "thread " << name_ << " is running";

    subscribeTopic("NAV_COMMAND", CALLBACK(&NavigationModule::onCommandMsg2));

    subscribeTopic("NAV_PATH_BUILD", CALLBACK(&NavigationModule::onGenerateNetPath));

    subscribeTopic("OBSTACLES", CALLBACK(&NavigationModule::onObaPoints));

    subscribeTopic("NAV_PARAMETER", CALLBACK(&NavigationModule::onParameterMsg));

    subscribeTopic("DEBUG_CMD", CALLBACK(&NavigationModule::onDebugCmdMsg));

    subscribeTopic("TIMER_50MS", CALLBACK(&NavigationModule::avoidObaLoop));

    subscribeTopic("TIMER_100MS", CALLBACK(&NavigationModule::onTimer_100ms));

    subscribeTopic("FEATURE_RECOG_RESULT", CALLBACK(&NavigationModule::onFeatureRecognitionMsg));

    subscribeTopic("RACK_INFO", CALLBACK(&NavigationModule::onRackMsg));

    subscribeTopic("GULF_GOODS_AVDOBA_CMD", CALLBACK(&NavigationModule::gulfAvdobaCmdMsg));

    subscribeTopic("RELEASE_MAP", CALLBACK(&NavigationModule::onReleaseMap));

    initParameter();
    resetAvoidObaPara();
    // 初始化完成后发送IDLE状态
    updateState(STATE_NAV_IDLE);
    nav_cfg_.updateNavParam();
    // 初始化局部路径规划。
    if (enable_free_nav_) {
        //sr_local_Planner.initLocalMap(400);  // 初始化局部地图
        //sr_local_Planner.tebLocalPlannerInit(nav_cfg_);
        planning_fsm_.planning_fsm_init();
    }

    boost::this_thread::sleep_for(boost::chrono::milliseconds(50));

    
    src_sdk->setVelocityCallback(boost::bind(&NavigationModule::onVelocityMsg, this, _1));
    src_sdk->setOptPoseCallback(boost::bind(&NavigationModule::onOptPoseMsg, this, _1));
    src_sdk->setCheckManualVelocityCallback(boost::bind(&NavigationModule::onCheckManualVelocity, this, _1,_2));

    // 建立一个线程，用于跟踪轨迹。
    boost::thread path_follow(boost::bind(&NavigationModule::pathFollowFunc,this));

    dispatch();

    // never return
}

void NavigationModule::onGenerateNetPath(base_msg_ptr m) {
    auto msg = dynamic_pointer_cast<CommonCommandMsg<NavigationCommand>>(m);
    Pose cur_pos = src_sdk->getCurPose();
    auto &settings = Settings::getInstance();
    auto mark_group = navigate_.getNavigationMap()->getStationMarkGroup();

    // 找到当前站点
    auto cur_station_no = g_state.station_no;

    auto dst_station_no = (StationNo_t)msg->param0;
    auto avoid_policy = (sros::core::ObstacleAvoidPolicy)msg->param1;
    auto nav_map_name = msg->str0;

    auto dst_pose = msg->pose;

    bool force_nav_on_map = msg->param_boolean;

   
    initNavigation(nav_map_name);

    LOG(INFO) << "Navigation 正在初始化 ";

    sros::core::Location_Vector empty_vector;  // 传入空的vector。

    Pose output_dst_pose;  // 导航任务的最终pose
    auto nav_paths = getNavigationPath(cur_station_no,
                                        dst_station_no,        // 目标站点编号
                                        src_sdk->getCurPose(),  // 当前位姿
                                        dst_pose,              // 目标位姿
                                        avoid_policy,          // 整体避障策略
                                        output_dst_pose,
                                        empty_vector,
                                        force_nav_on_map);

    if(nav_paths.size() == 0){
        LOG(WARNING)<<"GenerateNetPath is Empty!";
    }

    auto mm = make_shared<PathMsg>("NAV_PATH_BUILD_RESULT");
    NavigationPath_vector paths;
    mm->paths = nav_paths;
    sendMsg(mm);

}

void NavigationModule::onCommandMsg2(base_msg_ptr m) {
    failed_code_ = ERROR_CODE_NONE;

    std::string file_path;
    auto msg = dynamic_pointer_cast<CommonCommandMsg<NavigationCommand>>(m);
    switch (msg->command) {
        // LOG(WARNING) << "msg->command " << msg->command;
        case COMMAND_NAV_NAVIGATE_TO: {
            Pose cur_pos = src_sdk->getCurPose();
            auto &settings = Settings::getInstance();
            auto mark_group = navigate_.getNavigationMap()->getStationMarkGroup();

            // 找到当前站点
            auto cur_station_no = g_state.station_no;

            auto dst_station_no = (StationNo_t)msg->param0;
            auto avoid_policy = (sros::core::ObstacleAvoidPolicy)msg->param1;
            auto nav_map_name = msg->str0;

            auto dst_pose = msg->pose;
            bool force_nav_on_map = msg->param_boolean;

            if (avoid_policy != OBSTACLE_AVOID_WAIT && avoid_policy != OBSTACLE_AVOID_REPLAN &&
                avoid_policy != OBSTACLE_AVOID_NONE) {
                avoid_policy = OBSTACLE_AVOID_WAIT;
            }

            // 初始化,加载地图
            updateState(STATE_NAV_INITIALIZING);

            initNavigation(nav_map_name);

            if(g_state.load_state == sros::core::LOAD_FULL){
                nav_cfg_.robot.length = nav_cfg_.rack_length;
                nav_cfg_.robot.width = nav_cfg_.rack_width;
                LOG(INFO)<<"当前处于背货状态，避障尺寸为：长："<<nav_cfg_.robot.length<<";宽："<<nav_cfg_.robot.width;
            }
            else{
                nav_cfg_.robot.length = sros::core::Settings::getInstance().getValue<double>("nav.vehicle_length", 0.8);                          
                nav_cfg_.robot.width =  sros::core::Settings::getInstance().getValue<double>("nav.vehicle_width", 0.6);
                LOG(INFO)<<"当前处于空车状态，避障尺寸为：长："<<nav_cfg_.robot.length<<";宽："<<nav_cfg_.robot.width;
            }

            LOG(INFO) << "Navigation 正在初始化 ";

            updateState(STATE_NAV_PATH_FINDING);

            sros::core::Location_Vector empty_vector;  // 传入空的vector。

            Pose output_dst_pose;  // 导航任务的最终pose
            auto nav_paths = getNavigationPath(cur_station_no,
                                               dst_station_no,        // 目标站点编号
                                               src_sdk->getCurPose(),  // 当前位姿
                                               dst_pose,              // 目标位姿
                                               avoid_policy,          // 整体避障策略
                                               output_dst_pose,
                                               empty_vector,
                                               force_nav_on_map);
            
            // FIXME: 临时添加，实现通过chip-web下发任务后，路径direction属性自动修改为0x21，使用完可删除
            auto mission_manager = MissionManager::getInstance();
            auto next_step = mission_manager->getMissionNextStepInfo();
            if (next_step && next_step->getStepType() == MissionStepType::StepAction) {
                // 如果下一个步骤是执行旋转顶升动作，那么修改最后一条旋转路径的direction为0x21
                LOG(INFO) << "Mission: next_step: " << (int)next_step->getStepType() << "," << (int)nav_paths.size()
                          << "," << next_step->toString();
                StepActionPtr step_action = std::dynamic_pointer_cast<StepAction>(next_step);
                if (!nav_paths.empty() && step_action) {
                    if (step_action->getActionId() == 4 && step_action->getActionParam0() == 1 &&
                        step_action->getActionParam1() == 0) {
                        LOG(INFO) << "Mission: Next step is Action(4,1,0)";
                        auto last_path = nav_paths.back();

                        if (last_path.type_ == sros::core::PATH_ROTATE) {
                            LOG(INFO) << "Mission: last path is Rotate, turn last rotate path direction to 0x21";

                            last_path.direction_ = 0x21;

                            nav_paths.pop_back();
                            nav_paths.push_back(last_path);
                        }
                    }
                }
            }
            // FIXME: 以上代码可删除

            updateCurrPath(nav_paths);
            if(cur_paths_.size() ==1&&navigation_type_ == NAV_TYPE_MAP){
                if(cur_paths_[0].type_ == PATH_ROTATE){
                    navigation_type_ = NAV_TYPE_NET;
                    LOG(INFO)<<"开始规划路径时，只有一条原地旋转路径。";
                }
            }

            if (navigation_type_ == NAV_TYPE_MAP && !cur_paths_.empty()) {
                LOG(INFO) << "初次规划路径，考虑激光雷达检测到的障碍物。";
                Location_Vector oba_points;
                avoidoba::AvdobaPoseInfo obstacle_points;
                avoidoba_processor->getObstaclePoints(obstacle_points);
                updateObaPoints(oba_points, obstacle_points.oba_points);
                LOG(INFO)<<"用于初次规划的临时障碍点个数1："<<oba_points.size();
                std::vector<Eigen::Vector2d> vector_oba_points;
                avoidoba_processor->getObstaclePoints(vector_oba_points);
                LOG(INFO)<<"用于初次规划的临时障碍点个数2："<<vector_oba_points.size();

                cur_dst_pose_ = output_dst_pose;
                cur_paths_.clear();
                LOG(INFO) << "初次规划路径的起点坐标：x:" << cur_pos.x() << ";y:" << cur_pos.y()<<";yaw:"<< cur_pos.yaw();
                LOG(INFO) << "初次规划路径的终点坐标：x:" << cur_dst_pose_.x() << ";y:" << cur_dst_pose_.y()<<";yaw:"<<cur_dst_pose_.yaw();

                // 重新规划从起点到终点的全局路径，和初始规划不同的就是考虑了临时障碍点。
                cur_paths_ = navigate_.replanPath(cur_pos, cur_dst_pose_, NAV_PATH_TYPE::LINE_ROTATE_TYPE, oba_points);

                // 路网路径规划结束后，执行路径跟踪任务
                if(cur_paths_.size() == 1 && navigation_type_ == NAV_TYPE_MAP){
                    if(cur_paths_[0].type_ == PATH_ROTATE){
                        navigation_type_ = NAV_TYPE_NET;
                        LOG(INFO)<<"加入临时障碍点再规划时只有一条原地旋转路径。";

                        auto path_msg = make_shared<PathMsg>("NAV_PATH");
                        path_msg->paths = cur_paths_;
                        sendMsg(path_msg);

                        cur_start_station_no_ = cur_station_no;
                        cur_dst_station_no_ = dst_station_no;

                        cur_dst_pose_ = output_dst_pose;
                        cur_avoid_policy_ = avoid_policy;

                        g_state.station_no = 0;  // 将当前站点清零

                        updateState(STATE_NAV_WAITING_FOR_FINISH);
                        navigationModeInit();
                        if(navigation_type_ == NAV_TYPE_MAP){
                            startLocalPlanner();
                            planning_fsm_.resetNavState(target_vel_);

                            planning_fsm_.loadPaths(final_path_,opt_paths_);

                            planning_fsm_.traj_follow_init( cur_pos,opt_paths_);
                            planning_fsm_.changeNavState(NAV_STATE ::EXEC_TRAJ);
                            src_sdk->sendCommandMsg(COMMAND_SET_NAVALGORITHM, SRC_LOCAL_PLANNER, 0);  //设置为
                        }
                        else{
                            src_sdk->sendCommandMsg(COMMAND_SET_NAVALGORITHM, SRC_PATH_FOLLOW, 0);  //设置为
                        }
                        return;
                    }
                }

                if (cur_paths_.empty()) {
                    LOG(INFO) << "重新规划全局路径失败";
                    // 无论是否有路径都要发送，空路径用于传递无法到达状态
                    auto path_msg = make_shared<PathMsg>("NAV_PATH");
                    path_msg->paths = cur_paths_;
                    sendMsg(path_msg);
                    // 无路径直接结束
                    station_check_state_ = STATION_CHECK_NONE;
                    failed_code_ = ERROR_CODE_FREE_NAV_NO_WAY;
                    updateState(STATE_NAV_IDLE);
                    navigation_type_ = sros::core::NAV_TYPE_NONE;
                    navigate_.replanPath_del_obs();
                    return;
                }

                // 对全局路径的全面部分 采用hybrid进行规划。并合并后优化，最后完成路径。
                for (auto path : cur_paths_) {
                    LOG(INFO) << "初次规划路径类型：" << path.type_ << ";sx:" << path.sx_ << ";sy:" << path.sy_
                              << ";ex:" << path.ex_ << ";ey:" << path.ey_;
                }

                planning_fsm_.replan_free_path(navigate_, cur_paths_, cur_pos, global_map_,
                                                    oba_points,cur_dst_pose_, final_path_);

                COLLISION_OBS obs_pose;
                if(planning_fsm_.check_replan_path_collosion(navigate_,final_path_,global_map_,obs_pose)){
                    sendObs(obs_pose); // 将碰撞检测的障碍物信息发送给main模块。
                    cur_paths_.clear();
                    auto path_msg = make_shared<PathMsg>("NAV_PATH");
                    path_msg->paths = cur_paths_;
                    sendMsg(path_msg);
                    // 无路径直接结束
                    station_check_state_ = STATION_CHECK_NONE;
                    failed_code_ = ERROR_CODE_FREE_NAV_NO_WAY;
                    updateState(STATE_NAV_IDLE);
                    navigation_type_ = sros::core::NAV_TYPE_NONE;
                    navigate_.replanPath_del_obs();
                    return ;
                }
                navigate_.replanPath_del_obs();

            }

            // 无论是否有路径都要发送，空路径用于传递无法到达状态
            auto path_msg = make_shared<PathMsg>("NAV_PATH");
            path_msg->paths = cur_paths_;
            sendMsg(path_msg);

            if (cur_paths_.empty()) {
                // 无路径直接结束
                station_check_state_ = STATION_CHECK_NONE;
                updateState(STATE_NAV_IDLE);
                navigation_type_ = sros::core::NAV_TYPE_NONE;
            } else {
                // 更新当前导航任务信息
                cur_start_station_no_ = cur_station_no;
                cur_dst_station_no_ = dst_station_no;

                cur_dst_pose_ = output_dst_pose;
                cur_avoid_policy_ = avoid_policy;

                g_state.station_no = 0;  // 将当前站点清零


                updateState(STATE_NAV_WAITING_FOR_FINISH);
                navigationModeInit();
                if (navigation_type_ == NAV_TYPE_MAP) {

                    g_state.enable_station_feature_recognize = false;

                    startLocalPlanner();
                    planning_fsm_.resetNavState(target_vel_);

                    planning_fsm_.loadPaths(final_path_, opt_paths_);

                    planning_fsm_.traj_follow_init(cur_pos, opt_paths_);
                    planning_fsm_.changeNavState(NAV_STATE ::EXEC_TRAJ);
                    src_sdk->sendCommandMsg(COMMAND_SET_NAVALGORITHM, SRC_LOCAL_PLANNER, 0);  //设置为
                } else {
                    //判断当前目标站点是否需要特征识别
                    g_state.enable_station_feature_recognize = 
                        navigate_.getNavigationMap()->getFeatureMark(dst_station_no, g_state.feature_mark);
                    LOG(INFO) << "g_state.enable_station_feature_recognize:" << g_state.enable_station_feature_recognize;

                    src_sdk->sendCommandMsg(COMMAND_SET_NAVALGORITHM, SRC_PATH_FOLLOW, 0);  //设置为
                }
            }
            break;
        }
        case COMMAND_NAV_PAUSE: {
            LOG(INFO) << "COMMAND_NAV_PAUSE";
            if(navigation_type_ == NAV_TYPE_MAP){
                LOG(INFO)<<"1111111进入pause状态。";
                target_vel_.vx() = 0.0f;
                target_vel_.vtheta() = 0.0f;
                target_vel_.a()=0.0f;
                target_vel_.w_a() = 0;
                planning_fsm_.no_replan_pause = 1;
                planning_fsm_.changeNavState(NAV_STATE ::TRAJ_PAUSE);
            }
            break;
        }
        case COMMAND_NAV_CONTINUE:{

            LOG(INFO)<<"COMMAND_NAV_CONTINUE current,vx()"<<current_vel_.vx()<<";vtheta:"<<current_vel_.vtheta();
            if(navigation_type_ == NAV_TYPE_MAP) {
                target_vel_.vx() = current_vel_.vx();
                target_vel_.vtheta() = current_vel_.vtheta();
                target_vel_.a() = 0;
                target_vel_.w_a() = 0;

                planning_fsm_.changeNavState(NAV_STATE::EXEC_TRAJ);
                planning_fsm_.no_replan_pause = 0;
            }

            break;
        }
        case COMMAND_NAV_FINISH: {  // 到达导航目标点
            LOG(INFO) << "COMMAND_NAV_FINISH";

            finishLocalPlanner();
            planning_fsm_.resetNavState(target_vel_);

            Pose cur_pos = src_sdk->getCurPose();

            auto mark_group = navigate_.getNavigationMap()->getStationMarkGroup();
            if (cur_dst_station_no_ != 0) {
                auto dst_station = mark_group->getItem(cur_dst_station_no_);

                if (dst_station.id != 0 && dst_station.pos_dynamic && station_check_state_ == STATION_CHECK_WAITING) {
                    // 到达CheckPoint, 发送检测站点请求

                    LOG(INFO) << "到达CheckPoint, 发送检测站点请求";
                    auto mm = make_shared<CommonCommandMsg<NavigationCommand>>("FEATURE_RECOG_COMMAND");
                    mm->command = COMMAND_NAV_FIND_FEATURE;
                    // TODO(nobody): 根据目标站点位置计算feature位置
                    mm->pose =
                        Pose(Location(dst_station.pos.x / 100, dst_station.pos.y / 100), Rotation(dst_station.pos.yaw));
                    sendMsg(mm);

                    updateState(STATE_NAV_LOCATING_STATION);
                    break;
                }

                // 避免空路径下 更新站点信息
                if (cur_paths_.empty()) {
                    LOG(INFO) << "No path to destination!";
                    return;
                }
            }

            station_check_state_ = STATION_CHECK_NONE;
            updateState(STATE_NAV_IDLE);
            break;
        }
        case COMMAND_NAV_CONVERT_MAP: {
            LOG(INFO) << "COMMAND_NAV_CONVERT_MAP";
            updateState(STATE_NAV_CONVERTING_MAP);

            sros::map::NavigationMap convert_nav_map;
            convert_nav_map.loadGrayMap((msg->str0).c_str());  // str0为绘制生成的graymap文件路径
            convert_nav_map.saveFile((msg->str1).c_str());     // str1为navigationmap的保存路径

            updateState(STATE_NAV_IDLE);
            break;
        }
        case COMMAND_NAV_CANCEL: {
            planning_fsm_.resetNavState(target_vel_);
            navigation_type_ = sros::core::NAV_TYPE_NONE;
            local_navigation_state_ = sros::core::STATE_LOCAL_NAV_IDLE;
            LOG(INFO) << "COMMAND_NAV_CANCEL";
            station_check_state_ = STATION_CHECK_NONE;
            updateState(STATE_NAV_IDLE);
            break;
        }
        case COMMAND_NAV_MANUAL_PATH: {
            LOG(INFO) << "Nav收到手动路径" << msg->paths.size() << "条";

            // 初始化,加载地图
            updateState(STATE_NAV_INITIALIZING);
            initNavigation(msg->str0);

            updateCurrPath(msg->paths);
            // get destination pose
            sros::core::NavigationPath<double> pp;
            pp = cur_paths_.back();
            cur_dst_pose_.x() = pp.ex_;
            cur_dst_pose_.y() = pp.ey_;
            //            cur_dst_pose_.yaw() = pp.e_facing_;
            cur_dst_station_no_ = 0;

            // clear station_no when nav is begin
            g_state.station_no = 0;

            //判断当前目标站点是否需要特征识别
            g_state.enable_station_feature_recognize = 
                navigate_.getNavigationMap()->getFeatureMark(sros::core::StationNo_t(msg->param0), g_state.feature_mark);
            LOG(INFO) << "g_state.enable_station_feature_recognize:" << g_state.enable_station_feature_recognize;

            updateState(STATE_NAV_MANUAL_WAITING_FOR_START);

            //设置局部路径规划的参数。

            initNavigation(g_state.getCurMapName());
            navigation_type_ = sros::core::NAV_TYPE_NET;
            startLocalPlanner();
            break;
        }
        case COMMAND_NAV_SINGLE_APTH_REPLACE: {
            const auto &new_path = msg->paths;  // 新的路径
            updateCurrPath(new_path);
            break;
        }
        default: {
            LOG(INFO) << "ERROR MSG";
            break;
        }
    }
}

void NavigationModule::onTimer_100ms(sros::core::base_msg_ptr msg) {
    checkRackDetectLoop(msg);

    checkFeatureRecognizeLoop(msg);
    
    static uint8_t replan_time_couter=0;

    replan_time_couter++;
    if(replan_time_couter > nav_cfg_.replan_wait_time_s*10+5){
        replan_time_couter = 0;
        free_nav_replan();
    }
}

void NavigationModule::pathFollowFunc(){
    while(1){
        int64_t time_first2 = sros::core::util::get_time_in_ms();

        //LOG(INFO)<<"1111111111";
        Pose cur_pos = src_sdk->getCurPose();

        if (update_path_flag_) {
            LOG(INFO) << "此时正在更新局部路径！！！";
        } else {
            if (navigation_type_ == NAV_TYPE_MAP) {
                // 全局速度级别。g_state.speed_level;
                // LOG(INFO)<< "~~~~~~~~:" << (uint16_t)g_state.speed_level;
                planning_fsm_.navigateFSM(cur_pos, current_vel_, final_path_, cur_dst_pose_, opt_paths_, target_vel_);

                src_sdk->sendVelocityBack(target_vel_);
                final_target_vel_ = target_vel_;
            }
        }

        //src_sdk->sendVelocityBack(final_target_vel_);
        int64_t time_second2 = sros::core::util::get_time_in_ms();

        int64_t last_time = 20 - (time_second2 - time_first2);

        if(last_time<0){
            last_time = 0;
        }

        boost::this_thread::sleep_for(boost::chrono::milliseconds(last_time)); // 延时剩余时间
    }


}

void NavigationModule::onVelocityMsg(sros::core::Velocity vel) {
    current_vel_ = vel;
    if (!enable_free_nav_) {
        return;
    }




    // LOG(INFO)<<"TARGET V::::::::::::"<<target_vel_.vx()<<"  :theta: "<<target_vel_.vtheta();

    // LOG(INFO)<<"GOAL X::::::::::::"<<cur_dst_pose_.x()<<"  :Y: "<<cur_dst_pose_.y()<<"  :yaw:"<<cur_dst_pose_.yaw();

    //    if(navigation_mode_ == sros::core::NAV_MODE_LOCAL_PLAN&&navigation_type_ == sros::core::NAV_TYPE_MAP) {
    //        sr_local_Planner.onVelocityMsg(vel, rotate_enable_);
    //    }
    //    if(navigation_mode_ == sros::core::NAV_MODE_LOCAL_PLAN&&navigation_type_ == sros::core::NAV_TYPE_NET)
    //    {
    //        sr_local_Planner.localNavOnNet(vel);
    //    }
}

void NavigationModule::onCheckManualVelocity(int &v, int &w) {
    auto enable_manual_avoid_obstacle =
        (Settings::getInstance().getValue<std::string>("nav.enable_manual_control_avoid_obstacle", "False") ==
         "True");
    if (g_state.isManualControl()&&enable_manual_avoid_obstacle) {
        if (v != 0 || w != 0) {
            Eigen::Vector2d velocity;
            velocity[0] = (double)v / 1000.0;
            velocity[1] = (double)w / 1000.0;
            if (manual_avoidoba_processor) {
                avoidoba_para->manual_stop_distance = Settings::getInstance().getValue("nav.manual_control_stop_distance", 0.2);
                avoidoba_para->manual_slow_distance = Settings::getInstance().getValue("nav.manual_control_slow_distance", 0.2);
                if (avoidoba_para->manual_slow_distance - avoidoba_para->manual_stop_distance > 0.2) {
                    avoidoba_para->manual_slow_distance =
                        avoidoba_para->manual_slow_distance - avoidoba_para->manual_stop_distance;
                }
                bool curr_syn_rotate_state = false;
                updateAvdPara(manual_avoidoba_processor, carbody_size, curr_syn_rotate_state);
                avoidoba_para->enable_auto_stop_dist_by_vel =
                    Settings::getInstance().getValue<std::string>("nav.increase_stop_dist_every_velocity", "True") ==
                    "True";
                if (avoidoba_para->enable_auto_stop_dist_by_vel) {
                    double stop_dist_offset = 0.0, slow_dist_offset = 0.0;
                    updateStopDistByVel(current_vel_, stop_dist_offset, slow_dist_offset);
//                    avoidoba_para->manual_stop_distance = avoidoba_para->manual_stop_distance + stop_dist_offset;
                    avoidoba_para->manual_slow_distance = avoidoba_para->manual_slow_distance + slow_dist_offset;
                }
                manual_avoidoba_processor->updatePara();
                avoidoba::AvdobaPoseInfo avdoba_pose_info;
                avoidoba_processor->getObstaclePoints(avdoba_pose_info, true, false);
                auto curr_pose = src_sdk->getCurPose();
                Eigen::Vector3d curr_car_pose(curr_pose.x(), curr_pose.y(), curr_pose.yaw());
                manual_avoidoba_processor->checkManualVelocity(curr_car_pose, carbody_size, avdoba_pose_info, velocity);
                if (g_state.need_avoid_obstacle_prediction) {
                    Polygons polygons;
                    convertToPolygon(avdoba_pose_info, curr_syn_rotate_state, polygons);
                    newCarSimulateMovePoses(std::move(polygons));
                }
                if (avdoba_pose_info.result_info.collide_value >= avoidoba_para->min_slow_collide_value) {
                    g_state.in_manual_obstacle_paused = true;
                    if (avoidoba_processor) {
                        auto info = avoidoba_processor->outputObstacleInfo(
                            Eigen::Vector3d(curr_pose.x(), curr_pose.y(), curr_pose.yaw()), carbody_size, avdoba_pose_info);
                        if (avdoba_pose_info.result_info.oba_name == "PS_OBA" ) {
                            g_state.obstacle_direction = sros::core::ObstacleDirection::OBSTACLE_DIRECTION_BACKWARD;
                        }else {
                            g_state.obstacle_direction = (sros::core::ObstacleDirection)info.first;
                        }
                    }
                }else{
                    g_state.in_manual_obstacle_paused = false;
                }
                v = velocity[0] * 1000.0;
                w = velocity[1] * 1000.0;
                return;
            }
        }
    }
    g_state.in_manual_obstacle_paused = false;
}

sros::core::NavigationPath_vector NavigationModule::getNavigationPath(StationNo_t start_station_no,
                                                                      StationNo_t dst_station_no, Pose input_start_pose,
                                                                      Pose input_dst_pose, ObstacleAvoidPolicy policy,
                                                                      Pose &out_dst_pose,
                                                                      Location_Vector &radar_points, bool force_nav_on_map) {
    LOG(INFO) << start_station_no << input_start_pose << " => " << dst_station_no << input_dst_pose
              << " policy: " << policy << " force_nav_on_map: " << force_nav_on_map;

    auto &s = sros::core::Settings::getInstance();

    auto enable_free_nav_when_net_no_way =
        (s.getValue<std::string>("nav.enable_free_nav_when_net_no_way", "False") == "True");

    bool allow_backward = false;
    NavigationPath_vector paths;

    auto station_group = navigate_.getNavigationMap()->getStationMarkGroup();
    auto start_station = station_group->getItem(start_station_no);
    auto dst_station = station_group->getItem(dst_station_no);

    if ((start_station_no != 0 && start_station.id == 0) || (dst_station_no != 0 && dst_station.id == 0)) {
        // 地图中不存在该站点,直接返回
        LOG(INFO) << "地图中不存在该站点,直接返回: " << start_station_no << " -> " << dst_station_no;
        failed_code_ = ERROR_CODE_NAV_FIND_PATH_NO_STATION_IN_MAP;
        out_dst_pose = input_dst_pose;
        return paths;
    }

    Pose cur_pose = src_sdk->getCurPose();  // TODO(nobody): 提取到输入

    LOG(INFO) << cur_pose;
    //    double cur_pos_yaw = (cur_pose.yaw() >= 0) ? cur_pose.yaw() : (cur_pose.yaw() + 2 * M_PI);

    if (start_station_no != 0 && start_station_no == dst_station_no) {
        LOG(INFO) << "起点与终点站点为同一站点, 调整车朝向";
        double rotate_value = dst_station.pos.yaw;
        paths.push_back(RotatePath(start_station.pos.x / 100, start_station.pos.y / 100, rotate_value));
        out_dst_pose = input_dst_pose;
        return paths;
    }

    //起点与终点为同一目标点，调整车朝向
    Pose calc_pose_src;
    Pose calc_pose_dst;
    if (start_station_no != 0) {
        calc_pose_src.x() = start_station.pos.x / 100;
        calc_pose_src.y() = start_station.pos.y / 100;
        calc_pose_src.yaw()  = start_station.pos.yaw;
    } else {
        calc_pose_src = input_start_pose;
    }

    if(dst_station_no != 0) {
        calc_pose_dst.x() = dst_station.pos.x / 100;
        calc_pose_dst.y() = dst_station.pos.y / 100;
        calc_pose_dst.yaw()  = dst_station.pos.yaw;
    } else {
        calc_pose_dst = input_dst_pose;
    }

    auto distance = get2PointDistance(calc_pose_src.x(), calc_pose_src.y(), calc_pose_dst.x(), calc_pose_dst.y());
    auto dst_point_distance = s.getValue<double>("main.dst_point_distance", 0.02);
    LOG(INFO) << "起点与终点站点的计算距离: " << distance << ", min_dst_distance: " << dst_point_distance;

    if (distance < dst_point_distance) {
        LOG(INFO) << "起点与终点站点为同一位置, 调整车朝向";
        paths.push_back(RotatePath(calc_pose_src.x(), calc_pose_src.y(), calc_pose_dst.yaw()));
        out_dst_pose = calc_pose_src;
        return paths;
    }

    auto pointCm2MFunc = [](sros::map::MarkPointf &pointf) -> sros::map::MarkPointf {
        pointf.x *= CM_TO_METER;
        pointf.y *= CM_TO_METER;
        return pointf;
    };

    auto stationCm2MFunc = [&](sros::map::StationMarkGroup stations) -> sros::map::StationMarkGroup {
        for (auto &it : stations) {
            auto &station = it.second;
            pointCm2MFunc(station.pos);
            pointCm2MFunc(station.enter_pos);
            pointCm2MFunc(station.exit_pos);
            pointCm2MFunc(station.check_pos);
        }
        return stations;
    };

    auto edgeCm2MFunc = [&](sros::map::net::EdgeGroup edges) {
        for (auto &it : edges) {
            auto &edge = it.second;
            edge.sx *= CM_TO_METER;
            edge.sy *= CM_TO_METER;
            edge.ex *= CM_TO_METER;
            edge.ey *= CM_TO_METER;
            edge.cx *= CM_TO_METER;
            edge.cy *= CM_TO_METER;
            edge.dx *= CM_TO_METER;
            edge.dy *= CM_TO_METER;
            edge.cost *= CM_TO_METER;
        }
        return edges;
    };

    auto nodeCm2MFunc = [&](sros::map::net::NodeGroup nodes) {
        for (auto &it : nodes) {
            auto &node = it.second;
            node.x *= CM_TO_METER;
            node.y *= CM_TO_METER;
        }
        return nodes;
    };

    auto &setting = Settings::getInstance();
    NavigationOnNet navigation_no_net(stationCm2MFunc(*navigate_.getNavigationMap()->getStationMarkGroup()),
                                      edgeCm2MFunc(*navigate_.getNavigationMap()->getNetEdgeGroup()),
                                      nodeCm2MFunc(*navigate_.getNavigationMap()->getNetNodeGroup()),
                                      g_state.load_state, navigate_.getNavigationMap()->getMapVersion());
    navigation_no_net.setParameters(
        setting.getValue<double>("nav.nearest_edge_distance_threshold", 0.16),
        setting.getValue<double>("nav.start_pose_rotate_threshold", 10) / 10.0 * DEGREE_TO_RAD,
        setting.getValue<double>("nav.rotate_between_path_threshold", 10) / 10.0 * DEGREE_TO_RAD,
        setting.getValue<double>("nav.end_pose_rotate_threshold", 1) / 10.0 * DEGREE_TO_RAD,
        setting.getValue<std::string>("nav.move_to_nearest_object_first", "EDGE_END_NODE") == "EDGE_END_NODE",
        setting.getValue<double>("nav.nearest_station_distance_threshold", 0.16),
        setting.getValue<string>("nav.enable_regress_keep_agv_facing_follow_edge", "False") == "True",
        Settings::getInstance().getValue("nav.vehicle_width", 0.5),
        Settings::getInstance().getValue("nav.rotate_cost_rate", 1.0));
    bool start_pose_is_on_net =
        navigation_no_net.checkIsOnNet(start_station_no, input_start_pose);  // 起始点是否在路网上
    bool dst_pose_is_on_net = navigation_no_net.checkIsOnNet(dst_station_no, input_dst_pose);  // 目标点是否在路网上

    // 起点和终点是否都在路网上,且开启了路网导航，此时我们认为客户的需求就是路网导航，不管是否开启自由导航，自由导航此时都是被禁用的
    if (!force_nav_on_map && enable_net_nav_ && start_pose_is_on_net && dst_pose_is_on_net) {
        // 路网导航时，是否始终要在生成路径的末尾保留旋转路径
        bool keep_rotate_path_in_end = s.getValue<string>("nav.net_keep_rotate_path_in_end", "False") == "True";

        // 在路网上进行导航
        paths = navigation_no_net.getNavigationPathOnNet(start_station_no, dst_station_no, input_start_pose,
                                                         input_dst_pose, keep_rotate_path_in_end, out_dst_pose);
        if (paths.empty()) {
            failed_code_ = navigation_no_net.getFailedCode();
        }
        navigation_type_ = sros::core::NAV_TYPE_NET;
        // 如果在路网上没有搜索到路径，且启用了nav.enable_free_nav_when_net_no_way
        // 那么使用自由导航进行搜索，为了解决歌尔提出的“跨路网路径生成”需求
        if (paths.empty() && enable_free_nav_when_net_no_way && enable_free_nav_) {
            LOG(INFO) << "Net navigation no way!, start free navigation.";
            paths = getNavigationPathOnMap(start_station_no, dst_station_no, input_start_pose, input_dst_pose, policy,
                                           out_dst_pose, radar_points, allow_backward);
            if (paths.empty()) {
                failed_code_ = sros::core::ERROR_CODE_FREE_NAV_NO_WAY;
            }
            navigation_type_ = sros::core::NAV_TYPE_MAP;
        }

    } else if (enable_free_nav_) {
        LOG(INFO) << "input_start_pose x  " << input_start_pose.x() << " input_start_pose y " << input_start_pose.y()
                  << " input_start_pose yaw " << input_start_pose.yaw();
        LOG(INFO) << "input_dst_pose x  " << input_dst_pose.x() << " input_dst_pose y " << input_dst_pose.y()
                  << " input_dst_pose yaw " << input_dst_pose.yaw();

        paths = getNavigationPathOnMap(start_station_no, dst_station_no, input_start_pose, input_dst_pose, policy,
                                       out_dst_pose, radar_points, allow_backward);
        if (paths.empty()) {
            failed_code_ = sros::core::ERROR_CODE_FREE_NAV_NO_WAY;
        }

        navigation_type_ = sros::core::NAV_TYPE_MAP;
    } else if (enable_net_nav_) {  // 若启动了路网导航，但是起点或终点没有在路径上
        if (!start_pose_is_on_net) {
            failed_code_ = ERROR_CODE_NET_NAV_START_POSE_NOT_ON_NET;
        } else if (!dst_pose_is_on_net) {
            failed_code_ = ERROR_CODE_NET_NAV_DST_POSE_NOT_ON_NET;
        }
    }

    if (!enable_net_nav_ && !enable_free_nav_) {
        LOG(INFO) << "fail to find path, net navigation and free nav both not enabled!";
        failed_code_ = ERROR_CODE_NAV_FIND_PATH_NET_NAV_AND_FREE_NAV_BOTH_DISABLED;
    }

    debugOutputPathsInWorldMap(paths);
    return paths;
}

NavigationPath_vector NavigationModule::getNavigationPathOnMap(StationNo_t start_station_no, StationNo_t dst_station_no,
                                                               Pose input_start_pose, Pose input_dst_pose,
                                                               ObstacleAvoidPolicy policy, Pose &out_dst_pose,
                                                               Location_Vector &radar_points, bool allow_backward) {
    LOG(INFO) << "enter into free navigation!";
    NavigationPath_vector paths;

    auto station_group = navigate_.getNavigationMap()->getStationMarkGroup();
    auto start_station = station_group->getItem(start_station_no);
    auto dst_station = station_group->getItem(dst_station_no);
    NAV_PATH_TYPE nav_path_type;
    nav_path_type = nav_path_type_;
    Pose start_pose, dst_pose;
    LinePath enter_path, exit_path;
    out_dst_pose = input_dst_pose;

    if (start_station_no == 0) {        // 当前不在任何站点
        start_pose = input_start_pose;  // 以当前Pose作为起点
    } else {
        if (start_station.no_rotate) {
            // 以当前站点的离开Pose作为起点
            start_pose.x() = start_station.exit_pos.x / 100;
            start_pose.y() = start_station.exit_pos.y / 100;
            start_pose.yaw() = start_station.exit_pos.yaw;

            auto direction = start_station.exit_backward ? PATH_BACKWARD : PATH_FORWARD;

            // 离开站点路径
            if (start_station_no == charge_station_no_) {
                exit_path =
                    sros::core::LinePath(start_station.pos.x / 100, start_station.pos.y / 100,
                                         start_station.exit_pos.x / 100, start_station.exit_pos.y / 100, direction);
            } else {
                exit_path =
                    sros::core::LinePath(input_start_pose.x(), input_start_pose.y(), start_station.exit_pos.x / 100,
                                         start_station.exit_pos.y / 100, direction);
            }
            // 遇到障碍强制等待障碍消失
            exit_path.avoid_policy_ = exit_avoid_policy_;
        } else {
            // 以当前站点的Pose作为起点
            start_pose.x() = start_station.pos.x / 100;
            start_pose.y() = start_station.pos.y / 100;
            start_pose.yaw() = start_station.pos.yaw;
        }
    }

    if (dst_station_no == 0) {      // 没有设置目标站点id
        dst_pose = input_dst_pose;  // 使用msg中的位姿作为dst
        nav_path_type = LINE_ROTATE_TYPE;
    } else {
        if (dst_station.pos_dynamic && station_check_state_ == STATION_CHECK_NONE) {
            // 如果是动态站点,且还没有到达CheckPoint, 则以检查点作为导航目标点
            dst_pose.x() = dst_station.check_pos.x / 100;
            dst_pose.y() = dst_station.check_pos.y / 100;
            dst_pose.yaw() = dst_station.check_pos.yaw;
            LOG(ERROR) << " dst_station.check_pos.yaw " << dst_station.check_pos.yaw
                       << " dst_station.check_pos.y / 100 " << dst_station.check_pos.y / 100
                       << " dst_station.check_pos.x / 100 " << dst_station.check_pos.x / 100;

            station_check_state_ = STATION_CHECK_WAITING;
        } else if (dst_station.no_rotate) {
            // 如果目标站点为静态站点或者是已到达CheckPoint的动态站点, 则以目标站点的进入Pose作为起点
            dst_pose.x() = dst_station.enter_pos.x / 100;
            dst_pose.y() = dst_station.enter_pos.y / 100;
            dst_pose.yaw() = dst_station.enter_pos.yaw;

            auto direction = dst_station.enter_backward ? sros::core::PATH_BACKWARD : sros::core::PATH_FORWARD;

            // 进入站点路径
            enter_path = sros::core::LinePath(dst_station.enter_pos.x / 100, dst_station.enter_pos.y / 100,
                                              dst_station.pos.x / 100, dst_station.pos.y / 100, direction);
            // 遇到障碍强制等待障碍消失
            enter_path.avoid_policy_ = enter_avoid_policy_;
        } else {
            // 以目标站点位姿作为终点
            dst_pose.x() = dst_station.pos.x / 100;
            dst_pose.y() = dst_station.pos.y / 100;
            dst_pose.yaw() = dst_station.pos.yaw;
        }
        out_dst_pose.x() = dst_station.pos.x / 100;
        out_dst_pose.y() = dst_station.pos.y / 100;
        out_dst_pose.yaw() = dst_station.pos.yaw;
    }

    //    LOG(WARNING) << "StartPose: " << start_pose.x() << " " << start_pose.y()
    //    << "  Dst_Pose: " << dst_pose.x() << " " <<dst_pose.y() << std::endl;
    // 根据起点和终点搜寻路径
    paths = navigate_.getNavigationPaths(start_pose, dst_pose, nav_path_type, radar_points, allow_backward);
    if (paths.empty()) {
        failed_code_ = navigate_.getFiledCode();
        return paths;
    }

    // 根据输入设置路径的避障方式
    for (auto it = paths.begin(); it != paths.end(); ++it) {
        it->avoid_policy_ = policy;
    }

    if (start_station_no != 0 && start_station.no_rotate) {
        // 当前是从某个站点出发,而且需要插入离开路径
        // 将离开路径插入到路径的开始处
        paths.insert(paths.begin(), exit_path);
    }

    if ((!dst_station.pos_dynamic && dst_station_no != 0 && dst_station.no_rotate) ||
        (dst_station.pos_dynamic && station_check_state_ == STATION_CHECK_CHECKED)) {
        // (目标是站点 && 需要插入进入路径) || 动态站点且已经到达CheckPoint
        paths.push_back(enter_path);
    }

    return paths;
}

void NavigationModule::initNavigation(const string &map_name) {
    if (map_name.empty()) {
        LOG(WARNING) << "map_name is empty";
        return;
    }
    LOG(INFO) << "AGV is navigating on map " << map_name << " !";

    MapManager map_manager;
    map_manager.freshMapList();
    MapInfo map_info = map_manager.getMapInfo(map_name);

    // 修复每次自由导航都更新地图导致规划时间变长的问题
    ///////////////////////////////test 临时测试
    // cur_map_info_.name_ = "";
    // LOG(INFO) << "自由导航内存稳定性测试";
    ///////////////////////////////test 临时测试

    // 如果地图没有变化,则不需要重新初始化
    if (!(cur_map_info_ == map_info)) {  // MapInfo没有重载!=运算符
        cur_map_info_ = map_info;

        LOG(INFO) << "正在初始化navigation: " << map_info.file_path_;
        navigate_.loadMapFile(map_info.file_path_, enable_free_nav_);

        if (enable_free_nav_) {
            // 只有当启用自由导航时才需要初始化地图数据, 不初始化地图可以节省内存空间
            LOG(INFO) << "enable_free_nav_ = true, initMapData()";
            navigate_.initMapData();
            LOG(INFO)<<"地图大小：x:"<<navigate_.getNavigationMap()->getMapSizeX()<<";y:"<<navigate_.getNavigationMap()->getMapSizeY();
            // 更新用于smoother的地图。该地图没有膨胀。            
            
            global_map_.init_check_collision(navigate_.getNavigationMap(),&nav_cfg_);
            // LOG(INFO)<<"1111:"<<(float)global_map_.collision_map[1][1].value;
            //LOG(INFO)<<"XXX"<<xxx;
            //CheckCollision_Ptr global_map_ptr = CheckCollision_Ptr(&global_map_);
            planning_fsm_.initMap(navigate_, nav_cfg_);
            //LOG(INFO)<<"CESHI";
        }

        //         ////////////////////读取地图和地图中的参考线，初始化势场地图///////////
        // int offset_x, offset_y;
        // offset_x = navigate_.getNavigationMap()->getMapZeroOffset().x;
        // offset_y = navigate_.getNavigationMap()->getMapZeroOffset().y;
        // int mx, my;
        // double wx, wy;
        // navigate_.reverseMapCoords(offset_x, offset_y, 0, 0, &wx, &wy);
        // auto init_paths = navigate_.getNavigationMap()->getNetEdgeGroup();
        // LOG(INFO) << "路网线段条数: " << init_paths->getItemList().size();
        // sros::map::net::EdgeGroup paths;
        // for(int i = 0; i < init_paths->getItemList().size(); i++) {
        //     //LOG(INFO) << i << ", " << init_paths->getItemList()[i].execute_type << ", " << init_paths->getItemList()[i].sx << ", " << init_paths->getItemList()[i].sy;
        //     if(init_paths->getItemList()[i].execute_type == 1) {
        //         paths.addItem(init_paths->getItemList()[i]);
        //     }
        // }
     //   g_gradient_of_ref_path.initTheMap(sros::core::Pose(wx, wy, 0), navigate_.getNavigationMap()->getMapSizeX() * navigate_.getNavigationMap()->getMapResolution() / 100.0, navigate_.getNavigationMap()->getMapSizeY() * navigate_.getNavigationMap()->getMapResolution() / 100.0, 0.05, paths,global_map_ptr);
        ////////////////////

    }

    auto map_ptr = navigate_.getNavigationMap();
    if (!map_ptr) {
        LOG(WARNING) << "Map not set !";
        return;
    }
}


void NavigationModule::updateRegionSwitchState(int curr_pose_no) {
    auto enable_region_switch_avoid_obstacle =
        (Settings::getInstance().getValue<std::string>("obstacle.enable_switch_region_avoid_obstacle", "False") ==
         "True");
    if (!enable_region_switch_avoid_obstacle) {
        return;
    }
    if (avoidoba_processor) {
        std::shared_ptr<sros::core::CommonCommandMsg<std::string>> region_switch_cmd(
            new sros::core::CommonCommandMsg<std::string>("OBSTACLE_REGION_SWITCH"));
        if (curr_pose_no < 0) {
            region_switch_cmd->command = "DEFAULT";
        }else{
            const auto curr_pose = src_sdk->getCurPose();
            Eigen::Vector3d curr_pose_eigen(curr_pose.x(),curr_pose.y(),curr_pose.yaw());
            auto path_type = avoidoba_processor->getPathTypeByNumber(curr_pose_no,curr_pose_eigen);
            //目前只检测path type,等后期,加上与任务耦合逻辑

            region_switch_cmd->command = path_type;
            if (g_state.load_state == sros::core::LOAD_FULL) {
                region_switch_cmd->param0 = 1;
            }else{
                region_switch_cmd->param0 = 0;
            }
            g_state.gpio_output;
        }

        int rotate_value = 0;
        if(avoidoba_para->vehicle_type == "gulf") {
            if (src_sdk->getVersion() == sdk::SRC_PROTO_VERSION_V1) {
                src_sdk->getParameterInTime(0x1410, rotate_value);
            }else{
                rotate_value = g_state.rotate_value;
            }

            bool left_photoelectric_switch = false;//左边叉尖光电开关,第5位io,下标从0开始,常闭信号,默认为1
            bool right_photoelectric_switch = false;//右边叉尖光电开关,第6位io,下标从0开始,常闭信号,默认为1
            
            bool enable_1353 = (Settings::getInstance().getValue<std::string>("main.enable_1353", "False") == "True");
            if(enable_1353) {
                uint16_t io_1353 = g_state.io_1353;
                left_photoelectric_switch = (io_1353 >> 4) & 0x01;
                right_photoelectric_switch = (io_1353 >> 5) & 0x01;
            }else {
                uint8_t gpio_output = g_state.gpio_input;
                left_photoelectric_switch = (gpio_output >> 5) & 0x01;
                right_photoelectric_switch = (gpio_output >> 6) & 0x01;
            }


            region_switch_cmd->str0 = left_photoelectric_switch ? "PS LEFT OFF" : "PS LEFT ON" ;
            region_switch_cmd->str1 = right_photoelectric_switch ? "PS RIGHT OFF" : "PS RIGHT ON" ;

//            LOG(INFO) << " left_photoelectric:" << left_photoelectric_switch
//                      << ", right_photoelectric:" << right_photoelectric_switch
//                      << ", "<< gpio_output << ", " << g_state.rotate_value;

            // 光电触发时检查是否在禁止触发区域内
            bool is_disable_photoelectric_switch_area = false;
            if (!left_photoelectric_switch | !right_photoelectric_switch) {
                Pose cur_pose = src_sdk->getCurPose();
                auto area_list =
                        MapManager::getInstance()->getInsideArea(cur_pose.x() * 100, cur_pose.y() * 100);

//                LOG(INFO) << "area list size = " << area_list.size();
                for (const auto &area : area_list) {
//                    LOG(INFO) << "  type = " << area.type;
                    if (sros::map::AreaMark::AREA_TYPE_DISABLE_PHOTOELECTRIC_SWITCH_OBA == area.type) {
                        LOG(INFO) << "enter disable photoelectric switch area.";
                        is_disable_photoelectric_switch_area = true;    // AGV在禁止光叉尖光电触发区域内
                        break;
                    }
                }

                // 只有在非禁止光叉尖光电触发的区域内触发了光电时才打印输出日志.
                if (false == is_disable_photoelectric_switch_area){
                    // LOG(INFO) << "left_photoelectric_switch:" << region_switch_cmd->str0
                    //           << " right_photoelectric_switch:" << region_switch_cmd->str1;
                } else {    // 如果AGV在禁止光叉尖光电触发区域内则屏蔽触发信号
                    LOG(INFO) << "shielding photoelectric switch!";
                    region_switch_cmd->str0 = "PS LEFT OFF";
                    region_switch_cmd->str1 = "PS RIGHT OFF";
                }
            }
        }
        region_switch_cmd->param1 = g_state.rotate_value;//1进叉，2退叉；0为默认
        sendMsg(region_switch_cmd);
    }
}

void NavigationModule::convertToPolygon(avoidoba::AvdobaPoseInfo& avdoba_pose_info,bool curr_syn_rotate_state,Polygons &polygons){
    polygons.reserve(avdoba_pose_info.collide_poses.size() + avdoba_pose_info.slow_poses.size());
    avoidoba::OriginCollideSize_Ptr collide_size(new avoidoba::OriginCollideSize);
    *collide_size = *carbody_size;

    for (auto &pose : avdoba_pose_info.collide_poses) {
        if (pose.move_state == avoidoba::ROTATE_MOVE) {
            if (curr_syn_rotate_state) {
                avoidoba::OriginCollideSize_Ptr rotate_size(new avoidoba::OriginCollideSize);
                *rotate_size = *collide_size;
                rotate_size->rotate_angle = 0.0;
                rotate_size->head_length = collide_size->car_length / 2.0;
                rotate_size->back_length = -collide_size->car_length / 2.0;
                rotate_size->left_width = collide_size->car_width / 2.0;
                rotate_size->right_width = -collide_size->car_width / 2.0;
                convertRotateParticleToPolygons(pose.pose, pose.start_angle, pose.end_angle,
                                                rotate_size, polygons);
                collide_size->rotate_angle -=
                    rotate::RotateCurveOperator::normalizeAngle(pose.end_angle - pose.start_angle);
            } else {
                convertRotateParticleToPolygons(pose.pose, pose.start_angle, pose.end_angle,
                                                collide_size, polygons);
            }

        } else {
            polygons.emplace_back();
            convertParticleToPolygon(pose.pose, collide_size, polygons.back());
        }
    }
    for (auto &pose : avdoba_pose_info.slow_poses) {
        if (pose.move_state == avoidoba::ROTATE_MOVE) {
            if (curr_syn_rotate_state) {
                avoidoba::OriginCollideSize_Ptr rotate_size(new avoidoba::OriginCollideSize);
                *rotate_size = *collide_size;
                rotate_size->rotate_angle = 0.0;
                rotate_size->length = collide_size->car_length;
                rotate_size->width = collide_size->car_width;
                convertRotateParticleToPolygons(pose.pose, pose.start_angle, pose.end_angle,
                                                rotate_size, polygons);
                collide_size->rotate_angle -=
                    rotate::RotateCurveOperator::normalizeAngle(pose.end_angle - pose.start_angle);
            } else {
                convertRotateParticleToPolygons(pose.pose, pose.start_angle, pose.end_angle,
                                                collide_size, polygons);
            }

        } else {
            polygons.emplace_back();
            convertParticleToPolygon(pose.pose, collide_size, polygons.back());
        }
    }
}

void NavigationModule::avoidObaLoop(sros::core::base_msg_ptr m) {
    if (!g_state.isManualControl()) {
        g_state.in_manual_obstacle_paused = false;
    }
    if (!enable_path_oba_) {  // 若没有启用基于路径检测的避障, 直接返回
        LOG_EVERY_N(WARNING, 1000) << "Disabled path obstacle!";  // 每5秒打印一次
        return;
    }

    free_nav_check_obstacle();  // 当进行自由导航时，检测自由导航轨迹上是否有障碍物。

    if (navigation_mode_ == sros::core::NAV_MODE_LOCAL_PLAN)  //如果当前处于局部路径规划模式下则直接返回。
    {
        LOG_EVERY_N(INFO, 1000) << "local plan mode";  // 每5秒打印一次
        return;
    }

    if (!enable_net_nav_ && !enable_free_nav_) {                                       // 若没有启用自动导航,
        LOG_EVERY_N(WARNING, 1000) << "Disabled net navigation and free navigation!";  // 每5秒打印一次
        // 直接返回，没启动自由导航时，栅格地图没有加载，无法进行基于路径的避障
        return;
    }

    if (!g_state.isNeedAvoidObaNavState()) {  // 若当前的导航状态不需要避障，就直接返回
        updateRegionSwitchState(-1);
        return;
    }

    auto src_state = src_sdk->getSRCState();

    int cur_path_no = src_state.path_no;  // 获取当前执行路径编号,注意该路径
    if (cur_path_no < 0) {
        LOG(ERROR) << "curr path no is wrong!" << cur_path_no;
        return;
    }
    if (cur_path_no == 0 && g_state.nav_state != STATE_NAV_PATH_PAUSED &&
        g_state.nav_state != STATE_NAV_PATH_REFINDING_PAUSED && g_state.nav_state != STATE_NAV_MANUAL_PAUSED &&
        g_state.nav_state != STATE_NAV_MANUAL_WAITING_FOR_START_PAUSED &&
        g_state.nav_state != STATE_NAV_MANUAL_WAITING_FOR_START) {
        return;
    }

    int cur_path_idx = (cur_path_no > 0) ? (cur_path_no - 1) : 0;

    if (cur_path_idx >= cur_paths_.size()) {  // cur_path 存在等于0的情况,说明无法到达目标点
        LOG(ERROR) << "NavigationModule: cur_path_no >= cur_paths_.size()";
        return;
    }

    BLOCK_TYPE collision = SEGMENT_FREE;
    BLOCK_TYPE check_collision = SEGMENT_FREE;
    CollideState collide_state;
    // 取放货避障结束判断
    auto curr_pose = src_sdk->getCurPose();
    forklift_state = (curr_pose.distance_to(cmd_position) <=  1.0 && is_enable_avd)? DetectAndAvdObaState::ACTION_FINISH : forklift_state;
    LOG_EVERY_N(INFO, 100) << "curr_pose.distance_to(cmd_position): " << curr_pose.distance_to(cmd_position);
    //if reached cmd_position, disable camera
    if (forklift_state == DetectAndAvdObaState::ACTION_FINISH && is_enable_avd){
        // disable 03d camera
        LOG(INFO)  << "reached target position, disable camera.";
        std::shared_ptr<sros::core::CommonMsg> cmd_msg(new sros::core::CommonMsg("TOPIC_SVC100_ENABLE_PUBLISH"));
        cmd_msg->str_0_ = sros::device::DEVICE_CAMERA_O3D303;
        cmd_msg->flag = false;
        sendMsg(cmd_msg); 

        //disable back laser   
        auto msg = std::make_shared<sros::core::CommonMsg>("TOPIC_BACK_LASER_ENABLE_PUBLISH");
        msg->str_0_ = sros::device::DEVICE_UST_LIDAR_BACK;
        msg->flag = false;
        sros::core::MsgBus::sendMsg(msg);

        is_enable_avd = false;
    }


    avoidoba::AvdobaPoseInfo avdoba_pose_info;
    int avdoba_value = 0;
    // 仅当启用避障时才检测,否则认为无障碍
    if (obstacle_avoid_enable_) {
        auto forward_slow_distance = Settings::getInstance().getValue("nav.slow_distance", 1.5);
        auto forward_stop_distance = Settings::getInstance().getValue("nav.stop_distance", 1.0);
        auto backward_stop_distance = Settings::getInstance().getValue("nav.backward_stop_distance", 0.5);
        updateRegionSwitchState(cur_path_idx);

        if (forward_slow_distance != avoidoba_para->origin_forward_slow_distance ||
            forward_stop_distance != avoidoba_para->origin_forward_stop_distance ||
            backward_stop_distance != avoidoba_para->origin_backward_stop_distance) {
            LOG(INFO) << "slow or stop para is change:" << forward_slow_distance << "," << forward_stop_distance << ","
                      << backward_stop_distance << "origin para is:" << avoidoba_para->origin_forward_slow_distance << ","
                      << avoidoba_para->origin_forward_stop_distance << "," << avoidoba_para->origin_backward_stop_distance;
            avoidoba_para->forward_slow_distance = forward_slow_distance;
            avoidoba_para->forward_stop_distance = forward_stop_distance;
            avoidoba_para->backward_stop_distance = backward_stop_distance;
            avoidoba_para->origin_forward_slow_distance = forward_slow_distance;
            avoidoba_para->origin_forward_stop_distance = forward_stop_distance;
            avoidoba_para->origin_backward_stop_distance = backward_stop_distance;

            avoidoba_processor->updatePara();
        }
        avoidoba_para->enable_auto_stop_dist_by_vel =
            Settings::getInstance().getValue<std::string>("nav.enable_auto_stop_dist_by_velocity", "True") == "True";
        if (avoidoba_para->enable_auto_stop_dist_by_vel) {
            double stop_dist_offset = 0.0, slow_dist_offset = 0.0;
            updateStopDistByVel(current_vel_, stop_dist_offset, slow_dist_offset); // 基于速度改变避障距离

            avoidoba_para->forward_stop_distance = avoidoba_para->origin_forward_stop_distance + stop_dist_offset;
            avoidoba_para->forward_slow_distance = avoidoba_para->origin_forward_slow_distance + slow_dist_offset;
            avoidoba_processor->updatePara();
        }
        // 支持在线修改offset
        Eigen::Vector3d curr_car_pose(curr_pose.x(), curr_pose.y(), curr_pose.yaw());
        bool curr_syn_rotate_state = false;
        updateAvdPara(avoidoba_processor, carbody_size, curr_syn_rotate_state);
        if (avoidoba_processor) {
            avoidoba_processor->updateSynRotateState(curr_syn_rotate_state);
            //            LOG(INFO) << "curr path index:" << cur_path_idx <<
            //            ","<<curr_car_pose[0]<<","<<curr_car_pose[1]<<","<<curr_car_pose[2];
            //目前没有锁来保持同步
            avoidoba_processor->updateCheckPoint(g_state.checkpoint_no);//为了配合调度系统开发基于checkpoint的避障系统
            avdoba_value =
                avoidoba_processor->getAvdObaValue(cur_path_idx, curr_car_pose, carbody_size, avdoba_pose_info);

            // 当启用的避障调试模式时，将模拟的小车位置发送给chip显示
            if (g_state.need_avoid_obstacle_prediction) {
                static int count = 0;
                ++count;
                // 控制200ms上传一次，上传速度太快网络带宽大、Matrix处理不过来，tcmalloc调试时，网络现场直接卡死
                // 上传粒子间隔其实也存在很大的优化空间
                if (count % 4 == 0) {
                    Polygons polygons;
                    convertToPolygon(avdoba_pose_info, curr_syn_rotate_state, polygons);
                    newCarSimulateMovePoses(std::move(polygons));
                }
            }
            //            LOG(INFO) << "avdoba value:" << avdoba_value;
            collide_state = computeBlockTypeByCollissionValue(avdoba_value);
            collision = collide_state.type;
        } else {
            LOG(INFO) << "cannot find processor! maybe have not been initialized";
        }
    }

    pre_collision_ = collision;

    Location_Vector oba_points;

    updateObaPoints(oba_points, avdoba_pose_info.oba_points);  //
    bool need_output_obstacle_info = false;
    bool need_send_obstacle_info = false;
    if (g_state.nav_state == STATE_NAV_WAITING_FOR_FINISH) {  //自动路径检测障碍
        if (collision == SEGMENT_SLOW) {
            LOG(INFO) << "RUNNING"
                      << " => "
                      << "RUNNING_SLOW";
            need_output_obstacle_info = true;
            need_send_obstacle_info = true;
            updateState(STATE_NAV_WAITING_FOR_FINISH_SLOW);
        } else if (collision == SEGMENT_STOP) {
            LOG(INFO) << "RUNNING"
                      << " => "
                      << "PATH_PAUSED (handleObstacleStop)";
            need_output_obstacle_info = true;
            need_send_obstacle_info = true;
            // 避障处理逻辑
            handleObstacleStop(cur_paths_[cur_path_idx].avoid_policy_, oba_points, collide_state.stop_param);
            last_collide_state = collide_state;
        }
    } else if (g_state.nav_state == STATE_NAV_WAITING_FOR_FINISH_SLOW) {
        if (collision == SEGMENT_FREE) {
            LOG(INFO) << "RUNNING_SLOW"
                      << " => "
                      << "RUNNING";
            //need_output_obstacle_info = true;
            updateState(STATE_NAV_WAITING_FOR_FINISH);
        } else if (collision == SEGMENT_STOP) {
            LOG(INFO) << "RUNNING_SLOW"
                      << " => "
                      << "PATH_PAUSED (handleObstacleStop)";
            need_output_obstacle_info = true;
            need_send_obstacle_info = true;
            // 避障处理逻辑
            handleObstacleStop(cur_paths_[cur_path_idx].avoid_policy_, oba_points, collide_state.stop_param);
        }
    } else if (g_state.nav_state == STATE_NAV_PATH_REFINDING_PAUSED) {
        // 根据当前视野内的障碍点重新规划路径
        handleObstacleStop(OBSTACLE_AVOID_REPLAN, oba_points, collide_state.stop_param);
    } else if (g_state.nav_state == STATE_NAV_PATH_PAUSED) {
        if (collision == SEGMENT_FREE) {  // 障碍消除,继续运动
            LOG(INFO) << "PATH_PAUSED"
                      << " => "
                      << "RUNNING";
            //need_output_obstacle_info = true;
            updateState(STATE_NAV_WAITING_FOR_FINISH);
        } else if (collision == SEGMENT_SLOW) {
            LOG(INFO) << "PATH_PAUSED"
                      << " => "
                      << "RUNNING_SLOW";
            need_output_obstacle_info = true;
            need_send_obstacle_info = true;
            updateState(STATE_NAV_WAITING_FOR_FINISH_SLOW);
        } else if (collision == SEGMENT_STOP) {
            if (last_collide_state.stop_param != collide_state.stop_param) {
                LOG(INFO) << "PATH_PAUSED"
                          << " => "
                          << "PATH_PAUSED (handleObstacleStop)";
                LOG(INFO) << "stop param updated: " << last_collide_state.stop_param << " => "
                          << collide_state.stop_param;
                need_output_obstacle_info = true;
                handleObstacleStop(cur_paths_[cur_path_idx].avoid_policy_, oba_points, collide_state.stop_param);
                last_collide_state = collide_state;
            }
            need_send_obstacle_info = true;
        }
    } else if (g_state.nav_state == STATE_NAV_MANUAL_WAITING_FOR_START) {
        if (collision == SEGMENT_FREE) {
            // 无障碍，开始运动
            LOG(INFO) << "MANUAL_WAITING_FOR_START"
                      << " => "
                      << "MANUAL_RUNNING";
            updateState(STATE_NAV_MANUAL_WAITING_FOR_FINISH);
        } else if (collision == SEGMENT_STOP || collision == SEGMENT_SLOW) {
            // 有障碍，停车等待障碍消失
            LOG(INFO) << "MANUAL_WAITING_FOR_START"
                      << " => "
                      << "MANUAL_WAITING_FOR_START_PAUSED";
            need_output_obstacle_info = true;
            need_send_obstacle_info = true;
            updateState(STATE_NAV_MANUAL_WAITING_FOR_START_PAUSED);
        }
    } else if (g_state.nav_state == STATE_NAV_MANUAL_WAITING_FOR_START_PAUSED) {
        if (collision == SEGMENT_FREE || collision == SEGMENT_SLOW) {
            // 无障碍，开始运动, 減少的情况下也可运动
            LOG(INFO) << "MANUAL_WAITING_FOR_START_PAUSED"
                      << " => "
                      << "MANUAL_RUNNING";
            updateState(STATE_NAV_MANUAL_WAITING_FOR_FINISH);
        }
    } else if (g_state.nav_state == STATE_NAV_MANUAL_WAITING_FOR_FINISH) {
        if (collision == SEGMENT_SLOW) {
            // 减速运动
            LOG(INFO) << "MANUAL_RUNNING"
                      << " => "
                      << "MANUAL_RUNNING_SLOW";
            need_output_obstacle_info = true;
            need_send_obstacle_info = true;
            updateState(STATE_NAV_MANUAL_WAITING_FOR_FINISH_SLOW);
        } else if (collision == SEGMENT_STOP) {
            // 停车
            LOG(INFO) << "MANUAL_RUNNING"
                      << " => "
                      << "MANUAL_PAUSED";
            need_output_obstacle_info = true;
            need_send_obstacle_info = true;
            updateState(STATE_NAV_MANUAL_PAUSED, collide_state.stop_param);
            LOG(INFO) << "paused level: " << collide_state.stop_param;
        }
    } else if (g_state.nav_state == STATE_NAV_MANUAL_WAITING_FOR_FINISH_SLOW) {
        if (collision == SEGMENT_FREE) {
            // 障碍消除，全速前进
            LOG(INFO) << "MANUAL_RUNNING_SLOW"
                      << " => "
                      << "MANUAL_RUNNING";
            updateState(STATE_NAV_MANUAL_WAITING_FOR_FINISH);
        } else if (collision == SEGMENT_STOP) {
            // 停车
            LOG(INFO) << "MANUAL_RUNNING_SLOW"
                      << " => "
                      << "MANUAL_PAUSED";
            need_output_obstacle_info = true;
            need_send_obstacle_info = true;
            updateState(STATE_NAV_MANUAL_PAUSED, collide_state.stop_param);
            LOG(INFO) << "paused level: " << collide_state.stop_param;
        }
    } else if (g_state.nav_state == STATE_NAV_MANUAL_PAUSED) {
        // 仅当障碍完全消除时才开始运动
        if (collision == SEGMENT_FREE) {
            LOG(INFO) << "MANUAL_PAUSED"
                      << " => "
                      << "RUNNING";
            updateState(STATE_NAV_MANUAL_WAITING_FOR_FINISH);
        } else if (collision == SEGMENT_SLOW) {
            LOG(INFO) << "MANUAL_PAUSED"
                      << " => "
                      << "MANUAL_RUNNING_SLOW";
            need_output_obstacle_info = true;
            need_send_obstacle_info = true;
            updateState(STATE_NAV_MANUAL_WAITING_FOR_FINISH_SLOW);
        } else if (collision == SEGMENT_STOP) {
            // 如果 stop_param 有更新，那么需要重新执行 updateState() 以应用新的 stop_param
            if (last_collide_state.stop_param != collide_state.stop_param) {
                LOG(INFO) << "stop param updated: " << last_collide_state.stop_param << " => "
                          << collide_state.stop_param;
                LOG(INFO) << "MANUAL_PAUSED"
                          << " => "
                          << "PATH_PAUSED";
                need_output_obstacle_info = true;
                updateState(STATE_NAV_MANUAL_PAUSED, collide_state.stop_param);

                last_collide_state = collide_state;
            }

            need_send_obstacle_info = true;

        }
    }

    if (need_send_obstacle_info) {
        if (avdoba_pose_info.result_info.oba_name.empty()) {
            LOG(INFO) << "oba name is empty!";
        }
        sendObs(avdoba_pose_info.result_info);
    }

    if (need_output_obstacle_info && obstacle_avoid_enable_ && avoidoba_processor) {

        auto info = avoidoba_processor->outputObstacleInfo(
            Eigen::Vector3d(curr_pose.x(), curr_pose.y(), curr_pose.yaw()), carbody_size, avdoba_pose_info);
        if (avdoba_pose_info.result_info.oba_name == "PS_OBA" ) {
            g_state.obstacle_direction = sros::core::ObstacleDirection::OBSTACLE_DIRECTION_BACKWARD;
        }else {
            g_state.obstacle_direction = (sros::core::ObstacleDirection)info.first;
        }
        LOGGER(INFO, SROS) << info.second;
    }
}

void NavigationModule::handleObstacleStop(const ObstacleAvoidPolicy &avoid_policy, Location_Vector &radar_points,
                                          int stop_param) {
    if (avoid_policy == OBSTACLE_AVOID_WAIT) {
        // 暂停运动直至障碍消失415,nav.ignore_line,可容忍直线偏差的最小值,2,2,cm,float,,不产生长度短于该值的直线,30,,,1

        updateState(STATE_NAV_PATH_PAUSED, stop_param);
        LOG(INFO) << "handleObstacleStop() : stop param:" << stop_param;
    } else if (avoid_policy == OBSTACLE_AVOID_REPLAN) {
        // updateNavState(STATE_NAV_PATH_REFINDING);
        // 重新规划路径
        LOG(INFO) << "replan***"
                  << "radar point size:" << radar_points.size();
        // 等待彻底停止运动
        boost::this_thread::sleep_for(boost::chrono::milliseconds(500));

        Pose cur_pose = src_sdk->getCurPose();  // 以当前位姿为起点

        Pose dst_pose;
        auto paths = getNavigationPath(0, cur_dst_station_no_, cur_pose, cur_dst_pose_, cur_avoid_policy_, dst_pose,
                                       radar_points);

        if (paths.empty()) {
            // 没有新路径,暂停等待障碍消失后重新规划路径
            updateState(STATE_NAV_PATH_REFINDING_PAUSED);
        } else {
            // 发送新路径
            auto path_msg = make_shared<PathMsg>("NAV_PATH");
            path_msg->paths = paths;
            sendMsg(path_msg);

            updateCurrPath(paths);
            updateState(STATE_NAV_WAITING_FOR_FINISH);
        }

    } else if (avoid_policy == OBSTACLE_AVOID_NONE) {
    }
}

// 当state为避障暂停类状态时，param_int表示暂停减速级别，默认值为0
// 其他情况param_int不需要设置，默认为0
void NavigationModule::updateState(NavigationState state, int param_int) {
    auto mm = make_shared<CommonStateMsg<NavigationState>>("NAV_STATE");
    mm->state = state;
    mm->param_int = param_int;
    mm->failed_code_ = failed_code_;
    sendMsg(mm);
}

void NavigationModule::initParameter() {
    auto &s = Settings::getInstance();

    auto enter_avoid_policy_str = s.getValue<string>("nav.enter_avoid_policy", "NONE");
    if (enter_avoid_policy_str == "NONE") {
        enter_avoid_policy_ = sros::core::OBSTACLE_AVOID_NONE;
    } else {
        enter_avoid_policy_ = sros::core::OBSTACLE_AVOID_WAIT;
    }

    auto exit_avoid_policy_str = s.getValue<string>("nav.exit_avoid_policy", "NONE");
    if (exit_avoid_policy_str == "NONE") {
        exit_avoid_policy_ = sros::core::OBSTACLE_AVOID_NONE;
    } else {
        exit_avoid_policy_ = sros::core::OBSTACLE_AVOID_WAIT;
    }

    auto navigate_path_type_str = s.getValue<string>("nav.navigate_path_type", "NONE");
    if (navigate_path_type_str == "NONE") {
        nav_path_type_ = BEZIER_TYPE;
    } else {
        if ("LINE_ROTATE_TYPE" == navigate_path_type_str) {
            nav_path_type_ = LINE_ROTATE_TYPE;
        } else if ("LINE_ARC_TYPE" == navigate_path_type_str) {
            nav_path_type_ = LINE_ARC_TYPE;
        } else if ("BEZIER_TYPE" == navigate_path_type_str) {
            nav_path_type_ = BEZIER_TYPE;
        }
    }

    enable_path_oba_ = s.getValue<string>("nav.enable_path_oba", "True") == "True";
    enable_net_nav_ = s.getValue<string>("nav.enable_net_nav", "True") == "True";
    // tk1平台开放自由导航功能，nxp平台暂不开放此功能
    enable_free_nav_ = (CHIP_VERSION == 1) ? (s.getValue<string>("nav.enable_free_nav", "True") == "True") : false;

    enter_exit_station_ = s.getValue<int>("main.enter_exit_station", 0);
    charge_station_no_ = s.getValue<int>("main.charge_station_no", 0);
    dynamicPointToCheckPointLength = s.getValue<int>("nav.dynamicpoint_to_checkpoint_length", 100);
    charging_pile_type_ = s.getValue<string>("main.charging_pile_type", "flat_surface_charging_type");
    charging_point_to_center_length_ = s.getValue<double>("main.charging_point_to_center", 0.55);
    adjust_to_charging_point_length_ = s.getValue<double>("main.adjust_to_charging_point_length_", 1.5);

    navigate_.initNavConfig(nav_cfg_);

    // 取放货避障模型更改 1. 读取二维码信息 2.读取栈板信息
    loadGoodsParam();

}

void NavigationModule::loadGoodsParam() {
    auto &s = sros::core::Settings::getInstance();
    auto get_card_param = [&](const std::string &card_name, const std::string &default_value) {
        auto pallet_info_str = s.getValue<std::string>(card_name, default_value);
        perception::Card card;
        auto infos = common_func::splitStringToVector<std::string>(pallet_info_str, ';');
        if(infos.size() != 6){
            LOGGER(ERROR, SROS) << "card info setting error";
        }else{
            card.setId(std::stoi(infos[0]));
            card.setLength(std::stof(infos[1]));
            card.setPalletHeight(std::stof(infos[2]));
            card.setHoleHeight(std::stof(infos[3]));
            std::vector<std::string> holes_width_str = common_func::splitStringToVector<std::string>(infos[4], ',');
            std::vector<std::string> pallets_width_str = common_func::splitStringToVector<std::string>(infos[5], ',');
            std::vector<float> holes_width;
            float width = .0f, temp;
            for (auto const &iter : holes_width_str) {
                temp = std::stof(iter);
                width += temp;
                holes_width.push_back(temp);
            }
            std::vector<float> pallets_width;
            for (auto const &iter : pallets_width_str) {
                temp = std::stof(iter);
                width += temp;
                pallets_width.push_back(temp);
            }
            card.setWidth(width);
            card.setHoleWidth(holes_width);
            card.setPalletWidth(pallets_width);
            cards_map_.insert(std::make_pair(card.getId(), std::move(card)));
        }
    };

    get_card_param("perception.card_first_info", "0");
    get_card_param("perception.card_second_info", "0;");
    get_card_param("perception.card_third_info", "0;");
    get_card_param("perception.card_fourth_info", "0;");
    get_card_param("perception.card_fifth_info", "0;");
    get_card_param("perception.card_sixth_info", "0;");


    auto QR_Code_info = [&](const std::string &name, const std::string &default_value) {
        auto QR_code_info_str = s.getValue<std::string>(name, default_value);
        nav::QRCode QR_Code;
        auto infos = common_func::splitStringToVector<std::string>(QR_code_info_str, ';');
        if(infos.size() != 3){
            LOGGER(ERROR, SROS) << "qrcode info setting error";
        }else{
            QR_Code.setId(infos[0]);
            QR_Code.setLength(std::stof(infos[1]));
            QR_Code.setWidth(std::stof(infos[2]));
            qrcodes_map_.insert(std::make_pair(QR_Code.getId(), std::move(QR_Code)));
        }
    };

    
    QR_Code_info("obstacle.qrcode_first_info", "0");

    
}

void NavigationModule::onParameterMsg(sros::core::base_msg_ptr m) {
    auto msg = dynamic_pointer_cast<ParameterMsg>(m);
    string name = msg->name;
    string value = msg->value;

    // 此函数仅用于配置 Navigate 的参数
    LOG(INFO) << "接收到来自navigate的参数！！！";
    navigate_.setParam(name, value);
}

void NavigationModule::onDebugCmdMsg(sros::core::base_msg_ptr m) {
    auto msg = dynamic_pointer_cast<CommandMsg>(m);
    //    LOG(INFO) << "NavigationModule 收到调试指令: " << msg->command;

    if (msg->command == CMD_ENABLE_OBSTACLE_AVOID) {
        obstacle_avoid_enable_ = true;
    } else if (msg->command == CMD_DISABLE_OBSTACLE_AVOID) {
        obstacle_avoid_enable_ = false;
    }
}

void NavigationModule::checkFeatureRecognizeLoop(sros::core::base_msg_ptr m) {
    if(!g_state.enable_station_feature_recognize) {
        return;
    }

    double dist_to_end = 0;
    dist_to_end = g_src_state.cur_remain_distance;
    LOG_EVERY_N(INFO,10) << "checkFeatureRecognizeLoop, dist_to_end:" << dist_to_end;
    //距离目标站点1m的路径上开启特征对接
    if ((dist_to_end < 100) && (dist_to_end > 10)) {
        if (enable_feature_detect) {
            LOG(INFO) << "开启目标站点特征信息识别功能";
            auto msg = std::make_shared<sros::core::CommonCommandMsg<std::string>>("TOPIC_EXTRACT_COMMAND");
            msg->command = "START_EXTRACTOR";
            msg->str0 = g_state.feature_mark.feature_name;
            msg->str1 = g_state.feature_mark.sensor_name;
            msg->pose.x() = g_state.feature_mark.pos_x;
            msg->pose.y() = g_state.feature_mark.pos_y;
            msg->pose.z() = g_state.feature_mark.pos_z;
            msg->pose.yaw() = g_state.feature_mark.pos_yaw;
            msg->pose.pitch() = g_state.feature_mark.pos_pitch;
            msg->pose.roll() = g_state.feature_mark.pos_roll;

            LOG(INFO) << "站点特征名称:" << g_state.feature_mark.feature_name;
            LOG(INFO) << "站点特征传感器:" << g_state.feature_mark.sensor_name;
            LOG(INFO) << "站点特征信息的坐标x:" << g_state.feature_mark.pos_x 
                << ", y:" << g_state.feature_mark.pos_y
                << ", z:" << g_state.feature_mark.pos_z
                << ", yaw:" << g_state.feature_mark.pos_yaw
                << ", pitch:" << g_state.feature_mark.pos_pitch
                << ", roll:" << g_state.feature_mark.pos_roll;

            sendMsg(msg);
            enable_feature_detect = false;
        }
    } else {
        enable_feature_detect = true;
    }
}

void NavigationModule::checkRackDetectLoop(sros::core::base_msg_ptr m) {
    bool enable_rack_query =
        (Settings::getInstance().getValue<std::string>("main.enable_rack_query", "False") == "True");
    if (!enable_rack_query) {
        return;
    }

    double dist_to_end = 0;
    dist_to_end = g_src_state.cur_remain_distance;
    //  LOG(INFO)<<"接受到的距离终点的值:"<<g_src_state.cur_remain_distance ;
    //距离终点200cm以内.并且在最后一条直线上才开启.如果最后一条是原地旋转则在倒数第二条路径上.
    // g_src_state.cur_path_no //当前路径编号.
    int cur_path_no = (int)g_src_state.cur_path_no;
    size_t path_size = cur_paths_.size();

    if (path_size >= 2) {
        if (cur_paths_.back().type_ == PATH_ROTATE && cur_paths_.back().direction_ == 0x21 &&
            cur_paths_[path_size - 2].type_ == PATH_LINE && cur_path_no == path_size - 1) {
            if ((dist_to_end < 200) && (dist_to_end > 30)) {
                if (enable_rack_detect_) {
                    LOG(INFO) << "开启货架中心位置识别功能";
                    auto mm = std::make_shared<sros::core::CommonCommandMsg<std::string>>("RACK_QUERY_COMMAND");
                    mm->command = "RACK_DETECT";
                    mm->pose.x() = cur_dst_pose_.x();
                    mm->pose.y() = cur_dst_pose_.y();
                    mm->pose.yaw() = cur_dst_pose_.yaw();

                    LOG(INFO) << "目标点的坐标x:" << cur_dst_pose_.x();
                    LOG(INFO) << "目标点的坐标y:" << cur_dst_pose_.y();
                    LOG(INFO) << "目标点的坐标yaw:" << mm->pose.yaw();

                    sendMsg(mm);
                    enable_rack_detect_ = false;
                }
            } else {
                enable_rack_detect_ = true;
            }
        }
    }
}

void NavigationModule::onRackMsg(sros::core::base_msg_ptr m) {
    auto cmd = std::dynamic_pointer_cast<sros::core::CommonCommandMsg<std::vector<RackInfo>>>(m);

    LOG(INFO) << "给src发送的货架的中心位置x:" << cmd->pose.x();
    LOG(INFO) << "给src发送的货架的中心位置y:" << cmd->pose.y();
    LOG(INFO) << "给src发送的货架的中心位置yaw:" << cmd->pose.yaw();

    //将接受到的值发送给src.如果没有接受到值,则不发送.
    //    std::vector<int>  value;
    //    value.clear();
    //    value.push_back(int(cmd->pose.x()*1000));
    //    value.push_back(int(cmd->pose.y()*1000));
    //    value.push_back(int(cmd->pose.yaw()/3.1415f*1800));
    Eigen::Vector3f target_pose(cmd->pose.x(), cmd->pose.y(), cmd->pose.yaw());
    if (target_pose.norm() > 0.001f) {
        recomputEnterRackPath(target_pose);  //重新计算路径
        src_sdk->setPgvExecInfo(cmd->pose.x(), cmd->pose.y(), cmd->pose.yaw(), 1);
    } else {
        src_sdk->setPgvExecInfo(0, 0, 0, 0);
    }

}

void NavigationModule::onReleaseMap(sros::core::base_msg_ptr m) {
    //清空地图数据
    navigate_.releaseMapData();

    //将当前地图信息初始化，使下次导航任务能读取新的地图
    cur_map_info_ = MapInfo();
}

bool NavigationModule::checkStationIsRackStation() {
    string rack_location_stations_str = Settings::getInstance().getValue<string>("main.rack_location_stations", "");
    vector<int> rack_location_stations = spilt_strs_to_ints(rack_location_stations_str, ',');

    auto iter = find(rack_location_stations.begin(), rack_location_stations.end(), cur_dst_station_no_);
    if (iter != rack_location_stations.end()) {
        return true;
    } else {
        return false;
    }
}

void NavigationModule::onFeatureRecognitionMsg(sros::core::base_msg_ptr m) {
    auto msg = dynamic_pointer_cast<CommonCommandMsg<NavigationCommand>>(m);
    Pose navFinishedPos;

    switch (msg->command) {
        case COMMAND_NAV_FIND_FEATURE_RESULT: {
            LOG(INFO) << "COMMAND_NAV_FIND_FEATURE_RESULT";

            // 站点检测失败
            if (!msg->param0) {
                station_check_state_ = STATION_CHECK_NONE;
                updateState(STATE_NAV_IDLE);
                break;
            }

            // 已检测到目标点
            station_check_state_ = STATION_CHECK_CHECKED;

            auto mark_group = navigate_.getNavigationMap()->getStationMarkGroup();
            auto dst_station = mark_group->getItem(cur_dst_station_no_);
            Pose f_pose = msg->pose;  // feature pose

            if (charge_station_no_ == cur_dst_station_no_ && charging_pile_type_ == "angle_charging_type") {
                double station_offset = dst_station.station_offset / 100;

                // 根据feature pose和offset计算得到站点的pose
                sros::map::MarkPointf e(cos(f_pose.yaw()), sin(f_pose.yaw()));  // 单位方向向量
                sros::map::MarkPointf delta_p(station_offset * e.x, station_offset * e.y);
                sros::map::MarkPointf p(f_pose.x() + delta_p.x, f_pose.y() + delta_p.y, f_pose.yaw());

                sros::map::MarkPointf new_pose(p.x * 100, p.y * 100, p.yaw);

                // 根据检测结果重新设置站点信息
                if (dst_station.no_rotate) {
                    // 如果站点不可旋转, 需要重新计算进入点与离开点坐标
                    double delta_yaw;

                    // 进入点的朝向与站点朝向关系, 如果后退进入则与站点朝向相同,否则相反
                    delta_yaw = dst_station.enter_backward ? 0 : M_PI;
                    dst_station.enter_pos =
                        getNewEnterExitPos(dst_station.pos, new_pose, dst_station.enter_pos, delta_yaw);

                    // 离开点的朝向与站点朝向关系, 如果后退进入则与站点朝向相反,否则相同
                    delta_yaw = dst_station.exit_backward ? M_PI : 0;
                    dst_station.exit_pos =
                        getNewEnterExitPos(dst_station.pos, new_pose, dst_station.exit_pos, delta_yaw);
                }
                dst_station.pos = new_pose;
                mark_group->updateItem(dst_station.id, dst_station);

                // 生成到目标站点的路径
                Pose nav_dst_pose;                         // 导航任务的最终pose
                sros::core::Location_Vector empty_vector;  // 传入空的vector。
                nav_dst_pose.x() = dst_station.pos.x;
                nav_dst_pose.y() = dst_station.pos.y;
                nav_dst_pose.yaw() = dst_station.pos.yaw;

                updateCurrPath(getNavigationPath(0,
                                                 cur_dst_station_no_,   // 目标站点编号

                                                 src_sdk->getCurPose(),  // 当前位姿

                                                 nav_dst_pose,          // 目标位姿
                                                 cur_avoid_policy_,     // 整体避障策略
                                                 navFinishedPos, empty_vector));
                cur_dst_pose_ = navFinishedPos;

            } else if (checkStationIsRackStation()) {
                sros::map::MarkPointf new_pose(f_pose.x() * 100, f_pose.y() * 100, f_pose.yaw());

                if (dst_station.no_rotate) {
                    // 如果站点不可旋转, 需要重新计算进入点与离开点坐标
                    double delta_yaw;

                    // 进入点的朝向与站点朝向关系, 如果后退进入则与站点朝向相同,否则相反
                    delta_yaw = dst_station.enter_backward ? 0 : M_PI;
                    dst_station.enter_pos =
                        getNewEnterExitPos(dst_station.pos, new_pose, dst_station.enter_pos, delta_yaw);

                    // 离开点的朝向与站点朝向关系, 如果后退进入则与站点朝向相反,否则相同
                    delta_yaw = dst_station.exit_backward ? M_PI : 0;
                    dst_station.exit_pos =
                        getNewEnterExitPos(dst_station.pos, new_pose, dst_station.exit_pos, delta_yaw);
                }
                // cannot direct to dst pose ,it is better to go to entrance point first.
                Pose entrancePoint, entrancePointPart1RealPos;  // 导航任务的最终pose
                entrancePoint.x() = (new_pose.x + dynamicPointToCheckPointLength * cos(new_pose.yaw + M_PI)) / 100.0;
                entrancePoint.y() = (new_pose.y + dynamicPointToCheckPointLength * sin(new_pose.yaw + M_PI)) / 100.0;
                entrancePoint.yaw() = -new_pose.yaw;
                Pose dstPos;
                dstPos.x() = new_pose.x / 100.0;
                dstPos.y() = new_pose.y / 100.0;
                dstPos.yaw() = dst_station.pos.yaw;
                LOG(INFO) << "new_pose.x " << new_pose.x << " new_pose.y() " << new_pose.y << " new_pose.yaw "
                          << new_pose.yaw;

                LOG(INFO) << "entrancePoint.x " << entrancePoint.x() << " entrancePoint.y() " << entrancePoint.y()
                          << " entrancePoint.yaw " << entrancePoint.yaw();
                sros::core::Location_Vector empty_vector;  // 传入空的vector。
                sros::core::NavigationPath_vector cur_paths_1, cur_paths_2, cur_path_dst;
                cur_paths_1 = getNavigationPath(0,
                                                0,                     // 目标站点编号

                                                src_sdk->getCurPose(),  // 当前位姿
                                                entrancePoint,         // 目标位姿
                                                cur_avoid_policy_,     // 整体避障策略
                                                entrancePointPart1RealPos, empty_vector);
                if (!cur_paths_1.empty() && cur_paths_1.back().type_ == PATH_ROTATE) {
                    cur_paths_1.pop_back();
                }

                cur_paths_2 = getNavigationPath(0,
                                                0,                          // 目标站点编号
                                                entrancePointPart1RealPos,  // 当前位姿
                                                dstPos,                     // 目标位姿
                                                cur_avoid_policy_,          // 整体避障策略
                                                navFinishedPos, empty_vector);

                cur_path_dst.insert(cur_path_dst.end(), cur_paths_1.begin(), cur_paths_1.end());
                cur_path_dst.insert(cur_path_dst.end(), cur_paths_2.begin(), cur_paths_2.end());
                updateCurrPath(cur_path_dst);

                debugOutputPathsInWorldMap(cur_paths_);

                //                LOG(INFO)<<"display cur_paths " << " cur_paths_.size " << cur_paths_.size();
            } else if (dst_station.id != 0 && dst_station.pos_dynamic && cur_dst_station_no_ == charge_station_no_ &&
                       charging_pile_type_ == "flat_surface_charging_type") {
                auto src_state = src_sdk->getSRCState();
                auto src_pose = src_sdk->getCurPose();
                int cur_path_no = src_state.path_no;  // 获取当前执行路径编号
                auto cur_path = cur_paths_[cur_path_no];
                LOG(INFO) << " f_pose.x() " << f_pose.x() << " f_pose.y() " << f_pose.y() << " f_pose.yaw() "
                          << f_pose.yaw();
                LOG(INFO) << " src_pose.x() " << src_pose.x() << " src_pose.y() " << src_pose.y() << " src_pose.yaw() "
                          << src_pose.yaw();
                sros::core::Pose pose = src_sdk->getCurPose();
                //                if((src_pose.yaw()-f_pose.yaw())>60.0f/180.0*M_PI) {
                //                    //角度偏差過大
                //                    LOG(ERROR) << "please setting the direction correctly;";
                //                }else{
                if (local_real_time_slam_state_ == LocalRealTimeSlamState::REAL_TIME_SLAM_STATE_IDLE) {
                    charging_pile_pose_ = f_pose;  //////////////////!!!!!!!!!!!!!!!!!!
                    // 旋转180
                    double rotate_value = dst_station.pos.yaw;
                    cur_paths_.clear();
                    //                    cur_paths_.push_back(RotatePath(pose.x(), pose.y(), rotate_value));
                    auto mm = make_shared<SlamCommandMsg>();
                    mm->slam_command = COMMAND_START_LOCAL_REAL_TIME_SLAM_LOCATION;
                    mm->pose = src_pose;  // 初始位姿
                    sendMsg(mm);
                    LOG(INFO) << "new a rotate path "
                              << " src_pose.x() " << src_pose.x() << " src_pose.y() " << src_pose.y()
                              << " src_pose.yaw() " << src_pose.yaw();
                    local_real_time_slam_state_ = LocalRealTimeSlamState::REAL_TIME_SLAM_STATE_IN_USE;

                    double charging_pile_normal_angle = charging_pile_pose_.yaw();
                    auto current_pos = src_sdk->getCurPose();
                    double charging_pile_center_to_current_pose_angle =
                        atan2((charging_pile_pose_.y() - current_pos.y()), (charging_pile_pose_.x() - current_pos.x()));
                    double delta_angle = charging_pile_center_to_current_pose_angle - charging_pile_normal_angle;
                    normalizeAngle(delta_angle);
                    LOG(INFO) << "charging_pile_center_to_current_pose_angle "
                              << charging_pile_center_to_current_pose_angle << "charging_pile_normal_angle "
                              << charging_pile_normal_angle;
                    auto charging_pile_angle = M_PI + charging_pile_pose_.yaw();
                    normalizeAngle(charging_pile_angle);
                    Eigen::Affine2f origin_tf(Eigen::Translation2f(charging_pile_pose_.x(), charging_pile_pose_.y()) *
                                              Eigen::Rotation2Df(charging_pile_angle));

                    {
                        // 位置调整位置 之后倒退
                        // 走到充电站点正前方1.5m出

                        Eigen::Vector3f adjust_delta_pose(adjust_to_charging_point_length_, 0, 0);

                        auto adjust_pose_point = origin_tf * adjust_delta_pose.head<2>();
                        sros::core::Pose adjust_pose;
                        adjust_pose.x() = adjust_pose_point[0];
                        adjust_pose.y() = adjust_pose_point[1];
                        adjust_pose.yaw() = charging_pile_angle;

                        Eigen::Vector3f dst_delta_pose(charging_point_to_center_length_, 0, 0);
                        auto dst_pose_point = origin_tf * dst_delta_pose.head<2>();

                        sros::core::Pose dst_pose;
                        dst_pose.x() = dst_pose_point[0];
                        dst_pose.y() = dst_pose_point[1];
                        dst_pose.yaw() = charging_pile_angle;

                        Pose navFinishedPos;
                        sros::core::Location_Vector empty_vector;  // 传入空的vector。
                        sros::core::NavigationPath_vector paths_to_charging_pile;
                        paths_to_charging_pile = getNavigationPath(0,
                                                                   0,                  // 目标站点编号
                                                                   current_pos,        // 当前位姿
                                                                   adjust_pose,        // 目标位姿
                                                                   cur_avoid_policy_,  // 整体避障策略
                                                                   navFinishedPos, empty_vector);
                        paths_to_charging_pile.push_back(sros::core::LinePath(
                            adjust_pose.x(), adjust_pose.y(), dst_pose.x(), dst_pose.y(), PATH_BACKWARD));
                        //                        cur_paths_ = paths_to_charging_pile;
                        updateCurrPath(paths_to_charging_pile);
                        debugOutputPathsInWorldMap(cur_paths_);
                        station_check_state_ = STATION_CHECK_NONE;
                        LOG(INFO) << "charging_pile_pose_x " << charging_pile_pose_.x() << " charging_pile_pose_ y "
                                  << charging_pile_pose_.y() << " charging_pile_angle " << charging_pile_angle;
                    }
                }
            }

            // 更新站点坐标

            // 更新MarkGroup中目标站点信息, 不保存到文件中

            // 生成并发送路径msg
            auto path_msg = make_shared<PathMsg>("NAV_PATH");
            path_msg->paths = cur_paths_;
            sendMsg(path_msg);  // 即使path列表是空也要发送

            if (path_msg->paths.empty()) {  // 若无法到达目标点,直接结束导航任务
                LOG(INFO) << "NavigationModule 无法到达目标点,直接结束导航任务";
                station_check_state_ = STATION_CHECK_NONE;
                updateState(STATE_NAV_IDLE);
            } else {
                // 更新当前导航任务信息
                cur_dst_pose_ = navFinishedPos;

                LOG(INFO) << "NavigationModule 等待路径执行结束...";
                updateState(STATE_NAV_WAITING_FOR_FINISH);
            }
            break;
        }
        default:
            break;
    }
}

// 将新的模拟车走过的地方发出来，给chip显示
void NavigationModule::newCarSimulateMovePoses(Polygons &&car_poses) {
    auto common_poses_info_msg = make_shared<sros::core::CommonPosesInfoMsg>("TOPIC_COMMON_POSES_INFO");
    common_poses_info_msg->type = CommonPosesInfoMsg::Type::AVOID_OBSTACLE;
    common_poses_info_msg->car_simulate_poses = std::move(car_poses);
    common_poses_info_msg->sensor_name = "";
    //    LOG(INFO)<<"car pose size:"<<common_poses_info_msg->car_simulate_poses.size();
    sendMsg(common_poses_info_msg);
}

sros::map::MarkPointf NavigationModule::getNewEnterExitPos(sros::map::MarkPointf old_pose,
                                                           sros::map::MarkPointf new_pose,
                                                           sros::map::MarkPointf enter_exit_pos,
                                                           double delta_yaw) const {
    // 新的朝向单位向量
    double ex = cos(new_pose.yaw + delta_yaw);
    double ey = sin(new_pose.yaw + delta_yaw);

    // 计算原进入点/离开点向量的长度
    double dx = enter_exit_pos.x - old_pose.x;
    double dy = enter_exit_pos.y - old_pose.y;
    double d = ::std::sqrt(dx * dx + dy * dy);

    // 根据路径长度和方向得到新坐标
    enter_exit_pos.x = ex * d + new_pose.x;
    enter_exit_pos.y = ey * d + new_pose.y;
    enter_exit_pos.yaw = new_pose.yaw + delta_yaw;

    return enter_exit_pos;
}

void debugOutputPathsInWorldMap(sros::core::NavigationPath_vector paths) {
    int seq = 0;
    printf("------------displayPath-------------------\n");
    for (auto p : paths) {
        if (p.type_ == sros::core::PATH_LINE) {
            printf("seq: %d line:(%f, %f) --> (%f, %f)   direction %d \n", ++seq, p.sx_, p.sy_, p.ex_, p.ey_,
                   p.direction_);
        } else if (p.type_ == sros::core::PATH_ROTATE) {
            printf("seq: %d Rotate: %lf\n", ++seq, p.rotate_angle_);
        } else if (p.type_ == sros::core::PATH_ARC) {
            printf("seq: %d Arc:(%f, %f) --> (%f, %f), center:(%f %f), arc_radius: %lf  direction %d  \n", ++seq, p.sx_,
                   p.sy_, p.ex_, p.ey_, p.cx_, p.cy_, p.radius_, p.direction_);
        } else if (p.type_ == sros::core::PATH_BEZIER) {
            printf("seq: %d Control_Point:(%f, %f)(%f %f)(%f %f)(%f %f), radius: %lf    direction %d \n", ++seq, p.sx_,
                   p.sy_, p.cx_, p.cy_, p.dx_, p.dy_, p.ex_, p.ey_, p.radius_, p.direction_);
        }
    }
}

void NavigationModule::resetAvoidObaPara() {
    avoidoba_para.reset(new avoidoba::AvdobaModulePara);
    auto &s = Settings::getInstance();
    avoidoba_para->forward_slow_distance = s.getValue("nav.slow_distance", 1.5);
    avoidoba_para->forward_stop_distance = s.getValue("nav.stop_distance", 1.0);
    avoidoba_para->backward_stop_distance = s.getValue("nav.backward_stop_distance", 0.5);
    avoidoba_para->car_body_width = s.getValue("nav.vehicle_width", 0.5);
    avoidoba_para->car_body_length = s.getValue("nav.vehicle_length", 0.7);
    avoidoba_para->rack_body_width = s.getValue("rack.max_contour_width", 600) * MM_TO_M;
    avoidoba_para->rack_body_length = s.getValue("rack.max_contour_length", 1200) * MM_TO_M;
    avoidoba_para->slow_width_offset = s.getValue("nav.slow_width_offset", 0.25);
    avoidoba_para->stop_width_offset = s.getValue("nav.stop_width_offset", 0.02);
    avoidoba_para->stop_length_forward_offset = s.getValue("nav.stop_forward_offset", 0.05);  //旋转时,前侧余量,防止压脚
    avoidoba_para->delta_stop_time_stamp_in_us = Settings::getInstance().getValue<double>("nav.detect_obstacle_stop_time", 0.5)*1.0e6;
    avoidoba_para->station_stop_offset = Settings::getInstance().getValue("nav.station_stop_offset",
                                                                                 0.05);  //旋转时,前侧余量,防止压脚
    auto motion_type = Settings::getInstance().getValue("src.continuous_mode",6);
    avoidoba_para->vehicle_type = Settings::getInstance().getValue<std::string>("main.vehicle_type","oasis");
    if (motion_type == 24) {//动作类型24为叉车
        LOG(INFO) << "will change to gulf!";
        avoidoba_para->vehicle_type = "gulf";
    }
    LOG(INFO) << "vehicle type：" << avoidoba_para->vehicle_type;
    avoidoba_para->enable_auto_enhance_oba_width =
        (s.getValue<string>("nav.enable_auto_enhance_oba_width", "False") == "True");
    avoidoba_para->deviation_dist_thresh = s.getValue("nav.deviation_dist_thresh", 0.5);  //旋转时,前侧余量,防止压脚
    avoidoba_para->deviation_large_head_stop_offset =
        s.getValue("nav.deviation_large_head_stop_offset", 1.0);  //旋转时,前侧余量,防止压脚
    avoidoba_para->deviation_large_back_stop_offset =
        s.getValue("nav.deviation_large_back_stop_offset", 0.5);  //旋转时,前侧余量,防止压脚
    avoidoba_para->deviation_large_width_offset =
        s.getValue("nav.deviation_large_width_offset", 0.5);  //旋转时,前侧余量,防止压脚
    avoidoba_para->oba_laser_range_max = s.getValue("obstacle.oba_laser_range_max", 5.0);

    auto vehicle_asymmetric_offset_str = s.getValue<string>("nav.vehicle_asymmetric_offset", "0;0;0;0;");
    auto rack_asymmetric_offset_str = s.getValue<string>("rack.rack_asymmetric_offset", "0;0;0;0;");
    auto computeAsymmetricPara = [](const std::string &para_str, const int para_num = 4) {
        auto asymmetric_paras = avoidoba::splitStrsToDoubles(para_str, ';');
        int asymmetric_para_size = asymmetric_paras.size() < para_num ? asymmetric_paras.size() : para_num;
        std::vector<double> vehicle_paras(para_num, 0);
        for (int i = 0; i < asymmetric_para_size; ++i) {
            vehicle_paras[i] = asymmetric_paras[i];
        }
        return vehicle_paras;
    };
    const int para_num = 4;
    auto vehicle_paras = computeAsymmetricPara(vehicle_asymmetric_offset_str, para_num);
    auto rack_paras = computeAsymmetricPara(rack_asymmetric_offset_str, para_num);
    load_free_asymetric_region.reset(new avoidoba::RegionAsymmetricOffsetPara);
    load_full_asymetric_region.reset(new avoidoba::RegionAsymmetricOffsetPara);
    load_free_asymetric_region->forward_offset = vehicle_paras[0];
    load_free_asymetric_region->backward_offset = vehicle_paras[1];
    load_free_asymetric_region->left_offset = vehicle_paras[2];
    load_free_asymetric_region->right_offset = vehicle_paras[3];

    load_full_asymetric_region->forward_offset = rack_paras[0];
    load_full_asymetric_region->backward_offset = rack_paras[1];
    load_full_asymetric_region->left_offset = rack_paras[2];
    load_full_asymetric_region->right_offset = rack_paras[3];

    LOG(INFO) << "all para:" << avoidoba_para->forward_slow_distance << "," << avoidoba_para->forward_stop_distance
              << "," << avoidoba_para->backward_stop_distance << "," << avoidoba_para->car_body_width << ","
              << avoidoba_para->car_body_length << "," << avoidoba_para->rack_body_width << ","
              << avoidoba_para->rack_body_length << "," << avoidoba_para->slow_width_offset << ","
              << avoidoba_para->stop_width_offset << "," << avoidoba_para->stop_length_forward_offset << ","
              << load_full_asymetric_region->forward_offset << "," << load_full_asymetric_region->backward_offset << ","
              << load_full_asymetric_region->left_offset << "," << load_full_asymetric_region->right_offset;
    carbody_size.reset(new avoidoba::OriginCollideSize);
    carbody_size->length = avoidoba_para->car_body_length;
    carbody_size->width = avoidoba_para->car_body_width;
    carbody_size->car_length = avoidoba_para->car_body_length;
    carbody_size->car_width = avoidoba_para->car_body_width;

    avoidoba_processor.reset(new avoidoba::AvoidobaProcessor(avoidoba_para));
    avoidoba_processor->updateAsymetricOffsetPara(
        load_free_asymetric_region->left_offset, load_free_asymetric_region->right_offset,
        load_free_asymetric_region->forward_offset, load_free_asymetric_region->backward_offset);
    manual_avoidoba_processor.reset(new avoidoba::ManualAvdobaProcessor(avoidoba_para));
//    manu.reset(new avoidoba::AvoidobaProcessor(avoidoba_para));
    manual_avoidoba_processor->updateAsymetricOffsetPara(
        load_free_asymetric_region->left_offset, load_free_asymetric_region->right_offset,
        load_free_asymetric_region->forward_offset, load_free_asymetric_region->backward_offset);
    
    last_load_free_asymetric_region.reset(new avoidoba::RegionAsymmetricOffsetPara);

}

void NavigationModule::updateCurrPath(const NavigationPath_vector &paths) {
    cur_paths_ = paths;
    avoidoba_processor->updatePathInfo(cur_paths_);
}

void NavigationModule::updateObaPoints(Location_Vector &oba_points,
                                       std::map<std::string, std::vector<Eigen::Vector2d>> &points) {
    oba_points.reserve(points.size());
    for (auto &point_iter : points) {
        auto &points_vector = point_iter.second;
        for (auto &point : points_vector) {
            oba_points.push_back(sros::core::Location(point[0], point[1]));
        }
    }
}

NavigationModule::CollideState NavigationModule::computeBlockTypeByCollissionValue(int collision_value) {
    CollideState collide_state;
    const int min_stop_param = 1;  // 等于最小停止参数减去1
    const int max_stop_param = 5;  // 等于最大停止参数

    double min_stop_value = min_stop_param - 1;
    double max_stop_value = max_stop_param;

    collide_state.type = SEGMENT_FREE;
    if (collision_value >= avoidoba_para->min_stop_collide_value) {
        avoidoba::LineInterplation line_iterplation;
        line_iterplation.min_x_ = avoidoba_para->min_stop_collide_value;
        line_iterplation.max_x_ = avoidoba_para->max_stop_collide_value;
        line_iterplation.min_value_ = min_stop_value;
        line_iterplation.max_value_ = max_stop_value;
        double curr_stop_param = line_iterplation.getDegreInterplation(collision_value);
        collide_state.stop_param = floor(curr_stop_param) + 1;
        if (collide_state.stop_param > max_stop_param) {
            collide_state.stop_param = max_stop_param;
        }
        if (collide_state.stop_param < min_stop_param) {
            collide_state.stop_param = min_stop_param;
        }
        //        collide_state.stop_param = 0;
        //        LOG(INFO) << "param:" << collide_state.stop_param;
        collide_state.type = SEGMENT_STOP;
        return collide_state;
    } else if (collision_value >= avoidoba_para->min_slow_collide_value) {
        collide_state.type = SEGMENT_SLOW;
    }
    return collide_state;
}

void NavigationModule::onObaPoints(sros::core::base_msg_ptr m) {
    ObstacleMsg_ptr msg = dynamic_pointer_cast<ObstacleMsg>(m);
    avoidoba::ObstaclePoints_Ptr oba_points(new avoidoba::ObstaclePoints);
    oba_points->obstacle_name = msg->oba_name;
    oba_points->time = msg->time_;
    auto &points = oba_points->points;
    if((msg->oba_name == sros::device::DEVICE_UST_LIDAR_BACK || msg->oba_name == sros::device::DEVICE_LIDAR_1) && !g_state.enable_back_main_laser_oba){
        avoidoba_processor->updateObstaclePoints(oba_points);
        return ;
    }

    points.reserve(msg->point_cloud.size());
    static std::shared_ptr<slam::tf::TFOperator> base_to_world_tf;
    if (!base_to_world_tf) {
        LOG(INFO) << "will creat tf to transform!";
        slam::tf::FrameToFrame base_to_world_frame;
        base_to_world_frame.parent_frame = "world";
        base_to_world_frame.child_frame = "base_link";
        base_to_world_tf.reset(new slam::tf::TFOperator(base_to_world_frame));
    }

    slam::tf::TransForm curr_pose;
    bool filter_max_distance = true;
    if(!base_to_world_tf->lookUpTransForm(m->time_, curr_pose,60000)){
        filter_max_distance = false;
    }
    if (!avoidoba_para) {
        filter_max_distance = false;
    }
    Eigen::Vector2d center_point(curr_pose.position.x(), curr_pose.position.y());
    for (auto &point : msg->point_cloud) {
        auto curr_point = Eigen::Vector2d(point.x(), point.y());
        if (filter_max_distance) {
            if ((curr_point - center_point).norm() > avoidoba_para->oba_laser_range_max) {
                continue;
            }
        }
        points.push_back(curr_point);
    }
    oba_points->is_region_oba = msg->is_region_oba;
    oba_points->oba_state = (avoidoba::ObstaclePoints::ObaState)msg->oba_state;
    //    LOG(INFO)<<"msg name:"<<oba_points->obstacle_name;
    avoidoba_processor->updateObstaclePoints(oba_points);
}

void NavigationModule::convertParticleToPolygon(const Eigen::Vector3d &curr_pose,
                                                const std::shared_ptr<avoidoba::OriginCollideSize> &car_size,
                                                Polygon &polygon) {
    double unit_times = 1000;

    Eigen::Affine2d curr_tf =
        Eigen::Translation2d(curr_pose[0], curr_pose[1]) * Eigen::Rotation2Dd(curr_pose[2] + car_size->rotate_angle);
    Eigen::Vector2d first_corner(car_size->back_length, car_size->left_width);
    Eigen::Vector2d secon_corner(car_size->head_length, car_size->left_width);
    Eigen::Vector2d third_corner(car_size->head_length, car_size->right_width);
    Eigen::Vector2d forth_corner(car_size->back_length, car_size->right_width);

    polygon.clear();
    polygon.reserve(4);
    auto curr_point = curr_tf * first_corner;
    polygon.push_back(point(curr_point[0] * unit_times, curr_point[1] * unit_times));
    curr_point = curr_tf * secon_corner;
    polygon.push_back(point(curr_point[0] * unit_times, curr_point[1] * unit_times));
    curr_point = curr_tf * third_corner;
    polygon.push_back(point(curr_point[0] * unit_times, curr_point[1] * unit_times));
    curr_point = curr_tf * forth_corner;
    polygon.push_back(point(curr_point[0] * unit_times, curr_point[1] * unit_times));
}

void NavigationModule::convertRotateParticleToPolygons(const Eigen::Vector3d &curr_pose, double start_angle,
                                                       double end_angle,
                                                       const std::shared_ptr<avoidoba::OriginCollideSize> &car_size,
                                                       std::vector<Polygon> &polygons) {
    rotate::RotateInfo_Ptr info(new rotate::RotateInfo);
    info->start_angle = start_angle;
    info->end_angle = end_angle;
    info->center_info = curr_pose.head<2>();
    std::vector<Eigen::Vector3d> poses;
    rotate::RotateCurveOperator::splitCurveByStep(info, poses, avoidoba_para->sample_step);
    for (auto &pose : poses) {
        polygons.emplace_back();
        convertParticleToPolygon(pose, car_size, polygons.back());
    }
}

void NavigationModule::navigationModeInit() {
    if (navigation_type_ == sros::core::NAV_TYPE_MAP) {
        src_sdk->sendCommandMsg(COMMAND_SET_NAVALGORITHM, SRC_LOCAL_PLANNER, 0);  //设置为
        navigation_mode_ = sros::core::NAV_MODE_LOCAL_PLAN;
        last_navigation_mode_ = sros::core::NAV_MODE_LOCAL_PLAN;
        //并且去使能避障
    } else {
        src_sdk->sendCommandMsg(COMMAND_SET_NAVALGORITHM, SRC_PATH_FOLLOW, 0);  //设置为
        navigation_mode_ = sros::core::NAV_MODE_PATH;
        last_navigation_mode_ = sros::core::NAV_MODE_PATH;
        //并且打开使能避障。
    }
}

void NavigationModule::startLocalPlanner() {
    if (!enable_free_nav_) {
        return;
    }

    navigationModeInit();
    //sr_local_Planner.update_local_target_point_or_not_ = true;

    // 如果导航方式是固定路网的话，不需要原地旋转。
    // if (navigation_mode_ != sros::core::NAV_MODE_PATH) {
    //     rotate_enable_ = true;
    // } else {
    //     rotate_enable_ = false;
    // }
    //sr_local_Planner.start_rotate_w_ = 0.0;

    //需要在开始时获取一遍地图信息中的主动绕障区域的全部信息。只需要开机获取一次就行了。
    //保存在local_planner_area 中即可。
    local_planner_area_.clear();
    auto area_mark_group = navigate_.getNavigationMap()->getAreaMarkGroup();
    std::vector<sros::map::AreaMark> area_mark;
    area_mark = area_mark_group->getItemList();
    std::vector<sros::map::AreaMark>::iterator it;
    for (it = area_mark.begin(); it != area_mark.end(); it++) {
        if (it->type == sros::map::AreaMark::AREA_TYPE_LOCAL_PLANNER) {
            local_planner_area_.push_back(*it);
        }
    }
    LOG(INFO) << "xxxxxxxxxlocal_planner_area的大小：" << local_planner_area_.size();

    //sr_local_Planner.pathSamples(cur_paths_, cur_dst_pose_);  //根据规划的路径得到一段局部路径。
    //local_navigation_state_ = sros::core::STATE_LOCAL_NAV_RUNNING;
    //sr_local_Planner.end_rotate_enable_ = 1;
}

void NavigationModule::finishLocalPlanner() {
    if (!enable_free_nav_) {
        return;
    }
    //sr_local_Planner.end_rotate_enable_ = 0;
    navigation_type_ = sros::core::NAV_TYPE_NONE;
    //local_navigation_state_ = sros::core::STATE_LOCAL_NAV_IDLE;
    //sr_local_Planner.global_trajectory.clear();
}

//在局部规划和路径跟踪之间切换
// void NavigationModule::changeLocalPlannerOrPathFollow() {
//     if (navigation_mode_ == sros::core::NAV_MODE_LOCAL_PLAN) {
//         if (sr_local_Planner.checkLocalPlannerToPathFollow()) {
//             LOG(INFO) << "从局部规划切换到了固定路径～～～～～～～～～～～～～～～";
//             navigation_mode_ = sros::core::NAV_MODE_PATH;
//         }
//     }
//     // 如果配置文件在有货时关闭避障，且运行时有货则不切换到局部规划。

//     if (navigation_mode_ == sros::core::NAV_MODE_PATH) {
//         if (sr_local_Planner.checkPathFollowToLocalPlanner()) {
//             LOG(INFO) << "从固定路径切换到了局部规划～～～～～～～～～～～～～～～";
//             navigation_mode_ = sros::core::NAV_MODE_LOCAL_PLAN;
//             sr_local_Planner.local_rotate_end_ = false;
//             //如果处于path_pause状态下  则直接进入到继续执行状态。
//         }
//     }
//     if (navigation_mode_ != last_navigation_mode_) {
//         if (navigation_mode_ == sros::core::NAV_MODE_PATH) {
//             src_sdk->sendCommandMsg(COMMAND_SET_NAVALGORITHM, SRC_PATH_FOLLOW, 0);  //设置为
//         } else {
//             LOG(INFO) << "向main模块发送waiting for finish 指令。````````::" << g_state.nav_state;
//             if (g_state.nav_state == STATE_NAV_MANUAL_WAITING_FOR_START_PAUSED ||
//                 g_state.nav_state == STATE_NAV_MANUAL_WAITING_FOR_FINISH_SLOW) {
//                 updateState(STATE_NAV_MANUAL_WAITING_FOR_FINISH);
//             } else if (g_state.nav_state == STATE_NAV_WAITING_FOR_FINISH_SLOW ||
//                        g_state.nav_state == STATE_NAV_PATH_PAUSED) {
//                 updateState(sros::core::STATE_NAV_WAITING_FOR_FINISH);
//             }
//             src_sdk->sendCommandMsg(COMMAND_SET_NAVALGORITHM, SRC_LOCAL_PLANNER, 0);  //设置为
//         }
//         last_navigation_mode_ = navigation_mode_;
//     }
// }

void NavigationModule::onOptPoseMsg(sros::core::Pose opt_pose) {
    //LOG(INFO)<<"22222222222222222222222222222: "<<opt_pose.x()<<" "<<opt_pose.y()<<" "<<opt_pose.yaw() ;
    //sr_local_Planner.opt_Pose_Msg(opt_pose);
    planning_fsm_.opt_current_pose_ = opt_pose;
}
void NavigationModule::recomputEnterRackPath(const Eigen::Vector3f target_pose) {
    const auto &curr_pose = src_sdk->getCurPose();
    Eigen::Vector2f curr_point(curr_pose.x(), curr_pose.y());
    Eigen::Vector2f norm(cos(target_pose[2]), sin(target_pose[2]));
    Eigen::Vector2f origin_point = norm.dot(curr_point - target_pose.head<2>()) * norm + target_pose.head<2>();
    Eigen::Vector2f enter_line_vector = origin_point - target_pose.head<2>();
    Eigen::Vector2f verticle_vector = curr_point - origin_point;

    NavigationPath<double> path;
    if (verticle_vector.norm() > 0.1) {
        auto tan_theta = atan2(enter_line_vector.norm(), verticle_vector.norm());
        double sin_value = sin(M_PI - 2 * tan_theta);
        if (sin_value < 0.0001) {
            sin_value = 0.0001;
        }
        auto circle_radius = enter_line_vector.norm() / sin_value;
        auto center_point = circle_radius * verticle_vector / verticle_vector.norm() + target_pose.head<2>();
        auto first_radius_vector = curr_point - center_point;
        auto second_radius_vector = target_pose.head<2>() - center_point;
        auto direction =
            first_radius_vector[0] * second_radius_vector[1] - first_radius_vector[1] * second_radius_vector[0];
        if (direction < 0) {
            circle_radius *= -1.0;
        }
        path.type_ = PathType::PATH_ARC;
        path.direction_ = 1;
        path.cx_ = center_point[0];
        path.cy_ = center_point[1];
        path.radius_ = circle_radius;
        path.sx_ = curr_point[0];
        path.sy_ = curr_point[1];
        path.ex_ = target_pose[0];
        path.ey_ = target_pose[1];
        path.e_facing_ = target_pose[2] + M_PI;

        LOG(INFO) << "radius:" << circle_radius << ",center:" << center_point[0] << "," << center_point[1];
    } else {
        path.type_ = PathType::PATH_LINE;
        path.direction_ = 1;
        path.sx_ = origin_point[0];
        path.sy_ = origin_point[1];
        path.ex_ = target_pose[0];
        path.ey_ = target_pose[1];
        path.e_facing_ = target_pose[2] + M_PI;
    }

    LOG(INFO) << "path info:" << verticle_vector.norm() << "," << origin_point[0] << "," << origin_point[1] << ","
              << target_pose[0] << "," << target_pose[1] << "," << curr_point[0] << "," << curr_point[1];
    auto src_state = src_sdk->getSRCState();
    int cur_path_no = src_state.path_no;  // 获取当前执行路径编号,注意该路径
    LOG(INFO) << "curr path no:" << cur_path_no;
    avoidoba_processor->updateCurrPath(path, src_state.path_no);
}

void NavigationModule::free_nav_replan() {
    // 根据起点  终点 以及实时障碍点 重新规划一条路径。
    if (planning_fsm_.nav_state_ == NAV_STATE::REPLAN_TRAJ) {
        LOG(INFO) << "开始重新规划路径，考虑激光雷达检测到的障碍物。";
        Pose cur_pos = src_sdk->getCurPose();
        COLLISION_OBS obs_pose;
        Location_Vector oba_points;
        avoidoba::AvdobaPoseInfo obstacle_points;
        avoidoba_processor->getObstaclePoints(obstacle_points);
        updateObaPoints(oba_points, obstacle_points.oba_points);
        std::vector<Eigen::Vector2d> vector_obstacle_points;
        avoidoba_processor->getObstaclePoints(vector_obstacle_points);
        LOG(INFO) << "重新规划路径的起点坐标：x:" << cur_pos.x() << ";y:" << cur_pos.y();
        LOG(INFO) << "重新规划路径的终点坐标：x:" << cur_dst_pose_.x() << ";y:" << cur_dst_pose_.y();

        // 重新规划从起点到终点的全局路径，和初始规划不同的就是考虑了临时障碍点。
        cur_paths_ = navigate_.replanPath(cur_pos,cur_dst_pose_,NAV_PATH_TYPE::LINE_ROTATE_TYPE,oba_points);
        if(cur_paths_.empty())
        {
            navigate_.replanPath_del_obs();
            LOG(INFO) << "重新规划全局路径失败" ;
            return;
        }

        // 对全局路径的全面部分 采用hybrid进行规划。并合并后优化，最后完成路径。
        for (auto path : cur_paths_) {
            LOG(INFO) << "重新规划路径类型：" << path.type_ << ";sx:" << path.sx_ << ";sy:" << path.sy_
                      << ";ex:" << path.ex_ << ";ey:" << path.ey_;
        }

        // 此处需要考虑当cur_paths_。size == 1 时 且为原地旋转的情况。
        if(cur_paths_.size()==1&&cur_paths_[0].type_ == PathType::PATH_ROTATE){
            auto path_msg = make_shared<PathMsg>("NAV_PATH");
            path_msg->paths = cur_paths_;
            sendMsg(path_msg);
            updateState(STATE_NAV_WAITING_FOR_FINISH);

            planning_fsm_.resetNavState(target_vel_);
            planning_fsm_.exec_step_ = END_ROTATE;
            planning_fsm_.changeNavState(EXEC_TRAJ);
            LOG(INFO)<<"重新规划只有一条原地旋转。";
            return;
        }

        planning_fsm_.replan_free_path(navigate_, cur_paths_, cur_pos, global_map_,
                                         oba_points,cur_dst_pose_, final_path_);

        // 完成路径重规划之后，计算总长度。如果和之前差别太大则直接返回。
        double all_path_dist = planning_fsm_.calc_path_dist1(final_path_);
        if((all_path_dist-planning_fsm_.remain_dist)>nav_cfg_.replan_dist_error){
            LOG(INFO)<<"!!!!!!重规划的路径与剩余路径长度差大于"<<nav_cfg_.replan_dist_error<<"m,新规划的总距离:"<<all_path_dist<<"m;之前剩余距离："<<planning_fsm_.remain_dist<<"m.";
            return;
        } else{
            LOG(INFO)<<"!!!!!!重规划的路径与剩余路径长度差小于"<<nav_cfg_.replan_dist_error<<"m,新规划的总距离:"<<all_path_dist<<"m;之前剩余距离："<<planning_fsm_.remain_dist<<"m.";
        }
        if(planning_fsm_.check_replan_path_collosion(navigate_,final_path_,global_map_,obs_pose)){
            // 将obs_pose 发送出去。
            sendObs( obs_pose);
            return ;
        }
        planning_fsm_.ConvertType(cur_paths_,final_path_);
        planning_fsm_.loadPaths(final_path_,opt_paths_);
        planning_fsm_.traj_follow_init( cur_pos,opt_paths_);
        

        // 将final_path.格式转换为cur_paths_格式。


        auto path_msg = make_shared<PathMsg>("NAV_PATH");
        path_msg->paths = cur_paths_;
        sendMsg(path_msg);
        updateState(STATE_NAV_WAITING_FOR_FINISH);

        planning_fsm_.resetNavState(target_vel_);
        planning_fsm_.changeNavState(EXEC_TRAJ);
        LOG(INFO) << " 重新规划了路径 并且上传！！";
    }
}

void NavigationModule::free_nav_check_obstacle() {
    auto current_pose = src_sdk->getCurPose();
    Location_Vector oba_points1;
    avoidoba::AvdobaPoseInfo obstacle_points1;
    avoidoba_processor->getObstaclePoints(obstacle_points1);
    updateObaPoints(oba_points1, obstacle_points1.oba_points);  //
    std::vector<Eigen::Vector2d> cur_obstalces;
    avoidoba_processor->getObstaclePoints(cur_obstalces);
//    std::map<std::string,std::vector<Eigen::Vector2d>> all_obstalces;
//    avoidoba_processor->avoidoba_manager->getObstaclePoints(all_obstalces);

    global_map_.update_collision_map(navigate_ , obstacle_points1.oba_points);
    COLLISION_OBS obs_pose;
    if(planning_fsm_.nav_state_ ==NAV_STATE::EXEC_TRAJ||planning_fsm_.nav_state_==NAV_STATE::TRAJ_PAUSE){


        // 如果此处有原地旋转，如果有障碍物，则暂停。如果没有且在暂停状态 则继续执行。
        // 关于原地旋转直接采用30度为step计算 未来点是否有障碍物。

        if ((planning_fsm_.exec_step_ == EXEC_STEP::START_ROTATE ||
             planning_fsm_.exec_step_ == EXEC_STEP::END_ROTATE)) {
            bool rotate_collision_check;
            if (planning_fsm_.exec_step_ == EXEC_STEP::START_ROTATE) {
                float angle_error = planning_fsm_.start_rotate_angle - current_pose.yaw();
                angle_error = normalizeYaw(angle_error);
                for (int i = 1; i < 11; i++) {
                    float check_angle = planning_fsm_.start_rotate_angle + i * 1.0f / 10.0f * angle_error;
                    //Pose check_pose = current_pose;
                    Pose check_pose = opt_paths_[0];
                    check_pose.yaw() = normalizeYaw(check_angle);
                    rotate_collision_check = global_map_.check_point_collision(navigate_ ,check_pose,nav_cfg_.robot.length,nav_cfg_.robot.width,obs_pose);
                    if(rotate_collision_check) {
                        LOG(INFO)<<"非终点原地旋转遇到障碍物：此时AGV预测的角度为："<< check_pose.yaw();
                        sendObs(obs_pose);
                        break;}
                }

            } else if (planning_fsm_.exec_step_ == EXEC_STEP::END_ROTATE) {
                float angle_error = planning_fsm_.end_rotate_angle - current_pose.yaw();
                angle_error = normalizeYaw(angle_error);
                for (int i = 1; i < 11; i++) {
                    float check_angle = planning_fsm_.end_rotate_angle + i * 1.0f / 10.0f * angle_error;
                    //Pose check_pose = current_pose;
                    Pose check_pose = opt_paths_[opt_paths_.size()-1];
                    check_pose.yaw() = normalizeYaw(check_angle);
                    rotate_collision_check = global_map_.check_point_collision(navigate_ ,check_pose,nav_cfg_.robot.length,nav_cfg_.robot.width,obs_pose);
                    if(rotate_collision_check) { 
                        LOG(INFO)<<"终点原地旋转遇到障碍物：此时AGV预测的角度为："<< check_pose.yaw();
                        sendObs(obs_pose);
                        break;}
                }
            }

            // 如果在执行原地旋转，并且当前位置有障碍物。则进入暂停状态。
            if (rotate_collision_check) {
                LOG(INFO) << "原地旋转出现障碍物";
                planning_fsm_.changeNavState(NAV_STATE::TRAJ_PAUSE);
                //updateState(STATE_NAV_PATH_PAUSED, 1);
            } else {
                if (planning_fsm_.nav_state_ == NAV_STATE::TRAJ_PAUSE) {
                    updateState(STATE_NAV_WAITING_FOR_FINISH);
                    planning_fsm_.changeNavState(NAV_STATE::EXEC_TRAJ);

                }
            }
        }

        if (planning_fsm_.check_collosion(current_pose, navigate_, opt_paths_, global_map_,obs_pose)) {

            sendObs(obs_pose);
            LOG(INFO)<<"！！！！！！！！！！！障碍物距离"<<planning_fsm_.obstacle_dist_;
            if(planning_fsm_.obstacle_dist_>1.5&&planning_fsm_.obstacle_dist_<2.5&&planning_fsm_.nav_state_ ==NAV_STATE::EXEC_TRAJ&&planning_fsm_.exec_step_ == EXEC_STEP::FOLLOW_TRAJ)
            {
                // 先屏蔽掉这个功能
                //return;

                LOG(INFO)<<"自由导航 障碍物离agv距离在1.5-2.0之间:"<<planning_fsm_.obstacle_dist_;
                // 由于规划局部路径可能出现10s都无法规划出的问题，导致障碍物检测堵塞而撞到障碍物。
                // 新建一个线程用于规划局部路径。



                // 采用jsp+astar和teb 规划出一条局部路径。
                // 先计算需要规划的起点和终点坐标。
                Pose start_pose, end_pose;
                int start_index,end_index;

                int64_t time_first = sros::core::util::get_time_in_ms();
                planning_fsm_.traj_follow_init1( current_pose,opt_paths_);
                LOG(INFO)<<"当检测到1.5外障碍物时，AGV当前坐标：x："<<current_pose.x()<<";y:"<<current_pose.y()<<";yaw:"<<current_pose.yaw();
                LOG(INFO)<<"此时，min_index 和min pose 分别为："<<planning_fsm_.path_index_<<";x:"<<opt_paths_[planning_fsm_.path_index_].x()<<
                ";y:"<<opt_paths_[planning_fsm_.path_index_].y()<<";x:"<<planning_fsm_.min_pose_.x()<<";y:"<<planning_fsm_.min_pose_.y();
                planning_fsm_.calc_local_start_end_point(opt_paths_, start_pose, end_pose,start_index,end_index);

                 // 由于calc_local_start_end_point计算的终点坐标有可能会超过终点，所以打算用如下方式获取局部终点坐标。
                 // 主要是为了计算前视目标点而定的。
                 end_pose.x() = opt_paths_[end_index].x();
                 end_pose.y() = opt_paths_[end_index].y();

                 LOG(INFO)<<"local.start_pose:"<<start_pose.x()<<":"<<start_pose.y()<<":"<<start_pose.yaw();
                 LOG(INFO)<<"local.end_pose:"<<end_pose.x()<<":"<<end_pose.y()<<":"<<end_pose.yaw();
                 LOG(INFO)<<"当前agv的位置:"<<current_pose.x()<<":"<<current_pose.y()<<":"<<current_pose.yaw();
                 LOG(INFO) << "开始优化局部轨迹!" ;


                int64_t time_first2 = sros::core::util::get_time_in_ms();
                NavigationPath_vector tmp_path = navigate_.replanPath(start_pose,end_pose,NAV_PATH_TYPE::LINE_ROTATE_TYPE,oba_points1,false,true);
                int64_t time_second2 = sros::core::util::get_time_in_ms();
                LOG(INFO)<<"``````````````````````````````a star 规划不停车全局时间;"<<time_second2-time_first2<<" ms";
                //LOG(INFO)<<"1111111111"<<end_index<<";"<<opt_paths_.size()-1;
                if(tmp_path.empty()||end_index == opt_paths_.size()-2) 
                {
                    LOG(INFO)<<"无法规划出合理的路径";
                    planning_fsm_.obstacle_dist_ = 1.5;
                    planning_fsm_.changeNavState(NAV_STATE::TRAJ_PAUSE);
                    return;
                }
                vector<sros::core::Pose> initial_plan;
                initial_plan.clear();
                // 打印规划的轨迹
                for(auto path:tmp_path)
                {
                    if(path.type_==1)
                    {
                        if(!initial_plan.empty())
                        {
                            initial_plan.pop_back();
                        }

                        sros::core::Pose point(path.sx_,path.sy_);
                        initial_plan.push_back(point);
                        sros::core::Pose point1(path.ex_,path.ey_);
                        initial_plan.push_back(point1);
                        LOG(INFO)<<"规划的不停车局部轨迹的起点x：;"<<path.sx_<<":y:"<<path.sy_<<"终点坐标x："<<path.ex_<<":y:"<<path.ey_;
                    }
                }

               // planning_fsm_.path_smoother_.addObstacleFromGridMap( navigate_,cur_obstalces);
                // 往gridmap中添加障碍点。并在路径优化结束之后 删除这些障碍点。

                planning_fsm_.path_smoother_.pathSample(tmp_path,navigate_);
                planning_fsm_.path_smoother_.convertPath(navigate_,tmp_path,local_opt_paths_);

                std::vector<Node3D> path_1,path_2;
                Node3D first_node;
                first_node.setX(opt_paths_[start_index-1].x());
                first_node.setY(opt_paths_[start_index-1].y());
                first_node.setPrim(1);
                path_2.push_back(first_node);
                for(auto point:local_opt_paths_){
                    Node3D tmp_node;
                    tmp_node.setX(point.x());
                    tmp_node.setY(point.y());
                    tmp_node.setPrim(1);
                    path_2.push_back(tmp_node);
                }
                Node3D second_node;
                second_node.setX(opt_paths_[end_index+1].x());
                second_node.setY(opt_paths_[end_index+1].y());
                second_node.setPrim(1);
                path_2.push_back(second_node);

                int offset_x = navigate_.getNavigationMap()->getMapZeroOffsetX();
                int offset_y = navigate_.getNavigationMap()->getMapZeroOffsetY();
                for(auto point:path_2) {
                    Node3D traj_point ;
                    int x,y;
                    navigate_.convertMapCoords(offset_x,offset_y,point.getX(),point.getY(),&x,&y);

                    traj_point.setX(x);
                    traj_point.setY(y);
                    path_1.push_back(traj_point);
                }

                // for(auto point:path_1){

                //     LOG(INFO)<<"tiaoshidian x："<<point.getX()<<";y:"<<point.getY();
                // }

                planning_fsm_.path_smoother_.setPath(path_1);
                // 需要将首尾两个点添加进去。保证优化的结果满足要求
                planning_fsm_.path_smoother_.smoothPath(global_map_);
//                LOG(INFO) << "完成轨迹优化!" ;
//
                //planning_fsm_.path_smoother_.deleteObstacleFromGridMap( navigate_,cur_obstalces);

                planning_fsm_.path_smoother_.convertPath(navigate_,tmp_path,local_opt_paths_);

                for(auto point:local_opt_paths_){
                    LOG(INFO)<<"不停车局部路径经过优化后的点的坐标为x："<<point.x()<<";y:"<<point.y();
                }

                 for(int i= end_index+2;i<opt_paths_.size();i++){
                     local_opt_paths_.push_back(opt_paths_[i]);
                 }

                // 用于检查规划的局部路径是否满足要求。1，不允许有后退。2，相邻两点距离不能太小。
                if(!planning_fsm_.check_path_feasibility(local_opt_paths_)){
                    planning_fsm_.obstacle_dist_ = 1.5;
                    planning_fsm_.changeNavState(NAV_STATE::TRAJ_PAUSE);
                    return;
                }
                update_path_flag_ = 1;

                 opt_paths_.erase(opt_paths_.begin()+start_index-1,opt_paths_.end());
                 for(auto point:local_opt_paths_){
                     opt_paths_.push_back(point);
                 }
                 tmp_path.clear();
                 for(int i=0;i<opt_paths_.size()-1;i++){
                     auto xxx =  sros::core::LinePath(opt_paths_[i].x(),opt_paths_[i].y(),opt_paths_[i+1].x(),opt_paths_[i+1].y(),PATH_FORWARD);
                     tmp_path.push_back(xxx);
                 }
                update_path_flag_ = 0;
                
                for(auto point:opt_paths_){
                    LOG(INFO)<<"合并后的路径x:"<<point.x()<<";y:"<<point.y()<<";yaw:"<<point.yaw();
                }
                

                int64_t time_second = sros::core::util::get_time_in_ms();
                LOG(INFO)<<"``````````````````````````````不停车绕障规划时间ms;"<<time_second-time_first;
               // sr_local_Planner.RecordGlobalPath(opt_paths_);

                // 此处需要判断 如果障碍物很杂乱导致局部路径路径长度变化较大， 存在局部的曲率变化较大 则直接转为重新全局规划。



                auto path_msg = make_shared<PathMsg>("NAV_PATH");
                path_msg->paths = tmp_path;
                sendMsg(path_msg);

            } else if (planning_fsm_.obstacle_dist_ <= 1.5 && planning_fsm_.exec_step_ == EXEC_STEP::FOLLOW_TRAJ) {
                // 如果检测到障碍物。则进入到暂停状态。
                LOG(INFO) << "自由导航前方遇到障碍物障碍物距离小于1.5m,暂停." << planning_fsm_.obstacle_dist_;
                planning_fsm_.changeNavState(NAV_STATE::TRAJ_PAUSE);
                //updateState(STATE_NAV_PATH_PAUSED, 1);
            }
        } else if (planning_fsm_.nav_state_ == NAV_STATE::TRAJ_PAUSE &&
                   planning_fsm_.exec_step_ == EXEC_STEP::FOLLOW_TRAJ) {
            LOG(INFO) << "自由导航前方障碍物消失,继续运行.";
            planning_fsm_.changeNavState(NAV_STATE::EXEC_TRAJ);
            updateState(STATE_NAV_WAITING_FOR_FINISH);
        }
    }

}

void NavigationModule::sendObs(const avoidoba::CollideResultInfo& collide_info) {
    auto mm = make_shared<ObstacleMsg>("AVOID_OBSTACLES");
    mm->oba_name = collide_info.oba_name;
    sros::core::Location tmp_pose(collide_info.collide_point[0],collide_info.collide_point[1]);
    mm->point_cloud.push_back(tmp_pose);
    mm->is_region_oba = collide_info.is_region_oba;
    sros::core::MsgBus::sendMsg(mm);
}

void NavigationModule::sendObs(const COLLISION_OBS& obs_pose){

    auto mm = make_shared<ObstacleMsg>("AVOID_OBSTACLES");
    mm->oba_name = obs_pose.obs_name;
    sros::core::Location tmp_pose(obs_pose.obs_pose.x(),obs_pose.obs_pose.y());
    LOG(INFO)<<"11111111发送出去的障碍点坐标和类型.x:"<<tmp_pose.x()<<";y:"<<tmp_pose.y()<<";device_name:"<<mm->oba_name;
    mm->point_cloud.push_back(tmp_pose);
    sros::core::MsgBus::sendMsg(mm);
}

void NavigationModule::publishUstPoints() {
    if(avoidoba_processor){
        std::map<std::string,std::vector<Eigen::Vector2d>> obstacles;
        avoidoba_processor->getObstaclePoints(obstacles,false,false);
        sros::core::ObstacleMsg_ptr ust_oba(new sros::core::ObstacleMsg("TOPIC_OBSTACLE"));
        ust_oba->oba_name = "R2100_OBA";
        ust_oba->point_cloud.reserve(3600);
        ust_oba->is_real_time_ = true;
        for (auto &obstacle_iter : obstacles) {
            auto &obstacle = obstacle_iter.second;
            if(obstacle_iter.first!="LASER_OBA"){
                for (auto &point : obstacle) {
                    ust_oba->point_cloud.emplace_back(point[0], point[1]);
                }
            }
        }
        sendMsg(ust_oba);
    }
}


template <class AvdProcessor>
void NavigationModule::updateAvdPara(AvdProcessor &processor,
                                     std::shared_ptr<avoidoba::OriginCollideSize> &car_body_size,bool &curr_syn_rotate_state) {
    
    auto &s = Settings::getInstance();
    auto slow_width_offset = s.getValue("nav.slow_width_offset", 0.25);
    auto stop_width_offset = s.getValue("nav.stop_width_offset", 0.02);
    auto stop_length_forward_offset = s.getValue("nav.stop_forward_offset", 0.05);  //旋转时,前侧余量,防止压脚

    if(avoidoba_para->vehicle_type == "oasis"){
        //更新O车的增量
        processor->updateOffsetPara(slow_width_offset, stop_width_offset, stop_length_forward_offset);
        // 支持在线修改offset
        int action_mode = g_state.getActionControllerType();
        bool is_load_rack = true;
        if (action_mode != ACTION_CONTROLLER_TYPE_SRC_PUTTER_JACKING &&
            action_mode != ACTION_CONTROLLER_TYPE_SRC_JACKING_ROTATE) {
            // 如果动作机构不是顶升货架类机构，则不启用货架腿滤除功能
            bool enable_load_rack =
                Settings::getInstance().getValue<std::string>("rack.enable_load_rack", "False") == "True";
            //            LOG(INFO) << "src.continuous_mode isn't 4 or 6, disable rack leg filter function";
            if (enable_load_rack) {
                //                LOG(INFO) << "enable load rack!";
            } else {
                is_load_rack = false;
            }
        }

        if (is_load_rack && (g_state.load_state == sros::core::LOAD_FULL)) {
            curr_syn_rotate_state = g_state.sync_rotate;
            double rotate_angle = (double)g_state.rotate_value / 1000.0;
            if(avoidoba_para->vehicle_type == "gulf"){ //TO DELETE
                rotate_angle = 0;
            }
            auto rack_length = Settings::getInstance().getValue("rack.max_contour_length", 1200) * MM_TO_M;
            auto rack_width = Settings::getInstance().getValue("rack.max_contour_width", 600) * MM_TO_M;
            auto rack_op = rack::RackOperatorInstance::getInstance();
            if (rack_op) {
                auto rack_info = rack_op->getRackInfo();
                if (rack_info) {
                    rack_length = rack_info->avd_oba_length;
                    rack_width = rack_info->avd_oba_width;
                    
                }
            }
            car_body_size->rotate_angle = rotate_angle;
            car_body_size->length = rack_length;
            car_body_size->width = rack_width;
            processor->updateAsymetricOffsetPara(
                load_full_asymetric_region->left_offset, load_full_asymetric_region->right_offset,
                load_full_asymetric_region->forward_offset, load_full_asymetric_region->backward_offset);
        } else {
            car_body_size->length = Settings::getInstance().getValue("nav.vehicle_length", 0.7);
            car_body_size->width = Settings::getInstance().getValue("nav.vehicle_width", 0.5);
            car_body_size->rotate_angle = 0.0;
            processor->updateAsymetricOffsetPara(
                load_free_asymetric_region->left_offset, load_free_asymetric_region->right_offset,
                load_free_asymetric_region->forward_offset, load_free_asymetric_region->backward_offset);
        }
    }
    
    //TODO:如果当前处于栈板对接的状态，需要启用以下功能：
    std::string type = avoidoba_para->vehicle_type.substr(0,4);
    if(type == "gulf" || type == "Gulf"){
        // 取放货避障
        {   
            if(is_enable_avd){
                static bool temp_flag = false;
                //car_body_size->no_avoid_oba_back_length = fork_arm_length; // 这里需要确认一下
                auto curr_pose = src_sdk->getCurPose();
                double distance = curr_pose.distance_to(cmd_position); 
                LOG(INFO) << "stop distance: " << distance;
                if (forklift_state == DetectAndAvdObaState::AVDOBA_STAGE_FETCH_ON){
                    avoidoba_para->forward_stop_distance = distance;
                    avoidoba_para->backward_stop_distance = distance;
                    processor->updatePara();
                    //forklift_state = DetectAndAvdObaState::CHANGE_MODEL_FINISH;
                }else if (forklift_state == DetectAndAvdObaState::AVDOBA_STAGE_RELEASE_ON){
                    avoidoba_para->forward_stop_distance = distance;
                    avoidoba_para->backward_stop_distance = distance;
                    processor->updatePara();
                    //forklift_state = DetectAndAvdObaState::CHANGE_MODEL_FINISH;
                }else if (forklift_state == DetectAndAvdObaState::ACTION_FINISH || forklift_state == DetectAndAvdObaState::COMMAND_INVALID ){
                    //avoidoba_para->forward_stop_distance = avoidoba_para->origin_forward_stop_distance;
                    avoidoba_para->backward_stop_distance = avoidoba_para->origin_backward_stop_distance;
                    //car_body_size->no_avoid_oba_back_length = 0;
                }else{
                    LOG(INFO) << "DetectAndAvdObaState::CHANGE_MODEL_FINISH";
                }
            }
            
        }

        
        // 避障模型
        {
            static bool enable_change_avd_model = (s.getValue<std::string>("obstacle.enable_change_avd_model", "false") == "True");
            if(enable_change_avd_model){
                car_body_size->length = s.getValue("nav.vehicle_length", 0.7);
                car_body_size->width = s.getValue("nav.vehicle_width", 0.5);


                if(forklift_type == AvdObaCommandMsg::FORKLIFT_TYPE::FORKLIFT_1400){
                    if(g_state.load_state == LOAD_FREE){
                        restore_oba_model = true;
                    }else if(g_state.load_state == LOAD_FULL){
                        restore_oba_model = false;  
                    }
                    LOG_EVERY_N(INFO, 100) << "restore_oba_model: " << restore_oba_model << "  forklift_type:"  << forklift_type;
                }else if(forklift_type == AvdObaCommandMsg::FORKLIFT_TYPE::FORKLIFT_3000){
                    LOG_EVERY_N(INFO, 100) << "restore_oba_model: " << restore_oba_model << "  forklift_type:"  << forklift_type;
                }
            
                
                // 脱钩完后恢复模型
                if(restore_oba_model){
                    if(forklift_type == AvdObaCommandMsg::FORKLIFT_TYPE::FORKLIFT_1400){
                        car_body_size->length = car_body_size->length + stop_length_forward_offset;
                        car_body_size->width = car_body_size->width + stop_width_offset;
                        processor->updateAsymetricOffsetPara(
                            load_free_asymetric_region->left_offset, load_free_asymetric_region->right_offset,
                            load_free_asymetric_region->forward_offset, load_free_asymetric_region->backward_offset);
                        LOG_EVERY_N(INFO, 100) << "Restore avd model." << "avoidoba_para->car_body_width: " <<  avoidoba_para->car_body_width
                                                        << "avoidoba_para->car_body_length: " << avoidoba_para->car_body_length;
                    }else if(forklift_type == AvdObaCommandMsg::FORKLIFT_TYPE::FORKLIFT_3000){

                        load_free_asymetric_region->left_offset = last_load_free_asymetric_region->left_offset;
                        load_free_asymetric_region->right_offset = last_load_free_asymetric_region->right_offset;
                        load_free_asymetric_region->forward_offset = last_load_free_asymetric_region->forward_offset;
                        load_free_asymetric_region->backward_offset = last_load_free_asymetric_region->backward_offset;

                        avoidoba_processor->updateAsymetricOffsetPara(
                                    load_free_asymetric_region->left_offset, load_free_asymetric_region->right_offset,
                                    load_free_asymetric_region->forward_offset, load_free_asymetric_region->backward_offset);
                        processor->updateAsymetricOffsetPara(
                            load_free_asymetric_region->left_offset, load_free_asymetric_region->right_offset,
                            load_free_asymetric_region->forward_offset, load_free_asymetric_region->backward_offset);
                        LOG(INFO) << "Restore avd model.";  

                    }
                    restore_oba_model = false;  
                } 
                else{
                    if(forklift_type == AvdObaCommandMsg::FORKLIFT_TYPE::FORKLIFT_1400){
                        // 1. 针对栈板识别
                        // read the good length and width
                        // if no pallet_id recieved, select the largest one to change avd model    
                        if(g_state.cur_goal_id  == -1){
                            float temp_length = 0.0, temp_width = 0.0;
                            for(int i=0; i<cards_map_.size(); ++i){
                                auto temp_good_info = cards_map_[i];
                                if(temp_width < temp_good_info.getWidth()){
                                    temp_width = temp_good_info.getWidth(); //so far only consider length;
                                    g_state.cur_goal_id  = i;
                                }
                            }
                        }
                        auto good_info = cards_map_[g_state.cur_goal_id];
                        good_length = good_info.getLength();
                        good_width = good_info.getWidth();
                        LOG_EVERY_N(INFO, 100) << "good_length: " << good_length << "good_width: " << good_width; 
                        LOG(INFO) << "avoidoba_para->car_body_length: " << avoidoba_para->car_body_length;
                        LOG(INFO) << "avoidoba_para->car_body_width: " << avoidoba_para->car_body_width;
                        // 判断是否大于当前模型
                        avoidoba_para->stop_length_forward_offset = 0;
                        avoidoba_para->stop_width_offset = 0;
                        if(good_length > car_body_size->length){
                            car_body_size->length = good_length + stop_length_forward_offset;
                        }else{
                            car_body_size->length = car_body_size->length + stop_length_forward_offset;
                        }
                        if(good_width > car_body_size->width){
                            car_body_size->width = good_width + stop_width_offset;
                        }else{
                            car_body_size->width = car_body_size->width  + stop_width_offset;
                        }
                    }else if (forklift_type == AvdObaCommandMsg::FORKLIFT_TYPE::FORKLIFT_3000){
                        if(is_change_oba_model){
                            
                            last_load_free_asymetric_region->forward_offset = load_free_asymetric_region->forward_offset;
                            last_load_free_asymetric_region->backward_offset = load_free_asymetric_region->backward_offset;
                            last_load_free_asymetric_region->left_offset = load_free_asymetric_region->left_offset;
                            last_load_free_asymetric_region->right_offset = load_free_asymetric_region->right_offset;
                            
                            // 2. 针对二维码识别
                            // read the good length and width
                            auto good_info = qrcodes_map_[QR_Code_id];
                            good_length = good_info.getLength();
                            good_width = good_info.getWidth();
                            //LOG(INFO) << "good_length: " << good_length << "  good_width: " << good_width; 
                            
                            // 判断是否大于当前模型
                            avoidoba_para->stop_length_forward_offset = 0;
                            avoidoba_para->stop_width_offset = 0;
                            
                            load_free_asymetric_region->backward_offset += good_length;
                            // avoidoba_processor->updateAsymetricOffsetPara(
                            //     load_free_asymetric_region->left_offset, load_free_asymetric_region->right_offset,
                            //     load_free_asymetric_region->forward_offset, load_free_asymetric_region->backward_offset);
                            //car_body_size->length = good_length + stop_length_forward_offset;
                            
                            if(good_width > car_body_size->width){
                                load_free_asymetric_region->left_offset += (car_body_size->width - good_width)/2 + stop_width_offset;
                                load_free_asymetric_region->right_offset += (car_body_size->width - good_width)/2+ stop_width_offset ;
                                
                                avoidoba_processor->updateAsymetricOffsetPara(
                                    load_free_asymetric_region->left_offset, load_free_asymetric_region->right_offset,
                                    load_free_asymetric_region->forward_offset, load_free_asymetric_region->backward_offset);
                                   
                            }else{
                                car_body_size->width +=  stop_width_offset;
                                avoidoba_processor->updateAsymetricOffsetPara(
                                    load_free_asymetric_region->left_offset, load_free_asymetric_region->right_offset,
                                    load_free_asymetric_region->forward_offset, load_free_asymetric_region->backward_offset);
                                   
                            }

                            *load_free_asymetric_region -= *last_load_free_asymetric_region;
                            
                            is_change_oba_model = false;
                        }
                        
                    }
                    
                    
                }
                
            }
        } 
        
    
    }

}

void NavigationModule::updateStopDistByVel(const sros::core::Velocity& vel,double &stop_dist_offset, double &slow_dist_offset) {
    auto &s = Settings::getInstance();
    auto slow_distance = s.getValue("nav.slow_distance", 1.5);
    auto stop_distance = s.getValue("nav.stop_distance", 1.0);
    auto slow_distance_range_1 = s.getValue("nav.slow_distance_range_1", 1.5);
    auto stop_distance_range_1 = s.getValue("nav.stop_distance_range_1", 1.0);
    auto slow_distance_range_2 = s.getValue("nav.slow_distance_range_2", 1.5);
    auto stop_distance_range_2 = s.getValue("nav.stop_distance_range_2", 1.0);
    
    auto velocity = std::sqrt(vel.vx() * vel.vx() + vel.vy() * vel.vy())/1000;
    std::string type = avoidoba_para->vehicle_type.substr(0,4);
    LOG_EVERY_N(INFO, 100) << "velocity: " << velocity;
    if(type == "gulf" || type == "Gulf"){
        if (slow_distance_range_1 < slow_distance || slow_distance_range_2 < slow_distance_range_1){
                LOGGER(ERROR, SROS) << "Setting slow distance error";
            }
        if ( stop_distance_range_1 < stop_distance || stop_distance_range_2 < stop_distance_range_1){
                LOGGER(ERROR, SROS) << "Setting stop distance error";
            }

        // different speed response to different stop distance and slow distance
        if(velocity >= 0 && velocity < 0.5){
            return; // no need to change any stop or slow distance
        }else if(velocity >= 0.5 && velocity <= 1.0){
           stop_dist_offset = stop_distance_range_1 - avoidoba_para->origin_forward_stop_distance;
           slow_dist_offset = slow_distance_range_1 - avoidoba_para->origin_forward_slow_distance;
        }else if(velocity > 1.0){
            stop_dist_offset = stop_distance_range_2 - avoidoba_para->origin_forward_stop_distance;
            slow_dist_offset = slow_distance_range_2 - avoidoba_para->origin_forward_slow_distance;
        }else{
            LOG(INFO) << "forklift velocity is wrong!" << velocity;
        }
    }else{
        // for oasis, using ratio to change slow distance 
        auto slow_ratio = s.getValue("nav.increase_slow_dist_every_velocity", 1.0);
        if (velocity > 3.0) {
            LOG(INFO) << "oasis velocity is wrong!" << velocity;
            velocity = 3.0;
        }
        slow_dist_offset = velocity * slow_ratio;
    }
    
}

void NavigationModule::gulfAvdobaCmdMsg(sros::core::base_msg_ptr m){

    LOG(INFO) << "recieve gulf forklift avoid obstacle command.";

    AvdObaCommandMsgPtr cmd = std::dynamic_pointer_cast<AvdObaCommandMsg>(m);

    auto &s = sros::core::Settings::getInstance();

    avd_type = cmd->command.avdoba_type;
    
    // 判断避障功能 1. 取放货避障 2.避障模型更改
    if(avd_type == AvoidObstacleFunctionType::FETCH_RELEASE_AVD){
        forklift_state = cmd->command.avdoba_state;
    
        cmd_path_ = cmd->command.paths;

        cmd_position = cmd->command.goal_position;

        is_enable_avd = cmd->command.is_enable;

        fork_arm_length = s.getValue("forklift.fork_arm_length", 0.9);
        
        LOG(INFO) << " gulf forklift avoid obstacle state: " << forklift_state;

        LOG(INFO) << " gulf forklift avoid obstacle target position: " << "x: " << cmd_position.x()
                  << "y: " << cmd_position.y() << "yaw: " << cmd_position.yaw();

        // enable o3d camera
        if (forklift_state == DetectAndAvdObaState::AVDOBA_STAGE_FETCH_ON){
            LOG(INFO)  << "fetch goods avd; enable o3d camera";
            std::shared_ptr<sros::core::CommonMsg> cmd_msg(new sros::core::CommonMsg("TOPIC_SVC100_ENABLE_PUBLISH"));
            cmd_msg->str_0_ = sros::device::DEVICE_CAMERA_O3D303;
            cmd_msg->flag = true;
            sendMsg(cmd_msg);    
        }else if (forklift_state == DetectAndAvdObaState::AVDOBA_STAGE_RELEASE_ON){
            LOG(INFO)  << "release goods avd; enable back laser";
            auto msg = std::make_shared<sros::core::CommonMsg>("TOPIC_BACK_LASER_ENABLE_PUBLISH");
            msg->str_0_ = sros::device::DEVICE_UST_LIDAR_BACK;
            msg->flag = true;
            sros::core::MsgBus::sendMsg(msg);
        }else{
            // disable 03d camera
            LOG(INFO)  << "change camera state: send TOPIC_SVC100_DISABLE_PUBLISH msg";
            std::shared_ptr<sros::core::CommonMsg> cmd_msg(new sros::core::CommonMsg("TOPIC_SVC100_ENABLE_PUBLISH"));
            cmd_msg->str_0_ = sros::device::DEVICE_CAMERA_O3D303;
            cmd_msg->flag = false;
            sendMsg(cmd_msg);
        } 

    }else if(avd_type == AvoidObstacleFunctionType::CHANGE_MDOEL_AVD){

        is_change_oba_model = cmd->command.oba_model.is_change_oba_model;
        
        restore_oba_model = cmd->command.oba_model.restore_oba_model;

        forklift_type = cmd->command.oba_model.forklif_type;

        LOG(INFO) << "recieve avoid obstacle command--> is_change_oba_model: " << is_change_oba_model
                  << "  restore_oba_model: " << restore_oba_model;

        // 有二维码的读取二维码信息，无二维码需要识别货物大小
        if(forklift_type == AvdObaCommandMsg::FORKLIFT_TYPE::FORKLIFT_1400){
            pallet_id = cmd->command.oba_model.pallet_id;
            LOG(INFO) << "pallet_id is: " << pallet_id;
        }else if (forklift_type == AvdObaCommandMsg::FORKLIFT_TYPE::FORKLIFT_3000){
            QR_Code_id = cmd->command.oba_model.QR_Code_id;
            LOG(INFO) << "QR_Code_id is: " << QR_Code_id;
        }else{
            LOG(INFO) << "INVALID FORKLIFT TYPE";
        }
        
    }
    else{
         LOG(INFO) << "avdoba type invalid ";
    }
    
    
}

}  // namespace nav
