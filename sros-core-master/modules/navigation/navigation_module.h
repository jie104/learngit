/**
 * @file navigation_module.h
 *
 * @author lhx
 * @date 16-1-28.
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */
#ifndef MODULES_NAVIGATION_NAVIGATION_MODULE_H_
#define MODULES_NAVIGATION_NAVIGATION_MODULE_H_

#include <vector>

#include "lib/include/navigation.h"

#include "core/core.h"
#include "core/map_manager.h"
#include "core/msg/common_poses_info_msg.hpp"
#include "core/msg/gulf_avdoba_msg.hpp"
#include "qrcode.hpp"
#include "core/pose.h"
#include "core/msg_bus.h"
#include "costmap/costmap_model.h"
#include "../perception/common/common_func.hpp"
#include "path_smooth/smoother.h"
#include "planning_fsm.h"
#include "nav_config.h"
#include "check_collision.h"
#include "../perception/lib/object_detectors/card_detector/card.hpp"
#include"avoid_oba/particle_info_msg.hpp"


namespace avoidoba {
class AvoidobaProcessor;
class ManualAvdobaProcessor;

class AvdobaModulePara;

class OriginCollideSize;
}  // namespace avoidoba

namespace nav {

using sros::core::Location;
using sros::core::Location_Vector;
using sros::core::NavigationPath;
using sros::core::NavigationPath_vector;
using sros::core::ObstacleAvoidPolicy;
using sros::core::Pose;
using sros::core::StationNo_t;
using AvdObaCommandMsgPtr = sros::core::AvdObaCommandMsgPtr;   
using DetectAndAvdObaState = sros::core::AvdObaCommandMsg::DetectAndAvdObaState; 
using AvoidObstacleFunctionType = sros::core::AvdObaCommandMsg::AvoidObstacleFunctionType; 

class NavigationModule : public sros::core::Module {
 public:
    struct CollideState {
        BLOCK_TYPE type;
        int stop_param = 0;
    };

    NavigationModule();

    virtual ~NavigationModule();

    void onCommandMsg2(sros::core::base_msg_ptr m);

    void onGenerateNetPath(sros::core::base_msg_ptr m);

    void avoidObaLoop(sros::core::base_msg_ptr m);

    void updateRegionSwitchState(int curr_pose_no);

    void onTimer_100ms(sros::core::base_msg_ptr msg);

    void localPathPlanner(sros::core::base_msg_ptr m);

    void processLocalPlannerPause();

    void initParameter();

    void onParameterMsg(sros::core::base_msg_ptr m);

    void onDebugCmdMsg(sros::core::base_msg_ptr m);

    void onFeatureRecognitionMsg(sros::core::base_msg_ptr m);

    void checkRackDetectLoop(sros::core::base_msg_ptr m);

    void onRackMsg(sros::core::base_msg_ptr m);

    void onReleaseMap(sros::core::base_msg_ptr m);

    void checkFeatureRecognizeLoop(sros::core::base_msg_ptr m);

    virtual void run();

    void updateState(sros::core::NavigationState state, int param_int = 0);
    void sendObs(const COLLISION_OBS& obs_pose);
    void sendObs(const avoidoba::CollideResultInfo& collide_info);

 private:
    template <class AvdProcessor>
    void updateAvdPara(AvdProcessor &processor,std::shared_ptr<avoidoba::OriginCollideSize>& car_body_size,bool &curr_syn_rotate_state);

    void updateStopDistByVel(const sros::core::Velocity& vel,double &stop_dist_offset, double &slow_dist_offset);

    void publishUstPoints();

    void recomputEnterRackPath(const Eigen::Vector3f target_pose);

    void onObaPoints(sros::core::base_msg_ptr m);

    void gulfAvdobaCmdMsg(sros::core::base_msg_ptr m);

    void resetAvoidObaPara();

    void convertToPolygon(avoidoba::AvdobaPoseInfo& avdoba_pose_info,bool curr_syn_rotate_state,Polygons &polygons);

    void updateCurrPath(const NavigationPath_vector &paths);

    CollideState computeBlockTypeByCollissionValue(int collision_value);

    void initNavigation(const std::string &map_name);

    void updateObaPoints(Location_Vector &oba_points, std::map<std::string, std::vector<Eigen::Vector2d>> &points);

    bool checkStationIsRackStation();

    void convertParticleToPolygon(const Eigen::Vector3d &curr_pose,
                                  const std::shared_ptr<avoidoba::OriginCollideSize> &car_size, Polygon &polygon);

    void convertRotateParticleToPolygons(const Eigen::Vector3d &curr_pose, double start_angle, double end_angle,
                                         const std::shared_ptr<avoidoba::OriginCollideSize> &car_size,
                                         std::vector<Polygon> &polygons);

    NavigationPath_vector getNavigationPath(StationNo_t start_station_no, StationNo_t dst_station_no,
                                            Pose input_start_pose, Pose input_dst_pose, ObstacleAvoidPolicy policy,
                                            Pose &out_dst_pose, Location_Vector &radar_points, bool force_nav_on_map = false);

    NavigationPath_vector getNavigationPathOnMap(StationNo_t start_station_no, StationNo_t dst_station_no,
                                                 Pose input_start_pose, Pose input_dst_pose, ObstacleAvoidPolicy policy,
                                                 Pose &out_dst_pose, Location_Vector &radar_points,
                                                 bool allow_backward = false);

    void handleObstacleStop(const ObstacleAvoidPolicy &avoid_policy, Location_Vector &radar_points, int stop_param = 0);

    void free_nav_replan();
    void free_nav_check_obstacle();

    Navigation navigate_;

    PlanningFSM planning_fsm_;

    NAV_PATH_TYPE nav_path_type_;  //产生路径类型

    NavigationPath_vector cur_paths_;
    vector<Pose> opt_paths_;
    int update_path_flag_=0;
    vector<Pose> local_opt_paths_;

    //Smoother  hybrid_smoother_;
    vector<HybridAStar::Node3D> final_path_;

    //局部路径规划相关的变量.
    //std::shared_ptr<TebOptimalPlanner> planner;

    sros::core::NavigationMode navigation_mode_;       // 导航模式
    sros::core::NavigationMode last_navigation_mode_;  // 上一次的导航模式。
    bool rotate_enable_;  // 用来表示使能起步的原地旋转。原地旋转结束后才执行局部规划。
    // double start_rotate_w_; //起步原地旋转角速度。

    std::vector<sros::map::AreaMark> local_planner_area_;

    sros::core::NavigationType navigation_type_ = sros::core::NAV_TYPE_NONE;
    sros::core::LocalNavigationState local_navigation_state_ = sros::core::STATE_LOCAL_NAV_IDLE;  //局部规划运行状态。

    //teb_local_planner::TebLocalPlanner sr_local_Planner;
    NavConfig nav_cfg_;

    CheckCollision global_map_;

    int enter_exit_station_;
    int charge_station_no_;
    int dynamicPointToCheckPointLength;

    bool obstacle_avoid_enable_;  // 是否启用避障, 优先级高于避障策略

    double charging_point_to_center_length_;
    double adjust_to_charging_point_length_;
    StationNo_t cur_start_station_no_;      //当前导航任务的出发站点编号
    StationNo_t cur_dst_station_no_;        //当前导航任务的目标站点编号
    Pose cur_dst_pose_;                     //当前导航任务的出发位姿
    ObstacleAvoidPolicy cur_avoid_policy_;  //当前导航任务的整体避障策略

    int pre_collision_free_cnt_ = 5;
    BLOCK_TYPE pre_collision_;

    ObstacleAvoidPolicy enter_avoid_policy_;  // 进入路径段的避障策略
    ObstacleAvoidPolicy exit_avoid_policy_;   // 离开路径段的避障策略

    sros::core::MapInfo cur_map_info_;  // 当前使用的地图信息

    bool enable_path_oba_;  // 是否启用基于路径检测的避障
    bool enable_free_nav_;  // 是否启用自由导航
    bool enable_net_nav_;   // 是否启用路网导航
    bool enable_rack_detect_ = true;

    bool enable_feature_detect = true;

    /** @brief The state of the forklift. */
    int forklift_state = 0; 

    /** @brief The flag of fetch and release avd. */
    bool is_enable_avd = false;

    /** @brief The length of the forklift arm. */
    float  fork_arm_length = 0.9;

    /** @brief The tatget path */
    NavigationPath_vector cmd_path_;
    
    /** @brief The target of the position. */
    Pose cmd_position;

    /** @brief flag of changing oba model */
    bool is_change_oba_model = false;

    /** @brief flag of restore oba model */
    bool restore_oba_model = false;

    /** @brief The info of the QRCode. */
    std::string QR_Code_id;

    /** @brief The pallet id. */
    int pallet_id;

    /** @brief The map of the qrcode. */
    std::map<std::string, nav::QRCode> qrcodes_map_;

    /** @brief The map of the card. */    
    std::map<int, perception::Card> cards_map_;
    
    /** @brief The type of the avoiding obstacle. */
    int avd_type = 0;

    /** @brief The length of target */
    float good_length = 0;

    /** @brief The width of target */
    float good_width = 0;

    /** @brief The type of forklift */
    int forklift_type = AvdObaCommandMsg::FORKLIFT_TYPE::FORKLIFT_1400; // 默认1400 调试

    enum StationCheckState {
        STATION_CHECK_NONE,     // 空
        STATION_CHECK_WAITING,  // 等待到达Check点
        STATION_CHECK_CHECKED,  // 已经到达CheckPoint
    };

    enum class LocalRealTimeSlamState {
        REAL_TIME_SLAM_STATE_IDLE = 1,
        REAL_TIME_SLAM_STATE_IN_USE = 2,
        REAL_TIME_SLAM_STATE_END = 3
    };
    std::string charging_pile_type_;
    LocalRealTimeSlamState local_real_time_slam_state_;
    StationCheckState station_check_state_;  // 是否正在等待到达Check点
    Pose charging_pile_pose_;

    sros::map::MarkPointf getNewEnterExitPos(sros::map::MarkPointf old_pose, sros::map::MarkPointf new_pose,
                                             sros::map::MarkPointf enter_exit_pos, double delta_yaw) const;

    void newCarSimulateMovePoses(Polygons &&car_poses);

    void onVelocityMsg(sros::core::Velocity vel);
    void onOptPoseMsg(sros::core::Pose opt_pose);
    void onCheckManualVelocity(int& v,int& w);

    void navigationModeInit();
    void startLocalPlanner();
    void finishLocalPlanner();
    void changeLocalPlannerOrPathFollow();

    void pathFollowFunc();

    /** @brief load target infos */
    void loadGoodsParam();

    //MPCcontrol mpc_;
    sros::core::Velocity current_vel_;
    sros::core::Velocity target_vel_;
    sros::core::Velocity final_target_vel_;

 private:
    std::shared_ptr<avoidoba::AvdobaModulePara> avoidoba_para;
    std::shared_ptr<avoidoba::OriginCollideSize> carbody_size;
    std::shared_ptr<avoidoba::AvoidobaProcessor> avoidoba_processor;
    std::shared_ptr<avoidoba::ManualAvdobaProcessor> manual_avoidoba_processor;
    avoidoba::RegionAsymmetricOffsetPara_Ptr load_free_asymetric_region;
    avoidoba::RegionAsymmetricOffsetPara_Ptr last_load_free_asymetric_region;
    avoidoba::RegionAsymmetricOffsetPara_Ptr load_full_asymetric_region;
    CollideState last_collide_state;

    sros::core::ErrorCode failed_code_ = sros::core::ERROR_CODE_NONE;  // 失败代码
    sros::core::Pose localplan_currentpose;                            //局部规划每个周期ａｇｖ的位置。
};

void debugOutputPathsInWorldMap(NavigationPath_vector paths);

} /* namespace nav */

#endif  // MODULES_NAVIGATION_NAVIGATION_MODULE_H_
