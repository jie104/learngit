//
// Created by yj on 19-12-2.
//

#ifndef SROS_PLANNING_FSM_H
#define SROS_PLANNING_FSM_H

#include <vector>
#include "path_smooth/vector2d.h"
#include "path_smooth/smoother.h"
#include "hybrid_astar/hybrid_a_star.h"
#include "path_follow/path_follow.h"
#include "hybrid_astar/node3d.h"
#include "local_map.h"
#include "check_collision.h"
#include "speed_plan.h"
using namespace sros::core;


enum NAV_STATE
{
    INIT,
    WAIT_GOAL,
    GEN_TRAJ,
    REPLAN_TRAJ,
    EXEC_TRAJ,
    TRAJ_PAUSE
};
enum EXEC_STEP{
    EXEC_INIT,
    START_ROTATE,
    FOLLOW_TRAJ,
    END_ROTATE
};


class PlanningFSM{
public:
    PlanningFSM(){}
    ~PlanningFSM(){}
    void planning_fsm_init();
    void initMap(Navigation& nav,NavConfig& cfg);
    void navigateFSM(Pose current_pose,Velocity current_velocity,std::vector<HybridAStar::Node3D>& paths,Pose goal_pose,std::vector<Pose>& followed_path,Velocity& target_velocity);
    void changeNavState(NAV_STATE target_state);
    void printExecState();

    void resetNavState(Velocity& target_velocity);

    void traj_follow_init(Pose current_pose,std::vector<Pose>& paths);
    void traj_follow_init1(Pose current_pose,std::vector<Pose>& paths);

    bool pure_pursuit(Pose current_pose,
                       Velocity current_velocity,
                       std::vector<Pose>& paths,
                       Velocity& target_velocity);

    void pause_pure_pursuit(Pose current_pose,
        Velocity current_velocity,
        std::vector<Pose>& paths,
        Velocity& target_velocity);

    bool loadPaths(std::vector<HybridAStar::Node3D>& all_paths,std::vector<Pose>&  followed_path);

    bool exec_traj(Pose current_pose,Velocity current_velocity,std::vector<HybridAStar::Node3D>& all_paths,std::vector<Pose>& paths,Pose goal_pose,Velocity& target_velocity);

    bool check_collosion(Pose current_pose,Navigation& nav,vector<Pose> paths,CheckCollision& check_collision,COLLISION_OBS& obs_pose);

    bool check_replan_path_collosion(Navigation& nav,vector<HybridAStar::Node3D> paths,CheckCollision& check_collision,COLLISION_OBS& obs_pose);

    bool calc_local_start_end_point(std::vector<Pose>& paths,Pose& start_pose,Pose& end_pose,int& start_index,int& end_index);

    void calc_path_max_v(float& max_v,float max_w,std::vector<Pose>& paths,int first_index,int second_index);


    //void calc_free_path(Navigation& navigate_,sros::core::NavigationPath_vector& global_path,Pose cur_pose,std::vector<HybridAStar::Node3D>& all_paths);

    void replan_free_path(Navigation& navigate_,sros::core::NavigationPath_vector& global_path,Pose cur_pose,
    CheckCollision& obs_map, Location_Vector& oba_points,Pose dst_pose,std::vector<HybridAStar::Node3D>& all_paths);


    //void calc_smooth_path(Navigation& navigate_,std::vector<Eigen::Vector2d>& oba_points,uint8_t replan_or_not,sros::core::NavigationPath_vector& global_path,std::vector<HybridAStar::Node3D>& all_paths);

        bool check_path_feasibility(vector<Pose>& opt_paths);
    bool check_path_collision(int path_index,Pose min_pose,std::vector<Pose>& paths,Navigation& nav,float check_dist,float& obstacle_dist,CheckCollision& check_collision,COLLISION_OBS& obs_pose);

    double calc_path_dist1(std::vector<HybridAStar::Node3D>& path);

    void CalcSpeed(vector<Vector2D>& way_points);

    void calc_max_vel(vector<Pose>& way_points);

    void CalcSpeed(vector<Pose>& way_points);

    double CalcTargetSpeedBaseProfile(double current_time,std::vector<Pose>&  followed_path);

    void ConvertType(NavigationPath_vector& cur_paths,vector<HybridAStar::Node3D>& final_path);


    void calc_velocity_profile(std::vector<Pose>&  followed_path,
                                            int path_index,
                                            vector<double> times_profile,
                                            vector<double> max_vel_profile_,
                                            vector<vector<double>>&  poly_coff);

    double calc_target_vel(std::vector<Pose>&  followed_path,vector<double> times_profile,double dist_to_goal);

    void calc_velocity_profile1(std::vector<Pose>&  followed_path,
                                             vector<double> times_profile,
                                             vector<double> max_vel_profile_,
                                             vector<double> vel_profile,
                                             double& target_v,
                                             double& target_a
    );


    bool  process_rotate(double &w, double&w_a,double current_angle, double target_angle, double max_w,
                                      double acc_w);

    NAV_STATE nav_state_;
    bool has_goal_;
    double max_w_;
    double acc_w_;
    double run_period_;

    float obstacle_dist_; //当前位置到在路径上离最近障碍物的距离。
    int path_index_;
    Pose min_pose_;
    Pose opt_current_pose_; //从src上传回来的经过滤波后的位置。

    HybridAStar::hybrid_a_star a_star;
    HybridAStar::hybrid_a_star a_star1;

    sros::core::Velocity target_velocity_;
    EXEC_STEP exec_step_ ;
    PathFollow pathfollow_;
    uint8_t no_replan_pause;

    sros::core::Velocity last_target_velocity_;

    Smoother  path_smoother_;
    double remain_dist;// 表示规划好路径之后。剩余路径长度。 如果当前规划与上一次剩余路径差别很大则不考虑该路径。
    double dist_to_goal=0; // 每一段路径的剩余距离。
    double seg_dist=0; // 每一段路径的距离。
    double remain_seg_dist=0; // 下段总距离。

    bool direction_; //当前跟踪轨迹的移动方向 true 表示前进 false表示后退。
     double  start_rotate_angle; // 起点原地旋转的目标角度。
    double  end_rotate_angle; // 终点原地旋转的目标角度。

    NavConfig* cfg_;
    bool enable_backward_;

    vector<double> times_profile_;
    vector<double> vel_profile_;
    vector<double> max_vel_profile_;

    vector<vector<double>>  poly_coff_;

    std::vector<sros::map::AreaMark> slow_velocity_area_;

         VectorXd coff1_;
};






#endif //SROS_PLANNING_FSM_H
