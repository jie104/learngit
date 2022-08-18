//
// Created by yj on 19-12-2.
//

#include <modules/navigation/avoid_oba/particle_info_msg.hpp>
#include <modules/navigation/lib/include/geometry.h>
#include <modules/navigation/avoid_oba/avdoba_processor.hpp>
#include <core/src.h>
#include <src/sdk/protocol/src_protocol.h>
#include "planning_fsm.h"
#include "core/util/utils.h"


void PlanningFSM::planning_fsm_init() {
    nav_state_ = INIT;
    exec_step_ = EXEC_INIT ;
    has_goal_ = false;
    max_w_ = 0.6;
    acc_w_ = 0.6;
    run_period_ = 0.02;
    a_star.Init(800,800);
     a_star1.Init(800,800);
}

void PlanningFSM::initMap(Navigation& nav,NavConfig& cfg){
    path_smoother_.initGridMap(nav.getNavigationMap()->getMapSizeX(),nav.getNavigationMap()->getMapSizeY());
    path_smoother_.initNavConfig(cfg);
    cfg_ = &cfg;
    a_star.InitConfig(cfg);
    a_star1.InitConfig(cfg);

    slow_velocity_area_.clear();
    auto area_mark_group = nav.getNavigationMap()->getAreaMarkGroup();
    std::vector<sros::map::AreaMark> area_mark;
    area_mark = area_mark_group->getItemList();
    std::vector<sros::map::AreaMark>::iterator it;
    for (it = area_mark.begin(); it != area_mark.end(); it++) {
        if (it->type == sros::map::AreaMark::AREA_TYPE_SPEED_LEVEL_20||
        it->type == sros::map::AreaMark::AREA_TYPE_SPEED_LEVEL_40||
        it->type == sros::map::AreaMark::AREA_TYPE_SPEED_LEVEL_60||
        it->type == sros::map::AreaMark::AREA_TYPE_SPEED_LEVEL_80
        ) {
            slow_velocity_area_.push_back(*it);
        }
    }

    
    
}

void smoothvelocity(sros::core::Velocity &current_target_velocity,
                                    sros::core::Velocity &last_target_velocity, double acc_v,
                                    double acc_w, double time_inter) {

    sros::core::Velocity target_velocity;
    double dv = current_target_velocity.vx() - last_target_velocity.vx();
    dv = clipDouble(dv, -acc_v * time_inter, acc_v * time_inter);
    target_velocity.vx() = last_target_velocity.vx() + dv;

    double dw = current_target_velocity.vtheta() - last_target_velocity.vtheta();
    dw = clipDouble(dw, -acc_w * time_inter, acc_w * time_inter);

    target_velocity.vtheta() = last_target_velocity.vtheta() + dw;


    last_target_velocity = target_velocity;
    current_target_velocity = target_velocity;
}

void PlanningFSM::calc_path_max_v(float& max_v,float max_w,std::vector<Pose>& paths,int first_index,int second_index)
{
    std::vector<Pose>::iterator it;
    std::vector<Pose>::iterator it_next;

    for(it = paths.begin()+first_index;it<paths.begin()+second_index;it++)
    {
        it_next = (it+1);
        it->yaw() = atan2(it_next->y()-it->y(),it_next->x()-it->x());
    }
    for(it = paths.begin()+first_index;it<paths.begin()+second_index-1;it++)
    {
        Pose start_pose = *it;
        Pose end_pose =  *(it+1);
        float dist = hypot(start_pose.x()-end_pose.x(),start_pose.y()-end_pose.y());
        float dt = dist/max_v;
        float w ;
        w = normalizeYaw(start_pose.yaw()-end_pose.yaw())/dt;
       // LOG(INFO)<<"计算的最大w为："<<w;
        if(fabs(w)>max_w)
        {
            dt =   normalizeYaw(start_pose.yaw()-end_pose.yaw()) /max_w;
            float v = fabs(dist/dt);
            if(v<max_v){
                max_v = v;
//                LOG(INFO)<<"calc_path_max_ start_pose:"<<start_pose.x()<<";"<<start_pose.y()<<";"<<start_pose.yaw();
//                LOG(INFO)<<"calc_path_max_ end_pose:"<<end_pose.x()<<";"<<end_pose.y()<<";"<<end_pose.yaw();

              //  LOG(INFO)<<"first index:"<<first_index <<":second_index:"<<second_index;
              // LOG(INFO)<<"重新计算的最大线速度为："<<v<<":startpose_x:"<<start_pose.x()<<":startpose_y:"<<start_pose.y();
            }

        }
    }
    if(fabs(max_v)<0.1){
        LOG(INFO)<<"最大线速度不能低于0.1："<<max_v;
        max_v = 0.1;

    }

   // LOG(INFO)<<"重新计算的最大线速度为："<<max_v;
}

double PlanningFSM::calc_path_dist1(std::vector<HybridAStar::Node3D>& path){

    if(path.empty()){
        return 0;
    }
    if(path.size()==1)
    {
        return 0;
    }
    HybridAStar::Node3D pose = path[0];
    double dist=0;
    for(auto pose1:path){
        double dist1 = sqrt(pow((pose1.getX()-pose.getX()),2)+pow((pose1.getY()-pose.getY()),2));
        dist +=dist1;
        pose = pose1;
    }
    return dist;

}

double calc_path_dist(std::vector<Pose>& path){

    if(path.empty()){
        return 0;
    }
    if(path.size()==1)
    {
        return 0;
    }
    Pose pose = path[0];
    double dist=0;
    for(auto pose1:path){
        double dist1 = sqrt(pow((pose1.x()-pose.x()),2)+pow((pose1.y()-pose.y()),2));
        dist +=dist1;
        pose = pose1;
    }
    return dist;

}


// 从总的路径中单独取出前进或者后退的路径。
bool PlanningFSM::loadPaths(std::vector<HybridAStar::Node3D>& all_paths,std::vector<Pose>&  followed_path)
{
    remain_seg_dist = calc_path_dist1(all_paths);
    LOG(INFO)<<"进行路径装载时，此时的剩余路径长度："<<remain_seg_dist;
    remain_dist = remain_seg_dist;
    dist_to_goal = 0;
    seg_dist = 0;

    if(all_paths.empty()){
        return false;
    }

    if(all_paths.size()<2)
    {
        return false;
    }

    followed_path.clear();
    uint8_t init_direction;
    LOG(INFO)<<"all_paths[1]:"<<all_paths[1].getPrim()<<";"<<all_paths[1].getX()<<";"<<all_paths[1].getY();
    if(all_paths[1].getPrim()<3)
    {
        init_direction = 1;
    } else if(all_paths[1].getPrim()>2&&all_paths[1].getPrim()<6)
    {
        init_direction = 2;
    }
    else {
        init_direction = 3;
    }
    Pose pose;
    pose.x() = all_paths[0].getX();
    pose.y() = all_paths[0].getY();
    pose.yaw() = all_paths[0].getT();
    followed_path.push_back(pose);


    std::vector<HybridAStar::Node3D>::iterator it;
    for(it = all_paths.begin()+1;it!=all_paths.end();it++){
        uint8_t dir =0;
        if(it->getPrim()<3)
        {
            dir = 1;
        } else if(it->getPrim()>2&&it->getPrim()<6)
        {
            dir = 2;
        }
        else {
            dir = 3;
        }


        if(dir == init_direction)
        {
            pose.x() = it->getX();
            pose.y() = it->getY();
            pose.yaw() = it->getT();
            followed_path.push_back(pose);
        } else
        {
            direction_ = (init_direction==1||init_direction==3);
            all_paths.erase(all_paths.begin(),it-1);
            LOG(INFO)<<"当前路径是前进还是后退：前进为1 后退为0 ："<< direction_;
            for(auto path:followed_path){
                LOG(INFO)<<"分离出来的路径坐标："<< path.x()<<";"<<path.y();
            }
            seg_dist = calc_path_dist(followed_path);
            dist_to_goal = seg_dist;
//            for(auto path:all_paths){
//                LOG(INFO)<<" hybrid path direction:："<< path.getPrim();
//            }

            return true;
        }
    }
    direction_ = (init_direction==1||init_direction==3);


    all_paths.clear();
    LOG(INFO)<<"当前路径是前进还是后退：前进为1 后退为0 ："<< direction_;
    for(auto path:followed_path){
        LOG(INFO)<<"```````````分离出来的路径坐标："<< path.x()<<";"<<path.y();
    }
    calc_max_vel(followed_path);

    CalcSpeed(followed_path);

    seg_dist = calc_path_dist(followed_path);
    dist_to_goal = seg_dist;
    return true;
}


// 自由导航状态机  init初始化变量和地图信息
// wait_goal:等待接受新的目标位置
// gen_traj:用于计算全局路径，路径优化等
// exec_traj :用于执行规划出来的路径。
// pause  :遇到较近障碍物  暂停然后重新规划路径。
// 需要输入的信息 有 agv 当前的位姿，当前的线速度和角速度。 轨迹信息， 终点位姿。  目标速度。
// current_pose;
// current_velocity;
// std::vector<Pose> paths;
// goal_pose;
//  参数信息：param
//  输出为  target_velocity;
void PlanningFSM::navigateFSM(Pose current_pose,Velocity current_velocity,std::vector<HybridAStar::Node3D>& paths,Pose goal_pose,std::vector<Pose>& followed_path ,Velocity& target_velocity){

    //LOG(INFO)<<"nav_state_！"<<nav_state_;
    // LOG(INFO)<<"G:STATE:"<<g_src_state.src_state;
    //LOG(INFO)<<"g_state.nav_state:"<<g_state.nav_state;
    static uint8_t calc_pause_acc_flag=0;
    static double acc_v = 0.5 ;
    static double acc_w = 0.5;
    switch(nav_state_)
    {
        case INIT:
        {
            changeNavState(WAIT_GOAL);
            break;
        }
        case WAIT_GOAL:
        {
            if(has_goal_)
            { changeNavState(GEN_TRAJ);}
            else{
                return;
            }
            break;
        }
        case GEN_TRAJ:
        {
            //gen_trajectory();

            break;
        }
        case EXEC_TRAJ:
        {
            // collosion check. 根据实时雷达点判断障碍物上是否有障碍点。
            // 先写在这里。根据实时雷达点优化前方路径点
            // 首先应该选取当前点前方一段距离的点。 然后优化该路径，最后计算曲率是否满足要求。
            //  不在这里做。只在这里更新路径。

            bool success = exec_traj(opt_current_pose_,current_velocity,paths,followed_path,  goal_pose,target_velocity);

            // 如果src的状态异常 则线速度和角速度都设置为0；
           // LOG(INFO)<<"系统状态："<<g_state.isEmergency()<<";"<<g_state.isMotorError();
            if(g_state.isEmergency()||(g_state.isMotorError())){
                target_velocity.vx() = 0;
                target_velocity.a() = 0;
                target_velocity.vy() = 0;
                target_velocity.vtheta() = 0;
                LOG(INFO)<<"系统状态："<<g_state.isEmergency()<<";"<<g_state.isMotorError();
            }

            last_target_velocity_ = target_velocity;

            remain_dist =remain_seg_dist - seg_dist+dist_to_goal;

            // 由于总的路径有前进和后退的区别，所以需要分开执行。每执行完一段路径之后，通过loadPaths来装载路经。知道没有路径可以装载了就可以
            // 结束路径跟踪任务。
            if(success){
                bool have_path = loadPaths( paths,followed_path);

                LOG(INFO)<<"9999999999999999999剩余路径的条数"<<paths.size();
                if(have_path){
                    traj_follow_init( opt_current_pose_,followed_path);
                    // 如果还有路径没有执行完。
                    LOG(INFO)<<"继续执行下一段路径";
                    exec_step_ = EXEC_INIT;
                    return;
                } else{
                    LOG(INFO)<<"完成自由导航！";
                    src_sdk->sendCommandMsg(COMMAND_SET_NAVALGORITHM, 3, 0);//结束src的局部路径导航。
                    changeNavState(GEN_TRAJ);
                }
            }

            break;
        }
        case TRAJ_PAUSE:
        {
            double time_inter = 0.02;

            float stop_dist ;

            stop_dist = (obstacle_dist_-0.20);
            if(stop_dist<0.01) stop_dist = 0.01;
            acc_v =fabs(current_velocity.vx()*current_velocity.vx()/stop_dist/2.0);
            if(acc_v<1) acc_v = 1;
            if(acc_v>5)acc_v = 5;
            acc_w = 1.0;
            calc_pause_acc_flag=1;

            target_velocity.vx()  =0;
            target_velocity.a() = -acc_v;
            target_velocity.vtheta() = 0;
            smoothvelocity(target_velocity,last_target_velocity_,acc_v,acc_w,time_inter);

            if(exec_step_==FOLLOW_TRAJ){
                pause_pure_pursuit(opt_current_pose_,current_velocity,followed_path,target_velocity);
                if(fabs(current_velocity.vx())<0.01&&
                   (exec_step_ == EXEC_STEP::FOLLOW_TRAJ||exec_step_==EXEC_STEP::START_ROTATE||exec_step_==EXEC_STEP::EXEC_INIT)&&
                   no_replan_pause==0)
                {
                    target_velocity.vx()  =0;
                    target_velocity.vtheta() = 0;
                    target_velocity.a() = 0;
                    changeNavState(REPLAN_TRAJ);
                }
            }
            else{
                //LOG(INFO)<<"current_velocity.vx()"<<current_velocity.vx()<<"stop_dist"<<stop_dist<<"obstacle_dist_： "<< obstacle_dist_;
                //LOG(INFO)<<"目标减速度： "<< acc_v;
                if(fabs(current_velocity.vx())<0.01&&fabs(current_velocity.vtheta())<0.01&&
                        (exec_step_ == EXEC_STEP::FOLLOW_TRAJ||exec_step_==EXEC_STEP::START_ROTATE||exec_step_==EXEC_STEP::EXEC_INIT)&&
                no_replan_pause==0)
                {
                    target_velocity.vx()  =0;
                    target_velocity.vtheta() = 0;
                    target_velocity.a() = 0;
                    changeNavState(REPLAN_TRAJ);
                }
            }

            // 如果线速度为0，且
            break;
        }
        case REPLAN_TRAJ:
        {

            break;
        }
        default:
            break;
    }
    if(nav_state_ !=TRAJ_PAUSE){
        last_target_velocity_ = target_velocity;
        calc_pause_acc_flag = 0;
    }
}


std::string getStateString(int state){
    std::string state_str[6] = { "INIT", "WAIT_GOAL", "GENTRAJ", "REPLAN_TRAJ", "EXEC_TRAJ","TRAJ_PAUSE" };
    return state_str[int(state)];
}

void PlanningFSM::changeNavState(NAV_STATE target_state){
    nav_state_ = target_state ;

    LOG(INFO)<<"改变自由导航的状态 ："<<getStateString(nav_state_);
}


void PlanningFSM::resetNavState(Velocity& target_velocity){
    nav_state_ = INIT ;
    exec_step_ = EXEC_INIT;
    target_velocity.vtheta() = 0;
    target_velocity.vx() = 0;
    target_velocity.a() =0 ;
    target_velocity.w_a() = 0;
    no_replan_pause = 0;
}


void PlanningFSM::printExecState()
{
    std::string state_str[5] = { "INIT", "WAIT_GOAL", "GENTRAJ", "REPLAN_TRAJ", "EXEC_TRAJ" };

    std::cout << "[FSM]: state: " + state_str[int(nav_state_)] << std::endl;
}

bool PlanningFSM::exec_traj(Pose current_pose,Velocity current_velocity,std::vector<HybridAStar::Node3D>& all_paths,std::vector<Pose>& paths, Pose goal_pose,Velocity& target_velocity)
{
    bool success = false;

    static int64_t  time,time1;

    //LOG(INFO)<<"22222222222222222exec_step_"<<exec_step_;
    switch(exec_step_){
        case  EXEC_INIT:
        {
            start_rotate_angle = std::atan2(paths[1].y() - paths[0].y(),
                                            paths[1].x() - paths[0].x()); // direction to goal
        // 根据路径的第一个点和第二个点计算出起点旋转的目标角度。
        if(direction_){

        } else{
            start_rotate_angle = normalizeYaw(start_rotate_angle+M_PI) ; // direction to goal
        }

        exec_step_ = START_ROTATE;
        LOG(INFO)<<"`````````起点原地旋转目标角度：度。"<<start_rotate_angle/3.14*180.0<<";当前agv的角度："<<current_pose.yaw()/3.14*180.0;
        if(fabs(normalizeYaw(current_pose.yaw()-start_rotate_angle))<0.13825f&&cfg_->enable_backward){
            LOG(INFO)<<"开启后退模式下，旋转角度小于22.5度，直接忽略";
                exec_step_ = FOLLOW_TRAJ;
                time  = sros::core::util::get_time_in_ms();
            }
        pathfollow_.g_Scurve_plan_acc = 0;
        pathfollow_.start_rotate_flag = normalizeYaw(start_rotate_angle-current_pose.yaw());
        break;
        }
        case START_ROTATE:
        {
            pathfollow_.WConfigure_doubleS(cfg_->robot.max_vel_theta,
                                cfg_->robot.acc_lim_theta,
                                2.4,
                                fabs(normalizeYaw(start_rotate_angle -current_pose.yaw() ))-0.004,
                                fabs(target_velocity.vtheta()),
                                pathfollow_.g_Scurve_plan_acc);
		    target_velocity.vtheta() =pathfollow_.WNext_doubleS(&pathfollow_.g_Scurve_plan_acc);
            //LOG(INFO)<<"起点原地旋转时，目标角速度为："<<target_velocity.vtheta()<<"max_vel_theta"<<cfg_->robot.max_vel_theta<<"acc_lim_theta"<<cfg_->robot.acc_lim_theta;
 //           bool rotate_success = pathfollow_.process_rotate(target_velocity.vtheta(), current_pose.yaw(), start_rotate_angle, max_w_,acc_w_,run_period_);
//
//            bool  rotate_success = process_rotate(target_velocity.vtheta(), target_velocity.w_a(),current_pose.yaw(), start_rotate_angle, 1.5,
//                    1.5)    ;
//
//            target_velocity.vtheta() =  1.2*(target_velocity.vtheta()-current_velocity.vtheta())+current_velocity.vtheta();
//            LOG(INFO)<<"原地旋转时，目标角速度为："<<target_velocity.vtheta()<<";imu测量的实际角速度为："<<current_velocity.vtheta();
            if(normalizeYaw(start_rotate_angle -current_pose.yaw() )<0){
                target_velocity.vtheta() = -target_velocity.vtheta();
            }

            
            
            bool  rotate_success;
            if (normalizeYaw(start_rotate_angle -current_pose.yaw() )*pathfollow_.start_rotate_flag <=0&&
            fabs(normalizeYaw(start_rotate_angle -current_pose.yaw() ))<0.17) {
                rotate_success = true;
            }   
            else{
                rotate_success = false;
            }
            if (rotate_success) {
                    target_velocity.vx() = 0;
                    target_velocity.a() = 0;
                    target_velocity.vtheta() = 0;
                    exec_step_ = FOLLOW_TRAJ;
                    LOG(INFO) << "`````````切换到follow traj 模式。";
                     time  = sros::core::util::get_time_in_ms();

                }
            break;
        }
        case FOLLOW_TRAJ:
        {
            // 打印此时的系统时间
            time1  = sros::core::util::get_time_in_ms();
            //LOG(INFO)<<"运行到路径跟踪时的系统时间："<<time1-time;
            //target_velocity.vx() = CalcTargetSpeedBaseProfile((time1-time)*0.001,paths);
            target_velocity.vx() = fabs(target_velocity.vx());
            bool follow_traj_success = pure_pursuit( current_pose,current_velocity,paths,target_velocity);
            if(!direction_){
                target_velocity.vx() = -target_velocity.vx();
            }
           // LOG(INFO)<<"根据之前方式计算的目标线速度为："<<target_velocity.vx();
            if(follow_traj_success){
                target_velocity.vx() = 0;
                target_velocity.vtheta() = 0;
                end_rotate_angle = goal_pose.yaw();
                pathfollow_.g_Scurve_plan_acc = 0;
                pathfollow_.end_rotate_flag = normalizeYaw(end_rotate_angle-current_pose.yaw());
                     // 测试用。
                if(!all_paths.empty()){
                    target_velocity.vx() = 0;
                    target_velocity.a() = 0;
                    target_velocity.vtheta() = 0;
                    success = true;
                    return true;
                }
                exec_step_ = END_ROTATE;
            }
            break;
        }
        case END_ROTATE:
        {
            //end_rotate_angle = goal_pose.yaw();
            // 测试用。
            if(!all_paths.empty()){
                target_velocity.vx() = 0;
                target_velocity.a() = 0;
                target_velocity.vtheta() = 0;
                success = true;
                return true;
            }

            pathfollow_.WConfigure_doubleS(cfg_->robot.max_vel_theta,
                                cfg_->robot.acc_lim_theta,
                                2.4,
                                fabs(normalizeYaw(end_rotate_angle -current_pose.yaw() )),
                                fabs(target_velocity.vtheta()),
                                pathfollow_.g_Scurve_plan_acc);
		    target_velocity.vtheta() =pathfollow_.WNext_doubleS(&pathfollow_.g_Scurve_plan_acc);
            //LOG(INFO)<<"终点原地旋转时，目标角速度为："<<target_velocity.vtheta()<<"max_vel_theta"<<cfg_->robot.max_vel_theta<<"acc_lim_theta"<<cfg_->robot.acc_lim_theta;
 
            if(normalizeYaw(end_rotate_angle -current_pose.yaw() )<0){
                target_velocity.vtheta() = -target_velocity.vtheta();
            }         
            bool  rotate_success;
            if (normalizeYaw(end_rotate_angle -current_pose.yaw() )*pathfollow_.end_rotate_flag <=0&&
            fabs(normalizeYaw(end_rotate_angle -current_pose.yaw() ))<0.17) {
                rotate_success = true;
            }   
            else{
                rotate_success = false;
            }
            //bool rotate_success = pathfollow_.process_rotate(target_velocity.vtheta(), current_pose.yaw(), end_rotate_angle, max_w_, acc_w_, run_period_);
            if(rotate_success)
            {
                target_velocity.vx() = 0;
                target_velocity.a() = 0;
                target_velocity.vtheta() = 0;
                success = true;
                return true;
            }
            break;
        }
    }
    if(exec_step_==END_ROTATE&&success){

    }

    return false;
}




// 计算目标向量在基准向量上的投影距离。
double calc_radial_projection(Pose  aim_vector,Pose  base_vector){
    double min_dist;
    min_dist = (base_vector.x()*aim_vector.x()+base_vector.y()*aim_vector.y())/sqrt(base_vector.x()*base_vector.x()+base_vector.y()*base_vector.y());
    return min_dist;
}

// 计算目标向量在基准向量法向方向上的投影距离。
double calc_normal_projection(Pose  aim_vector,Pose  base_vector){
    double min_dist;
    min_dist = (base_vector.y()*aim_vector.x()-base_vector.x()*aim_vector.y())/sqrt(base_vector.x()*base_vector.x()+base_vector.y()*base_vector.y());
    return min_dist;
}

// 计算离当前点最近的路径点。 在线段内。
void  calc_min_dist_to_segment(Pose current_pose,Pose start_pose, Pose end_pos,Pose& min_pose)
{
    Pose aim_vector;
    Pose base_vector;
    aim_vector.x() = current_pose.x() - start_pose.x();
    aim_vector.y() = current_pose.y() - start_pose.y();

    base_vector.x() = end_pos.x() - start_pose.x();
    base_vector.y() = end_pos.y() - start_pose.y();

    double base_length = hypot(base_vector.x(),base_vector.y());
    float radial_dist = calc_radial_projection(aim_vector,base_vector);
    if(radial_dist>0)
    {
        if(radial_dist>base_length)
        {
            Pose error;
            error.y()= base_vector.y()-aim_vector.y();
            error.x()= base_vector.x()-aim_vector.x();
            //second_dist = hypot( error.x(),error.y());
            min_pose = end_pos;
        }
        else
        {
            //double dist = fabs(calc_normal_projection(aim_vector,base_vector));
            min_pose.x() = radial_dist/base_length*base_vector.x()+start_pose.x();
            min_pose.y() = radial_dist/base_length*base_vector.y()+start_pose.y();
        }
    }
    else
    {
        //second_dist =    hypot( aim_vector.x(),aim_vector.y());
        min_pose = start_pose;
    }
}

// 根据agv 当前位置  和离散点的path  计算出最近点的坐标  ，最近点所在线段的起点index
bool calc_min_to_paths(Pose current_pose,std::vector<Pose>& paths,int& path_index,Pose& min_pose)
{
    int near_id = 0;
    int i=0;
    float max_dist = 10000;
    auto size = paths.size();

    if(paths.empty()){
        return false;
    }
    std::vector<Pose>::iterator path_it;

    // 首先求出最近点的编号near_id 从零开始。 以及 最近点的距离。
    for(int j=0;j<3;j++){
        int num ;
        num = path_index+j;
        if(num>=size){
            num = size-1;
        }

        double dist = std::hypot((current_pose.x()-paths[num].x()),(current_pose.y()-paths[num].y()));
        if(dist<max_dist){
            max_dist = dist;
            near_id = num;
        }
    }

    if(size>1){
        if(near_id == 0)
        {
            Pose start_pose,end_pose ;
            start_pose = paths[0];
            end_pose = paths[1];
            calc_min_dist_to_segment( current_pose, start_pose, end_pose, min_pose);
            path_index = 0;
            return true;
        }
        else if(near_id == size-1)
        {
            Pose start_pose,end_pose ;
            start_pose = paths[size-2];
            end_pose = paths[size-1];
            calc_min_dist_to_segment( current_pose, start_pose, end_pose, min_pose);
            path_index = size-2;
            return true;
        } else{
            Pose start_pose,end_pose ,first_pose,second_pose;
            start_pose = paths[near_id-1];
            end_pose = paths[near_id];
            calc_min_dist_to_segment( current_pose, start_pose, end_pose, first_pose);
            float first_dist = hypot(current_pose.x()-first_pose.x(),current_pose.y()-first_pose.y());

            start_pose = paths[near_id];
            end_pose = paths[near_id+1];
            calc_min_dist_to_segment( current_pose, start_pose, end_pose, second_pose);
            float second_dist = hypot(current_pose.x()-second_pose.x(),current_pose.y()-second_pose.y());

            if(first_dist<second_dist){
                path_index = near_id-1;
                min_pose = first_pose;
            }else{
                path_index = near_id;
                min_pose = second_pose;
            }
            return true;
        }
    } else{
        min_pose = paths[0] ;
        path_index = 0;
        return true;
    }
    //near_id 表示距离上最近的waypoint

}

// 根据路径点index min_pose 以及paths 还有前视距离 计算出目标点的位置 target_index 目标点所在的直线上的起点的index 从0开始的。

bool calc_target_pose(int path_index,Pose min_pose,std::vector<Pose>& paths,float lookahead_dist,Pose& target_pose,int& target_index){
    if(paths.size()<2)
    {
        return false;
    }
    int near_id = path_index+1;
    double dist_to_target=0;
    dist_to_target =  std::hypot((min_pose.x()-paths[near_id].x()),(min_pose.y()-paths[near_id].y()));
    // 计算出目标点
    if(dist_to_target>lookahead_dist)
    {
        target_pose.x() = lookahead_dist/dist_to_target*(paths[near_id].x()-min_pose.x())+min_pose.x();
        target_pose.y() = lookahead_dist/dist_to_target*(paths[near_id].y()-min_pose.y())+min_pose.y();
        target_pose.yaw() = atan2(paths[near_id].y()-min_pose.y(),paths[near_id].x()-min_pose.x());
        target_index = path_index;
       // LOG(INFO)<<"计算的局部规划的起点用到的坐标x："<<paths[near_id].x()<<"; "<<paths[near_id].y()<<";"<<dist_to_target;
        return  true;
    }
    else
    {
        if(near_id == paths.size()-1)
        {
            target_pose.x() = lookahead_dist/dist_to_target*(paths[near_id].x()-min_pose.x())+min_pose.x();
            target_pose.y() = lookahead_dist/dist_to_target*(paths[near_id].y()-min_pose.y())+min_pose.y();
            target_pose.yaw() = atan2(paths[near_id].y()-min_pose.y(),paths[near_id].x()-min_pose.x());
            target_index = path_index;
            return  true;
        }
        double acc_dist = dist_to_target;
        for(int j=near_id;j<paths.size()-1;j++)
        {
            double dist = std::hypot((paths[j].x()-paths[j+1].x()),(paths[j].y()-paths[j+1].y()));
            acc_dist = acc_dist + dist;
            if(acc_dist > lookahead_dist){
                target_pose.x() = (lookahead_dist-(acc_dist-dist))/dist*(paths[j+1].x()-paths[j].x())+paths[j].x();
                target_pose.y() = (lookahead_dist-(acc_dist-dist))/dist*(paths[j+1].y()-paths[j].y())+paths[j].y();
                target_pose.yaw() = atan2(paths[j+1].y()-paths[j].y(),paths[j+1].x()-paths[j].x());
                target_index =j;
                return  true;
            }

            target_pose.x() = (lookahead_dist-(acc_dist-dist))/dist*(paths[j+1].x()-paths[j].x())+paths[j].x();
            target_pose.y() = (lookahead_dist-(acc_dist-dist))/dist*(paths[j+1].y()-paths[j].y())+paths[j].y();
            target_pose.yaw() = atan2(paths[j+1].y()-paths[j].y(),paths[j+1].x()-paths[j].x());
        }
        target_index =paths.size()-2;
        return  true;

    }


}

//根据路径点index min_pose 以及paths 计算从最近点的终点的距离。
bool calc_ramain_dist(int path_index,Pose min_pose,std::vector<Pose>& paths,double& dist_to_goal)
{
    if(paths.size()<2)
    {
        return false;
    }
    int near_id = path_index+1;
    dist_to_goal=0;
    dist_to_goal =  std::hypot((min_pose.x()-paths[near_id].x()),(min_pose.y()-paths[near_id].y()));

    for(int j=near_id;j<paths.size()-1;j++)
    {
        double dist = std::hypot((paths[j].x()-paths[j+1].x()),(paths[j].y()-paths[j+1].y()));
        dist_to_goal = dist_to_goal + dist;
    }
    return  true;
}

// 判断agv 在某个位置时是否会碰到障碍物。
// 返回true表示agv在该点有障碍物，false表示agv在该点没有障碍物。

bool collision_check(Pose agv_pose,Navigation& nav,float agv_length,float agv_width,CheckCollision& check_collision,COLLISION_OBS& obs_pose)
{
    float diag_length = hypot(agv_length,agv_width)/2.0f;
    const float margin = 0.01f;
    // 进行坐标转换。 将障碍点转换为agv坐标系下的坐标

    bool collision_or_not = check_collision.check_point_collision( nav,agv_pose,agv_length,agv_width,obs_pose);
    if(collision_or_not){
        return  true;
    }else {
        return  false;
    }

//    for(auto pose:local_obstacles)
//    {
//        float dx = pose.x() - agv_pose.x();
//        float dy = pose.y() - agv_pose.y();
//        if(fabs(dx)>diag_length+margin&&fabs(dy)>diag_length+margin){
//            continue;
//        }
//        float yaw = agv_pose.yaw();
//
//        // 进行坐标转换。 将障碍点转换为agv坐标系下的坐标
//        double new_x =  dx*cos(yaw)+dy*sin(yaw);
//        double new_y = -dx*sin(yaw)+dy*cos(yaw);
//
//        if((fabs(new_x)<(agv_length/2.0+margin))&&(fabs(new_y)<(agv_width/2.0+margin)))
//        {
//            LOG(INFO)<<"agv_pose.x(): "<<agv_pose.x()<<" agv_pose.y():"<<agv_pose.y()<<"agv_pose.yaw():"<<agv_pose.yaw();
//            LOG(INFO)<<"此时的障碍点坐标为："<<pose.x()<<":"<<pose.y();
//            return true;
//        }
//    }
//    return false;
}


// 检测路径前方是否有障碍物 并记录agv与障碍物的
bool PlanningFSM::check_path_collision(int path_index,Pose min_pose,std::vector<Pose>& paths,Navigation& nav,float check_dist,float& obstacle_dist,CheckCollision& check_collision,COLLISION_OBS& obs_pose)
{
    float check_step = 0.1; // 表示每隔check_step距离检查一次agv是否碰到障碍物。
    float agv_length = cfg_->robot.length;
    float agv_width = cfg_->robot.width;
    int near_id = path_index+1;
    float dist_to_target=0;
    Pose agv_pose;

    if(paths.size()<2)
    {
        return false;
    }

    dist_to_target =  std::hypot((min_pose.x()-paths[near_id].x()),(min_pose.y()-paths[near_id].y()));
    if(dist_to_target<check_step)
    {
        agv_pose = paths[near_id];
        agv_pose.yaw() = atan2(paths[near_id].y()-paths[near_id-1].y(),paths[near_id].x()-paths[near_id-1].x());
        if(collision_check(agv_pose, nav,agv_length,agv_width,check_collision,obs_pose))
        {
            //LOG(INFO)<<"1111111"<<agv_pose.x()<<" : "<<agv_pose.y()<<" : "<<agv_pose.yaw();
            obstacle_dist = dist_to_target;
            return true;
        }
    } else{
        int  check_num = floor(dist_to_target/check_step);
        for(int i=3;i<check_num-1;i++){
            agv_pose.x() =check_step*i/dist_to_target*(paths[near_id].x()-min_pose.x())+min_pose.x();
            agv_pose.y() = check_step*i/dist_to_target*(paths[near_id].y()-min_pose.y())+min_pose.y();
            agv_pose.yaw() = atan2(paths[near_id].y()-paths[near_id-1].y(),paths[near_id].x()-paths[near_id-1].x());
            if(collision_check(agv_pose, nav,agv_length,agv_width,check_collision,obs_pose))
            {
                //LOG(INFO)<<"22222"<<agv_pose.x()<<" : "<<agv_pose.y()<<" : "<<agv_pose.yaw();
                obstacle_dist = check_step*i;
                return true;
            }
        }

    }

    float acc_dist = dist_to_target;

    for(int j=near_id;j<paths.size()-1;j++)
    {
        double dist = std::hypot((paths[j].x()-paths[j+1].x()),(paths[j].y()-paths[j+1].y()));


        acc_dist = acc_dist + dist;
        if(acc_dist > check_dist){

            int  check_num = floor((check_dist-(acc_dist-dist))/check_step);
            for(int i=1;i<check_num+0.01;i++){
                agv_pose.x() =check_step*i/dist*(paths[j+1].x()-paths[j].x())+paths[j].x();
                agv_pose.y() = check_step*i/dist*(paths[j+1].y()-paths[j].y())+paths[j].y();
                agv_pose.yaw() = atan2(paths[j+1].y()-paths[j].y(),paths[j+1].x()-paths[j].x());
                if(collision_check(agv_pose, nav,agv_length,agv_width,check_collision,obs_pose))
                {
                    //LOG(INFO)<<"33333"<<agv_pose.x()<<" : "<<agv_pose.y()<<" : "<<agv_pose.yaw();
                    obstacle_dist = acc_dist-dist+check_step*i;
                    return true;
                }
            }
            return  false;
        }

        int check_num = floor(dist/check_step);

        for(int i=1;i<check_num+0.01;i++){
            agv_pose.x() =check_step*i/dist*(paths[j+1].x()-paths[j].x())+paths[j].x();
            agv_pose.y() = check_step*i/dist*(paths[j+1].y()-paths[j].y())+paths[j].y();
            agv_pose.yaw() = atan2(paths[j+1].y()-paths[j].y(),paths[j+1].x()-paths[j].x());
            if(collision_check(agv_pose, nav,agv_length,agv_width,check_collision,obs_pose)){
                obstacle_dist = acc_dist-dist+check_step*i;
                //LOG(INFO)<<"44444444"<<agv_pose.x()<<" : "<<agv_pose.y()<<" : "<<agv_pose.yaw();
                return true;
            }

        }

        agv_pose.x() =paths[j+1].x();
        agv_pose.y() = paths[j+1].y();
        agv_pose.yaw() = atan2(paths[j+1].y()-paths[j].y(),paths[j+1].x()-paths[j].x());
        if(collision_check(agv_pose, nav,agv_length,agv_width,check_collision,obs_pose)){
            //LOG(INFO)<<"555555555"<<agv_pose.x()<<" : "<<agv_pose.y()<<" : "<<agv_pose.yaw();
            obstacle_dist = acc_dist;
            return true;
        }

    }
    return false;
}

//每隔50ms就判断一次前方是否有障碍物 如果有则暂停。
// 如果没有  则继续执行轨迹

bool PlanningFSM::check_collosion(Pose current_pose,Navigation& nav,vector<Pose> paths,CheckCollision& check_collision,COLLISION_OBS& obs_pose)
{
    Pose min_pose;
    int path_index = 0;
    path_index = path_index_;
    std::vector<sros::core::Pose> local_obstacles;
    float check_dist = 2.8f;
    //check_dist  = fabs(target_velocity_.vx())*2.0+cfg_->robot.length/2.0;
    calc_min_to_paths(current_pose, paths, path_index, min_pose);
    bool collosion = check_path_collision( path_index, min_pose, paths, nav, check_dist, obstacle_dist_, check_collision,obs_pose);
    return collosion;
}


// 规划完成路径之后检测一下该路径点是否有障碍物。一般检测前方6米范围内。

bool PlanningFSM::check_replan_path_collosion(Navigation& nav,vector<HybridAStar::Node3D> paths,CheckCollision& check_collision,COLLISION_OBS& obs_pose)
{

    float check_dist = 2.5f;
    float check_step = 0.1; // 表示每隔check_step距离检查一次agv是否碰到障碍物。
    float agv_length = cfg_->robot.length + 0.05;
    float agv_width = cfg_->robot.width + 0.05;
    float dist_to_target=0;
    float total_dist = 0;
    Pose agv_pose;

    if(paths.size()<2){
        LOG(INFO)<<"规划的路径点条数小于2.不检测碰撞 直接返回。";
        return  true;
    }
    for(int j=0;j<paths.size()-1;j++){
        dist_to_target  = sqrt((paths[j+1].getX()-paths[j].getX())*(paths[j+1].getX()-paths[j].getX())+(paths[j+1].getY()-paths[j].getY())*(paths[j+1].getY()-paths[j].getY()));
        total_dist += dist_to_target;

        int  check_num = floor(dist_to_target/check_step);
        for(int i=1;i<check_num+0.01;i++){
            agv_pose.x() = check_step*i/dist_to_target*(paths[j+1].getX()-paths[j].getX())+paths[j].getX();
            agv_pose.y() = check_step*i/dist_to_target*(paths[j+1].getY()-paths[j].getY())+paths[j].getY();
            agv_pose.yaw() = atan2(paths[j+1].getY()-paths[j].getY(),paths[j+1].getX()-paths[j].getX());
            if(collision_check(agv_pose, nav,agv_length,agv_width,check_collision,obs_pose))
            {
                LOG(INFO)<<"规划路径之后检测到该点会导致agv发生碰撞：x:"<<agv_pose.x()<<";y:"<<agv_pose.y()<<";yaw:"<<agv_pose.yaw();
                return true;
            }
            else{
               // LOG(INFO)<<"检测该点没有障碍物"<<agv_pose.x()<<";y:"<<agv_pose.y()<<";yaw:"<<agv_pose.yaw();
            }
        }

        agv_pose.x() =paths[j+1].getX();
        agv_pose.y() = paths[j+1].getY();
        agv_pose.yaw() = atan2(paths[j+1].getY()-paths[j].getY(),paths[j+1].getX()-paths[j].getX());
        if(collision_check(agv_pose, nav,agv_length,agv_width,check_collision,obs_pose))
        {
            LOG(INFO)<<"规划路径之后检测到该点会导致agv发生碰撞：x:"<<agv_pose.x()<<";y:"<<agv_pose.y()<<";yaw:"<<agv_pose.yaw();
            return true;
        }
        else{
           // LOG(INFO)<<"检测该点没有障碍物"<<agv_pose.x()<<";y:"<<agv_pose.y()<<";yaw:"<<agv_pose.yaw();
        }

        if(total_dist>check_dist){
            LOG(INFO)<<"前方"<<check_dist<<"m.距离内，没有检测到障碍物！";
            return  false;
        }
    }
    LOG(INFO)<<"到路径终点都没有检测到障碍物！";
    return  false;

}

//每隔50ms就判断一次前方是否有障碍物 如果有则暂停。
// 如果没有  则继续执行轨迹

//bool PlanningFSM::check_collosion1(Pose current_pose,CheckCollision& check_collision,vector<Pose> paths)
//{
//    Pose min_pose;
//    int path_index = 0;
//    std::vector<sros::core::Pose> local_obstacles;
//    float check_dist = 2.3f;
//
//    // 找到最近点，并将需要检测障碍物的点保持在  std::vector<pose> check_points;
//    std::vector<Pose> check_points;
//    double max_dist  = 100;
//    if(paths.empty()){
//        return false;
//    }
//    uint16_t i=0;
//    uint16_t near_id = 0;
//    for(auto path:paths){
//        double dist = std::hypot((current_pose.x()-path.x()),(current_pose.y()-path.y()));
//        if(dist<max_dist){
//            max_dist = dist;
//            near_id = i;
//        }
//        i++;
//    }
//    Pose first_pose;
//    double total_dist = 0;
//    first_pose = current_pose;
//    check_points.push_back(current_pose);
//    for(int j = near_id+1;j<paths.size();j++){
//        double dist = std::hypot((first_pose.x()-paths[j].x()),(first_pose.y()-paths[j].y()));
//        total_dist += dist;
//        if(total_dist<check_dist){
//            check_points.push_back(paths[j]);
//        }
//        else{
//            // 需要计算
//            check_points.push_back(paths[j]);
//            break;
//        }
//        first_pose = paths[j];
//    }
//
//
//
//
//
//
//
//    // 然后依次判断check_points的点上是否有障碍物。
//
//
//    //LOG(INFO)<<"local_obstacles size:"<<local_obstacles.size();
//
//    calc_min_to_paths(current_pose, paths, path_index, min_pose);
//    bool collosion = check_path_collision( path_index, min_pose, paths, local_obstacles, check_dist, obstacle_dist_);
//    return collosion;
//}

//
//bool PlanningFSM::check_collosion_with_map(Pose current_pose,std::shared_ptr<avoidoba::AvoidobaProcessor> avoidoba_processor,vector<Pose> paths)
//{
//    Pose min_pose;
//    int path_index = 0;
//    std::vector<sros::core::Pose> local_obstacles;
//    float check_dist = 2.3f;
//
//    //LOG(INFO)<<"opt_path size:"<<paths.size();
//    avoidoba::AvdobaPoseInfo obstacle_points;
//    obstacle_points.oba_points.clear();
//    avoidoba_processor->getObstaclePoints(obstacle_points);
//
//    std::vector<Eigen::Vector2d>::iterator it;
//    for (it = obstacle_points.oba_points.begin(); it != obstacle_points.oba_points.end(); it++) {
//        double agv_obstacle_dist;
//        sros::core::Pose pose;
//        Eigen::Vector2d obs;
//        obs = *it;
//        pose.x() = obs.coeffRef(0);
//        pose.y() = obs.coeffRef(1);
//        agv_obstacle_dist = pointDistance(current_pose.x(), current_pose.y(), obs[0], obs[1]);
//        const double OBS_MAX_DIST_TO_AGV = 2.3;
//        if (agv_obstacle_dist < OBS_MAX_DIST_TO_AGV) {
//            // obstacles_.push_back(ObstaclePtr(new PointObstacle(obs)));
//            local_obstacles.push_back(pose);
//        }
//    }
//    //LOG(INFO)<<"local_obstacles size:"<<local_obstacles.size();
//
//    calc_min_to_paths(current_pose, paths, path_index, min_pose);
//    bool collosion = check_path_collision( path_index, min_pose, paths, local_obstacles, check_dist, obstacle_dist_);
//    return collosion;
//}

//float PlanningFSM::check_slowspeed_area(void){
//    // 该函数用于判断是否进入减速区。减速区。
//    // 如果进入减速区。则将最大速度降低到指定速度。
//}

// 路径跟踪，根据当前位置，当前速度，和需要跟踪的轨迹，计算目标速度。
bool PlanningFSM::pure_pursuit(Pose current_pose,
        Velocity current_velocity,
        std::vector<Pose>& paths,
        Velocity& target_velocity)
{
    //计算离当前agv最近的轨迹点
    Pose target_pose;

    double  lookahead_dist = 0.4;

    int target_index;
    lookahead_dist =  0.8*(fabs(target_velocity.vx())+0.1);
    if(lookahead_dist<0.2){
        lookahead_dist = 0.2;
    }
   // LOG(INFO)<<"11111111执行路径时，path_index:"<<path_index_;
    calc_min_to_paths(current_pose, paths, path_index_, min_pose_);
  //  LOG(INFO)<<"22222222执行路径时，path_index:"<<path_index_;

    calc_ramain_dist(path_index_,min_pose_, paths, dist_to_goal);

    calc_target_pose(path_index_, min_pose_,paths, lookahead_dist, target_pose,target_index);

    float velocity_dist = 3;
    Pose velocity_pose;
    int velocity_index;
    calc_target_pose(path_index_, min_pose_,paths, velocity_dist, velocity_pose,velocity_index);
   // check_path_collision(path_index, min_pose, paths,std::vector<sros::core::Pose>& local_obstacles,float check_dist,float& obstacle_dist);
    // 需要根据估计 当前最近点计算前面1m内是否有障碍物。每隔5cm判断一次

    sros::core::Pose current_position ;
    sros::core::Pose target_position;

    current_position = current_pose;
    velocity_plan velocity_param;
    velocity_param.acc_down = cfg_->robot.acc_lim_x;
    velocity_param.acc_up = cfg_->robot.acc_lim_x;
    velocity_param.end_v = 0;
    velocity_param.max_v = cfg_->robot.max_vel_x*g_state.speed_level*1.0f/100.0f;
    velocity_param.remain_dist = dist_to_goal;

   calc_path_max_v(velocity_param.max_v,0.5,paths,path_index_,velocity_index);
   pathfollow_.calc_velocity( velocity_param,run_period_,target_velocity.vx());

    // calc_max_vel(paths);

    // CalcSpeed(paths);

    
    for(auto x:times_profile_){
       // LOG(INFO)<<"每段的时间序列："<<x;
    }
    for(auto x:max_vel_profile_){
        //LOG(INFO)<<"每段的最大速度序列："<<x;
    }

    // calc_velocity_profile1(paths,
    //                        times_profile_,max_vel_profile_,
    //                        vel_profile_,
    //                        target_velocity.vx(),
    //                        target_velocity.a());

    if(fabs(target_velocity.vx())<0.01){
        if(target_velocity.vx()>0){
            target_velocity.vx() = 0.01;
        }else{

            target_velocity.vx() = -0.01;
        }

    }

    if(target_velocity.vx()>cfg_->robot.max_vel_x){
        target_velocity.vx() = cfg_->robot.max_vel_x;
    }
    //LOG(INFO)<<"-------------------------------------";
   // LOG(INFO)<<"当前位置："<<current_position.x()<<";"<<current_position.y()<<";"<<current_position.yaw();
    //LOG(INFO)<<"目标位置："<<target_pose.x()<<";"<<target_pose.y();

    double gama= pathfollow_.sr_calc_coeff ( current_position ,  direction_,  target_pose);


    target_velocity.vtheta() = target_velocity.vx()*gama;
    //LOG(INFO)<<"目标速度："<<target_velocity.vx()<<";"<<target_velocity.vtheta();
    //LOG(INFO)<<"当前AGV的速度："<<current_velocity.vx();
    //target_velocity.vx() =  1.3*(target_velocity.vx()-fabs(current_velocity.vx()))+fabs(current_velocity.vx());
    if(target_velocity.vx()<0){
        target_velocity.vx() = 0.008;
    }
    //LOG(INFO)<<"pid后的目标速度："<<target_velocity.vx();
    //LOG(INFO)<<"-------------------------------------";

    // 计算角速度完成任务。
    if(dist_to_goal < 0.02 && fabs(target_velocity.vx()) < 0.011)
    {
        return true;
    }

    return false;
}


void PlanningFSM::pause_pure_pursuit(Pose current_pose,
        Velocity current_velocity,
        std::vector<Pose>& paths,
        Velocity& target_velocity)
{
    //计算离当前agv最近的轨迹点
    Pose target_pose;

    double  lookahead_dist = 0.4;

    int target_index;
    lookahead_dist =  0.8*(fabs(target_velocity.vx())+0.1);
    if(lookahead_dist<0.2){
        lookahead_dist = 0.2;
    }
   // LOG(INFO)<<"11111111执行路径时，path_index:"<<path_index_;
    calc_min_to_paths(current_pose, paths, path_index_, min_pose_);
  //  LOG(INFO)<<"22222222执行路径时，path_index:"<<path_index_;

    calc_ramain_dist(path_index_,min_pose_, paths, dist_to_goal);

    calc_target_pose(path_index_, min_pose_,paths, lookahead_dist, target_pose,target_index);

    float velocity_dist = 3;
    Pose velocity_pose;
    int velocity_index;
    calc_target_pose(path_index_, min_pose_,paths, velocity_dist, velocity_pose,velocity_index);
   // check_path_collision(path_index, min_pose, paths,std::vector<sros::core::Pose>& local_obstacles,float check_dist,float& obstacle_dist);
    // 需要根据估计 当前最近点计算前面1m内是否有障碍物。每隔5cm判断一次

    sros::core::Pose current_position ;
    sros::core::Pose target_position;

    current_position = current_pose;
    
    double gama= pathfollow_.sr_calc_coeff ( current_position ,  direction_,  target_pose);


    target_velocity.vtheta() = target_velocity.vx()*gama;

}


//// 该函数用于判断agv是否应该进行重新规划全局路径。
//check_replan_or_not()
//
//该函数用于判断障碍物较远时，规划前方的局部路径。并更新需要执行的路径。
bool  PlanningFSM::calc_local_start_end_point(std::vector<Pose>& paths,Pose& start_pose,Pose& end_pose,int& start_index,int& end_index)
{
    float start_dist = 1.0;
    calc_target_pose(path_index_, min_pose_,paths, start_dist, start_pose,start_index);
    float end_dist = 5.0;
    calc_target_pose(path_index_, min_pose_,paths, end_dist, end_pose,end_index);

    LOG(INFO)<<"min_pose_ X:"<<min_pose_.x()<<"; y:"<<min_pose_.y();
    LOG(INFO)<<"path_index_:"<<path_index_<<"; start_index:"<<start_index;
    return  true;
}

void PlanningFSM::traj_follow_init(Pose current_pose,std::vector<Pose>& paths)
{
    path_index_ = 0;
    calc_min_to_paths(current_pose, paths, path_index_, min_pose_);
    poly_coff_.clear();

}

void PlanningFSM::traj_follow_init1(Pose current_pose,std::vector<Pose>& paths)
{
    calc_min_to_paths(current_pose, paths, path_index_, min_pose_);
}


//// 在执行路径前更新全局路径点。
void update_local_path(std::vector<Pose>& old_path,std::vector<Pose>& new_local_path,int start_index,int end_index)
{
    for(int i= end_index+1;i<old_path.size();i++){
        new_local_path.push_back(old_path[i]);
    }

    old_path.erase(old_path.begin()+start_index+1,old_path.end());
    for(auto point:new_local_path){
        old_path.push_back(point);
    }
}

//void convert_global_to_local_map(struct grid& global_map,struct grid& local_map){
//
//
//    for (int i = 0; i < local_map.width; ++i) {
//        for (int j = 0; j < local_map.height; ++j) {
//            struct node *tgrid = getNodeAt(&local_map, i, j);
//
//            tgrid->grid_layered_walkable_ = 0;
//            //需要将i j转化为全局栅格坐标系的位置，并判断位置是否超过边界，如果超过则直接忽略。
//            int global_i = current_gridmap_x - local_map.width / 2 + i;
//            int global_j = current_gridmap_y - local_map.height / 2 - 1 + j;
//            if (global_i >= global_width || global_i < 0 || global_j >= global_height || global_j < 0) {
//                tgrid->grid_layered_walkable_ = LETHAL_OBSTACLE;
//                continue;
//            }
//            if (getNodeAt(&global_map, global_i, global_j)->grid_layered_walkable_ == INIT_WALKABLE_FALSE) {
//
//                tgrid->grid_layered_walkable_ = LETHAL_OBSTACLE;
//            }
//        }
//    }
//}

void sample_global_path(sros::core::NavigationPath_vector& global_path, vector<HybridAStar::Node3D> sampled_path) {
    // g

}

// 计算轨迹的长度

float calc_path_length(vector<HybridAStar::Node3D>& global_path){
    float length=0;
    vector<HybridAStar::Node3D>::iterator it;
    for(it = global_path.begin();it!=global_path.end()-1;it++){
        HybridAStar::Node3D first_point = *it;
        HybridAStar::Node3D second_point = *(it+1);
        float dist = hypot(first_point.getX()-second_point.getX(),first_point.getY()-second_point.getY());
        length +=dist;
    }
    return  length;
}


// 根据轨迹和lookahead_dist计算局部目标点。
void calc_hybrid_target_point(vector<HybridAStar::Node3D>& global_path,float lookahead_dist,Pose& target_pose){
    float path_length = calc_path_length(global_path);
    float length = 0;
    if(lookahead_dist<path_length){
        vector<HybridAStar::Node3D>::iterator it;
        for(it = global_path.begin();it!=global_path.end()-1;it++){
            HybridAStar::Node3D first_point = *it;
            HybridAStar::Node3D second_point = *(it+1);
            float dist = hypot(first_point.getX()-second_point.getX(),first_point.getY()-second_point.getY());
            //LOG(INFO)<<"first_point"<<first_point.getX()<<";"<<first_point.getY();
            //LOG(INFO)<<"second_point"<<second_point.getX()<<";"<<second_point.getY();
            length += dist;
            if(length>lookahead_dist){
                float coeff = (dist-(length-lookahead_dist))/dist;
                target_pose.x() = first_point.getX()+coeff*(second_point.getX()-first_point.getX());
                target_pose.y() = first_point.getY()+coeff*(second_point.getY()-first_point.getY());
                target_pose.yaw() = atan2(second_point.getY()-first_point.getY(),second_point.getX()-first_point.getX());
                LOG(INFO)<<"hybrid 的局部目标点1 ："<<target_pose.x()<<";"<<target_pose.y();
                return;
            }
        }

    }else{
        target_pose.x() = global_path.back().getX();
        target_pose.y() = global_path.back().getY();
        LOG(INFO)<<"hybrid 的局部目标点2 ："<<target_pose.x()<<";"<<target_pose.y();
    }


}
// 截取全局路径的后面一段，用于和hybrid规划的轨迹合并。
void calc_hybrid_remain_path(vector<HybridAStar::Node3D>& global_path,float lookahead_dist){
    float length = 0;
    int num=0;
    vector<HybridAStar::Node3D>::iterator it;
    for(it = global_path.begin();it!=global_path.end()-2;it++){
        HybridAStar::Node3D first_point = *it;
        HybridAStar::Node3D second_point = *(it+1);
        float dist = hypot(first_point.getX()-second_point.getX(),first_point.getY()-second_point.getY());
        length += dist;
        if(length<lookahead_dist){
            num++;
        }else{
            break;
        }
    }
    global_path.erase(global_path.begin(),global_path.begin()+num+1);
}

//根据jps计算的全局路径 global_path 选取局部目标点作为hybrid算法的目标点规划包含agv运动学约束的轨迹。
// 然后将该轨迹与原先的global_path.后面的一段合并 再进行优化处理。
void PlanningFSM::replan_free_path(Navigation& navigate_,sros::core::NavigationPath_vector& global_path,Pose cur_pose,CheckCollision& obs_map, Location_Vector& oba_points,Pose dst_pose,std::vector<HybridAStar::Node3D>& all_paths){
    // 根据全局路径规划来。 首先对该路径进行采样。

    int offset_x = navigate_.getNavigationMap()->getMapZeroOffset().x;
    int offset_y = navigate_.getNavigationMap()->getMapZeroOffset().y;

    //path_smoother_.addObstacleFromGridMap(navigate_,oba_points);
    // 已经优化完的路径保存在path_smoother.对象中。

    path_smoother_.pathSample(global_path,navigate_);
    // path_smoother_.smoothPath1();

    // ///////////////////////////test
    // // 如果初始路径满足碰撞检测，则完成路径规划
    // all_paths.clear();
    // std::vector<Node3D> test_path;
    // vector<Pose> opt_path0;
    // path_smoother_.smoothPath1(obs_map);
    // path_smoother_.convertPath(navigate_, global_path, opt_path0);
    // test_path = path_smoother_.getPath();
    // for(auto it = test_path.begin(); it < test_path.end(); it++) {
    //     double point_x, point_y;
    //     navigate_.reverseMapCoords(offset_x,offset_y,it->getX(),it->getY(),&point_x,&point_y);
    //     it->setX(point_x);
    //     it->setY(point_y);
    //     all_paths.push_back(*it);
    //     LOG(INFO)<<"经过合并和优化之后轨迹,"<<"方向："<<it->getPrim()<<";x:"<<it->getX()<<";y:"<<it->getY();
    // }
    // COLLISION_OBS obstacle_pose;
    // if(check_replan_path_collosion(navigate_, all_paths, obs_map, obstacle_pose) == 0) {
    //     LOG(INFO) << "初始优化路径满足碰撞检测，完成路径规划";

    //     return;
    // }
    // LOG(INFO) << "初始优化路径不满足碰撞检测，将进行重规划";
    // ///////////////////////////test

    Pose target_pose;
    vector<HybridAStar::Node3D> final_path;
    for(auto point : path_smoother_.getPath()){
        double point_x,point_y;
        navigate_.reverseMapCoords(offset_x,offset_y,point.getX(),point.getY(),&point_x,&point_y);
        Node3D path;
        path.setX(point_x);
        path.setY(point_y);
        final_path.push_back(path);
    }
    // 接着选取一个局部目标点。距离为4m。
    calc_hybrid_target_point(final_path, 3.5, target_pose);

    //calc_hybrid_remain_path(final_path,3.5);
    // 得到目标点之后就开始规划hybrid路径。
    int startx,starty,endx,endy;
    LOG(INFO)<<"hybrid起点规划时的目标点坐标为："<<target_pose.x()<<";"<<target_pose.y();

    //从此处开始获取局部地图，并在局部地图中规划路径。
    LocalMap localmaptest(8,cfg_->nav_resolution);
    localmaptest.initLocalMap();
    localmaptest.getLocalMap1(obs_map,navigate_,cur_pose);
    navigate_.convertMapCoords(offset_x, offset_y, cur_pose.x(), cur_pose.y(), &startx, &starty);
    navigate_.convertMapCoords(offset_x, offset_y, target_pose.x(), target_pose.y(), &endx, &endy);
    LOG(INFO) << "hybrid start point: " << cur_pose << ": " << startx << "; " << starty ;
    LOG(INFO) << "hybrid end   point: " << target_pose << ": " << endx << "; " << endy ;
    sros::core::Pose start(startx, starty, -1* cur_pose.yaw());
    sros::core::Pose end(endx, endy, -1*target_pose.yaw());
    sros::core::Pose local_start, local_end;
    localmaptest.convertGlobalMapToLocalMap(local_start, start);
    localmaptest.convertGlobalMapToLocalMap(local_end, end);
    local_start.yaw() = start.yaw();
    local_end.yaw() = end.yaw();
    // 对于重新规划需要直接将障碍点映射到局部地图中。
    //a_star.plan(local_start, local_end, *localmaptest.local_map_);
    a_star.findOneRotatePointForAgv(local_start, local_end, *localmaptest.local_map_);    ////////////////////////////////////////test

    if(a_star.path.empty()){
        global_path.clear();
        //path_smoother_.deleteObstacleFromGridMap( navigate_,oba_points);
        LOG(INFO)<<"规划出来的hybrid path为 空。";
        return;
    }
    //hybrid_path = navigate_.replanHybridPath(cur_pose,target_pose,oba_points1);

    reverse(a_star.path.begin(),a_star.path.end());

    {
        vector<HybridAStar::Node3D>::iterator it;
        for(it = a_star.path.begin();it!=a_star.path.end();it++){
            Pose local_end1(it->getX(),it->getY());
            Pose end1;
            localmaptest.convertLocalMapToGlobalMap(local_end1,end1);
            //LOG(INFO)<<"混合规划出的轨迹点："<<end1.x()<<";"<<end1.y();
            it->setX(end1.x());
            it->setY(end1.y());
        }
    }

    path_smoother_.setPath(a_star.path);
    //path_smoother_.smoothPath();

    std::vector<Node3D> hybrid_path;
    std::vector<Node3D> hybrid_path1;
    hybrid_path = path_smoother_.getPath();
    NavigationPath_vector cur_paths ;
   
    // // 上面计算了起点无法原地旋转时采用hybrid 下面计算终点无法原地旋转时也采用hybrid找到可以原地旋转的点。
    // /////////////////////////////////////////////////////////////////////////////
    COLLISION_OBS obs_pose;
    LOG(INFO)<<"终点坐标："<<dst_pose.x()<<";"<<dst_pose.y()<<";"<<dst_pose.yaw();
    bool end_rotate_collision = false;
    for(int i=0;i<45;i++){
        Pose tmp_pose = dst_pose;
        tmp_pose.yaw()= i*4.0/180*3.14159;
        end_rotate_collision = obs_map.check_point_collision(navigate_,tmp_pose,cfg_->robot.length,cfg_->robot.width,obs_pose);
        if(end_rotate_collision){
            LOG(INFO)<<"终点坐标原地旋转检测到碰撞："<<tmp_pose.x()<<";"<<tmp_pose.y()<<";"<<tmp_pose.yaw();
            break;
        }
    }

    if(end_rotate_collision) {
        LOG(INFO) <<" 终点原地旋转会检测到碰撞，采用hybrid规划终点。";

        reverse(final_path.begin(), final_path.end());
        calc_hybrid_target_point(final_path, 3.0, target_pose);

        navigate_.convertMapCoords(offset_x, offset_y, target_pose.x(), target_pose.y(), &startx, &starty);
        navigate_.convertMapCoords(offset_x, offset_y, dst_pose.x(), dst_pose.y(), &endx, &endy);
        // LOG(INFO) << "hybrid start point :"<<startx<<";"<<starty ;
        // LOG(INFO) << "hybrid end   point :"<<endx<<";"<<endy ;
        sros::core::Pose end(startx,starty, -1.0*normalizeYaw(target_pose.yaw()+3.1415926));
        sros::core::Pose start(endx, endy, -1.0*normalizeYaw(dst_pose.yaw()+3.1415926));
        LOG(INFO)<<"终点无法原地旋转hybird规划起点坐标： " << dst_pose;
        LOG(INFO)<<"终点无法原地旋转hybird规划终点坐标： " << target_pose;       

        //sros::core::Pose local_start,local_end;
        localmaptest.getLocalMap1(obs_map,navigate_,dst_pose);
        localmaptest.convertGlobalMapToLocalMap(local_start, start);
        localmaptest.convertGlobalMapToLocalMap(local_end, end);
        local_start.yaw() = start.yaw();
        local_end.yaw() = end.yaw();

        // 对于重新规划需要直接将障碍点映射到局部地图中。
        //a_star1.plan(local_start, local_end, *localmaptest.local_map_);
        a_star1.findOneRotatePointForAgv(local_start, local_end, *localmaptest.local_map_);    ////////////////////////////////////test

        if(a_star1.path.empty()){
            //global_path.clear();
            //path_smoother_.deleteObstacleFromGridMap( navigate_,oba_points);
            LOG(INFO)<<"a_star1规划出来的hybrid path为 空。";
        }
        else{
            vector<HybridAStar::Node3D>::iterator it;
            for(it = a_star1.path.begin(); it != a_star1.path.end(); it++){
                Pose local_end1(it->getX(), it->getY());
                Pose end1;
                localmaptest.convertLocalMapToGlobalMap(local_end1, end1);
                //LOG(INFO)<<"混合规划出的轨迹点："<<end1.x()<<";"<<end1.y();
                it->setX(end1.x());
                it->setY(end1.y());
            }
            
            path_smoother_.setPath(a_star1.path);
            //path_smoother_.smoothPath();

            hybrid_path1 = path_smoother_.getPath();
            if(hybrid_path1.size()>1){
                if(hybrid_path1[hybrid_path1.size()-2].getPrim()<3){
                    hybrid_path1[hybrid_path1.size()-1].setPrim(2);
                }else{
                    hybrid_path1[hybrid_path1.size()-1].setPrim(4);
                }
            }

            // 不仅最后一个要改。当涉及到换向时也需要改。从后往前修改prim
            if(hybrid_path1.size()>2){
                for(int i=hybrid_path1.size()-2;i>0;i--){
                    bool changeflag =0;
                    if((hybrid_path1[i].getPrim()>2&&hybrid_path1[i-1].getPrim()<3)||
                    (hybrid_path1[i].getPrim()<3&&hybrid_path1[i-1].getPrim()>2)){
                        hybrid_path1[i].setPrim(hybrid_path1[i-1].getPrim());
                    }
                }
            }
        }

        if(!a_star1.path.empty()){
            double world_start_x,world_start_y;
            HybridAStar::Node3D final_point = hybrid_path.back();
            navigate_.reverseMapCoords(offset_x, offset_y, final_point.getX(),final_point.getY(), &world_start_x, &world_start_y);
            Pose start_pose(world_start_x,world_start_y);
            final_point = hybrid_path1.front();
            navigate_.reverseMapCoords(offset_x, offset_y, final_point.getX(),final_point.getY(), &world_start_x, &world_start_y);
            // navigate_.reverseMapCoords(offset_x, offset_y, final_point.getX(),final_point.getY(), &world_start_x, &world_start_y);
            Pose end_pose(world_start_x, world_start_y);
            LOG(INFO) << "二次规划：start: " << start_pose;
            LOG(INFO) << "二次规划：end: " << end_pose;
            cur_paths = navigate_.replanPath(start_pose, end_pose, NAV_PATH_TYPE::LINE_ROTATE_TYPE,  oba_points);
        }
        else{
            double world_start_x,world_start_y;
            HybridAStar::Node3D final_point = hybrid_path.back();
            navigate_.reverseMapCoords(offset_x, offset_y, final_point.getX(),final_point.getY(), &world_start_x, &world_start_y);
            Pose start_pose(world_start_x,world_start_y);
            //navigate_.reverseMapCoords(offset_x, offset_y, final_point.getX(),final_point.getY(), &world_start_x, &world_start_y);
            Pose end_pose = dst_pose;
            LOG(INFO)<<"二次规划：start:"<<start_pose.x()<<";"<<start_pose.y()<<";"<<start_pose.yaw();
            LOG(INFO)<<"二次规划：end:"<<end_pose.x()<<";"<<end_pose.y()<<";"<<end_pose.yaw();
            cur_paths = navigate_.replanPath(start_pose, end_pose, NAV_PATH_TYPE::LINE_ROTATE_TYPE,  oba_points); 
        }
    }
    else{
        LOG(INFO)<<"终点可以满足原地旋转的要求！！！！！";
        double world_start_x,world_start_y;
        HybridAStar::Node3D final_point = hybrid_path.back();
        navigate_.reverseMapCoords(offset_x, offset_y, final_point.getX(),final_point.getY(), &world_start_x, &world_start_y);
        Pose start_pose(world_start_x,world_start_y);
        //navigate_.reverseMapCoords(offset_x, offset_y, final_point.getX(),final_point.getY(), &world_start_x, &world_start_y);
        Pose end_pose = dst_pose;
        LOG(INFO)<<"二次规划：start:"<<start_pose.x()<<";"<<start_pose.y()<<";"<<start_pose.yaw();
        LOG(INFO)<<"二次规划：end:"<<end_pose.x()<<";"<<end_pose.y()<<";"<<end_pose.yaw();
         cur_paths = navigate_.replanPath(start_pose, end_pose, NAV_PATH_TYPE::LINE_ROTATE_TYPE,  oba_points);
    }
    ///////////////////////////////////////////////////////////////////////                                  

    path_smoother_.pathSample(cur_paths,navigate_);
    
    path_smoother_.smoothPath1( obs_map);
    // 如果a star 最后一个点是后退。且 且离终点距离小于0.9米。且 cur_paths.小于2米。则直接添加后退路径。
    LOG(INFO)<<"CCC";
    // 在此处判断规划的路径是否合理。不合理则返回false 合理则返回true。
    std::vector<Pose> temp_path;
    for(auto point:path_smoother_.getPath()){
        Pose temp_pose;
        temp_pose.x() = point.getX();
        temp_pose.y() = point.getY();
        temp_path.push_back(temp_pose);
    }

    if(!check_path_feasibility(temp_path)){
        LOG(INFO)<<"优化后的全局路径无法通过合理性检查。";
        return;
    }

    int move_first = 1;
    for(auto point : path_smoother_.getPath()){
        if(move_first == 1){
            move_first = 0;
        } else {
            point.setPrim(6);
            hybrid_path.push_back(point);
        }
    }

    move_first = 1;
    for(auto point:hybrid_path1){
            if(move_first==1){
                move_first = 0;
            } else {
                hybrid_path.push_back(point);
            }
    }

    // LOG(INFO)<<"最终的smoothpath 的size："<<path_smoother_.getPath().size();
    //path_smoother_.deleteObstacleFromGridMap( navigate_,oba_points);
    all_paths.clear();

    // 将最后的轨迹保存在global_path中。
    vector<Pose> opt_path;
    path_smoother_.setPath(hybrid_path);
    path_smoother_.convertPath(navigate_,global_path,opt_path);
    vector<HybridAStar::Node3D>::iterator it;

    std::vector<Node3D> tmp_path;
    tmp_path = path_smoother_.getPath();
    LOG(INFO)<<"CCC"<<tmp_path.size();
    for(it = tmp_path.begin(); it < tmp_path.end(); it++) {
        double point_x,point_y;
        navigate_.reverseMapCoords(offset_x,offset_y,it->getX(),it->getY(),&point_x,&point_y);
        it->setX(point_x);
        it->setY(point_y);
        all_paths.push_back(*it);
        LOG(INFO)<<"经过合并和优化之后轨迹,"<<"方向："<<it->getPrim()<<";x:"<<it->getX()<<";y:"<<it->getY();
    }
    // LOG(INFO)<<"最终的global——path 的size："<<global_path.size();
    // 程序结束。
}

//// 用于在规划路径之后检查用a star 规划的路径是否存在转弯半径较大的情况。根据夹角来判断。如果夹角大于70度则报错表示规划失败。
//bool PlanningFSM::check_path_feasibility(vector<Pose>& opt_paths){
//    if(opt_paths.empty())
//    {
//        return false;
//    }
//    if(opt_paths.size()<3)
//    {
//        return  false;
//    }
//
//
//}

// 用于检查规划的局部路径是否满足要求。1，不允许有后退。2，相邻两点距离不能太小。
bool PlanningFSM::check_path_feasibility(vector<Pose>& opt_paths)
{
    if(opt_paths.empty())
    {
        return false;
    }
    if(opt_paths.size()<3)
    {
        return  true;
    }
    vector<Pose>::iterator it;
    for(it=opt_paths.begin()+1;it!=opt_paths.end()-2;it++){
        Pose p1,p2,p3;
        p1 = *(it-1);
        p2 = *it;
        p3 = *(it+1);
        Pose p12,p23;
        p12.x() = p2.x()-p1.x();
        p12.y() = p2.y()-p1.y();
        p23.x() = p3.x() - p2.x();
        p23.y() = p3.y() - p2.y();

        double angle ;
        angle = p12.x()*p23.x()+p12.y()*p23.y();
        double p12_length =  sqrt(p12.x() * p12.x() + p12.y() * p12.y());
        double p23_length =  sqrt(p23.x() * p23.x() + p23.y() * p23.y());
        angle = acos(angle/p12_length/p23_length);
        LOG(INFO)<<"规划的路径=相邻两个向量夹角(度)："<<angle/3.1415*180.0;
        if(angle>70.0f/180*3.1415)
        {
            LOG(INFO)<<"相邻两个向量夹角大于70度规划的路径不满足要求。可以切换到重新规划。夹角为(度)："<<angle/3.1415*180.0f;
            return  false;
        }

    }
    for(it=opt_paths.begin();it!=opt_paths.end()-2;it++){
        Pose p1,p2;
        p1 = *it;
        p2 = *(it+1);
        Pose p12,p23;
        p12.x() = p2.x()-p1.x();
        p12.y() = p2.y()-p1.y();

        double dist = hypot(p12.x(),p12.y());
        if(dist<0.05)
        {
            LOG(INFO)<<"相邻两点的距离太小了，规划的路径不满足要求。可以切换到重新规划。";
            return  false;
        }

    }
    return  true;

}

//计算一个矩形区域内是否有障碍物。
bool check_Rect_obstacle(Navigation& nav,float width,float length){
// 需要考虑地图的静态障碍物 和雷达扫描的动态障碍物。 比如绘制了禁止区域 如果判断障碍物不考虑该区域则会导致AGV在该区域撞到雷达检测不到的障碍物。
// 因此，直接采用读取包含了实时障碍点和地图信息的会比较合适。采用局部地图 会每个周期都需要更新局部地图，比较耗时。 采用全局地图。每个周期先删除前一个
// 障碍点坐标，然后添加新的障碍点坐标。会极大的减小运算量。 然后以该地图为基础进行障碍物判断。
// 需要考虑多线程安全问题。可能存在地图更新时候，刚好需要读取地图判断障碍物，导致障碍物检测失败。 当更新地图时锁住地图。
    return false;
}


// 判断差速轮AGV原地旋转是否可行。
bool check_rotate_feasible(Pose cur_pose){
    // 不能采用膨胀的地图。否则会与膨胀尺寸关联。最好采用原始地图， 然后采用车尺寸对角线距离 然后+ margin 进行判断。
    float margin  = 0.1f; //判断原地旋转是否碰到障碍物的余量。
    float width;
    float length;
    return false;
}

void PlanningFSM::ConvertType(NavigationPath_vector& cur_paths,vector<HybridAStar::Node3D>& final_path){
    cur_paths.clear();
    for(int i=0;i<final_path.size()-1;i++){

        float point_x,point_y,point_next_x,point_next_y;

        point_x = final_path[i].getX();
        point_y = final_path[i].getY();

        point_next_x = final_path[i+1].getX();
        point_next_y = final_path[i+1].getY();

        sros::core::LinePath pose(point_x,point_y,point_next_x,point_next_y);
        cur_paths.push_back(pose);
    }
}

//// 根据起点和终点 以及给定的起点和终点的速度，采用TEB算法规划出从起点到终点的轨迹。
//// 调用teb算法的接口。
//bool  PlanningFSM::plan_local_traj(Pose& start_pose,Pose& end_pose)
//{
//
//
//    return  true;
//}

//
//bool  PlanningFSM::replan_global_path(Pose& start_pose,Pose& end_pose)
//{
//
//
//    return  true;
//}


// 计算三个离散点的近似曲率
double PJCurvature(Eigen::Vector3d xvals, Eigen::Vector3d yvals){
    double ta =sqrt(pow(xvals(1)-xvals(0),2)+pow(yvals(1)-yvals(0),2));
    double tb =sqrt(pow(xvals(2)-xvals(1),2)+pow(yvals(2)-yvals(1),2));
    Eigen::MatrixXd mat_;
    mat_.resize(3,3);
    mat_<<1,-ta,ta*ta,
            1,0,0,
            1,tb,tb*tb;
    Eigen::Vector3d ax,ay;
    ax = mat_.inverse()*xvals;
    ay = mat_.inverse()*yvals;
    double kappa = 2*(ax(2)*ay(1)-ax(1)*ay(2))/(pow(ax(1)*ax(1)+ay(1)*ay(1),1.5));

    return kappa;
}


//计算每个点的最大速度限制。考虑曲率 和 减速区。
void PlanningFSM::calc_max_vel(vector<Pose>& way_points)
{
    double coeff_vel = 0.4;
    double para_max_vel = cfg_->robot.max_vel_x;

    max_vel_profile_.clear();
    //vel_profile_.push_back(para_max_vel);
    for(int j=0;j<way_points.size()-2;j++){
        Eigen::Vector3d xvals,yvals;
        for(int i=0;i<3;i++){

            xvals(i) = way_points[i+j].x();
            yvals(i) = way_points[i+j].y();
        }
        double kappa = PJCurvature( xvals,  yvals);
        double temp_max_vel = sqrt(fabs(coeff_vel/kappa));
        if(temp_max_vel<para_max_vel){
            max_vel_profile_.push_back(temp_max_vel);
        }
        else{
            max_vel_profile_.push_back(para_max_vel);
        }
        //区域坐标点在笛卡尔坐标系下位置。
        //pa-----pb
        //--------
        //--------
        //pd-----pc
        std::vector<sros::map::AreaMark>::iterator it;
        for(it=slow_velocity_area_.begin();it!=slow_velocity_area_.end();it++){
            if(way_points[j+1].x()>it->pa.x*0.01&&
            way_points[j+1].x()<it->pb.x*0.01&&
            way_points[j+1].y()<it->pa.y*0.01&&
            way_points[j+1].y()>it->pd.y*0.01
            ){
                if(it->type ==sros::map::AreaMark::AREA_TYPE_SPEED_LEVEL_80){
                    max_vel_profile_[j] = min(max_vel_profile_[j],para_max_vel*0.8);
                     if(j!=0){
                     max_vel_profile_[j-1] = min(max_vel_profile_[j-1],para_max_vel*0.8);
                    }
                }
                if(it->type ==sros::map::AreaMark::AREA_TYPE_SPEED_LEVEL_60){
                    max_vel_profile_[j] = min(max_vel_profile_[j],para_max_vel*0.6);
                    if(j!=0){
                     max_vel_profile_[j-1] = min(max_vel_profile_[j-1],para_max_vel*0.6);
                    }
                }
                if(it->type ==sros::map::AreaMark::AREA_TYPE_SPEED_LEVEL_40){
                    max_vel_profile_[j] = min(max_vel_profile_[j],para_max_vel*0.4);
                    if(j!=0){
                     max_vel_profile_[j-1] = min(max_vel_profile_[j-1],para_max_vel*0.4);
                    }
                }
                if(it->type ==sros::map::AreaMark::AREA_TYPE_SPEED_LEVEL_20){
                    max_vel_profile_[j] = min(max_vel_profile_[j],para_max_vel*0.2);
                    if(j!=0){
                     max_vel_profile_[j-1] = min(max_vel_profile_[j-1],para_max_vel*0.2);
                    }
                }

                //LOG(INFO)<<"11111111路径点在减速区域。此时路径点的坐标为："<<way_points[j].x()<<";"<<way_points[j].y();
                //LOG(INFO)<<"减速区域顶点坐标："<<it->pa.x*0.01<<";"<<it->pb.x*0.01<<";"<<it->pa.y*0.01<<";"<<it->pd.y*0.01;
            }
        }

        //  cout<<"j:"<<j<<";kappa:"<<kappa<<endl;
    }
    max_vel_profile_.push_back(para_max_vel);

}
// 根据最大速度，距离，加速度，起点和终点速度都为0时的一段路径采用T型加减速度时需要的时间。
//double calc_time(double s,double acc,double vel)

void PlanningFSM::CalcSpeed(vector<Pose>& way_points){
    double para_max_vel = cfg_->robot.max_vel_x;
    double para_accel = cfg_->robot.acc_lim_x*0.8;
    double coeff_vel = 0.4;

    // 先计算各点的速度和时间。way_points 这个为最终的路径点。
    //假设加速度为0.3 最大速度0.8.考虑曲率对运行速度的影响。v<sqrt(fabs(0.3/kappa));
    vel_profile_.clear();
    times_profile_.clear();
    vel_profile_.push_back(para_max_vel);
    for(auto speed:max_vel_profile_){
        vel_profile_.push_back(speed);
    }
    // //vel_profile_.push_back(para_max_vel);
    // for(int j=0;j<way_points.size()-2;j++){
    //     Eigen::Vector3d xvals,yvals;
    //     for(int i=0;i<3;i++){

    //         xvals(i) = way_points[i+j].x();
    //         yvals(i) = way_points[i+j].y();
    //     }
    //     double kappa = PJCurvature( xvals,  yvals);
    //     double temp_max_vel = sqrt(fabs(coeff_vel/kappa));
    //     if(temp_max_vel<para_max_vel){
    //         vel_profile_.push_back(temp_max_vel);
    //     }
    //     else{
    //         vel_profile_.push_back(para_max_vel);
    //     }

    //     //  cout<<"j:"<<j<<";kappa:"<<kappa<<endl;
    // }
    //
    // vel_profile_.push_back(para_max_vel);
    vector<double> time;
    double total_dist = 0;
    // 根据T型加减速度来计算每个点的速度值。 从前往后。
    for(int i=0;i<vel_profile_.size();i++){
        if(i==0){
            vel_profile_[0]=0;
            continue;
        }
        if(i>0){
            double s,a,v0,v1;
            v0 = vel_profile_[i-1];
            a = para_accel;
            s = sqrt(pow(way_points[i].x()-way_points[i-1].x(),2)+pow(way_points[i].y()-way_points[i-1].y(),2));
            total_dist +=s;
            //cout<<"两点之间的距离："<<total_dist<<endl;
            v1 = sqrt(2.0*s*a+v0*v0);
            //cout<<"t"<<2.0*s/(v0+v1)<<endl;
            if(v1>para_max_vel){
                v1=para_max_vel;
            }
            if(v1>vel_profile_[i]){
                v1=vel_profile_[i];
            }
            vel_profile_[i] = v1;

        }
    }
    //cout<<"vel_profile_.size():"<<vel_profile_.size()<<endl;
    // 根据T型加减速度来计算每个点的速度值。 从前往后。
    for(int i=vel_profile_.size()-1;i>=0;i--){
        if(i== vel_profile_.size()-1){
            vel_profile_[vel_profile_.size()-1]=0;
            continue;
        }
        if(i<vel_profile_.size()-1){
            double s,a,v0,v1;
            v0 = vel_profile_[i+1];
            a = para_accel;
            s = sqrt(pow(way_points[i].x()-way_points[i+1].x(),2)+pow(way_points[i].y()-way_points[i+1].y(),2));


            v1 = sqrt(2.0*s*a+v0*v0);
           // cout<<"从后往前计算点 i："<<i<<";的v1值为:"<<v1<<endl;

            if(v1>para_max_vel){
                v1=para_max_vel;
            }
            if(v1>vel_profile_[i]){
                v1=vel_profile_[i];
            }
            vel_profile_[i] = v1;

        }
    }
    // 求每段路径的时间。

    for(int i=0;i<vel_profile_.size()-1;i++){
        double s,a,v0,v1;
        if(vel_profile_.size()==2){
            s = sqrt(pow(way_points[i].x()-way_points[i+1].x(),2)+pow(way_points[i].y()-way_points[i+1].y(),2));
            double time = sqrt(s/para_accel);
            times_profile_.push_back(time);
        }else{
            v0 = vel_profile_[i];
            v1 = vel_profile_[i+1];
            s = sqrt(pow(way_points[i].x()-way_points[i+1].x(),2)+pow(way_points[i].y()-way_points[i+1].y(),2));
            double time = 2.0*s/(v0+v1);
            //cout<<"t"<<2.0*s/(v0+v1)<<endl;
            times_profile_.push_back(time);
        }

    }

    for(auto v:vel_profile_){
        //LOG(INFO)<<"规划出来的速度值："<<v;
    }
    double all_time = 0;
    for(auto time:times_profile_){
       // LOG(INFO)<<"规划出来的每段时间："<<time;
        all_time +=time;
    }
   // LOG(INFO)<<"！！！！！！！！分离出来的路径运行总时间："<<all_time;


}


double PlanningFSM::CalcTargetSpeedBaseProfile(double current_time,std::vector<Pose>&  followed_path){
    // 根据规划的速度和时间轮廓 计算下一个周期的目标速度。
    //同时根据时间计算下一个时间的目标位置。如果位置太靠前 则通过比例控制减小速度。

    //先根据时间计算出目标速度然后来控制速度。
    double time_index = 0;
    double total_times = 0;

    for(int i=0;i<times_profile_.size();i++){
        total_times += times_profile_[i];
        if(current_time<total_times){
            time_index = i;
            break;
        }
    }

    //  求出比例
    double base_time=0;

        base_time = total_times - times_profile_[time_index];

    double K = (current_time - base_time)/times_profile_[time_index];

    double target_vel = K*(vel_profile_[time_index+1] - vel_profile_[time_index])+vel_profile_[time_index];

    LOG(INFO)<<"通过速度轮廓和时间计算得到的目标线速度为："<<target_vel;

    // 计算current_time时 agv到了哪个位置。然后根据位置差根据比例控制求出新的目标速度。
    double target_x = K*(followed_path[time_index+1].x() - followed_path[time_index].x())+followed_path[time_index].x();

    double target_y = K*(followed_path[time_index+1].y() - followed_path[time_index].y())+followed_path[time_index].y();

    LOG(INFO)<<"当前位置："<<opt_current_pose_.x()<<" ; "<<opt_current_pose_.y()<<
    " ;目标位置： "<<target_x<<" ; "<<target_y;


    return  target_vel;

}


// 根据 路径。 path_index. time轮廓。 顶点速度轮廓。求出 两点之间的速度曲线。
// 因为经过优化之后，原先的s发生了一点点变化。所有需要根据AGV实际走过的s来判断采用哪个多项式。
// times_profile，则用来计算优化后的新分段。
// 该函数周期性执行。 每次添加5条完成的曲线。

void PlanningFSM::calc_velocity_profile(std::vector<Pose>&  followed_path,
        int path_index,
        vector<double> times_profile,
        vector<double> max_vel_profile_,
        vector<vector<double>>&  poly_coff){

    // 首先根据followed_path 计算一个距离值。后面的优化都是以该距离值为基础。
    vector<double> dists;
    double dist =0;
    for(int i=0;i<followed_path.size()-1;i++){
         dist += sqrt(pow(followed_path[i].x()-followed_path[i+1].x(),2)+ pow(followed_path[i].y()-followed_path[i+1].y(),2));
         dists.push_back(dist);
    }

    //LOG(INFO)<<"路径总长度："<<dist<<";dist_to_goal"<<dist_to_goal;
    double dist_from_start = dist - dist_to_goal;
    for(auto d:dists){
       // LOG(INFO)<<"距离计算："<<d;
    }

    if(poly_coff.empty()){


        vector<double> opt_times;
        vector<double> opt_dist;
        vector<double> opt_max_vel;
        Vector3d start_status;

        //10段路径，获取10段时间。 10个点的距离。还有后10个点的最大速度限制。
        if(times_profile.size()>=10){
            for(int i=0;i<10;i++){
                opt_times.push_back(times_profile[i]);
                opt_max_vel.push_back(max_vel_profile_[i]);
                opt_dist.push_back(dists[i]);
            }
        }
        else{
            for(int i=0;i<times_profile.size();i++){
                opt_times.push_back(times_profile[i]);
                opt_max_vel.push_back(max_vel_profile_[i]);
                opt_dist.push_back(dists[i]);
            }
        }



        SpeedPlan speed_planner(opt_times.size(),6);
        start_status(0) = dist_from_start;
        start_status(1) = 0;
        start_status(2) = 0;

        LOG(INFO)<<"第一次开始优化！！！";
        int64_t time1  = sros::core::util::get_time_in_ms();
        speed_planner.calc_P(opt_times);

        speed_planner.calc_A(start_status,opt_times,opt_dist,opt_max_vel,cfg_->robot.acc_lim_x*1.2);

        speed_planner.optimize();
        int64_t time2  = sros::core::util::get_time_in_ms();

        LOG(INFO)<<"第一次完成了速度曲线的优化！";
        LOG(INFO)<<"第一次多项式的个数："<<speed_planner.coff_.size();
        LOG(INFO)<<"第一次完成优化需要的时间ms："<<time2-time1;
        double start_dist = speed_planner.calc_dist(speed_planner.coff_[0],0);
        LOG(INFO)<<"起始点的距离值为："<<start_dist;
        if(speed_planner.coff_.size()>=5){
            for(int i=0;i<5;i++){ //
                poly_coff.push_back(speed_planner.coff_[i]);
            }
        }else{
            for(int i=0;i<speed_planner.coff_.size();i++){ //
                poly_coff.push_back(speed_planner.coff_[i]);
            }
        }

    }
    if(!poly_coff.empty()){

        if(poly_coff.size()-path_index<=5){

            //
            LOG(INFO)<<"```````````````````更新后续路径的速度曲线。";

            vector<double> opt_times;
            vector<double> opt_dist;
            vector<double> opt_max_vel;
            Vector3d start_status;



            if(times_profile.size()-poly_coff.size()<10&&times_profile.size()-poly_coff.size()>0){
                // 还没有规划完的线段数少于10条。则有多少条优化多少条。并且终点速度为0.
                //10段路径，获取10段时间。 10个点的距离。还有后10个点的最大速度限制。
                int opt_num = times_profile.size()-poly_coff.size();
                for(int i=0;i<opt_num;i++){
                    opt_times.push_back(times_profile[poly_coff.size()+i]);
                    opt_max_vel.push_back(max_vel_profile_[poly_coff.size()+i]);
                    opt_dist.push_back(dists[poly_coff.size()+i]);
                }

                SpeedPlan speed_planner(opt_num,6);

                start_status(0) = speed_planner.calc_dist(poly_coff[poly_coff.size()-1],times_profile[poly_coff.size()-1]);
                start_status(1) = speed_planner.calc_vel(poly_coff[poly_coff.size()-1],times_profile[poly_coff.size()-1]);
                start_status(2) = speed_planner.calc_a(poly_coff[poly_coff.size()-1],times_profile[poly_coff.size()-1]);

                speed_planner.calc_P(opt_times);

                speed_planner.calc_A(start_status,opt_times,opt_dist,opt_max_vel,cfg_->robot.acc_lim_x*1.2);

                speed_planner.optimize();
                if(speed_planner.coff_.size()>=5){
                    for(int i=0;i<5;i++){ //
                        poly_coff.push_back(speed_planner.coff_[i]);
                    }
                }else{
                    for(int i=0;i<speed_planner.coff_.size();i++){ //
                        poly_coff.push_back(speed_planner.coff_[i]);
                    }
                }
            }
            else if(times_profile.size()-poly_coff.size()>=10){
                //10段路径，获取10段时间。 10个点的距离。还有后10个点的最大速度限制。
                for(int i=0;i<10;i++){
                    opt_times.push_back(times_profile[poly_coff.size()+i]);
                    opt_max_vel.push_back(max_vel_profile_[poly_coff.size()+i]);
                    opt_dist.push_back(dists[poly_coff.size()+i]);
                }

                SpeedPlan speed_planner(10,6);

                start_status(0) = speed_planner.calc_dist(poly_coff[poly_coff.size()-1],times_profile[poly_coff.size()-1]);
                start_status(1) = speed_planner.calc_vel(poly_coff[poly_coff.size()-1],times_profile[poly_coff.size()-1]);
                start_status(2) = speed_planner.calc_a(poly_coff[poly_coff.size()-1],times_profile[poly_coff.size()-1]);

                speed_planner.calc_P(opt_times);

                speed_planner.calc_A(start_status,opt_times,opt_dist,opt_max_vel,cfg_->robot.acc_lim_x*1.2);

                speed_planner.optimize();

                if(speed_planner.coff_.size()>=5){
                    for(int i=0;i<5;i++){ //
                        poly_coff.push_back(speed_planner.coff_[i]);
                    }
                }else{
                    for(int i=0;i<speed_planner.coff_.size();i++){ //
                        poly_coff.push_back(speed_planner.coff_[i]);
                    }
                }
            }
            else{
                LOG(INFO)<<"22222222完成了所有路径段的 速度曲线优化！！！";
            }

            //计算poly_coff的终点的s v 和a 作为下一次优化的起始条件。



        }

    }

    LOG(INFO)<<"已经完成速度规划的段数："<<poly_coff.size();
}

void PlanningFSM::calc_velocity_profile1(std::vector<Pose>&  followed_path,
                                        vector<double> times_profile,
                                        vector<double> max_vel_profile_,
                                        vector<double> vel_profile,
                                        double& target_v,
                                        double& target_a
                                        ){
    vector<double> dists;
    double lookahead_time = 0.030;
    double dist =0;
    for(int i=0;i<followed_path.size()-1;i++){
        dist += sqrt(pow(followed_path[i].x()-followed_path[i+1].x(),2)+ pow(followed_path[i].y()-followed_path[i+1].y(),2));
        dists.push_back(dist);
    }

   // LOG(INFO)<<"路径总长度："<<dist<<";dist_to_goal"<<dist_to_goal;
   // LOG(INFO)<<followed_path.size()<<";"<<
    double dist_from_start = dist - dist_to_goal;

    int current_id=0;
    for(int i=0;i<dists.size();i++){
        if(dists[i]>dist_from_start){
            current_id = i;
            //LOG(INFO)<<"当前位置所在的段是第几段。从0 开始："<<current_id<<";总共有的段数："<<dists.size();
            break;
        }
    }

    // 构造这三个变量。然后给求解器求解。
    vector<double> opt_times;
    vector<double> opt_dist;
    vector<double> opt_max_vel;
    Vector3d start_status,end_status;

    //10段路径，获取10段时间。 10个点的距离。还有后10个点的最大速度限制。
    if(times_profile.size()-current_id>=11){
        for(int i=0;i<10;i++){

            //第一个时间需要变化一下。

            double s = dists[current_id] - dist_from_start;
            // if(s>0.15){
            //     if(i==0){
            //         double time_comp = 0;
            //         double time_comp1 = 0;
            //         if(fabs(target_v)-cfg_->robot.max_vel_x>0){
            //             time_comp = fabs(target_v)-cfg_->robot.max_vel_x;
            //         }
            //         if(fabs(target_a)-cfg_->robot.acc_lim_x>0){
            //             time_comp1 = fabs(target_a)-cfg_->robot.acc_lim_x;
            //         }
                
            //         double first_time = 2.0*s/(target_v+vel_profile[current_id+1])+time_comp*10.0+time_comp1*2.0;
            //         opt_times.push_back(first_time);
            //     }else{
            //         opt_times.push_back(times_profile[current_id+i]);
            //     }
            //     opt_max_vel.push_back(max_vel_profile_[current_id+i]);
            //     opt_dist.push_back(dists[current_id+i]);
            // }else
            {
                if(i==0){
                    double time_comp = 0;
                    double time_comp1 = 0;
                    if(fabs(target_v)-cfg_->robot.max_vel_x>0.01){
                        time_comp = fabs(target_v)-cfg_->robot.max_vel_x;
                    }
                    if(fabs(target_a)-cfg_->robot.acc_lim_x>0.01){
                        time_comp1 = fabs(target_a)-cfg_->robot.acc_lim_x;
                    }
                
                    double first_time = 2.0*s/(target_v+vel_profile[current_id+1])+time_comp*10.0+time_comp1*2.0;
                    opt_times.push_back(first_time+times_profile[current_id+1]);
                }else{
                    opt_times.push_back(times_profile[current_id+i+1]);
                }
                opt_max_vel.push_back(max_vel_profile_[current_id+i+1]);
                opt_dist.push_back(dists[current_id+i+1]);
            }

        }
    }
    else if(times_profile.size()-current_id>=4){

        for(int i=0;i<times_profile.size()-current_id-1;i++){

            double s = dists[current_id] - dist_from_start;
            {
                if(i==0){
                    double time_comp = 0;
                    double time_comp1 = 0;
                    if(fabs(target_v)-cfg_->robot.max_vel_x>0.01){
                        time_comp = fabs(target_v)-cfg_->robot.max_vel_x;
                    }
                    if(fabs(target_a)-cfg_->robot.acc_lim_x>0.01){
                        time_comp1 = fabs(target_a)-cfg_->robot.acc_lim_x;
                    }
                
                    double first_time = 2.0*s/(target_v+vel_profile[current_id+1])+time_comp*10.0+time_comp1*2.0;
                    opt_times.push_back(first_time+times_profile[current_id+1]);
                }else{
                    opt_times.push_back(times_profile[current_id+i+1]);
                }
                opt_max_vel.push_back(max_vel_profile_[current_id+i+1]);
                opt_dist.push_back(dists[current_id+i+1]);
            }
        }
    }
     else if(times_profile.size()-current_id==3){

        double s = dists[current_id] - dist_from_start;
    {
        double first_time;
        if(times_profile.size()==3){
            first_time = times_profile[current_id] * s/(dists[current_id]);
        }else{
            first_time = times_profile[current_id] * s/(dists[current_id]-dists[current_id-1]);
        }
        
        first_time =  first_time+times_profile[current_id+1]+times_profile[current_id+2];
        SpeedPlan speed_planner(1,6);
        start_status(0) = dist_from_start;
        start_status(1) = target_v;
        start_status(2) = target_a;
        end_status(0) = dist;
        end_status(1) = 0;
        end_status(2) = 0;
        speed_planner.calc_opt_analytical_coff( start_status, end_status, cfg_->robot.max_vel_x, cfg_->robot.acc_lim_x*1.1, first_time,coff1_);
        vector<double> coff2;
        for(int i=0;i<6;i++){
            coff2.push_back(coff1_(i));
        }
        target_v = speed_planner.calc_vel(coff2,lookahead_time);
        target_a = speed_planner.calc_a(coff2,lookahead_time);
        //LOG(INFO)<<"11113333TARGET_V:"<<target_v<<";TARGET_A:"<<target_a;
        return;
    }


        
    }

    else if(times_profile.size()-current_id==2){

        double s = dists[current_id] - dist_from_start;
    {
        double first_time;
        if(times_profile.size()==2){
            first_time = times_profile[current_id] * s/(dists[current_id]);
        }else{
            first_time = times_profile[current_id] * s/(dists[current_id]-dists[current_id-1]);
        }
        
        first_time =  first_time+times_profile[current_id+1];
        SpeedPlan speed_planner(1,6);
        start_status(0) = dist_from_start;
        start_status(1) = target_v;
        start_status(2) = target_a;
        end_status(0) = dist;
        end_status(1) = 0;
        end_status(2) = 0;
        speed_planner.calc_opt_analytical_coff( start_status, end_status, cfg_->robot.max_vel_x, cfg_->robot.acc_lim_x*1.1, first_time,coff1_);
        vector<double> coff2;
        for(int i=0;i<6;i++){
            coff2.push_back(coff1_(i));
        }
        target_v = speed_planner.calc_vel(coff2,lookahead_time);
        target_a = speed_planner.calc_a(coff2,lookahead_time);
        //LOG(INFO)<<"1111222TARGET_V:"<<target_v<<";TARGET_A:"<<target_a;
        return;
    }


        
    }
    else{
        for(int i=0;i<times_profile.size()-current_id;i++){

                //第一个时间需要变化一下。
                //LOG(INFO)<<"TEST1";
                double s = dists[current_id] - dist_from_start;
                double first_time=0;
                if(times_profile.size()==1){
                    first_time = sqrt(s/0.3);
                    if(first_time<0.2){
                        first_time=0.2;
                    }
                    first_time = times_profile[0]*s/dist;
                   
                    SpeedPlan speed_planner(1,6);
                    start_status(0) = dist_from_start;
                    start_status(1) = target_v;
                    start_status(2) = target_a;
                    end_status(0) = dist;
                    end_status(1) = 0;
                    end_status(2) = 0;
                    speed_planner.calc_opt_analytical_coff( start_status, end_status, cfg_->robot.max_vel_x, cfg_->robot.acc_lim_x*1.1, first_time,coff1_);
                    vector<double> coff2;
                    for(int i=0;i<6;i++){
                        coff2.push_back(coff1_(i));
                    }
                    target_v = speed_planner.calc_vel(coff2,lookahead_time);
                    target_a = speed_planner.calc_a(coff2,lookahead_time);
                    return;

                }else{
                     first_time = 2.0*s/(target_v+vel_profile[current_id+1]);
                }
            first_time = times_profile[current_id]*s/(dists[current_id]-dists[current_id-1]);
            SpeedPlan speed_planner(1,6);
            start_status(0) = dist_from_start;
            start_status(1) = target_v;
            start_status(2) = target_a;
            end_status(0) = dist;
            end_status(1) = 0;
            end_status(2) = 0;
            speed_planner.calc_opt_analytical_coff( start_status, end_status, cfg_->robot.max_vel_x, cfg_->robot.acc_lim_x*1.1, first_time,coff1_);
            vector<double> coff2;
            for(int i=0;i<6;i++){
                coff2.push_back(coff1_(i));
            }
            target_v = speed_planner.calc_vel(coff2,lookahead_time);
            target_a = speed_planner.calc_a(coff2,lookahead_time);
            //LOG(INFO)<<"1111111TARGET_V:"<<target_v<<";TARGET_A:"<<target_a;
            return;



            opt_times.push_back(first_time);


            opt_max_vel.push_back(max_vel_profile_[current_id+i]);
            opt_dist.push_back(dists[current_id+i]);
        }
    }


//    for(auto t:opt_times){
//        LOG(INFO)<<t;
//    }

//    LOG(INFO)<<"opt_times.size:"<<opt_times.size();
//    LOG(INFO)<<"opt_dist.size:"<<opt_dist.size();
//    LOG(INFO)<<"opt_max_vel.size:"<<opt_max_vel.size();
   // LOG(INFO)<<"规划时，起始点的状态："<<dist_from_start<<";v:"<<target_v<<";a:"<<target_a;
    //LOG(INFO)<<"剩余距离："<<dist_to_goal<<"第一段的时间："<<opt_times[0]<<"第一段的距离"<<opt_dist[0];

    SpeedPlan speed_planner(opt_times.size(),6);
    start_status(0) = dist_from_start;
    start_status(1) = target_v;
    start_status(2) = target_a;
    if(dist_to_goal<0.01){
        target_v = 0.01;
        target_a =0;
        return;
    }
   // LOG(INFO)<<"第一次开始优化！！！";
    int64_t time1  = sros::core::util::get_time_in_ms();
    speed_planner.calc_P(opt_times);

    speed_planner.calc_A(start_status,opt_times,opt_dist,opt_max_vel,cfg_->robot.acc_lim_x*1.1);

    speed_planner.optimize();
    int64_t time2  = sros::core::util::get_time_in_ms();


    // VectorXd coff1;
    // SpeedPlan speed_planner1(1,6);
    // start_status(0) = dist_from_start;
    // start_status(1) = target_v;
    // start_status(2) = target_a;
    // end_status(0) = speed_planner.calc_dist(speed_planner.coff_[0],opt_times[0]);
    // double end_v=0;
    // if(speed_planner.calc_vel(speed_planner.coff_[0],opt_times[0])>cfg_->robot.max_vel_x){
    //     end_v = cfg_->robot.max_vel_x;
    // }
    // else{
    //     end_v = speed_planner.calc_vel(speed_planner.coff_[0],opt_times[0]);
    // }

    // double end_a;
    // if(fabs(speed_planner.calc_a(speed_planner.coff_[0],opt_times[0]))>cfg_->robot.acc_lim_x){
    //     if(speed_planner.calc_a(speed_planner.coff_[0],opt_times[0])>0){
    //         end_a = cfg_->robot.acc_lim_x;
    //     }else{
    //         end_a = -cfg_->robot.acc_lim_x;
    //     }
        
    // }
    // else{
    //     end_a = speed_planner.calc_a(speed_planner.coff_[0],opt_times[0]);
    // }
    // end_status(1) = end_v;
    // end_status(2) = end_a;
    // LOG(INFO)<<"start_status:"<<start_status<<";end_status:"<<end_status;
    // double first_time;
    // if(opt_times[0]>0.3){
    //     first_time = opt_times[0]-0.3;
    // }
    // else{
    //     first_time = 0.01;
    // }

    // speed_planner1.calc_opt_analytical_coff1( start_status, end_status, cfg_->robot.max_vel_x, cfg_->robot.acc_lim_x*1.1, opt_times[0],coff1);
    // vector<double> coff2;
    // for(int i=0;i<6;i++){
    //     coff2.push_back(coff1(i));
    // }
    // target_v = speed_planner1.calc_vel(coff2,lookahead_time);
    // target_a = speed_planner1.calc_a(coff2,lookahead_time);

    if(opt_times[0]>lookahead_time){
        target_v = speed_planner.calc_vel(speed_planner.coff_[0],lookahead_time);
        target_a = speed_planner.calc_a(speed_planner.coff_[0],lookahead_time);
    }
    else if(speed_planner.coff_.size()>1){
        target_v = speed_planner.calc_vel(speed_planner.coff_[1],lookahead_time-opt_times[0]);
        target_a = speed_planner.calc_a(speed_planner.coff_[1],lookahead_time-opt_times[0]);
    }
    else if(speed_planner.coff_.size()==1){
        target_v = speed_planner.calc_vel(speed_planner.coff_[0],opt_times[0]);
        target_a = speed_planner.calc_a(speed_planner.coff_[0],opt_times[0]);
    }

   //LOG(INFO)<<current_id<<";TARGET_V:"<<target_v<<";TARGET_A:"<<target_a;
   // LOG(INFO)<<"1111111111111111OPTTIME0:"<<opt_times[0]<<";dists0:"<<opt_dist[0]-dist_from_start<<";opt_max_vel:"<<opt_max_vel[0];



}


// vector<double> after_opt_dist 由于优化后每一段的距离和原来的不一样。 所以需要根据AGV走过的s 和after_opt_dist来判断采用哪个多项式系数。
// current_dist 表示当前已经走过的距离。这个值非常重要。
// poly_coff 表示每一段的5次多项式系数。
// 然后采用伴随矩阵求多项式根。在根据根求目标速度和目标加速度。
//
double PlanningFSM::calc_target_vel(std::vector<Pose>&  followed_path,vector<double> times_profile,double dist_to_goal){
    // 首先计算优化完成之后新的距离
    SpeedPlan speed_planner(10,6);

    vector<double> dists;
    double dist =0;
    for(int i=0;i<followed_path.size()-1;i++){
        dist += sqrt(pow(followed_path[i].x()-followed_path[i+1].x(),2)+ pow(followed_path[i].y()-followed_path[i+1].y(),2));
    }
    double dist1=0;
    for(int i=0;i<poly_coff_.size();i++){
        dist1 = speed_planner.calc_dist(poly_coff_[i],times_profile[i]);
        //LOG(INFO)<<"经过优化后每段的距离值："<<dist1;
        dists.push_back(dist1);
    }


    // 根据path_index 和当前位置计算下已经走过的距离值。然后出去下一时刻的目标速度。
    //LOG(INFO)<<"路径总长度："<<dist<<";dist_to_goal"<<dist_to_goal;
    double dist_from_start = dist - dist_to_goal;
    int current_id=0;
    for(int i=0;i<dists.size();i++){
        if(dists[i]>dist_from_start){
            current_id = i;
           // LOG(INFO)<<"当前位置所在的段是第几段。从0 开始："<<current_id;
            break;
        }
    }


    // 根据距离反求t 然后在求速度。
    double current_t = speed_planner.solve_poly_root(poly_coff_[current_id],dist_from_start,times_profile[current_id]);



    LOG(INFO)<<"DIST_FROM_START："<<dist_from_start<<";current_T:"<<current_t;

    double target_v = speed_planner.calc_vel(poly_coff_[current_id],current_t+0.02);

    LOG(INFO)<<"计算的前方0.02s的目标速度为："<<target_v;
    return target_v;
}


bool  PlanningFSM::process_rotate(double &w, double&w_a,double current_angle, double target_angle, double max_w,
                                double acc_w){
   double angle_error = target_angle - current_angle;
    angle_error = normalizeYaw(angle_error);
    VectorXd coff1;
    Vector3d start_status,end_status;
    double first_time;
    SpeedPlan speed_planner(1,6);
    start_status(0) = 0;
    start_status(1) = fabs(w);
    start_status(2) = w_a;
    end_status(0) = fabs(angle_error);
    end_status(1) = 0;
    end_status(2) = 0;
    LOG(INFO)<<"START_STATUSw:"<<fabs(w)<<";wa:"<<w_a<<";angle_error:"<<fabs(angle_error);
    speed_planner.calc_opt_analytical_coff( start_status, end_status, max_w,acc_w, first_time,coff1_);
    vector<double> coff2;
    for(int i=0;i<6;i++){
        coff2.push_back(coff1(i));
        LOG(INFO)<<"XISHU:"<<coff1(i);
    }
    w = speed_planner.calc_vel(coff2,0.02);
    w_a = speed_planner.calc_a(coff2,0.02);

    const double VELOCITY_THRESHOLD = 0.02;
    if (w < VELOCITY_THRESHOLD) {
        w = VELOCITY_THRESHOLD;
    }
    if(w>max_w)w = max_w;
    LOG(INFO)<<"w:"<<w<<";w_a:"<<w_a;
    if (angle_error > 0) {
    } else {
        w = -1.0 * w;
    }

    const double ANGLE_GOAL_TOLERANCE=0.02;
    if (abs(angle_error) < ANGLE_GOAL_TOLERANCE&&fabs(w)<0.03) {
        w = 0;
        // LOG(INFO) << "local_planner:完成原地旋转";
        return true;
    }

    return false;
}


// 问题.当AGV进入pause还没有减速到0时障碍物消失。此时AGV需要重新加速。但是有时候当前点到最近前方点的距离较小。
// 根据之前的速度轮廓计算的t会导致AGV的加速度过大。所以采用一个新函数来计算第一段的时间。

