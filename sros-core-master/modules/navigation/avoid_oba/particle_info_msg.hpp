//
// Created by lfc on 18-12-10.
//

#ifndef PROJECT_PARTICLE_INFO_MSG_HPP
#define PROJECT_PARTICLE_INFO_MSG_HPP

#include <Eigen/Dense>
#include <memory>
#include <map>
#include <vector>

namespace avoidoba {
enum ParticleMoveState{//标记采样粒子的状态,后退,前进,旋转;因为他们避障尺寸不一样
    FORWARD_MOVE = 1,
    BACKWARD_MOVE = 2,
    ROTATE_MOVE = 3,
    NO_MOVE_STATE = 4,
};

struct ParticleInfo{
    ParticleInfo(){

    }
    ParticleInfo(const Eigen::Vector3d& curr_pose,ParticleMoveState& state):pose(curr_pose),move_state(state){

    }
    Eigen::Vector3d pose;//每个采样粒子的中心位姿
    ParticleMoveState move_state;//采样粒子状态
    double start_angle;//主要服务于旋转粒子
    double end_angle;//主要服务于旋转粒子
};

struct CollideResultInfo {
    int index = 0;//标记采样粒子id
    bool is_collide_sample = false;
    bool is_region_oba = false;
    Eigen::Vector2d collide_point = Eigen::Vector2d::Zero();
    int collide_value = 0;//采样粒子的碰撞值
    std::string oba_name;
};

struct AvdobaPoseInfo {//避障处理器的传入参数,包含,停止采样粒子,减速采样粒子,障碍点信息,站点粒子,主要用于外部演示以及获取障碍点
    std::vector<ParticleInfo> collide_poses;
    std::vector<ParticleInfo> slow_poses;
    std::map<std::string,std::vector<Eigen::Vector2d>> oba_points;
    ParticleInfo station_pose;
    CollideResultInfo result_info;
};


struct OriginCollideSize {//避障尺寸,考虑到有可能会顶升货架,所以该尺寸是避障处理器的输入参数
    double width;
    double length;
    double car_width = 0.0;
    double car_length = 0.0;
    double rotate_angle = 0.0;
    double left_width;
    double right_width;
    double head_length;
    double back_length;
    double no_avoid_oba_back_length = 0.0;//从车体后端计算，屏蔽避障的长度，主要是考虑叉车前叉避障用
    double no_avoid_oba_head_length = 0.0;//从车体前端计算，屏蔽避障的长度
};

struct RegionOffsetPara {//避障尺寸的offset
    double slow_width_offset = 0;
    double stop_width_offset = 0;
    double stop_length_forward_offset = 0;
    //小车车头朝向offset
    double stop_length_backward_offset = 0;//小车车尾offset,这两个值主要处理小车前进/倒退
};
struct RegionAsymmetricOffsetPara {  //避障尺寸的offset
    double left_offset = 0;
    double right_offset = 0;
    double forward_offset = 0;
    //小车车头朝向offset
    double backward_offset = 0;  //小车车尾offset,这两个值主要处理小车前进/倒退

    RegionAsymmetricOffsetPara& operator -= (const RegionAsymmetricOffsetPara& para)
    {
        
        this->left_offset -= para.left_offset;
        this->right_offset -= para.right_offset;
        this->forward_offset -= para.forward_offset;
        this->backward_offset -= para.backward_offset;

        return *this;
    }
};

struct LineInterplation {//线性差值求解器,将距离差值成碰撞值
    LineInterplation(double min, double max, double min_val, double max_val) : min_x_(min), max_x_(max),
                                                                               min_value_(min_val),
                                                                               max_value_(max_val) { }

    LineInterplation() {

    }

    void updatePara(double min, double max, double min_val, double max_val){
        min_x_ = min;
        max_x_ = max;
        min_value_ = min_val;
        max_value_ = max_val;
    }

    double min_x_ = 0;
    double max_x_ = 0.5;
    double min_value_ = 0;
    double max_value_ = 900;

    double getDegreInterplation(double x) {//单调递减
        if (min_x_ == max_x_) {
            return min_value_;
        }
        if (x > max_x_) {
            return min_value_;
        }
        if (x < min_x_) {
            return max_value_;
        }
        return (x - min_x_) / (max_x_ - min_x_) * (min_value_ - max_value_) + max_value_;
    }

    double getIncreInterplation(double x) {
        if (min_x_ == max_x_) {
            return min_value_;
        }
        if (x > max_x_) {
            return max_value_;
        }
        if (x < min_x_) {
            return min_value_;
        }
        return (x - min_x_) / (max_x_ - min_x_) * (max_value_ - min_value_) + min_value_;
    }
};

typedef std::shared_ptr<OriginCollideSize> OriginCollideSize_Ptr;
typedef std::shared_ptr<RegionOffsetPara> RegionOffsetPara_Ptr;
typedef std::shared_ptr<LineInterplation> LineInterplation_Ptr;
typedef std::shared_ptr<RegionAsymmetricOffsetPara> RegionAsymmetricOffsetPara_Ptr;

}


#endif //PROJECT_PARTICLE_INFO_MSG_HPP
