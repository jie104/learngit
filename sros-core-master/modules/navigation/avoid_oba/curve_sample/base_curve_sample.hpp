//
// Created by lfc on 18-12-10.
//

#ifndef PROJECT_BASECURVESAMPLE_HPP
#define PROJECT_BASECURVESAMPLE_HPP
#include "core/navigation_path.h"
#include <Eigen/Dense>
#include "../particle_info_msg.hpp"
#include <memory>
#include <glog/logging.h>
namespace sample{
enum CurveSampleType{
    TYPE_LINE_CURVE = 1,
    TYPE_CIRCLE_CURVE = 2,
    TYPE_ROTATE_CURVE = 3,
    TYPE_BEZIER_CURVE = 4,
};
class BaseCurveSample {
public:
    BaseCurveSample(CurveSampleType type):type_(type){

    }

    virtual ~BaseCurveSample(){

    }

    CurveSampleType getType() {
        return type_;
    }

    virtual bool isForward() = 0;//是否是前进路径.

    virtual Eigen::Vector2d getEndPoint() = 0;//获取端点位置

    virtual Eigen::Vector3d getStartPose() {  //获取起始点位姿
        if (poses_.size()) {
            return poses_[0].pose;
        }
        LOG(INFO) << "cannot get start pose! will return zero!";
        return Eigen::Vector3d::Zero();
    }

    virtual void getSomeSamples(std::vector<avoidoba::ParticleInfo> &samples, const int max_size, int &curr_real_size){//生成采样粒子,除原地旋转外,所有获取采样粒子方法均使用该接口
        if (samples.size() == 0) {
            int sum_count = 0;
            for (auto &pose:poses_) {
                samples.push_back(pose);
                sum_count++;
                if (sum_count >= max_size) {
                    break;
                }
            }
            curr_real_size = sum_count;
            return;
        }else{
            double percent = getPercent(samples.back());
            int pose_size = (int)std::floor((double) poses_.size() * percent + 0.01) + 1;//加上0.01的目的是,防止出现0.999999而被削成0的情况
            if (pose_size >= poses_.size()) {//以下是为了保证pose size不大于最大采样个数
                curr_real_size = 0;
                return;
            }
            int max_pose_size = poses_.size();

            int sum_count = 0;
            for (int i = pose_size; i < max_pose_size; ++i) {
                if (samples.size() < max_size) {
                    samples.push_back(poses_[i]);
                }else {
                    break;
                }
                sum_count++;
            }
            curr_real_size = sum_count;
            return;
        }
    }


protected:
    std::vector<avoidoba::ParticleInfo> poses_;//用于存储曲线按照一定步长采样出的粒子(原地旋转除外),原地旋转只需要设置开始角度和终止角度

    void updatePoseDirection(int direction){
        if(direction==3||direction==4){
            auto delta_angle = M_PI_2;
            if(direction==4){
                delta_angle = -M_PI_2;
            }
            for(auto& pose:poses_){
                pose.pose[2] += delta_angle;
            }
        }
    }

    virtual double getPercent(const avoidoba::ParticleInfo& curr_pose) = 0;//获取当前位姿在poses中的位置,用来确定采样个数,在子类实现
private:
    CurveSampleType type_;//曲线类型

};

typedef std::shared_ptr<BaseCurveSample> BaseCurveSample_Ptr;
}



#endif //PROJECT_BASECURVESAMPLE_HPP
