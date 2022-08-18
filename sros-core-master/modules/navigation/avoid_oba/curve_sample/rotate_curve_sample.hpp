//
// Created by lfc on 18-12-10.
//

#ifndef PROJECT_ROTATE_CURVE_SAMPLE_HPP
#define PROJECT_ROTATE_CURVE_SAMPLE_HPP

#include "base_curve_sample.hpp"
#include "rotate_curve_operator.hpp"

namespace sample{
class RotateCurveSample: public BaseCurveSample {
public:
    RotateCurveSample(const sros::core::CirclePath& path,double step,int direction):BaseCurveSample(TYPE_ROTATE_CURVE){
        sample_step_ = step;
        rotate_info_.reset(new rotate::RotateInfo);
        rotate_info_->center_info = Eigen::Vector2d(path.sx_, path.sy_);
        rotate_info_->end_angle = path.rotate_angle_;
        if (path.direction_ == 0x21) {
            LOG(INFO) << "curr path doesnt need build samples! no is:" << path.direction_;
            build_rotate_samples_ = false;
        }
        LOG(INFO) << "rotate!";
    }

    virtual void getSomeSamples(std::vector<avoidoba::ParticleInfo> &samples, const int max_size, int &curr_real_size) {//旋转因为没有初始位姿，所以处理与其他类型不一样
        if (samples.size() == 0) {
            LOG(INFO) << "cannot distribute rotate particle! cannot get start pose!";
            curr_real_size = 0;
            return;
        }
        rotate_info_->start_angle = samples.back().pose[2];
        samples.emplace_back();
        auto& rotate_pose = samples[samples.size() - 2];
        samples.back().start_angle = rotate_info_->start_angle;
        samples.back().end_angle = rotate_info_->end_angle;
        samples.back().pose = rotate_pose.pose;
        if (build_rotate_samples_) {
            samples.back().move_state = avoidoba::ROTATE_MOVE;
        }else{
            samples.back().move_state = avoidoba::NO_MOVE_STATE;
        }

//        double start_angle = rotate_info_->start_angle;
//        poses_.clear();
//        rotate_opera_.splitCurveByStep(rotate_info_, poses_, sample_step_);
//        int sum_count = 0;
//        for (auto &pose:poses_) {
//            if (sum_count > 0) {
//                samples.push_back(pose);
//            }
//            sum_count++;
//            if (samples.size() >= max_size) {
//                break;
//            }
//        }
//        curr_real_size = sum_count;
    }

    virtual bool isForward() {
        return true;
    }


    virtual Eigen::Vector2d getEndPoint(){
        return rotate_info_->center_info;
    }

    virtual Eigen::Vector3d getStartPose() {  //获取起始点位姿
        return Eigen::Vector3d(rotate_info_->center_info[0], rotate_info_->center_info[1], rotate_info_->end_angle);
    }

 private:
    virtual double getPercent(const avoidoba::ParticleInfo& curr_pose){
        return 0;
    }

    rotate::RotateCurveOperator rotate_opera_;
    rotate::RotateInfo_Ptr rotate_info_;
    double sample_step_;
    bool build_rotate_samples_ = true;
};


}


#endif //PROJECT_ROTATE_CURVE_SAMPLE_HPP
