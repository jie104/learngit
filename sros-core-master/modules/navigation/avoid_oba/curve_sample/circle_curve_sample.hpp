//
// Created by lfc on 18-12-10.
//

#ifndef PROJECT_CIRCLE_CURVE_SAMPLE_HPP
#define PROJECT_CIRCLE_CURVE_SAMPLE_HPP

#include "base_curve_sample.hpp"
#include "circle_curve_operator.hpp"

namespace sample{
class CircleCurveSample: public BaseCurveSample {
public:
    CircleCurveSample(const sros::core::CirclePath& path,double step,int direction):BaseCurveSample(TYPE_CIRCLE_CURVE){
        circle_info_.reset(new circle::CircleInfo);
        circle_info_->start_point = Eigen::Vector2d(path.sx_, path.sy_);
        circle_info_->center_point = Eigen::Vector2d(path.cx_, path.cy_);
        circle_info_->end_point = Eigen::Vector2d(path.ex_, path.ey_);
        circle_info_->radius = path.radius_;
        if(path.direction_ == 2){
            circle_info_->is_forward = false;
        }
        poses_.clear();
        circle_opera_.splitCurveByStep(circle_info_, poses_, step);//将当前路径分割成采样粒子,只有在生成路径时生成一次,后期在避障判断时,不需要重新计算
        LOG(INFO) << "pose size:" << poses_.size();
        updatePoseDirection(direction);
    }


    virtual bool isForward() {
        return circle_info_->is_forward;
    }

    virtual Eigen::Vector2d getEndPoint(){
        return circle_info_->end_point;
    }

private:
    virtual double getPercent(const avoidoba::ParticleInfo& curr_pose){
        circle::CirclePointInfo circle_point;
        circle_opera_.projectToCurve(circle_info_, curr_pose.pose, circle_point);//目前还没有加当前位姿投影到该曲线上的偏差判断,将来可以进行优化.因为,偏差太大,是需要特殊处理的
        double total_length = circle_opera_.computeCurveLength(circle_info_);
        double percent = (double)fabs(circle_point.arc_length/total_length);
        return percent;
    }

    circle::CircleCurveOperator circle_opera_;

    circle::CircleInfo_Ptr circle_info_;
};


}


#endif //PROJECT_CIRCLE_CURVE_SAMPLE_HPP
