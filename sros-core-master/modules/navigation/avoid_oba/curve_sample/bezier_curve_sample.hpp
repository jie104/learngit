//
// Created by lfc on 18-12-10.
//

#ifndef PROJECT_BEZIER_CURVE_SAMPLE_HPP
#define PROJECT_BEZIER_CURVE_SAMPLE_HPP

#include "base_curve_sample.hpp"
#include "bezier_curve_operator.hpp"

namespace sample{
class BezierCurveSample: public BaseCurveSample {
public:
    BezierCurveSample(const sros::core::NavigationPath<double> &path,double step,int direction):BaseCurveSample(TYPE_BEZIER_CURVE) {
        bezier_info_.reset(new bezier::BezierInfo);
        bezier_info_->start = Eigen::Vector2d(path.sx_, path.sy_);
        bezier_info_->control_1 = Eigen::Vector2d(path.cx_, path.cy_);
        bezier_info_->control_2 = Eigen::Vector2d(path.dx_, path.dy_);
        bezier_info_->end = Eigen::Vector2d(path.ex_, path.ey_);

        LOG(INFO) << "bezier info point:" << bezier_info_->start[0] << "," << bezier_info_->start[1] << "," << bezier_info_->end[0] <<
        "," << bezier_info_->end[1] << "," << bezier_info_->control_2[0] << "," << bezier_info_->control_2[1];
        if(path.direction_ == 2){
            bezier_info_->is_forward = false;
        }
        poses_.clear();
        bezier_opera_.splitCurveByStep(bezier_info_, poses_, step);//将当前路径分割成采样粒子,只有在生成路径时生成一次,后期在避障判断时,不需要重新计算
        updatePoseDirection(direction);
    }

    virtual bool isForward() {
        return bezier_info_->is_forward;
    }

    virtual Eigen::Vector2d getEndPoint(){
        return bezier_info_->end;
    }

private:
    virtual double getPercent(const avoidoba::ParticleInfo& curr_pose){
        bezier::BezierPointInfo bezier_point;
        bezier_opera_.projectToCurve(bezier_info_, curr_pose.pose, bezier_point);//目前还没有加当前位姿投影到该曲线上的偏差判断,将来可以进行优化.因为,偏差太大,是需要特殊处理的
        double percent = (double)fabs(bezier_point.t);
        return percent;
    }

    bezier::BezierCurveOperator bezier_opera_;
    bezier::BezierInfo_Ptr bezier_info_;

};
}



#endif //PROJECT_BEZIER_CURVE_SAMPLE_HPP
