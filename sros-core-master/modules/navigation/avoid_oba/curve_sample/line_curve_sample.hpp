//
// Created by lfc on 18-12-10.
//

#ifndef PROJECT_LINE_CURVE_SAMPLE_HPP
#define PROJECT_LINE_CURVE_SAMPLE_HPP

#include "base_curve_sample.hpp"
#include "line_curve_operator.hpp"

namespace sample{
class LineCurveSample: public BaseCurveSample {
public:
    LineCurveSample(const sros::core::NavigationPath<double>& path,double sample_step,int direction):BaseCurveSample(TYPE_LINE_CURVE) {
        line_info_.reset(new line::LineInfo);
        line_info_->start_point[0] = path.sx_;
        line_info_->start_point[1] = path.sy_;
        line_info_->end_point[0] = path.ex_;
        line_info_->end_point[1] = path.ey_;
        if(path.direction_ == 2){
            line_info_->is_forward = false;
        }
        poses_.clear();
        line_opera_.splitCurveByStep(line_info_, poses_, sample_step);//将当前路径分割成采样粒子,只有在生成路径时生成一次,后期在避障判断时,不需要重新计算
        updatePoseDirection(direction);
        LOG(INFO) << "line!";
    }

    virtual Eigen::Vector2d getEndPoint(){
        return line_info_->end_point;
    }

    virtual bool isForward() {
        return line_info_->is_forward;
    }

private:
    virtual double getPercent(const avoidoba::ParticleInfo& curr_pose){
        line::LinePointInfo line_point;
        line_opera_.projectToCurve(line_info_, curr_pose.pose, line_point);
        double total_length = line_opera_.computeCurveLength(line_info_);//目前还没有加当前位姿投影到该曲线上的偏差判断,将来可以进行优化.因为,偏差太大,是需要特殊处理的
        double percent = (double)std::fabs(line_point.curr_length / total_length);
        return percent;
    }

    line::LineCurveOperator line_opera_;

    line::LineInfo_Ptr line_info_;


};


}


#endif //PROJECT_LINE_CURVE_SAMPLE_HPP
