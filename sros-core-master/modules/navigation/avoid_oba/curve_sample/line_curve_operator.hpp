//
// Created by lfc on 18-12-10.
//

#ifndef PROJECT_LINE_CURVE_OPERATOR_HPP
#define PROJECT_LINE_CURVE_OPERATOR_HPP

#include <Eigen/Dense>
#include <glog/logging.h>
#include "../particle_info_msg.hpp"

namespace line{
struct LineInfo{
    Eigen::Vector2d start_point;
    Eigen::Vector2d end_point;
    bool is_forward = true;
};

struct LinePointInfo{
    Eigen::Vector3d pose;
    double curr_length;
};
typedef std::shared_ptr<LineInfo> LineInfo_Ptr;

class LineCurveOperator {//用于处理直线操作,包括将当前点投影到曲线上,计算曲线长度,将曲线以一定步长划分采样位姿;其中采样位姿方向相切于曲线
public:
    void projectToCurve(const LineInfo_Ptr& info,const Eigen::Vector3d &cand_pose,LinePointInfo &point_info){
        Eigen::Vector2d cand_point = cand_pose.head<2>();
        if (inCurve(info, cand_point)) {
            computePointInfo(info,cand_point, point_info);
        }else{
            LinePointInfo start_info;
            LinePointInfo end_info;
            computePointInfo(info, info->start_point, start_info);
            computePointInfo(info, info->end_point, end_info);
            double start_to_curr_norm = (info->start_point - cand_point).norm();
            double end_to_curr_norm = (info->end_point - cand_point).norm();
            point_info = start_to_curr_norm > end_to_curr_norm ? end_info : start_info;
        }
    }

    bool inCurve(const LineInfo_Ptr& info,const Eigen::Vector2d &cand_point){
        Eigen::Vector2d line_vector = info->end_point - info->start_point;
        if (line_vector.norm() == 0) {
            return false;
        }
        auto unit_vector = line_vector * (1 / line_vector.norm());
        auto curr_point = unit_vector.dot(cand_point - info->start_point) * unit_vector + info->start_point;
        auto first_vector = curr_point - info->start_point;
        auto second_vector = curr_point - info->end_point;
        if (first_vector.dot(second_vector)<=0) {
            return true;
        }else{
            return false;
        }
    }

    void computePointInfo(const LineInfo_Ptr &info,const Eigen::Vector2d &cand_point,LinePointInfo &point_info){
        Eigen::Vector2d line_vector = info->end_point - info->start_point;
        if (line_vector.norm() == 0) {
            point_info.pose.setZero();
            point_info.curr_length = 0;
            LOG(INFO) << "vector length is zero! will return!";
            return;
        }
        auto unit_vector = line_vector * (1 / line_vector.norm());
        auto curr_point = unit_vector.dot(cand_point - info->start_point) * unit_vector + info->start_point;
        point_info.pose.head<2>() = curr_point;
        point_info.pose[2] = atan2(unit_vector[1], unit_vector[0]);
        point_info.curr_length = (curr_point - info->start_point).norm();
        if (!info->is_forward) {
            point_info.pose[2] += M_PI;
        }
    }


    void splitCurveByStep(const LineInfo_Ptr &info, std::vector<Eigen::Vector3d>& points, double step) {
        auto curve_length = computeCurveLength(info);
        if (curve_length <= 0) {
            LOG(INFO)<<"length is err:"<<curve_length;
            return;
        }
        double line_step = step;
        if (line_step == 0) {
            LOG(INFO) << "line step is zero! will set to 0.01!";
            line_step = 0.01;
        }
        double curr_length = 0;
        Eigen::Vector2d line_vector = info->end_point - info->start_point;
        if (line_vector.norm() == 0) {
            LOG(INFO) << "vector length is zero! will return!";
            return;
        }
        auto unit_vector = line_vector * (1 / line_vector.norm());

        while (curr_length < curve_length) {
            Eigen::Vector3d curr_pose;
            curr_pose.head<2>() = unit_vector * curr_length + info->start_point;
            curr_pose[2] = atan2(unit_vector[1], unit_vector[0]);
            if (!info->is_forward) {
                curr_pose[2] += M_PI;
            }
            points.push_back(curr_pose);
            curr_length += line_step;
        }
    }

    void splitCurveByStep(const LineInfo_Ptr &info, std::vector<avoidoba::ParticleInfo>& points, double step) {
        auto curve_length = computeCurveLength(info);
        if (curve_length <= 0) {
            LOG(INFO)<<"length is err:"<<curve_length;
            return;
        }
        double line_step = step;
        if (line_step == 0) {
            LOG(INFO) << "line step is zero! will set to 0.01!";
            line_step = 0.01;
        }
        double curr_length = 0;
        Eigen::Vector2d line_vector = info->end_point - info->start_point;
        if (line_vector.norm() == 0) {
            LOG(INFO) << "vector length is zero! will return!";
            return;
        }
        auto unit_vector = line_vector * (1 / line_vector.norm());

        while (curr_length <= curve_length) {
            avoidoba::ParticleInfo particle;
            Eigen::Vector3d curr_pose;
            curr_pose.head<2>() = unit_vector * curr_length + info->start_point;
            curr_pose[2] = atan2(unit_vector[1], unit_vector[0]);
            if (!info->is_forward) {
                curr_pose[2] += M_PI;
            }
            setToParticle(info, curr_pose, particle);
            points.push_back(particle);
            curr_length += line_step;
        }
    }

    void setToParticle(const LineInfo_Ptr &info, const Eigen::Vector3d& pose, avoidoba::ParticleInfo& particle) {
        if (info->is_forward) {
            particle.move_state = avoidoba::FORWARD_MOVE;
        }else {
            particle.move_state = avoidoba::BACKWARD_MOVE;
        }
        particle.pose = pose;
    }

    double computeCurveLength(const LineInfo_Ptr &info) {
        return (info->end_point - info->start_point).norm();
    }
private:


};

}


#endif //PROJECT_LINE_CURVE_OPERATOR_HPP
