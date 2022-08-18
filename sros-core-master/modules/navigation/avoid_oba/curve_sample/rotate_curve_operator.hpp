//
// Created by lfc on 18-12-10.
//

#ifndef PROJECT_ROTATE_CURVE_OPERATOR_HPP
#define PROJECT_ROTATE_CURVE_OPERATOR_HPP

#include <Eigen/Dense>
#include <memory>
#include "../particle_info_msg.hpp"

namespace rotate{
struct RotateInfo{
    Eigen::Vector2d center_info;
    double start_angle;
    double end_angle;//无法预测旋转方向,只能做两次判断,是否在旋转范围内.这里,如果方向相反,只能按照倒退处理.
};
struct RotatePointInfo{
    Eigen::Vector3d pose;
    double curr_length;
};

typedef std::shared_ptr<RotateInfo> RotateInfo_Ptr;

const double rotate_equival_distance = 0.5;
class RotateCurveOperator {////用于处理旋转操作,包括将当前点投影到旋转上,将旋转以一定步长划分采样位姿;
public:

    static void projectToCurve(const RotateInfo_Ptr& info,const Eigen::Vector3d &cand_pose,RotatePointInfo &point_info){
        if (inCurve(info, cand_pose)) {
            computePointInfo(info,cand_pose, point_info);
        }else{
            double first_delta_angle = fabs(normalizeAngle(cand_pose[2] - info->start_angle));
            double second_delta_angle = fabs(normalizeAngle(info->end_angle - cand_pose[2]));
            RotatePointInfo start_point,end_point;
            start_point.pose.head<2>() = info->center_info;
            end_point.pose.head<2>() = info->center_info;
            start_point.pose[2] = info->start_angle;
            end_point.pose[2] = info->end_angle;
            computePointInfo(info, start_point.pose, start_point);
            computePointInfo(info, end_point.pose, end_point);
            point_info = first_delta_angle > second_delta_angle ? end_point : start_point;
        }
    }

    static bool inCurve(const RotateInfo_Ptr& info,const Eigen::Vector3d &cand_pose){
        double first_delta_angle = normalizeAngle(cand_pose[2] - info->start_angle);
        double second_delta_angle = normalizeAngle(info->end_angle - cand_pose[2]);
        if (first_delta_angle * second_delta_angle >= 0) {
            return true;
        }else {
            return false;
        }
    }

    static void computePointInfo(const RotateInfo_Ptr &info,const Eigen::Vector3d &cand_point,RotatePointInfo &point_info){
        double first_delta_angle = normalizeAngle(cand_point[2] - info->start_angle);
        double second_delta_angle = normalizeAngle(info->end_angle - cand_point[2]);
        double delta_angle = normalizeAngle(info->end_angle - info->start_angle);
        if (first_delta_angle * delta_angle >= 0 && second_delta_angle * delta_angle >= 0) {//如果相同,说明两个方向相同
            point_info.pose = cand_point;
        }else {
            point_info.pose = cand_point;
            point_info.pose[2] = normalizeAngle(cand_point[2] + M_PI);
        }
        point_info.curr_length = (double) fabs(first_delta_angle) * rotate_equival_distance;
    }


    static void setToParticle(const RotateInfo_Ptr &info, const Eigen::Vector3d& pose, avoidoba::ParticleInfo& particle) {
        particle.move_state = avoidoba::ROTATE_MOVE;
        particle.pose = pose;
    }


    static void splitCurveByStep(const RotateInfo_Ptr &info, std::vector<avoidoba::ParticleInfo>& points, double step) {
        auto curve_length = computeCurveLength(info);
        if (curve_length <= 0) {
//            LOG(INFO) << "length is err:" << curve_length << info->start_angle << "," << info->end_angle;
            return;
        }
        double delta_angle = normalizeAngle(info->end_angle - info->start_angle);
        double max_size = curve_length / step;

        double angle_step = step / rotate_equival_distance;
        angle_step = angle_step * (delta_angle) / fabs(delta_angle);

        double curr_length  = 0;

        double point_count = 0;
        double start_angle = info->start_angle;
        while (curr_length < curve_length) {
            Eigen::Vector3d curr_point;
            curr_point.head<2>() = info->center_info;

            curr_point[2] = start_angle;
            start_angle += angle_step;

            RotatePointInfo point_info;
            computePointInfo(info, curr_point, point_info);
            curr_length = point_info.curr_length;//存在死循环的情况,因为是角度直接转化的,所以有可能永远加不完
            avoidoba::ParticleInfo particle;
            setToParticle(info, point_info.pose, particle);
            points.push_back(particle);
            point_count++;
            if (point_count > max_size) {
                break;//因为length是循环的,如果旋转是180°,有可能导致永远在死循环里,出不来.
            }
        }
    }
    static void splitCurveByStep(const RotateInfo_Ptr &info, std::vector<Eigen::Vector3d>& points, double step) {
        auto curve_length = computeCurveLength(info);
        if (curve_length <= 0) {
//            LOG(INFO)<<"length is err:"<<curve_length;
            return;
        }
        double delta_angle = normalizeAngle(info->end_angle - info->start_angle);
        double max_size = curve_length / step;

        double angle_step = step / rotate_equival_distance;
        angle_step = angle_step * (delta_angle) / fabs(delta_angle);

        double curr_length  = 0;

        double point_count = 0;
        double start_angle = info->start_angle;
        while (curr_length < curve_length) {
            Eigen::Vector3d curr_point;
            curr_point.head<2>() = info->center_info;

            curr_point[2] = start_angle;
            start_angle += angle_step;

            RotatePointInfo point_info;
            computePointInfo(info, curr_point, point_info);
            curr_length = point_info.curr_length;//存在死循环的情况,因为是角度直接转化的,所以有可能永远加不完
            points.push_back(point_info.pose);
            point_count++;
            if (point_count > max_size) {
                break;//因为length是循环的,如果旋转是180°,有可能导致永远在死循环里,出不来.
            }
        }
    }


    template<class T>
    static double normalizeAngle(T angle) {
        const T &max_angle = (T) (2 * M_PI);
        angle = fmod(angle, (max_angle));
        if (angle >= (T) (M_PI)) {
            angle -= max_angle;
        } else if (angle < -(T) (M_PI)) {
            angle += max_angle;
        }
        return angle;
    }

    static double computeCurveLength(const RotateInfo_Ptr &info) {
        return (double)fabs(normalizeAngle(info->start_angle - info->end_angle) * rotate_equival_distance);
    }
private:
};
}



#endif //PROJECT_ROTATE_CURVE_OPERATOR_HPP
