//
// Created by lfc on 18-12-9.
//

#ifndef PROJECT_CIRCLE_CURVE_OPERATOR_HPP
#define PROJECT_CIRCLE_CURVE_OPERATOR_HPP

#include <Eigen/Dense>
#include "../particle_info_msg.hpp"

namespace circle{
struct CircleInfo{
    Eigen::Vector2d start_point;
    Eigen::Vector2d end_point;
    Eigen::Vector2d center_point;
    double radius;//有±
    bool is_forward = true;
};

struct CirclePointInfo{
    Eigen::Vector3d pose;
    double angle;
    double arc_length;
};

typedef std::shared_ptr<CircleInfo> CircleInfo_Ptr;
class CircleCurveOperator {//用于处理圆形曲线操作,包括将当前点投影到曲线上,计算曲线长度,将曲线以一定步长划分采样位姿;其中采样位姿方向相切于曲线
public:
    void projectToCurve(const CircleInfo_Ptr &info, const Eigen::Vector3d &cand_pose, CirclePointInfo &point_info) {
        Eigen::Vector2d cand_point = cand_pose.head<2>();
        if (inCurve(info, cand_point)) {
            computePointInfo(info, cand_point, point_info);
        }else{
            double start_angle = getCurrPointAngle(info, info->start_point);
            double end_angle = getCurrPointAngle(info, info->end_point);
            double curr_angle = getCurrPointAngle(info, cand_point);
            double delta_curve_angle = normalizeAngle(end_angle - start_angle);
            double delta_first_angle = normalizeAngle(curr_angle - start_angle);
            double delta_second_angle = normalizeAngle(end_angle - curr_angle);

            CirclePointInfo start_info,end_info;
            computePointInfo(info, info->start_point, start_info);
            computePointInfo(info, info->end_point, end_info);
            point_info = fabs(delta_first_angle) > fabs(delta_second_angle) ? end_info : start_info;
        }
    }


    double computeCurveLength(const CircleInfo_Ptr &info) {
        CirclePointInfo end_info;
        computePointInfo(info, info->end_point, end_info);
        return end_info.arc_length;
    }


    void splitCurveByStep(CircleInfo_Ptr &info, std::vector<Eigen::Vector3d>& points, double step) {
        if (info->radius==0) {
            LOG(INFO) << "radius is wrong!";
            return;
        }
        double angle_step = step / info->radius;
        double start_angle = getCurrPointAngle(info, info->start_point);
        double total_length = computeCurveLength(info);
        double max_size = total_length / step;
        int point_count = 0;
        double curr_arc_length = 0;
        while ((curr_arc_length < total_length)) {

            auto curr_point = getCurvePoint(info, start_angle);
            CirclePointInfo curr_info;
            computePointInfo(info, curr_point.head<2>(), curr_info);
            curr_arc_length = curr_info.arc_length;
            if (curr_arc_length < total_length) {
                points.push_back(curr_point);
            }
            start_angle += angle_step;
            point_count++;
            if (point_count > max_size) {
                break;
            }
        }
    }

    void splitCurveByStep(CircleInfo_Ptr &info, std::vector<avoidoba::ParticleInfo>& points, double step) {
        if (info->radius==0) {
            LOG(INFO) << "radius is wrong!";
            return;
        }
        double angle_step = step / info->radius;
        double start_angle = getCurrPointAngle(info, info->start_point);
        start_angle += angle_step;
        double total_length = computeCurveLength(info);
        double max_size = total_length / step;
        int point_count = 0;
        double curr_arc_length = 0;
        while ((curr_arc_length < total_length)) {
            auto curr_point = getCurvePoint(info, start_angle);
            CirclePointInfo curr_info;
            computePointInfo(info, curr_point.head<2>(), curr_info);
            curr_arc_length = curr_info.arc_length;
            if (curr_arc_length < total_length) {
                avoidoba::ParticleInfo particle;
                setToParticle(info, curr_point, particle);
                points.push_back(particle);
            }
            start_angle += angle_step;
            point_count++;
            if (point_count > max_size) {
                break;
            }
        }
    }


    void setToParticle(const CircleInfo_Ptr &info, const Eigen::Vector3d& pose, avoidoba::ParticleInfo& particle) {
        if (info->is_forward) {
            particle.move_state = avoidoba::FORWARD_MOVE;
        }else {
            particle.move_state = avoidoba::BACKWARD_MOVE;
        }
        particle.pose = pose;
    }

    bool inCurve(const CircleInfo_Ptr &circle_info,const Eigen::Vector2d& point){
        double start_angle = getCurrPointAngle(circle_info, circle_info->start_point);
        double end_angle = getCurrPointAngle(circle_info, circle_info->end_point);
        double curr_angle = getCurrPointAngle(circle_info, point);
        double delta_curve_angle = normalizeAngle(end_angle - start_angle);
        double delta_first_angle = normalizeAngle(curr_angle - start_angle);
        double delta_second_angle = normalizeAngle(end_angle - curr_angle);

        if (circle_info->radius >= 0) {
            if (delta_curve_angle >= 0) {
                if (delta_first_angle >= 0 && delta_second_angle >= 0) {
                    return true;
                }else{
                    return false;
                }
            }else{
                if (delta_first_angle < 0 && delta_second_angle < 0) {
                    return false;
                }else{
                    return true;
                }
            }
        }else {
            if (delta_curve_angle > 0) {
                if (delta_first_angle > 0 && delta_second_angle > 0) {
                    return false;
                }else{
                    return true;
                }
            }else{
                if (delta_first_angle <= 0 && delta_second_angle <= 0) {
                    return true;
                }else{
                    return false;
                }
            }
        }
        return false;
    }

    void computePointInfo(const CircleInfo_Ptr &info,const Eigen::Vector2d &cand_point,CirclePointInfo &point_info){
        double start_angle = getCurrPointAngle(info, info->start_point);
        double curr_angle = getCurrPointAngle(info, cand_point);
        double delta_angle = normalizeAngle(curr_angle - start_angle);

        point_info.pose = getCurvePoint(info, curr_angle);

        if (delta_angle * info->radius >= 0) {//angle方向与半径方向一致,说明是小角
            point_info.angle = delta_angle;
            point_info.arc_length = delta_angle * info->radius;
        }else {//angle方向与半径方向不一致,说明是大角
            if (info->radius == 0) {
                LOG(INFO)<<"radius is err! will set to 1";
                info->radius = 1.0;
            }
            point_info.angle = M_PI * 2.0 * info->radius / (double)fabs(info->radius) + delta_angle;//将角度化成合适角度
            point_info.arc_length = M_PI * 2.0 * info->radius - (double)fabs(delta_angle * info->radius);//将弧长化成合适弧长
        }
    }


    Eigen::Vector3d getCurvePoint(const CircleInfo_Ptr &info,double angle) {
        /* Formula from Wikipedia article on Bezier curves. */
        Eigen::Vector2d curve_point = info->center_point + Eigen::Vector2d(fabs(info->radius) * cos(angle), fabs(info->radius) * sin(angle));
        double cand_angle = angle + M_PI / 2.0;
        if (info->radius < 0.0) {
            cand_angle += M_PI;
        }
        if (!info->is_forward) {
            cand_angle += M_PI;
        }
        return Eigen::Vector3d(curve_point[0], curve_point[1], normalizeAngle(cand_angle));
    }

    double getCurrPointAngle(const CircleInfo_Ptr &circle_info,const Eigen::Vector2d& other_point){
        Eigen::Vector2d to_center_vector = other_point - circle_info->center_point;
        return atan2(to_center_vector[1], to_center_vector[0]);
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

private:
    double computeCrossValue(const Eigen::Vector2d& base_point,const Eigen::Vector2d& other_point) {
        return base_point[0] * other_point[1] - base_point[1] * other_point[0];
    }

};
}



#endif //PROJECT_CIRCLE_CURVE_OPERATOR_HPP
