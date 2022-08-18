//
// Created by lfc on 18-12-7.
//

#ifndef PROJECT_BEZIER_CURVE_OPERATOR_HPP
#define PROJECT_BEZIER_CURVE_OPERATOR_HPP

#include <Eigen/Dense>
#include <glog/logging.h>

namespace bezier {
struct BezierInfo {
    Eigen::Vector2d start;
    Eigen::Vector2d end;
    Eigen::Vector2d control_1;
    Eigen::Vector2d control_2;
    bool is_forward = true;
};

struct BezierPointInfo {
    Eigen::Vector3d pose;
    Eigen::Vector2d unit_tangent;
    //单位切线向量
    double direction;
    double project_dist;
    double t;
};
typedef std::shared_ptr<BezierInfo> BezierInfo_Ptr;

class BezierCurveOperator {//用于处理贝赛尔曲线操作,包括将当前点投影到曲线上,计算贝赛尔曲线长度,将贝赛尔曲线以一定步长划分采样位姿;其中采样位姿方向相切于贝赛尔曲线
public:
    void projectToCurve(const BezierInfo_Ptr &info, const Eigen::Vector3d &cand_pose, BezierPointInfo &point_info) {
        Eigen::Vector2d cand_point = cand_pose.head<2>();
        int iteration = 20;
        BezierPointInfo start_info, end_info;
        start_info.t = 0;
        end_info.t = 1.0;
        computeBezierPointInfo(info, start_info);
        computeBezierPointInfo(info, end_info);
        auto initial_t = getInitialT(info, cand_point);
        if (initial_t == 0) {
            point_info = start_info;
            return;
        }
        if (initial_t == 1.0) {
            point_info = end_info;
            return;
        }

        computeInitialBezierPairs(info, cand_point, start_info, end_info);//初始化二分法的初始端点,目的是在局部求解最优值
        double start_dot_value = getVectorDotValue(cand_point, start_info);//计算始端点的向量内积
        double end_dot_value = getVectorDotValue(cand_point, end_info);//计算终端点向量内积.
        if (start_dot_value * end_dot_value >= 0) {
            if (start_dot_value == 0) {
                point_info = start_info;
                return;
            }
            if (end_dot_value == 0) {
                point_info = end_info;
                return;
            }
            LOG(INFO) << "cannot find real point!" << info->start[0] << "," << info->start[1] << "," << info->end[0] <<
            "," << info->end[1] << "," << cand_pose[0] << "," << cand_pose[1] << "," << info->control_1[0] << "," <<
            info->control_1[1] << "," << info->control_2[0] << "," << info->control_2[1];
            auto t = getInitialT(info, cand_point);
            point_info.t = t;
            computeBezierPointInfo(info, point_info);
            LOG(INFO) << "initial t is:" << t;
            return;
        }
        BezierPointInfo mid_bezier;
        for (int i = 0; i < iteration; ++i) {
            mid_bezier.t = (start_info.t + end_info.t) / 2.0;
            computeBezierPointInfo(info, mid_bezier);
            double tmp_dot_value = getVectorDotValue(cand_point, mid_bezier);
            if (fabs(tmp_dot_value) < 0.0001) {
                point_info = mid_bezier;
                return;
            }
            if (start_dot_value * tmp_dot_value < 0) {
                end_info = mid_bezier;
                end_dot_value = tmp_dot_value;
            } else {
                start_info = mid_bezier;
                start_dot_value = tmp_dot_value;
            }
        }
        point_info = mid_bezier;
    }

/**
  * @brief  计算贝塞尔曲线长度
  * @param  pstPathPara:ST_PATH_PARA结构体中fpStartX，fpBezierC1X，fpBezierC2X，fpEndX
                以及fpStartY，fpBezierC1Y，fpBezierC2Y，fpEndY 用于计算贝塞尔曲线的四个点。
            Bezier_t :贝塞尔曲线参数，一般取值1.
  * @Description 通过高斯-勒让德方法计算∫（x'2+y'2）^(1/2)dx 的结果
  * @retval 贝塞尔曲线长度
  */
    double computeGaussCalBezierLength(const BezierInfo_Ptr &info, double Bezier_t) {
        double a0, a1, a2;
        double length;
        a0 = 0.887299f * Bezier_t;
        a1 = 0.5f * Bezier_t;
        a2 = 0.112701f * Bezier_t;
        length = (5.0f * computeBezierEucheanDist(info, a0) + 8.0f * computeBezierEucheanDist(info, a1) +
                  5.0f * computeBezierEucheanDist(info, a2)) * Bezier_t * 0.0555555556f;
        LOG(INFO) << "dist:" << length;
        return length;
    }

    double computeCurveLength(BezierInfo_Ptr &info, double Bezier_t = 1.0) {
        return computeGaussCalBezierLength(info, Bezier_t);
    }

    void splitCurveByStep(BezierInfo_Ptr &info, std::vector<Eigen::Vector3d>& points, double step) {
        auto curve_length = computeGaussCalBezierLength(info,1);
        if (curve_length <= 0) {
            LOG(INFO)<<"length is err:"<<curve_length;
            return;
        }
        double t_step = step / curve_length;
        double start_t = 0;
        while (start_t <= 1) {
            const Eigen::Vector3d &curr_point = getCurvePoint(info, start_t);
            points.push_back(curr_point);
            start_t += t_step;
        }
    }

    void splitCurveByStep(BezierInfo_Ptr &info, std::vector<avoidoba::ParticleInfo>& points, double step) {
        auto curve_length = computeGaussCalBezierLength(info,1);
        if (curve_length <= 0) {
            LOG(INFO)<<"length is err:"<<curve_length;
            return;
        }
        double t_step = step / curve_length;
        double start_t = 0;
        while (start_t <= 1) {
            const Eigen::Vector3d &curr_point = getCurvePoint(info, start_t);
            avoidoba::ParticleInfo particle;
            setToParticle(info, curr_point, particle);
            points.push_back(particle);
            start_t += t_step;
        }
    }


    void setToParticle(const BezierInfo_Ptr &info, const Eigen::Vector3d &pose, avoidoba::ParticleInfo& particle) {
        if (info->is_forward) {
            particle.move_state = avoidoba::FORWARD_MOVE;
        }else {
            particle.move_state = avoidoba::BACKWARD_MOVE;
        }
        particle.pose = pose;
    }

    Eigen::Vector3d getCurvePoint(const BezierInfo_Ptr &info,double t) {
        /* Formula from Wikipedia article on Bezier curves. */
        auto& start = info->start;
        auto& end = info->end;
        auto& control_1 = info->control_1;
        auto& control_2 = info->control_2;
        double t_1 = 1 - t;
        double t_squre = t * t;
        double t_1_squre = t_1 * t_1;
        double t_1_t = t_1 * t;


        Eigen::Vector2d bezier1 = t_1_squre * start + 2 * t_1_t * control_1
                                  + t_squre * control_2;

        Eigen::Vector2d bezier2 = t_1_squre * control_1 + 2 * t_1_t * control_2
                                  + t_squre * end;



        Eigen::Vector2d bezier_tangent = bezier2 - bezier1;
        Eigen::Vector2d unit_tangent = bezier_tangent * (1 / bezier_tangent.norm());
        Eigen::Vector3d pose;
        pose.head<2>() = (1 - t) * bezier1 + t * bezier2;
        pose[2] = atan2(unit_tangent[1], unit_tangent[0]);
        if(!info->is_forward) {
            pose[2] += M_PI;
        }
        return pose;
    }

private:
    /**
   * @brief  计算贝塞尔曲线长度
   * @param  pstPathPara:ST_PATH_PARA结构体中fpStartX，fpBezierC1X，fpBezierC2X，fpEndX
                 以及fpStartY，fpBezierC1Y，fpBezierC2Y，fpEndY 用于计算贝塞尔曲线的四个点。
             Bezier_t :贝塞尔曲线参数，一般取值1.
   * @Description 计算(x'2+y'2)^(1/2)的结果
   */
    double computeBezierEucheanDist(const BezierInfo_Ptr &info, double Bezier_t) {
        double bezier_ctrp[8];  //贝塞尔两个起始点+两个控制点
        double bezier_k[12] = {-1, 3, -3, 1, 2, -4, 2, 0, -1, 1, 0, 0};

        bezier_ctrp[0] = info->start[0];
        bezier_ctrp[1] = info->start[1];
        bezier_ctrp[2] = info->control_1[0];
        bezier_ctrp[3] = info->control_1[1];
        bezier_ctrp[4] = info->control_2[0];
        bezier_ctrp[5] = info->control_2[1];
        bezier_ctrp[6] = info->end[0];
        bezier_ctrp[7] = info->end[1];
        Eigen::Matrix<double, 4, 3> bezier_k_mat = (Eigen::Map<Eigen::Matrix<double, 4, 3> >(bezier_k));
        Eigen::Matrix<double, 2, 4> bezier_ctrp_mat = (Eigen::Map<Eigen::Matrix<double, 2, 4> >(bezier_ctrp));
        Eigen::Vector3d bezier_ts(Bezier_t * Bezier_t, Bezier_t, 1);

        Eigen::Vector4d bezier_tmpt;
        Eigen::Vector2d bezier_out;
        bezier_tmpt = bezier_k_mat * bezier_ts;
        bezier_out = bezier_ctrp_mat * bezier_tmpt;
        auto dist = bezier_out.norm() * 3.0f;
        return dist;
    }

    void computeBezierPointInfo(const BezierInfo_Ptr &info, BezierPointInfo &point_info) {
        double t_1 = 1 - point_info.t;
        double t_squre = point_info.t * point_info.t;
        double t_1_squre = t_1 * t_1;
        double t_1_t = t_1 * point_info.t;


        Eigen::Vector2d bezier1 = t_1_squre * info->start + 2 * t_1_t * info->control_1
                                  + t_squre * info->control_2;

        Eigen::Vector2d bezier2 = t_1_squre * info->control_1 + 2 * t_1_t * info->control_2
                                  + t_squre * info->end;



        Eigen::Vector2d bezier_tangent = bezier2 - bezier1;
        point_info.unit_tangent = bezier_tangent * (1 / bezier_tangent.norm());

        point_info.pose.head<2>() = (1 - point_info.t) * bezier1 + point_info.t * bezier2;
        point_info.pose[2] = atan2(point_info.unit_tangent[1], point_info.unit_tangent[0]);
        if(!info->is_forward) {
            point_info.pose[2] += M_PI;
        }
    }

    double getVectorDotValue(const Eigen::Vector2d &point, const BezierPointInfo &bezier) {
        return bezier.unit_tangent.dot(point - bezier.pose.head<2>());
    }

    double getInitialT(const BezierInfo_Ptr &info, const Eigen::Vector2d &cand_point) {
        auto bezier_vector = info->end - info->start;
        auto tmp_vector = cand_point - info->start;
        double bezier_length = bezier_vector.norm();
        double norm_dist = bezier_vector.dot(tmp_vector)/bezier_length;
        if (norm_dist < 0) {
            return 0;
        }
        if (norm_dist > bezier_length) {
            return 1;
        }
        if (bezier_length == 0) {
            return 0;
        }
        return norm_dist / bezier_length;
    }

    void computeInitialBezierPairs(const BezierInfo_Ptr &info, const Eigen::Vector2d &cand_point,
                                   BezierPointInfo &start_info, BezierPointInfo &end_info) {
        BezierPointInfo initial_info, mid_info_1,mid_info_2,mid_info;
        auto initial_t = getInitialT(info, cand_point);
        start_info.t = 0;
        end_info.t = 1;

        initial_info.t = initial_t;
        mid_info_1.t = initial_t - 0.25 > 0 ? (initial_t - 0.25) : 0;
        mid_info_2.t = (initial_t + 0.25) < 1.0 ? (initial_t + 0.25) : 1.0;
        mid_info.t = 0.5;

        if (initial_info.t == mid_info.t) {
            return;
        }
        computeBezierPointInfo(info, mid_info_1);
        computeBezierPointInfo(info, mid_info_2);
        computeBezierPointInfo(info, initial_info);
        computeBezierPointInfo(info, mid_info);
        double mid_dot_value = getVectorDotValue(cand_point, mid_info);
        double initial_dot_value = getVectorDotValue(cand_point, initial_info);
        double mid_1_dot_value = getVectorDotValue(cand_point, mid_info_1);
        double mid_2_dot_value = getVectorDotValue(cand_point, mid_info_2);

        if (mid_1_dot_value * initial_dot_value <= 0) {
            start_info = mid_info_1;
            end_info = initial_info;
            return;
        }
        if (mid_2_dot_value * initial_dot_value <= 0) {
            start_info = initial_info;
            end_info = mid_info_2;
            return;
        }

        if (mid_1_dot_value * mid_2_dot_value <= 0) {
            start_info = mid_info_1;
            end_info = mid_info_2;
            return;
        }

        if (initial_info.t < mid_info.t) {
            if (mid_2_dot_value * mid_dot_value <= 0) {
                start_info = mid_info;
                end_info = mid_info_2;
                return;
            }
        }else{
            if (mid_1_dot_value * mid_dot_value <= 0) {
                start_info = mid_info_1;
                end_info = mid_info;
                return;
            }
        }

        if (mid_dot_value * initial_dot_value <= 0) {
            if (initial_info.t < mid_info.t) {
                end_info = mid_info;
                start_info = initial_info;
            }else{
                start_info = mid_info;
                end_info = initial_info;
            }
            return;
        }

        computeBezierPointInfo(info, start_info);
        computeBezierPointInfo(info, end_info);
        double start_dot_value = getVectorDotValue(cand_point, start_info);
        double end_dot_value = getVectorDotValue(cand_point, end_info);
        if (initial_t > mid_info.t) {
            if (end_dot_value * initial_dot_value <= 0) {
                start_info = initial_info;
                return;
            }
            if (mid_dot_value * end_dot_value <= 0) {
                start_info = mid_info;
                return;
            }
        }else{
            if (start_dot_value * initial_dot_value <= 0) {
                end_info = initial_info;
                return;
            }
            if (mid_dot_value * start_dot_value <= 0) {
                end_info = mid_info;
                return;
            }
        }
        if (initial_t > 0.9) {
            start_info = mid_info;
        }else if (initial_t < 0.1) {
            end_info = mid_info;
        }
    }
};


}


#endif //PROJECT_BEZIER_CURVE_OPERATOR_HPP
