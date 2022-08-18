//
// Created by lfc on 2022/1/8.
//

#ifndef LIVOX_SLAM_NORM_COMPUTER_HPP
#define LIVOX_SLAM_NORM_COMPUTER_HPP
#include <Eigen/Dense>

namespace standard {
    class NormComputer {
    public:
        template <class EigenType>
        static void buildNormByTwoVector(const EigenType& first_vector,const EigenType& second_vector,EigenType& norm){
            norm = first_vector.cross(second_vector);
        }

        template <class EigenType>
        static void buildNormByCross(const EigenType& center,const std::vector<EigenType>& points,const EigenType& initial_norm,EigenType& norm,float& norm_weight){
            norm = initial_norm;
            for (auto& point : points) {
                EigenType delta_err = point - center;
                auto err_dist = delta_err.norm();
                if (err_dist >= 0.001) {
                    auto first_cross = delta_err.cross(norm);
                    auto second_cross = first_cross.cross(delta_err);
                    norm += err_dist * second_cross;
                }
            }
            norm.normalize();
            norm = center.dot(norm) > 0 ? norm : EigenType(-norm);
            double norm_sum = 0, dot_sum = 0;
            for (auto &point:points) {
                EigenType err = point - center;
                norm_sum += err.norm();
                dot_sum += fabs(norm.dot(err));
            }
            if (norm_sum != 0) {
                norm_weight = 1.0 - dot_sum / norm_sum;
            } else {
                norm_weight = 0;
            }
        }

        static bool
        computeNorm(const Eigen::Vector3d &center, const std::vector<Eigen::Vector3d> &points, Eigen::Vector3d &norm,
                    double &norm_weight) {
            if (points.size() <= 2) {
                LOG(INFO) << "cannot get enough to compute norm!";
                norm_weight = 0;
                norm = Eigen::Vector3d(0, 0, 1);
                return false;
            }
            Eigen::Matrix2d A_m;
            Eigen::Vector2d b_v;
            A_m.setZero(), b_v.setZero();
            for (auto &point:points) {
                Eigen::Vector3d err = point - center;
                Eigen::Vector2d err_2d = Eigen::Vector2d(err[0], err[1]);
                double err_z = err[2];
                A_m += err_2d * err_2d.transpose();
                b_v += err_z * err_2d;
            }
            LOG(INFO) << "abs:" << absDeterminant(A_m);
            if (absDeterminant(A_m) > 0.01) {
                norm.head<2>() = A_m.inverse() * b_v;
                if (!std::isfinite(norm[0]) || !std::isfinite(norm[1])) {
                    LOG(INFO) << "err! will return false!";
                    norm_weight = 0;
                    norm.setIdentity();
                    return false;
                }
                norm[2] = 1;
            } else if (fabs(A_m(0, 0) - A_m(1, 0)) > 0.01) {
                norm = Eigen::Vector3d((A_m(1, 1) - A_m(1, 0)) / (A_m(0, 0) - A_m(1, 0)), 1, 0);
            } else {
                norm = Eigen::Vector3d(1, 0, 0);
            }
            norm.normalize();
            norm = center.dot(norm) > 0 ? norm : Eigen::Vector3d(-norm);
            double norm_sum = 0, dot_sum = 0;
            for (auto &point:points) {
                Eigen::Vector3d err = point - center;
                norm_sum += err.norm();
                dot_sum += fabs(norm.dot(err));
            }
            if (norm_sum != 0) {
                norm_weight = 1.0 - dot_sum / norm_sum;
            } else {
                norm_weight = 0;
            }
            return true;
        }

        static double absDeterminant(const Eigen::Matrix2d &matrix) {
            return fabs(matrix(0, 0) * matrix(1, 1) - matrix(0, 1) * matrix(1, 0));
        }
    };
}


#endif //TSDF_SLAM_NORM_COMPUTER_HPP
