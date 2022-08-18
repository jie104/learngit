//
// Created by lfc on 17-7-3.
//

#ifndef PROJECT_GICP_H
#define PROJECT_GICP_H
#include <Eigen/Dense>
#include <glog/logging.h>
namespace icp{
struct MatchPair{
    Eigen::Vector2f origin_point;
    Eigen::Vector2f match_point;
    Eigen::Matrix2f info;
};
class Gicp {
public:
    Gicp(){

    }
    virtual ~Gicp(){

    }

    bool singleIcp(std::vector<MatchPair>& point_pairs,Eigen::Vector3f& pose){
        Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
        Eigen::Vector3f b = Eigen::Vector3f::Zero();
        float pose_cos = cosf(pose[2]);
        float pose_sin = sinf(pose[2]);

        Eigen::Affine2f delta_tf(
                Eigen::Translation2f(pose[0], pose[1]) * Eigen::Rotation2Df(pose[2]));
        for (auto &point_pair:point_pairs) {
            Eigen::Matrix<float, 3, 2> jacobi_mat;
            jacobi_mat(1, 0) = jacobi_mat(0, 1) = 0;
            jacobi_mat(0, 0) = jacobi_mat(1, 1) = 1;
            jacobi_mat(2, 0) = -point_pair.match_point[0] * pose_sin - point_pair.match_point[1] * pose_cos;
            jacobi_mat(2, 1) = point_pair.match_point[0] * pose_cos - point_pair.match_point[1] * pose_sin;

            H += jacobi_mat * point_pair.info * jacobi_mat.transpose();

            Eigen::Vector2f err = delta_tf * point_pair.match_point - point_pair.origin_point;
            b += jacobi_mat * point_pair.info * err;
        }
        Eigen::Vector3f delta_pose = H.inverse() * (-b);
        pose += delta_pose;
        return true;
    }

    bool processGIcp(std::vector<MatchPair>& point_pairs,Eigen::Vector3f& output_pose,int iter_times = 10) {
//        output_pose = Eigen::Vector3f::Zero();
        for (int i = 0; i < iter_times; ++i) {
            Eigen::Vector3f last_pose = output_pose;
            singleIcp(point_pairs, output_pose);
            Eigen::Vector3f delta_pose = output_pose - last_pose;
            if (delta_pose.norm() <= 0.0005) {
                return true;
            }
        }
        return true;
    }

    bool processNIcp(std::vector<MatchPair>& point_pairs,Eigen::Vector3f& output_pose,int iter_times){
        for (int i = 0; i < iter_times; ++i) {
            Eigen::Vector3f last_pose = output_pose;
            singleNIcp(point_pairs, output_pose);
            Eigen::Vector3f delta_pose = output_pose - last_pose;
            if (delta_pose.norm() <= 0.0001) {
                LOG(INFO) << "have converge! will return true";
                return true;
            }
            LOG(INFO) << "it is:" << i;
        }
        return true;
    }

    bool singleNIcp(std::vector<MatchPair>& point_pairs,Eigen::Vector3f& output_pose){
        float sum_xa = 0.0f;
        float sum_xb = 0.0f;
        float sum_ya = 0.0f;
        float sum_yb = 0.0f;
        float sxx = 0.0f;
        float sxy = 0.0f;
        float syx = 0.0f;
        float syy = 0.0f;
        float pair_count = (float) point_pairs.size();
        if (pair_count < 2.0f) {
            return false;
        }
        float count_inv = 1.0f / pair_count;

        for (auto &pair:point_pairs) {
            const float xa = pair.origin_point[0];
            const float ya = pair.origin_point[1];
            const float xb = pair.match_point[0];
            const float yb = pair.match_point[1];

            sum_xa += xa;
            sum_ya += ya;
            sum_xb += xb;
            sum_yb += yb;

            sxx += xa * xb;
            sxy += xa * yb;
            syx += ya * xb;
            syy += ya * yb;
        }

        const float mean_x_a = sum_xa * count_inv;
        const float mean_y_a = sum_ya * count_inv;
        const float mean_x_b = sum_xb * count_inv;
        const float mean_y_b = sum_yb * count_inv;

        const float auxi_x = pair_count * (sxx + syy) - sum_xa * sum_xb - sum_ya * sum_yb;
        const float auxi_y = pair_count * (syx - sxy) + sum_xa * sum_yb - sum_ya * sum_xb;

        float yaw = 0.0f;
        if (auxi_x != 0 || auxi_y != 0) {
            yaw = atan2(static_cast<double>(auxi_y), static_cast<double>(auxi_x));
        }
        output_pose[2] = yaw;
        const double ccos = cos(yaw);
        const double csin = sin(yaw);

        output_pose[0] = mean_x_a - mean_x_b * ccos + mean_y_b * csin;
        output_pose[1] = mean_y_a - mean_x_b * csin - mean_y_b * ccos;
        return true;
    }


};
}



#endif //PROJECT_GICP_H
