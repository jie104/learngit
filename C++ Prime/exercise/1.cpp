#include <iostream>
#include <eigen3/Eigen/Geometry>
#include <cmath>


int main() {
	Eigen::Vector3d delta_pose(1,0,1.57);
	Eigen::Vector3d curr_pose(1,2,1.57);
	
	Eigen::Affine2d origin_tf(Eigen::Translation2d(curr_pose[0], curr_pose[1]) *
            Eigen::Rotation2Dd(curr_pose[2]));
    std::cout << "delta_pose.head<2>: " << delta_pose.head<2>().matrix().transpose() << std::endl;
    auto point = origin_tf * delta_pose.head<2>();
    auto delta_yaw = delta_pose[2] + curr_pose[2];
    delta_yaw = atan2(sin(delta_yaw), cos(delta_yaw));
    std::cout  << "origin_tf: " << "\n" << origin_tf.matrix() << std::endl;
    std::cout << "point: " << point.matrix().transpose() << std::endl;
}