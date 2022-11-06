#include <eigen3/Eigen/Geometry>
#include <iostream>

int main()
{
    Eigen::Affine2d curr_tf=  Eigen::Rotation2Dd(1.57)*Eigen::Translation2d(1,1);
    // std::cout  << "curr_tf: " << std::endl <<  curr_tf.matrix() << std::endl;
    Eigen::Matrix3d a=curr_tf.matrix();

    Eigen::Vector2d first_corner(1,0);

    auto curr_point = curr_tf * first_corner;
    // auto b = a * first_corner;
    std::cout << "curr_point: " << curr_point.matrix().transpose() << std::endl;
    // auto b = a * first_corner;
    // std::cout << "b: " << b.matrix() << std::endl;


}