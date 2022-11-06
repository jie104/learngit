#include  "eigen3/Eigen/Core"
#include  "eigen3/Eigen/Geometry"
#include <iostream>

int main()
{    
    Eigen::Affine2d scan_tf;
    scan_tf = Eigen::Translation2d(2, 4) *Eigen::Rotation2Dd(1.57);
    std::cout << scan_tf.matrix() << std::endl;

}