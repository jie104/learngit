//
// Created by lfc on 19-7-25.
//

#ifndef SROS_STEREO_MODULE_PARA_HPP
#define SROS_STEREO_MODULE_PARA_HPP
#include "Eigen/Dense"

namespace stereo {
struct StereoModulePara {
    double d435_install_x = 0.29;
    double d435_install_y = 0.0;
    double d435_install_yaw = 0.0;
    double d435_install_height = 0.0;
    double d435_install_pitch = 0.0;
    double d435_install_roll = 0.0;
    double d435_calibration_height = 0.0;
    Eigen::Matrix3d d435_install_calibration_matrix = Eigen::Matrix3d::Zero();
    double rack_wheel_rotate_radius = 0.1;
    double rack_leg_wheel_center_length = 1.06f;
    double rack_leg_wheel_center_width = 0.6f;
    bool enable_remove_rack_leg = false;
    bool enable_filter_only_load_full = true;
    double obstacle_min_height = 0.03;
    double obstacle_max_height = 1.0;
    double obstacle_max_dist = 1.5;

};

}


#endif //SROS_STEREO_MODULE_PARA_HPP
