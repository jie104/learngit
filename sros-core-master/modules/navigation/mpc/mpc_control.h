//
// Created by yj on 19-11-6.
//

#ifndef SROS_MPC_CONTROL_H
#define SROS_MPC_CONTROL_H
#include <Eigen/Core>

#include <Eigen/Dense>

#include "../path_smooth/vector2d.h"
#include "mpc.h"
#include "core/pose.h"

class MPCcontrol{
public:
    MPCcontrol(){}
    ~MPCcontrol(){}

    Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);

    double polyeval(Eigen::VectorXd coeffs, double x);

    void mpcLoop(sros::core::Pose current_pose,sros::core::Velocity current_velocity,std::vector<Vector2D> opt_point);

private:
    MPC _mpc;
};

#endif //SROS_MPC_CONTROL_H
