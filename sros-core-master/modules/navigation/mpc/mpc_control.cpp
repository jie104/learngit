//
// Created by yj on 19-11-6.
//

#include "mpc_control.h"


// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd MPCcontrol::polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order)
{
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++)
        A(i, 0) = 1.0;

    for (int j = 0; j < xvals.size(); j++)
    {
        for (int i = 0; i < order; i++)
            A(j, i + 1) = A(j, i) * xvals(j);
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}

// Evaluate a polynomial.
double MPCcontrol::polyeval(Eigen::VectorXd coeffs, double x)
{
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++)
    {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}

// 根据轨迹点 ，agv当前的位置 速度信息。先通过曲线拟合 然后通过mpc方法计算出目标线速度和角速度。
void MPCcontrol::mpcLoop(sros::core::Pose current_pose,sros::core::Velocity current_velocity,std::vector<Vector2D> opt_point)
{
    double px,py,psi,v;
    px = current_pose.x();
    py = current_pose.y();
    psi = current_pose.yaw();
    v = current_velocity.vx();
    const int N = 5; // Number of waypoints
    double dist_thresh_sq = 10000;
    std::vector<Vector2D>::iterator  it=opt_point.begin();
    std::vector<Vector2D>::iterator erase_end;
    // 现在需要找到离agv最近的一个点。
    while (it != opt_point.end()) {
        double dx = px- it->getX();
        double dy = py - it->getY();
        double dist_sq = dx * dx + dy * dy;
        if (dist_sq < dist_thresh_sq) {
            erase_end = it;
            dist_thresh_sq = dist_sq;
            //break;
        }
        ++it;
    }
    if (erase_end == opt_point.end()) {
        opt_point.clear();
        return;
    }
    if (erase_end != opt_point.begin()) {
        opt_point.erase(opt_point.begin(), erase_end);
    }
    const double cospsi = cos(psi);
    const double sinpsi = sin(psi);
    // Convert to the vehicle coordinate system
    Eigen::VectorXd x_veh(N);
    Eigen::VectorXd y_veh(N);
    for(int i = 0; i < N; i++)
    {
        const double dx = opt_point[i].getX() - px;
        const double dy = opt_point[i].getY() - py;
        x_veh[i] = dx * cospsi + dy * sinpsi;
        y_veh[i] = dy * cospsi - dx * sinpsi;
        // std::cout<<"vehecl x:"<< x_veh[i]<<";y: "<<y_veh[i]<<std::endl;
    }

    auto coeffs = polyfit(x_veh, y_veh, 3);

    const double cte  = polyeval(coeffs, 0.0);
    const double epsi = atan(coeffs[1]);
    Eigen::VectorXd state(6);
    // 设置agv当前的状态，依次为px,py,psi.因为每次都将路径点转换到agv坐标系下所以 前面三个都为0.
    // 。 速度v。 相对于拟合曲线的横向距离cte，还有角度偏差 epsi。
    state << 0, 0, 0, v, cte, epsi;

    static double  velocity = 1.0;

    std::vector<double> mpc_results = _mpc.Solve(state, coeffs,velocity);
}