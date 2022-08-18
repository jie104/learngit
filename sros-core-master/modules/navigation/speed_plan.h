/*
 * @Author: your name
 * @Date: 2021-03-15 10:09:36
 * @LastEditTime: 2021-04-02 17:19:30
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /sros/modules/navigation/speed_plan.h
 */
//
// Created by yj on 21-2-22.
//

#ifndef SROS_SPEED_PLAN_H
#define SROS_SPEED_PLAN_H

#include <Eigen/Core>
#include <iostream>

#include "osqp/osqp.h"
#include <Eigen/Dense>
#include "glog/logging.h"

using namespace std;
using namespace Eigen;
class SpeedPlan{
public:
    SpeedPlan(int n,int m):n_(n),m_(m){};
    ~SpeedPlan(){};

    //

    bool calc_P(vector<double>& times);

    bool calc_A(Vector3d start_status,
                           vector<double>& times,
                           vector<double>& dist_from_start,
                           vector<double> point_max_vel,
                           double max_acc );


    void calc_coff(VectorXd& start,int order,double time);

    void matrixToCsc(c_float** M_x, c_int& M_nnz, c_int** M_i, c_int** M_p, Matrix<double, Dynamic, Dynamic> M);

    int upperMatrixToCsc(c_float** M_x, c_int& M_nnz, c_int** M_i, c_int** M_p, Matrix<double, Dynamic, Dynamic> M) ;

    bool optimize();

    double solve_poly_root(vector<double> coff,double s,double t);

    double calc_dist(vector<double> coff,double t);
    double calc_vel(vector<double> coff,double t);
    double calc_a(vector<double> coff,double t);

    bool calc_analytical_coff(Vector3d start_status,Vector3d end_status,double time,double max_v,double max_a,VectorXd& coff,double& cost);

    bool calc_analytical_coff1(Vector3d start_status,Vector3d end_status,double time,double max_v,double max_a,VectorXd& coff,double& cost);


    bool check_acc(VectorXd coff, double time,double max_a);
    bool check_vel(VectorXd coff, double time,double max_v);

    void calc_opt_analytical_coff(Vector3d start_status,Vector3d end_status,double max_v,double max_a,double& time,VectorXd& coff);
    void calc_opt_analytical_coff1(Vector3d start_status,Vector3d end_status,double max_v,double max_a,double& time,VectorXd& coff);

    //计算P矩阵。
    int n_ ;//  表示多少条条多项式函数。
    int m_ = 6 ;  // 表示5次多项式有6个变量。

    MatrixXd P;
    MatrixXd A ; // 首先初始化一个60*60的矩阵。

    VectorXd U ;
    VectorXd L ;

    vector<vector<double>> coff_;

    vector<double> vel_profile_;
    vector<double> times_profile_;



};


#endif //SROS_SPEED_PLAN_H
