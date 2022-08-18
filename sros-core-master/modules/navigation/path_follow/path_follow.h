/*
 * @Author: your name
 * @Date: 2020-11-24 15:59:34
 * @LastEditTime: 2021-03-26 14:18:18
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /sros/modules/navigation/path_follow/path_follow.h
 */
//
// Created by yj on 19-11-28.
//
// 完成路径规划之后 并且优化之后开始执行该路径。
//

#ifndef SROS_PATH_FOLLOW_H
#define SROS_PATH_FOLLOW_H

#include "core/util/utils.h"
#include "core/pose.h"

#include <glog/logging.h>
#include "math.h"

using namespace std;
typedef struct {
    float max_v;
    float end_v;
    float acc_up;
    float acc_down;
    float remain_dist;

}velocity_plan;

typedef struct
{
    /*
     * ָ�����:    so,se,S, vs, ve, as, ae, Ts, sigma;
     * ���Ʋ���:    Vmin, Vmax, Amin, Amax, Jmin, Jmax;
     * �滮����:    th, tk, sk, vk, ak, jk, vk1, ak1, jk1,_Amin,_Amax,_Jmin,_Jmax;
     */
    float sa, se, S, vs, ve, as, ae, Ts;
    float Vmin, Vmax, Amin, Amax, Jmin, Jmax;
    float sk, vk, ak, jk, vk1, ak1, jk1,_Amin,_Amax,_Jmin,_Jmax,Tj2a, Tj2b, Td, hk;
}S_double_data;

class PathFollow{

public:
    void calc_velocity(velocity_plan velocity,double dt,double& current_v);
    bool process_rotate(double &w, double current_angle, double target_angle, double max_w,
                                   double acc_w, double time);
    double sr_calc_coeff (sros::core::Pose current_position , unsigned char direction, sros::core::Pose target_position);
    
    void WConfigure_doubleS(double max_w,float max_acc_w,float Jmu,float distance,float vs,float as);

    float WNext_doubleS(float *aa);

    
    S_double_data DoubleS;
    bool is_InStopPhase, is_AccelerationBegin;

    float g_Scurve_plan_acc = 0;

    float start_rotate_flag =0;
    float end_rotate_flag = 0;
};


#endif //SROS_PATH_FOLLOW_H
