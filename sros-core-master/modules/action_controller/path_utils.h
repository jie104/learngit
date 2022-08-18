//
// Created by caoyan on 4/22/21.
//

#ifndef SROS_PATH_UTILS_H
#define SROS_PATH_UTILS_H

#include "core/navigation_path.h"
#include "core/src.h"
#include "core/pose.h"

using namespace std;
using namespace sros::core;

namespace ac {

void genRotateBetweenPaths(sros::core::NavigationPath_vector& dst_paths);   //在两条路径中生成旋转路径
void genStartRotatePath(sros::core::NavigationPath_vector& dst_paths, double srt_pose_yaw);    //在路径的起始生成旋转路径
void genEndRotatePath(sros::core::NavigationPath_vector& dst_paths, double dst_pose_yaw);   //在路径的结束生成旋转路径

//工具函数
void string_split(string s, string delim, std::vector<string>& ans);
void debugOuputPaths(const sros::core::NavigationPath_vector& dst_paths);

double calcGoodsLength(int goal_id);
double calcForkEndCoordinateLength();

double calcAdjustPoseOptimalDistance();     //计算调整位姿的最优距离长度
double calcDetectDistance();     //计算视觉检测货物的最优距离长度


bool isForkPoseAdjust();            //判断叉车是否需要进行位姿调整

void calcDstPose(const Pose& pose_src, Pose& pose_dst, double vec);  //vec 向量长度

BezierPath genBezierPath(const Pose& pose_src, const Pose& pose_dst, PathDirection d);

double distanceTwoPose (const Pose &p1,const Pose &p2);

}

#endif  // SROS_PATH_UTILS_H
