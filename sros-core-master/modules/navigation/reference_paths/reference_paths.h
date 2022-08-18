/**
 * @file reference_paths.h
 * @brief 主要类“ReferencePaths”的头文件
 * 
 * 生成参考线的人工势场地图，并势场地图的作用下，使全局路径沿参考线方向
 * 
 * @author 吴运才
 * @date 文件创建日期：2021-4-13
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef REFERENCE_PATHS_H_
#define REFERENCE_PATHS_H_

#include <iostream>
#include <vector>
#include <set>

#include "core/core.h"
#include "core/map_manager.h"
#include "core/msg/common_poses_info_msg.hpp"
#include "core/pose.h"
#include "core/src.h"
#include "core/module.h"
#include "modules/navigation/path_smooth/vector2d.h"
#include "core/map/net/edge.hpp"

using namespace std;
using sros::core::Pose;

/**
 * \brief 定义栅格地图中的一个栅格
 *
 * 一个栅格中，包含了多个势场、路径编号、路径角度等信息
 */
class Node {
public:
    vector<Pose> gradients_;    //多条参考线的势场
    vector<int> occupied_paths_;    //参考线的编号
    vector<double> paths_theta_;    //参考线的角度

    Node() {}

    void addPathAndGrandient(int num, Pose grandient, double theta) {    //给地图当前栅格增加参考线编号、势场、角度
        int add_flag = 1;    //增加参考线标志
        for(int i = 0; i < occupied_paths_.size(); i++) {    //当前栅格没有包含此条参考线时，增加此参考线
            if(num == occupied_paths_[i]) {
                add_flag = 0;
                break;
            }
        }
        if(add_flag == 1) {
            occupied_paths_.push_back(num);
            gradients_.push_back(grandient);
            paths_theta_.push_back(theta);
        }
    }
};

/**
 * \brief 定义栅格地图
 *
 * 栅格地图包括数据、地图起点、宽带、高度、分辨率等信息
 */
class Map {
public:
    vector<vector<Node> > map_;    //保存二维栅格地图数据
    Pose origin_pose_;    //origin_pose为地图右上角相对于世界坐标系的位置，宽用x坐标，高用y坐标
    double height_;    //地图高度
    double width_;    //地图宽带
    double resolution_;    //地图分辨率
};

/**
 * \brief 将参考路径的势场添加到地图中，并用于计算势场
 *
 * 通过读取地图中的路径做为参考线，并以参考线为基础生成人工势场，最终通过世界坐标计算势场
 */
class ReferencePaths {
public:
    ReferencePaths() {
        potential_field_width_ = 1.0;
    }

    ~ReferencePaths() {}

    /**
    * @brief 获得地图初始化信息和路径信息，生成人工势场地图
    * @param origin_pose 栅格地图起点相对于世界坐标的位姿，栅格地图的起点为地图左上角
    * @param map_width 地图宽度，与世界坐标x轴平行
    * @param map_heigth 地图高度，与世界坐标y轴平行，方向相反
    * @param resolution 地图分辨率
    * @param paths 地图中所有路径，只有路径为EXECUTE_FREE类型时，才被识别为参考线
    */
    void initTheMap(Pose origin_pose, double map_width, double map_height, double resolution, sros::map::net::EdgeGroup paths) {    //origin_pose为地图右上角相对于世界坐标系的位置
        LOG(INFO) << origin_pose << ", " << map_width << ", " << map_height << ", " << resolution;
        map_.map_.clear();
        map_.map_.resize(map_width / resolution);
        for(int i = 0; i < map_.map_.size(); i++) {
            map_.map_[i].resize(map_height / resolution);
        }
        map_.origin_pose_ = origin_pose;
        map_.height_ = map_height;
        map_.width_ = map_width;
        map_.resolution_ = resolution;

        setReferencePaths(paths);

        setPathGradientInMap();

        // vector<vector<double> > datas;
        // datas.resize(map_.map_.size());
        // for(int i = 0; i < map_.map_.size(); i++) {
        //     for(int j = 0; j < map_.map_[i].size(); j++) {
        //         datas[i].push_back(hypot(map_.map_[i][j].gradient_.y(), map_.map_[i][j].gradient_.x()) / 0.3);
        //     }
        // }
        // saveTheData(datas);
    }

    /**
    * @brief 通过世界坐标得到势场强度
    * @param x0 前一个路径点x坐标
    * @param y0 前一个路径点y坐标
    * @param x 路径点x坐标
    * @param y 路径点y坐标
    * @param x1 后一个路径点x坐标
    * @param y1 后一个路径点y坐标
    * @return 一个二维向量表示的势场强度和方向
    */
    Vector2D getGrimMapGradient(double x0, double y0, double x, double y, double x1, double y1) {    //注意网格的起点和世界坐标的起点的位置不重合，方向也不相同，函数的参数均为世界坐标系下的相邻路径点
        int mx, my;
        Pose gradient(0, 0, 0);
        if(worldToMap(x, y, mx, my) == 1) {
            if(map_.map_[mx][my].gradients_.size() == 1) {    //判断此栅格是否只有一个势场
                
                double theta0 = map_.map_[mx][my].paths_theta_[0];
                double theta1 = atan2(y1 - y0, x1 - x0);
                double theta = detAngle(theta0, theta1);
                if(theta >= -0.25 * PI && theta <= 0.25 * PI) {    //判断路径点的角度和参考线的夹角是否小于45°
                    gradient = map_.map_[mx][my].gradients_[0];
                }
            }
            else if(map_.map_[mx][my].gradients_.size() > 1) {    //判断此栅格是否有多个势场
                int ref_path = calcuRefPath(x0, y0, x, y, x1, y1);    //选取其中一条参考线计算势场
                for(int j = 0; j < map_.map_[mx][my].occupied_paths_.size(); j++) {
                    if(ref_path == map_.map_[mx][my].occupied_paths_[j]) {
                        double theta0 = map_.map_[mx][my].paths_theta_[j];
                        double theta1 = atan2(y1 - y0, x1 - x0);
                        double theta = detAngle(theta0, theta1);
                        if(theta >= -0.25 * PI && theta <= 0.25 * PI) {    //判断路径点的角度和参考线的夹角是否小于45°
                            gradient = map_.map_[mx][my].gradients_[j];
                        }
                    }
                }
            }
        }

        Vector2D gradient1;
        gradient1.setX(gradient.x());
        gradient1.setY(-gradient.y());    //网格y方向与世界坐标y方向相反
        double cofigration = 30;
        double theta0 = atan2(gradient.y(), gradient.x());    //梯度的方向角
        double theta1 = atan2(y1 - y0, x1 - x0);
        double theta = theta1 - theta0;
        theta = theta >= PI ? theta - 2 * PI : theta;
        theta = theta <= -PI ? theta + 2 * PI : theta;
        theta = theta < 0 ? -theta : theta;
        theta = theta > 0.5 * PI ? PI - theta : theta;
        if(theta < 0.25 * PI) {
            cofigration = 0;
        }

        return cofigration * gradient1;
    }

private:
    double PI = 3.14159265;    
    Map map_;    //栅格地图
    vector<vector<Pose> > reference_paths_;    //以离散的方式记录所有参考线
    double potential_field_width_;    //以参考线为中心，势场影响宽带

    /**
    * @brief 将世界坐标系位置映射到栅格地图中
    * @param wx 世界坐标系x坐标
    * @param wy 世界坐标系y坐标
    * @param mx 栅格地图宽度方向第mx格
    * @param mx 栅格地图高度方向第my格
    * @return bool类型 0表示无效，1表示有效
    */
    bool worldToMap(double wx, double wy, int& mx, int& my) {
        if(wx < map_.origin_pose_.x() || map_.origin_pose_.x() + map_.width_ < wx || wy < map_.origin_pose_.y() - map_.height_ || map_.origin_pose_.y() < wy) {
            mx = 0;
            my = 0;
            LOG(INFO) << "地图坐标越界。(" << wx << ", " << wy << ")";
            return false;
        }
        mx = int((wx - map_.origin_pose_.x()) / map_.resolution_ + 0.5);
        my = int((map_.origin_pose_.y() - wy) / map_.resolution_ + 0.5);
        if(mx >= map_.map_.size() || my >= map_.map_.begin()->size()) {
            LOG(INFO) << "地图数值越界。(" << wx << ", " << wy << "), (" << mx << ", " << my << "), (" << map_.map_.size() << ", " << map_.map_.begin()->size() << ")";
            mx = 0;
            my = 0;
            
            return false;
        }
        return true;
    }

    /**
    * @brief 将栅格地图映射到世界坐标中
    * @param wx 世界坐标系x坐标
    * @param wy 世界坐标系y坐标
    * @param mx 栅格地图宽度方向第mx格
    * @param mx 栅格地图高度方向第my格
    * @return bool类型 0表示无效，1表示有效
    */
    bool mapToWorld(int mx, int my, double& wx, double& wy) {
        if(mx >= map_.width_ / map_.resolution_ || my >= map_.height_ / map_.resolution_) {
            LOG(INFO) << "地图数值越界。";
            return false;
        }
        wx = mx * map_.resolution_ + map_.origin_pose_.x();
        wy = map_.origin_pose_.y() - my * map_.resolution_;
        return true;
    }

    /**
    * @brief 读取地图中所有路径，选取其中EXECUTE_FREE类型的路径，以离散的方式保存在reference_paths_中
    * @param paths 地图中所有的路径
    * @note 地图中的路径坐标单位为cm
    */
    void setReferencePaths(sros::map::net::EdgeGroup paths) {
        reference_paths_.clear();
        vector<Pose> path;

        for(int i = 0; i < paths.getItemList().size(); i++) {
            if(paths.getItemList()[i].execute_type == sros::map::net::EdgeExecuteType::EXECUTE_FREE) {
                path.clear();
                if(paths.getItemList()[i].type == sros::map::net::EdgeType::EDGE_LINE) {    //直线路径
                    path.push_back(Pose(paths.getItemList()[i].sx / 100.0, paths.getItemList()[i].sy / 100.0, 0));
                    path.push_back(Pose(paths.getItemList()[i].ex / 100.0, paths.getItemList()[i].ey / 100.0, 0));
                    reference_paths_.push_back(path);
                }
                else if(paths.getItemList()[i].type == sros::map::net::EdgeType::EDGE_CIRCLE) {    //圆弧路径

                }
                else if(paths.getItemList()[i].type == sros::map::net::EdgeType::EDGE_BEZIER) {    //3阶贝塞尔路径
                    Pose pose0(paths.getItemList()[i].sx / 100.0, paths.getItemList()[i].sy / 100.0, 0);
                    Pose pose1(paths.getItemList()[i].cx / 100.0, paths.getItemList()[i].cy / 100.0, 0);
                    Pose pose2(paths.getItemList()[i].dx / 100.0, paths.getItemList()[i].dy / 100.0, 0);
                    Pose pose3(paths.getItemList()[i].ex / 100.0, paths.getItemList()[i].ey / 100.0, 0);
                    vector<Pose> poses;
                    for(double t = 0.01; t < 1; t += 0.01) {
                        double x = pow(1-t, 3)*pow(t, 0)*pose0.x() + 3*pow(1-t, 2)*pow(t, 1)*pose1.x() + 3*pow(1-t, 1)*pow(t, 2)*pose2.x() + pow(1-t, 0)*pow(t, 3)*pose3.x();
                        double y = pow(1-t, 3)*pow(t, 0)*pose0.y() + 3*pow(1-t, 2)*pow(t, 1)*pose1.y() + 3*pow(1-t, 1)*pow(t, 2)*pose2.y() + pow(1-t, 0)*pow(t, 3)*pose3.y();
                        path.push_back(Pose(x, y, 0));
                    }
                    double d = 0;
                    for(int i = 0; i < int(poses.size()) - 1; i++) {
                        d += hypot(poses[i].y() - poses[i + 1].y(), poses[i].x() - poses[i + 1].x());
                        if(d >= 0.1) {
                            path.push_back(poses[i]);
                            d = 0;
                        }
                    }

                    reference_paths_.push_back(path);
                }
            }
        }

        LOG(INFO) << "参考路径条数: " << reference_paths_.size();
    }

    /**
    * @brief 以参考线reference_paths_为基础，生成人工势场地图
    * @param reference_paths_ 地图中所有的路径
    */
    void setPathGradientInMap() {
        if(reference_paths_.size() == 0) {
            LOG(INFO) << "No reference path!";
            return;
        }

        vector<vector<Pose> > reference_paths;
        reference_paths.resize(reference_paths_.size());
        for(int i = 0; i < reference_paths_.size(); i++) {    //将路径细化
            for(int j = 0; j < int(reference_paths_[i].size()) - 1; j++) {
                double d = hypot(reference_paths_[i][j].y() - reference_paths_[i][j + 1].y(), reference_paths_[i][j].x() - reference_paths_[i][j + 1].x());
                double theta = atan2(reference_paths_[i][j + 1].y() - reference_paths_[i][j].y(), reference_paths_[i][j + 1].x() - reference_paths_[i][j].x());
                double det_d = 0.02;
                int n = d / det_d;
                for(int k = 0; k < n; k++) {
                    reference_paths[i].push_back(Pose(reference_paths_[i][j].x() + k * det_d * cos(theta), reference_paths_[i][j].y() + k * det_d * sin(theta), theta));
                }
            }
        }

        for(int i = 0; i < reference_paths.size(); i++) {    //为每条路径在地图上添加势场
            for(int j = 0; j < reference_paths[i].size(); j++) {
                for(double det_l = 0; det_l < potential_field_width_; det_l += map_.resolution_) {
                    double x = reference_paths[i][j].x() + det_l * cos(reference_paths[i][j].yaw() + 0.5 * PI);
                    double y = reference_paths[i][j].y() + det_l * sin(reference_paths[i][j].yaw() + 0.5 * PI);
                    int mx, my;
                    if(worldToMap(x, y, mx, my)) {
                        map_.map_[mx][my].addPathAndGrandient(i, Pose(0.3 * det_l * cos(reference_paths[i][j].yaw() - 0.5 * PI), 0.3 * det_l * sin(reference_paths[i][j].yaw() - 0.5 * PI), reference_paths[i][j].yaw() - 0.5 * PI), reference_paths[i][j].yaw());
                    }
                }
                for(double det_l = 0; det_l < potential_field_width_; det_l += map_.resolution_) {
                    double x = reference_paths[i][j].x() + det_l * cos(reference_paths[i][j].yaw() - 0.5 * PI);
                    double y = reference_paths[i][j].y() + det_l * sin(reference_paths[i][j].yaw() - 0.5 * PI);
                    int mx, my;
                    if(worldToMap(x, y, mx, my)) {
                        map_.map_[mx][my].addPathAndGrandient(i, Pose(0.3 * det_l * cos(reference_paths[i][j].yaw() + 0.5 * PI), 0.3 * det_l * sin(reference_paths[i][j].yaw() + 0.5 * PI), reference_paths[i][j].yaw() + 0.5 * PI), reference_paths[i][j].yaw());
                    }
                }
            }
        }
    }

    /**
    * @brief 路径点在参考线的交汇处时，选择出其中一条参考线做为参考
    * @param x0 前一个路径点x坐标
    * @param y0 前一个路径点y坐标
    * @param x 路径点x坐标
    * @param y 路径点y坐标
    * @param x1 后一个路径点x坐标
    * @param y1 后一个路径点y坐标
    * @return 参考线编号
    */
    int calcuRefPath(double x0, double y0, double x, double y, double x1, double y1) {    //路径点在参考线的交汇处时，选择出其中一条参考线做为参考
        int mx, my;
        if(worldToMap(x, y, mx, my)) {    //考虑双向路径的特殊情况
            if(map_.map_[mx][my].occupied_paths_.size() == 2) {
                double theta0 = atan2(y1 - y0, x1 - x0);
                double det_theta0 = detAngle(theta0, map_.map_[mx][my].paths_theta_[0]);    //路径点和第一条参考线的夹角
                double det_theta1 = detAngle(theta0, map_.map_[mx][my].paths_theta_[1]);    //路径点和第二条参考线的夹角
                if(fabs(det_theta0) <= 0.25 * PI && fabs(det_theta1) >= 0.25 * PI) {
                    return map_.map_[mx][my].occupied_paths_[0];
                }
                else if(fabs(det_theta1) <= 0.25 * PI && fabs(det_theta0) >= 0.25 * PI) {
                    return map_.map_[mx][my].occupied_paths_[1];
                }
            }
        }

        if(worldToMap(x, y, mx, my)) {    //考虑多条路径相交的特殊情况
            double min_det_theta = 999999;
            double ref_path = -1;
            double theta0 = atan2(y1 - y0, x1 - x0);
            for(int i = 0; i < map_.map_[mx][my].occupied_paths_.size(); i++) {
                double det_theta = fabs(detAngle(theta0, map_.map_[mx][my].paths_theta_[i]));
                if(min_det_theta > det_theta) {
                    min_det_theta = det_theta;
                    ref_path = map_.map_[mx][my].occupied_paths_[i];
                }
            }

            return ref_path;
        }

        return -1;
    }

    /**
    * @brief 计算两个夹角相减后的夹角
    * @param angle0
    * @param angle1
    * @return 两个夹角相差角度
    * @note 返回值的取值范围为[-3.14159265， 3.14159265]
    */
    double detAngle(double angle0, double angle1) {
        double theta = angle0 - angle1;
        theta = theta > PI ? theta - 2 * PI : theta;
        theta = theta < -PI ? theta + 2 * PI : theta;
        return theta;
    }

    /**
    * @brief 保存2为数组数据
    * @param data 数据
    * @param filename 文件路径和名称
    */
    void saveTheData(std::vector<std::vector<double> > data, char* filename) {
        FILE * fp;
        if((fp = fopen(filename, "wb")) == NULL) {
            LOG(INFO) << "cant open the file, write the data failed!";
            exit(0);
        }
        
        for(int i = 0; i < data.size(); i++) {
            for(int j = 0; j < data[i].size(); j++) {
                fprintf(fp, "%f ", data[i][j]);
            }
            fprintf(fp, "\n");
        }
        fclose(fp);
    }
};

//ReferencePaths g_gradient_of_ref_path;

#endif
