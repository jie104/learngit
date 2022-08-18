//
// Created by yj on 19-10-8.
//

#include <core/pose.h>
#include "collisiondetection.h"
#include "../lib/include/jps_grid.h"
using  namespace HybridAStar;

CollisionDetection::CollisionDetection() {
//    this->grid = nullptr;
   //Lookup::collisionLookup(collisionLookup);
}

bool CollisionDetection::configurationTest(float x, float y, float t) {
    int X = (int)x;
    int Y = (int)y;
    int iX = (int)((x - (long)x) * Constants::positionResolution);
    iX = iX > 0 ? iX : 0;
    int iY = (int)((y - (long)y) * Constants::positionResolution);
    iY = iY > 0 ? iY : 0;
    int iT = (int)(t / Constants::deltaHeadingRad);
    int idx = iY * Constants::positionResolution * Constants::headings + iX * Constants::headings + iT;
    int cX;
    int cY;

    cX = (X );
    cY = (Y );


        // make sure the configuration coordinates are actually on the grid
        if (cX >= 0 && (unsigned int)cX < grid_map_->width && cY >= 0 && (unsigned int)cY < grid_map_->height) {
            if (!isWalkableAt(grid_map_,cX,cY)) {
                return false;
            }
        }

    return true;
}


bool CollisionDetection::check_line_collision(Node3D* node){

    float line_angle=0;
    //LOG(INFO)<<"判断hybrid 当前点的坐标为："<<node->getX()<<";"<<node->getY()<<";"<<node->getT();

    if(node->getPred()!= nullptr){
        const Node3D* node11 = node->getPred();

                Node3D pre_node;
            pre_node.setX(node11->getX());
            pre_node.setY(node11->getY());
            line_angle = atan2(node->getY()-node11->getY(),node->getX()-node11->getX());
            pre_node.setT(line_angle);
        //LOG(INFO)<<"hybrid 检测的前一个点为"<<"x:"<<pre_node.getX()<<"；y:"<<pre_node.getY()<<";yaw:"<<pre_node.getT();

        // if( !check_point_collision1(&pre_node)){
        //     return  false;
        // }

                // 检测一下中间点

            Node3D pre_node2;
            pre_node2.setX((node->getX()+node11->getX())/2.0);
            pre_node2.setY((node->getY()+node11->getY())/2.0);
            pre_node2.setT(line_angle);
            //LOG(INFO)<<"hybrid 检测的后一个点为"<<"x:"<<pre_node1.getX()<<"；y:"<<pre_node1.getY()<<";yaw:"<<pre_node1.getT();
            if(!check_point_collision1(&pre_node2)){
                LOG(INFO)<<"hybrid 检测的中间点有障碍物";
                return false;
            }
        }



    Node3D pre_node1;
    pre_node1.setX(node->getX());
    pre_node1.setY(node->getY());
    pre_node1.setT(line_angle);
    //LOG(INFO)<<"hybrid 检测的后一个点为"<<"x:"<<pre_node1.getX()<<"；y:"<<pre_node1.getY()<<";yaw:"<<pre_node1.getT();
    if(!check_point_collision1(&pre_node1)){
        return false;
    }
    return true;

}

bool CollisionDetection::check_point_collision1(const Node3D* node) {

    double robot_length = cfg_.robot.length + 0.08;
    double robot_width = cfg_.robot.width + 0.08;

    double map_height = grid_map_->height;
    double map_width = grid_map_->width;

    int check_length;
    check_length = (unsigned int)(std::sqrt(robot_length * robot_length + robot_width * robot_width) / cfg_.nav_resolution*100.0) + 20;

    sros::core::Pose cur_map_pose;

    cur_map_pose.x() = node->getX();
    cur_map_pose.y() = node->getY();

    // 根据局部矩形的边长 计算四个角度的坐标
    sros::core::Pose up_left, up_right, down_left, down_right;

    up_left.x() = cur_map_pose.x() - check_length / 2;
    up_left.y() = cur_map_pose.y() - check_length / 2;

    if (up_left.x() < 0) { up_left.x() = 0.0; }
    if (up_left.y() < 0) { up_left.y() = 0.0; }

    up_right.y() = up_left.y();
    up_right.x() = cur_map_pose.x() + check_length / 2;

    if (up_right.x() >= map_width) { up_right.x() = map_width - 1; }

    down_left.x() = up_left.x();
    down_left.y() = cur_map_pose.y() + check_length / 2;

    if (down_left.y() >= map_height) { down_left.y() = map_height - 1; }

    down_right.x() = up_right.x();
    down_right.y() = down_right.y();



    int margin = (cfg_.nav_resolution+1)*2.0*0.01;
    // 然后将局部矩形内的障碍点保存在 local_obstacles.
    for (int i = (int) (up_left.x()); i <= up_right.x(); i++) {
        for (int j = (int) (up_left.y()); j <= down_left.y(); j++) {
            //LOG(INFO)<<"GRIDMAP VAULE:"<<  i<<";"<<j<<";"<<(int)grid_map_->nodes[j][i].grid_layered_walkable_;
            if (!isWalkableAt(grid_map_, i, j)) {

                // 将坐标点i,j 转换为机器人坐标系下。
                float dx = i - cur_map_pose.x();
                float dy = j - cur_map_pose.y();

                float yaw = node->getT();

                // 进行坐标转换。 将障碍点转换为agv坐标系下的坐标
                double new_x = dx * cos(yaw) + dy * sin(yaw);
                double new_y = -dx * sin(yaw) + dy * cos(yaw);

                if ((fabs(new_x) < (robot_length  + margin)/ (cfg_.nav_resolution*2.0/100.0)) && (fabs(new_y) < (robot_width+ margin) / (cfg_.nav_resolution*2.0/100.0))) {
                    LOG(INFO)<<"检查hybrid 障碍物:i: "<<i<<";j:"<<j<<";yaw;"<<yaw<<";grid_map_.value:"<<(int)grid_map_->nodes[j][i].grid_layered_walkable_;
                    LOG(INFO)<<"检查hybrid 障碍物:"<<new_x<<";"<<new_y;
                    return false;
                }
                else{
                    //LOG(INFO)<<"检查hybrid 障碍物:"<<new_x<<";"<<new_y<<";"<<yaw;
                }
            }
        }
    }
    return true;
}