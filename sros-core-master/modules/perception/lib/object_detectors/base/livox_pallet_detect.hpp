//
// Created by lfc on 2022/4/28.
//

#ifndef LIVOX_PALLET_DETECT_LIVOX_PALLET_DETECT_HPP
#define LIVOX_PALLET_DETECT_LIVOX_PALLET_DETECT_HPP
//#include <geometry_msgs/PoseArray.h>
//#include <sensor_msgs/PointCloud.h>
#include <Eigen/Dense>
#include <deque>
#include <memory>
#include "ground_plane_map.hpp"
#include "norm_computer.hpp"
#include "pickplace_frontier_edge_detect.hpp"
#include "standard_pallet_detect.hpp"
#include"../base/detector.h"

namespace standard {
class LivoxPalletDetect {
 public:


    template <class Quaternion>
    void EulerToEigenQuaternion(double yaw, double roll, double pitch, Quaternion& quaternion) {
        double cy = cos(yaw / 2.0);
        double sy = sin(yaw / 2.0);
        double cr = cos(roll / 2.0);
        double sr = sin(roll / 2.0);
        double cp = cos(pitch / 2.0);
        double sp = sin(pitch / 2.0);
        quaternion.w() = cr * cp * cy + sr * sp * sy;
        quaternion.x() = sr * cp * cy - cr * sp * sy;
        quaternion.y() = cr * sp * cy + sr * cp * sy;
        quaternion.z() = cr * cp * sy - sr * sp * cy;
    }

    LivoxPalletDetect() {
        //该地图为一个投影到地面的柱状图，每个网格高度信息会生成高低不同的柱，柱在高度方向上也会拆分成一个个小格子，如果小格子没有填充，则是-1，否则是点云的索引;在栈板识别成功后，该格子会被设置为0
        map_.reset(new GroundPlaneMap<Eigen::Vector3f>(map_resolution_, map_length_, map_width_, map_min_height_,
                                                       map_max_height_));
    }

    void onReceiveLivoxPoint(const PointCloudPtr pcl, std::vector<std::shared_ptr<FrontierEdgeInfo>> &frontier_edges, const Eigen::Vector3f& transf,
                             const Eigen::Vector3f& rotate) {
        auto& points = pcl->points;
        Eigen::Quaternionf quat;
        for (auto& point : points) {
            livox_points_.push_front(Eigen::Vector3f(point.x, point.y, point.z));
        }
        //这里需要根据实际需要多少雷达点来做切换，一般可能需要200000个点，同时下方也要修改。
        if (livox_points_.size() < 200000) {
            return;
        }
//        Eigen::Vector3f height(-0.08, -0.014, 0.118);//该值为传感器的安装位置信息，该值可被修改
        LOG(INFO) << "point size:" << livox_points_.size();
        LOG(INFO)<<"ground_camera_transf: (x:"<<transf[0]<<", y: "<<transf[1]<<", z: "<<transf[2]<<")";
        LOG(INFO)<<"ground_camera_rotate: (pitch:"<<rotate[0]<<", yaw: "<<rotate[1]<<", roll: "<<rotate[2]<<")";

        EulerToEigenQuaternion(rotate[1], rotate[2], rotate[0], quat);

        map_->clear();
        for (auto& point : livox_points_) {
            Eigen::Vector3f point_e = quat * point + transf;
            //将所有点云添加到地图上，不在范围内的点将会被去除
            map_->addPoint(point_e[0], point_e[1], point_e);
        }
//        LOG(INFO)<<"Begin to detect pallet";
        //在构建map后，接下来主要做的就是从所有点集中寻找所有栈板，优先行搜索
//        std::string detect_target = "pallet"; //两个检测不可混用。如果需要混用，需要构建两个map，map在检测过程中会发生修改。
            PickPlaceFrontierEdgeDetect edge_detect;
//            std::vector<std::shared_ptr<FrontierEdgeInfo>> frontier_edges;
            if(edge_detect.findFrontierEdge(map_, frontier_edges)){
//                LOG(INFO)<<"Succeed to find frontier edge.";
            }
        while (livox_points_.size() >= 100000) {
            livox_points_.pop_back();
        }
    }

    void onReceiveLivoxPoint(const PointCloudPtr& pcl, std::vector<PalletInfo>& pallet_info, const Eigen::Vector3f& transf,
                             const Eigen::Vector3f& rotate) {
        auto& points = pcl->points;
        Eigen::Quaternionf quat;
        for (auto& point : points) {
            livox_points_.push_front(Eigen::Vector3f(point.x, point.y, point.z));
        }
        //这里需要根据实际需要多少雷达点来做切换，一般可能需要200000个点，同时下方也要修改。
        // if (livox_points_.size() < 200000) {
        //     return;
        // }
//        Eigen::Vector3f height(-0.08, -0.014, 0.118);//该值为传感器的安装位置信息，该值可被修改
        LOG(INFO) << "point size:" << livox_points_.size();
        LOG(INFO) <<"ground_camera_transf: (x:"<<transf[0]<<", y: "<<transf[1]<<", z: "<<transf[2]<<")";
        LOG(INFO) <<"ground_camera_rotate: (pitch:"<<rotate[0]<<", yaw: "<<rotate[1]<<", roll: "<<rotate[2]<<")";

        EulerToEigenQuaternion(rotate[1], rotate[2], rotate[0], quat);

        map_->clear();
        for (auto& point : livox_points_) {
            Eigen::Vector3f point_e = quat * point + transf;
            //将所有点云添加到地图上，不在范围内的点将会被去除
            map_->addPoint(point_e[0], point_e[1], point_e);
        }
//        LOG(INFO)<<"Begin to detect pallet";
        //在构建map后，接下来主要做的就是从所有点集中寻找所有栈板，优先行搜索

            StandardPalletDetect pallet_detector;
//            std::vector<PalletInfo> pallets;
//            LOG(INFO)<<"Begin to tranverse Grid for pallet. ";
            if(pallet_detector.tranverseGridForPallet(map_, pallet_info)){
//                LOG(INFO) << "find pallet size:" << pallet_info.size();
//                publishPallets(pallets, "livox");
            }

        while (livox_points_.size() >= 100000) {
            livox_points_.pop_back();
        }

    }

 private:
    std::shared_ptr<GroundPlaneMap<Eigen::Vector3f>> map_;//投影到地面的柱状地图，将高度信息变成直方柱图
//    ros::NodeHandle node_;//ros发布用
//    ros::Publisher up_hole_point_pub_;//发布洞的上边沿数据+中心墩的点云
//    ros::Publisher down_hole_point_pub_;//发布洞的下边沿数据
//    ros::Publisher mean_array_pub_;//栈板中心位姿数据
//    ros::Publisher sum_livox_pub_;//积分的点云数据，这是做了变换后的原始点云数据
//    ros::Subscriber livox_point_sub_;//订阅的livox雷达数据
    std::deque<Eigen::Vector3f> livox_points_;//用于构造积分雷达数据的队列

    float map_resolution_ = 0.03f; //0.02f;//投影到地面上网格的分辨率，每个网格独立构造一个柱状条
    float map_length_ = 3.0f;//相机出射方向范围，定义成x轴方向
    float map_width_ = 3.0f;//相机左右方向范围，实际为2×map_width_，由负到正
    float map_min_height_ = -0.1f;//投影到地面上的点云最小过滤高度，小于该值则过滤
    float map_max_height_ = 3.0f;//投影到地面上的点云最大过滤高度，大于该值则过滤

};
}  // namespace livox

#endif  // LIVOX_PALLET_DETECT_LIVOX_PALLET_DETECT_HPP
