#include <sensor_msgs/PointCloud2.h>
#include <librealsense2/rs.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <glog/logging.h>
#include "../include/obstacle_detecter.hpp"

class ObstacleDetecterWrapper {
public:
    ObstacleDetecterWrapper() = default;

private:
    ros::Publisher point_cloud_publisher_;
    ros::Publisher obstacle_grid_publisher_;
    nav_msgs::OccupancyGrid obstacle_grid_;
    obstacle_detection::ObstacleDetecter detecter_;

public:
    bool setup(ros::NodeHandle &node, ros::NodeHandle &private_node) {
        point_cloud_publisher_ = node.advertise<sensor_msgs::PointCloud2>("/point_cloud", 1);
        obstacle_grid_.header.frame_id = "/camera";
        obstacle_grid_.info.origin.position.y = -0.6;
        obstacle_grid_publisher_ = node.advertise<nav_msgs::OccupancyGrid>("/obstacle_map", 1);
        return true;
    }

    void process() {
        rs2::config cfg;
        cfg.enable_stream(rs2_stream::RS2_STREAM_DEPTH, 848, 480, rs2_format::RS2_FORMAT_Z16, 15);
        cfg.enable_stream(rs2_stream::RS2_STREAM_COLOR, 848, 480, rs2_format::RS2_FORMAT_RGB8, 15);
        rs2::pointcloud pointcloud;
        rs2::points points;
        rs2::pipeline pipe;
        rs2::decimation_filter decimation_filter(2.f);
        rs2::threshold_filter threshold_filter(0.15f, 4.f);
        rs2::temporal_filter temporal_filter(0.4f, 20.f, 4);
        auto profile = pipe.start(cfg);
        auto sensor = profile.get_device().first<rs2::depth_sensor>();
        sensor.set_option(RS2_OPTION_VISUAL_PRESET, RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY);
        if (sensor.supports(RS2_OPTION_LASER_POWER)) {
            auto range = sensor.get_option_range(RS2_OPTION_LASER_POWER);
            sensor.set_option(RS2_OPTION_LASER_POWER, range.max);
        }
        while (ros::ok()) {
            auto frames = pipe.wait_for_frames();
            auto color = frames.get_color_frame();
            pointcloud.map_to(color);
            auto depth = frames.get_depth_frame();
            rs2::frame filtered = depth;
            filtered = decimation_filter.process(filtered);
            filtered = threshold_filter.process(filtered);
            filtered = temporal_filter.process(filtered);
            points = pointcloud.calculate(filtered);
            detecter_.process(points, color);
            publishPointCloud();
            publishObstacleGrid();
            ros::spinOnce();
        }
    }

private:
    void publishPointCloud() {
        sensor_msgs::PointCloud2 pc_msg;
        pcl::toROSMsg(detecter_.pointCloud(), pc_msg);
        pc_msg.header.stamp = ros::Time::now();
        pc_msg.header.frame_id = "/camera";
        point_cloud_publisher_.publish(pc_msg);
    }

    void publishObstacleGrid() {
        obstacle_grid_.header.stamp = ros::Time::now();
        detecter_.obstacleGrid().toOccupancyGrid(obstacle_grid_);
        this->obstacle_grid_publisher_.publish(obstacle_grid_);
    }
};