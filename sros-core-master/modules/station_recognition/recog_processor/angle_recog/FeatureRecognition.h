/*
 * FeatureRecognition.h
 *
 *  Created on: 2016年6月2日
 *      Author: hq
 */


//#define ROS_ENV

#ifndef GAZEBO_SIMULATOR_SRC_FEATURERECOGNITION_H_
#define GAZEBO_SIMULATOR_SRC_FEATURERECOGNITION_H_


#ifdef ROS_ENV

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/PoseArray.h"

#endif


#include <fstream>
#include <eigen3/Eigen/Dense>

#include "fit.h"

#ifndef ROS_ENV

#include "core/pose.h"
#include "core/msg/laser_scan_msg.hpp"

#endif

#define MAX_ARRAY_COUNT 1082

namespace slam {

struct AngleFeature {
    float angle;
    float line1_length;
    float line2_length;
    LinePara line1;
    LinePara line2;
    iPoint corner;
    LinePara bisector;
    bool is_reflexangle_side;

};

class FeatureRecognition {
public:
    FeatureRecognition();

    virtual ~FeatureRecognition();

#ifdef ROS_ENV

    void scanCallback(const sensor_msgs::LaserScan &scan_const);

    void decodeLidar(const sensor_msgs::LaserScan &scan);

    bool matchAngle(const sensor_msgs::LaserScan &scan_const, AngleFeature &inputangle, Eigen::Vector3f &laser_pose,
                    vector<Eigen::Vector3f> &pose_vec);

#else

    void decodeLidar(const sros::core::LaserScanMsg &scan);

    bool matchAngle(const sros::core::LaserScanMsg &scan_const, AngleFeature &inputangle,
                    Eigen::Vector3f &laser_pose, vector<Eigen::Vector3f> &pose_vec);

    sros::core::Pose_Vector matchAngle(const sros::core::LaserScanMsg &scan_const,
                                       const Eigen::Vector3f &laser_pose);

    void setAngleFeature(double theta, double d1, double d2);

    bool getInitialPose(const sros::core::LaserScanMsg &scan,Eigen::Vector3f& initialpose);

#endif

    bool getAngleFeature(vector<AngleFeature> &angle_feature_vector);

    void initialize(const sros::core::LaserScanMsg& scan);


    bool checkInputAngel(vector<AngleFeature> &angle_feature_vector,
                         vector<AngleFeature> &output_angfeature);

    bool matchAngle(int *radarrho, float *theta, int ridardatacnt, AngleFeature &inputangle,
                    vector<Eigen::Vector3f> &pose_vec);

    float getPointDist(iPoint &start, iPoint &end);

    LinePara findLongestLine(vector<LinePara>& line_vector,int& index);

private:
#ifdef ROS_ENV
    ros::NodeHandle node_;
    ros::Publisher breakpoint_pub;
    ros::Publisher pose_pub;
    ros::Publisher longline_pub;
    sensor_msgs::PointCloud laser_cloud;
#endif
    OpenRadar openRadar;
    AngleFeature input_anglefeature;


    int *RadarRho;
    float *RadarTheta;
    //sin cos
    float *RadarSinTheta;
    float *RadarCosTheta;

    int RadarDataCnt;
    int *RadarX;
    int *RadarY;

    float laser_max_dist;
    float laser_min_dist;
    float maxangle;
    float minangle;
    bool is_initialized;
    int scan_size;
};

} /* namespace slam */

#endif /* GAZEBO_SIMULATOR_SRC_FEATURERECOGNITION_H_ */
