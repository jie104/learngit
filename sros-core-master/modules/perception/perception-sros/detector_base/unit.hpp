//
// Created by zx on 2021/1/5.
//

#ifndef __SROS_UNIT_HPP__
#define __SROS_UNIT_HPP__
#include "core/settings.h"
#include "core/src.h"
#include "modules/perception/common/common_func.hpp"
#include "modules/navigation/lib/include/geometry.h"
#include "modules/perception/common/default_value.h"
#include <Eigen/Dense>
#include <vector>

const double EPS = 1e-6;

namespace common_func {
static int
decodeStringParam(const std::string &param_str, std::vector<float> &param_vct) {
    param_vct.clear();
    sros::core::Settings &s = sros::core::Settings::getInstance();
    const std::string range3d_str = s.getValue<std::string>(param_str, "");
    param_vct = common_func::splitStringToVector<float>(range3d_str, ';');
    return param_vct.size();
};

template <class Param>
void loadCommonParam(Param &param_) {
    sros::core::Settings &s = sros::core::Settings::getInstance();

    // load camera rotation parameter.
    param_.camera_fixed[0] = s.getValue<float>("camera.o3d303_install_x_offset", perception::DEFAULT_CAMERA_X_OFFSET);
    param_.camera_fixed[1] = s.getValue<float>("camera.o3d303_install_y_offset", perception::DEFAULT_CAMERA_Y_OFFSET);
    param_.camera_fixed[2] = s.getValue<float>("camera.o3d303_install_yaw", perception::DEFAULT_CAMERA_YAW);

    // load camera coordinate parameter.
    param_.laser_fixed[0] = s.getValue<float>("perception.laser_install_x_offset", perception::DEFAULT_LASER_X_OFFSET);
    param_.laser_fixed[1] = s.getValue<float>("perception.laser_install_y_offset", perception::DEFAULT_LASER_Y_OFFSET);
    param_.laser_fixed[2] = s.getValue<float>("perception.laser_install_yaw", perception::DEFAULT_LASER_YAW);

    // load fork end of coordinate parameter.
    std::vector<float> fork_end_coordinate_vct;
    const int fork_end_coordinate_param_num = sizeof(param_.fork_end) / sizeof(float);
    const int decode_fork_end_coordinate_param_num =
        decodeStringParam("forklift.fork_end_coordinate", fork_end_coordinate_vct);
    if (decode_fork_end_coordinate_param_num == fork_end_coordinate_param_num) {
        param_.fork_end[0] = fork_end_coordinate_vct[0];
        param_.fork_end[1] = fork_end_coordinate_vct[1];
        param_.fork_end[2] = fork_end_coordinate_vct[2];
    } else {
        param_.fork_end[0] = perception::DEFAULT_FORK_END_X;
        param_.fork_end[1] = perception::DEFAULT_FORK_END_Y;
        param_.fork_end[2] = perception::DEFAULT_FORK_END_Z;
        LOG(INFO) << "decode_fork_end_coordinate_param_num =" << decode_fork_end_coordinate_param_num 
        << " != " << fork_end_coordinate_param_num;
    }
}

template <class T>
inline T normalizeAngle(T &angle) {
    angle = fmod(angle, 2.0 * M_PI);
    if (angle >= M_PI) {
        angle -= 2.0f * M_PI;
    } else if (angle < -M_PI) {
        angle += 2.0f * M_PI;
    }
    return angle;
}

static Eigen::Vector3f
cvtToWorldPose(const Eigen::Vector3f &world_pose,
               const Eigen::Vector3f &delta_pose) {
    Eigen::Vector3f curr_pose;
    Eigen::Affine2f world_tf(Eigen::Translation2f(world_pose.head<2>()) * Eigen::Rotation2Df(world_pose[2]));
    curr_pose.head<2>() = world_tf * delta_pose.head<2>();
    curr_pose[2] = world_pose[2] + delta_pose[2];
    normalizeAngle(curr_pose[2]);
    return curr_pose;
}


static void
cvtPointsToWorld(const Eigen::Vector3d &curr_pose,
                 const std::vector<Eigen::Vector2d> &local_points,
                 std::vector<Eigen::Vector2d> &world_points) {
    Eigen::Translation2d translation2D(curr_pose[0], curr_pose[1]);
    Eigen::Rotation2Dd rotation2D(curr_pose[2]);
    Eigen::Affine2d world_tf = translation2D * rotation2D;

    world_points.clear();
    world_points.reserve(world_points.size());
    for (auto p : local_points){
        auto world_point = world_tf * p;
        world_points.push_back(world_point);
    }
}

static void
cvtSectorToPolygon(const double start_angle,
                   const double end_angle,
                   const double radius,
                   const double resolution,
                   std::vector<Eigen::Vector2d> &world_points){
    const size_t point_number = (end_angle - start_angle) / resolution;

    world_points.clear();
    world_points.reserve(point_number + 2);

    // push origin Point (x:0,y:0)
    Eigen::Vector2d temp_point;
    temp_point[0] = 0;
    temp_point[1] = 0;
    world_points.push_back(temp_point);

    double angle = start_angle;
    while (end_angle > angle+EPS){
        temp_point[0] = radius * cos(angle);
        temp_point[1] = radius * sin(angle);
        world_points.push_back(temp_point);
        angle += resolution;
    }
}


static void
convert2PointCloud(cv::Mat &mat_xyz, PointCloud::Ptr &cloud) {
    if (mat_xyz.type() != CV_16SC3) return;

    int image_height = mat_xyz.rows;
    int image_width = mat_xyz.cols;
    
    cloud->resize(image_width * image_height);
    for (int row = 0; row < image_height; row++) {
        for (int col = 0; col < image_width; col++) {
            const int idx = row * image_width + col;
            const auto pixel = mat_xyz.at<cv::Vec3s>(row, col);
            cloud->points[idx].x = static_cast<float>(pixel[0]) * 0.001f;
            cloud->points[idx].y = static_cast<float>(pixel[1]) * 0.001f;
            cloud->points[idx].z = static_cast<float>(pixel[2]) * 0.001f;
            cloud->image_indices[idx] = idx;
        }
    }
}

/**
 * @brief  convert ifm xyzimage to point cloud. downsample for avoiding obstacle. 
 */
static void
convertPointCloud(cv::Mat &mat_xyz, PointCloud::Ptr &cloud) {
    if (mat_xyz.type() != CV_16SC3) return;

    int image_height = mat_xyz.rows;
    int image_width = mat_xyz.cols;

    cloud->reserve(image_width * image_height);

    for (int row = 0; row < image_height; row=row+2) {
        for (int col = 0; col < image_width; col=col+2) {
            const int idx = row * image_width + col;
            const auto pixel = mat_xyz.at<cv::Vec3s>(row, col);
            cloud->points[idx].x = static_cast<float>(pixel[0]) * 0.001f;
            cloud->points[idx].y = static_cast<float>(pixel[1]) * 0.001f;
            cloud->points[idx].z = static_cast<float>(pixel[2]) * 0.001f;
            cloud->image_indices[idx] = idx;
        }
    }
}
// calculate |p1 p2| X |p1 p|
static double
cross(const Eigen::Vector2d &p1,
      const Eigen::Vector2d &p2,
      const Eigen::Vector2d &p) {
    return (p2.x() - p1.x()) * (p.y() - p1.y()) - (p.x() - p1.x()) * (p2.y() - p1.y());
}

// Judge whether the point P is in the rectangle
static bool
isPointInRectangle(const std::vector<Eigen::Vector2d> &rect,
                   const Eigen::Vector2d &p) {
    return cross(rect[0], rect[1], p) * cross(rect[2], rect[3], p) >= 0
           && cross(rect[1], rect[2], p) * cross(rect[3], rect[0], p) >= 0;
}

static void
cvtMeterUnits2MillimeterUnits(const std::vector<Eigen::Vector2d> &points) {
    const double METER_TO_MILLIMETER_UNIT = 1000;
    std::vector<Eigen::Vector2d> points_temp;
    points_temp.resize(points.size());
    for (const auto & point : points) {
        Eigen::Vector2d temp;
        temp[0] = point.x() * METER_TO_MILLIMETER_UNIT;
        temp[1] = point.y() * METER_TO_MILLIMETER_UNIT;
        points_temp.push_back(std::move(temp));
    }
    copy(points.begin(), points.end(), points_temp.begin());
}

static void
cvtPointToPolygon(const std::vector<Eigen::Vector2d> &points,
                  Polygon &polygon) {
    polygon.clear();
    polygon.reserve(points.size());
    for (auto p : points) {
        polygon.push_back(point(p[0], p[1]));
    }
}


template<typename Scalar>
struct Transformer
{
    const Eigen::Matrix<Scalar, 4, 4>& tf;

    /** Construct a transformer object.
     * The transform matrix is captured by const reference. Make sure that it does not go out of scope before this
     * object does. */
    Transformer (const Eigen::Matrix<Scalar, 4, 4>& transform) : tf (transform) { };

    /** Apply SO3 transform (top-left corner of the transform matrix).
     * \param[in] src input 3D point (pointer to 3 floats)
     * \param[out] tgt output 3D point (pointer to 4 floats), can be the same as input. The fourth element is set to 0. */
    void so3 (const Point3D& src, Point3D& tgt) const
    {
        const Scalar p[3] = { src.x, src.y, src.z};  // need this when src == tgt
        tgt.x = static_cast<float> (tf (0, 0) * p[0] + tf (0, 1) * p[1] + tf (0, 2) * p[2]);
        tgt.y = static_cast<float> (tf (1, 0) * p[0] + tf (1, 1) * p[1] + tf (1, 2) * p[2]);
        tgt.z = static_cast<float> (tf (2, 0) * p[0] + tf (2, 1) * p[1] + tf (2, 2) * p[2]);
        //tgt[3] = 0;
    }

    /** Apply SE3 transform.
     * \param[in] src input 3D point (pointer to 3 floats)
     * \param[out] tgt output 3D point (pointer to 4 floats), can be the same as input. The fourth element is set to 1. */
    void se3 (const Point3D& src, Point3D& tgt) const
    {
        const Scalar p[3] = { src.x, src.y, src.z };  // need this when src == tgt
        tgt.x = static_cast<float> (tf (0, 0) * p[0] + tf (0, 1) * p[1] + tf (0, 2) * p[2] + tf (0, 3));
        tgt.y = static_cast<float> (tf (1, 0) * p[0] + tf (1, 1) * p[1] + tf (1, 2) * p[2] + tf (1, 3));
        tgt.z = static_cast<float> (tf (2, 0) * p[0] + tf (2, 1) * p[1] + tf (2, 2) * p[2] + tf (2, 3));
        
        //tgt[3] = 1;
    }
};

static void 
transformPointCloud(const PointCloudPtr& cloud_in,
                    PointCloudPtr& cloud_out,
                    const Eigen::Vector3f& offset,
                    const Eigen::Vector3f& rotation)
{
    float offset_x = offset[0], offset_y = offset[1], offset_z = offset[2];
    float rotation_yaw = rotation[0], rotation_pitch = rotation[1], rotation_roll = rotation[2];

    Eigen::Translation3f translation(offset_x,offset_y,offset_z);
    Eigen::AngleAxisf yawAngle( Eigen::AngleAxisf(rotation_yaw,Eigen::Vector3f::UnitZ()));
    Eigen::AngleAxisf pitchAngle( Eigen::AngleAxisf(rotation_pitch,Eigen::Vector3f::UnitY()));
    Eigen::AngleAxisf rollAngle( Eigen::AngleAxisf(rotation_roll,Eigen::Vector3f::UnitX()));

    Eigen::Quaternionf quaternion = yawAngle * pitchAngle * rollAngle;

    Eigen::Affine3f transform = translation*quaternion.toRotationMatrix();

    Transformer<float> tf (transform.matrix());
    cloud_out->reserve(cloud_in->points.size ());
    Point3D temp_point;
    for (std::size_t i = 0; i < cloud_in->points.size (); ++i)
    {
        tf.se3 (cloud_in->points[i], temp_point);
        cloud_out->push_back(temp_point, cloud_in->image_indices[i]);

    }

    LOG(INFO) << "transformPointCloud done:  "  ; 

}

}
#endif  // SROS_UNIT_HPP
