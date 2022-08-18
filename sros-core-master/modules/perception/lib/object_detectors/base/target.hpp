/**
 * @file target.h
 * @brief target information
 *
 * include target detector_base information, such as target pose(x,y,z) and angel relative to camera,
 * target card pallet index of the clusters. card point cloud and normal cloud.
 *
 * @author zhangxu@standard-robots.com
 * @date create date：2020/12/16
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */


#ifndef TARGET_HPP
#define TARGET_HPP
//INCLUDE
#include "../../../common/cloud_memory.hpp"
#include <glog/logging.h>

//CODE
namespace perception {

/**
 * @brief target of the detect.
 */
class Target {
public:

    Target() : angle_(180)
              , normal_cloud_(new NormalCloud)
              , point_cloud_(new PointCloud) {

    }

    virtual ~Target() {
        if (normal_cloud_ != nullptr) {
            normal_cloud_->clear();
            normal_cloud_ = nullptr;
        }

        if (point_cloud_ != nullptr) {
            point_cloud_->clear();
            point_cloud_ = nullptr;
        }
    }

    Target& operator=(const Target &target){
        this->id_ = target.id_;
        this->angle_ = target.angle_;
        this->pose_ = target.pose_;
        this->pose_normal_ = target.pose_normal_;
        copyPointCloud(target.point_cloud_, this->point_cloud_);
        copyNormalCloud(target.normal_cloud_, this->normal_cloud_);
//        LOG(INFO)  << "target operator=!";
        return *this;
    }

    Target(const Target &target)
        : angle_(180)
        , normal_cloud_(new NormalCloud)
        , point_cloud_(new PointCloud) {
        this->id_ = target.id_;
        this->angle_ = target.angle_;
        this->pose_ = target.pose_;
        this->pose_normal_ = target.pose_normal_;
        copyPointCloud(target.point_cloud_, this->point_cloud_);
        copyNormalCloud(target.normal_cloud_, this->normal_cloud_);
//        LOG(INFO)  << "target copy constructor!";
    }

    /**
     * @brief reset all of the runtime variable.
     */
    virtual void reset()  {
        this->id_ = -1;
        this->angle_ = 180;
        this->pose_.x = 0;
        this->pose_.y = 0;
        this->pose_.z = 0;

        this->pose_normal_.normal_x = 0;
        this->pose_normal_.normal_y = 0;
        this->pose_normal_.normal_z = 0;

        this->normal_cloud_->clear();
        this->point_cloud_->clear();
    }

    /**
      * @brief set target id.
      * @param[in] id index
      */
    inline void setId(int id) {
        this->id_ = id;
    }

    /**
     * @brief get target's id.
     * @return the target index.
     */
    inline int getId() const {
        return this->id_;
    }

    /**
     * @brief set the target angle relative to the camera.
     * @param[in] angle input target angle
     */
    inline void setAngle(const float &angle) {
        this->angle_ = angle;
    }

    /**
     * @brief get the target angle relative to the camera.
     * @return target angle
     */
    inline float getAngle() const {
        return this->angle_;
    }

    /**
     * @brief set the target center point.
     * @param[in] pose point x, y, z.
     */
    inline void setPose(const Point3D &pose) {
        this->pose_ = pose;
    }

    /**
     * @brief get the target center point.
     * @return point x, y, z.
     */
    inline Point3D getPose() const {
        return this->pose_;
    }

    /**
     * @brief set the target center point normal.
     * @param[in] normal normal_x, normal_y, normal_z.
     */
    inline void setNormal(const Normal &normal) {
        this->pose_normal_ = normal;
    }

    /**
     * @brief get the target center point normal.
     * @return normal(normal_x, normal_y, normal_z).
     */
    inline Normal getNormal() const {
        return this->pose_normal_;
    }

    /**
     * @brief set point cloud.
     * @param[in] cloud point cloud.
     */
    inline void setPointCloud(const PointCloudConstPtr &cloud) {
        copyPointCloud(cloud, this->point_cloud_);
    }

    /**
     * @brief get point cloud.
     * @return point cloud ptr.
     */
    inline PointCloudPtr getPointCloud() {
        return this->point_cloud_;
    }

    /**
     * @brief set card normal cloud.
     * @param[in] pc_normal input normal cloud.
     */
    inline void setNormalCloud(const NormalCloudConstPtr &pc_normal) {
        copyNormalCloud(pc_normal, this->normal_cloud_);
    }

    /**
     * @brief get card normal cloud.
     * @return return normal cloud.
     */
    inline NormalCloudPtr getNormalCloud() const {
        return this->normal_cloud_;
    }

protected:

    /** @brief  target id. */
    int id_{};

    /** @brief  target angle. */
    float angle_{};

    /** @brief  target center pose(x, y, z). */
    Point3D pose_{};

    /** @brief  target center point normal. */
    Normal pose_normal_;

    /** @brief  target normal cloud. */
    NormalCloudPtr normal_cloud_;

    /** @brief  target point cloud. */
    PointCloudPtr point_cloud_;
};

} // end of namespace perception
#endif //PERCEPTION_SOLUTION_TARGET_HPP
