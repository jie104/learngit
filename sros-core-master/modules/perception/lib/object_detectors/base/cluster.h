/**
 * @file cluster.h
 * @brief cluster information
 *
 * cluster detector_base information, such as cluster point cloud, cluster center point. cluster rectangle in
 * deep image.
 *
 * @author zhangxu@standard-robots.com
 * @date create date：2020/8/31
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef __CLUSTER_HH__
#define __CLUSTER_HH__

#include <vector>
#include <opencv/cv.h>

#include "../../../common/point_cloud.hpp"
#include "../../../common/cloud_memory.hpp"

namespace perception {

/** @brief cluster information. */
class Cluster {
public:
    /**
     * @brief constructor.
     */
    Cluster()
    : point_cloud_(new PointCloud)
    , normal_cloud_(new NormalCloud) {

    }

    /**
    * @brief destructor.
    */
    ~Cluster() = default;

    /**
     * @brief clear runtime data.
     */
    void clear();

    /**
     * @brief sort by cluster's rectangle.x.
     * @return if this cluster rectangle.x > cluster.rectangle.x.
     */
    bool operator<(const perception::Cluster &cluster) const {
        return this->getRect().x > cluster.getRect().x;
    }

    /**
     * @brief get cluster center x.
     * @return output cluster center x.
     */
    inline float getAvgX() const {
        return this->avg_x_;
    }

    /**
     * @brief get cluster center y.
     * @return output cluster center y.
     */
    inline float getAvgY() const {
        return this->avg_y_;
    }

    /**
     * @brief get cluster center z.
     * @return output cluster center z.
     */
    inline float getAvgZ() const {
        return this->avg_z_;
    }

    /**
     * @brief get cluster rectangle.
     * @return output cluster rectangle.
     */
    inline cv::Rect getRect() const {
        return this->rect_;
    }

    /**
     * @brief set cluster rectangle.
     * @param[in] rect cluster bounding rectangle in deep image.
     * @param[in] width deep image width.
     */
    void setRect(const cv::Rect &rect, int width);

    /**
     * @brief get cluster point cloud.
     * @return output cluster point cloud.
     */
    inline PointCloudPtr getPointCloud() const {
        return this->point_cloud_;
    }

    /**
     * @brief set cluster point cloud.
     * @param[in] points input cluster point cloud.
     * @param[in] width deep image width.
     */
    void setPointCloud(const PointCloudConstPtr &points, int width);

    /**
     * @brief get cluster normal cloud.
     * @return output cluster normal cloud.
     */
    inline NormalCloudPtr getNormalCloud() const {
        return this->normal_cloud_;
    }

    /**
     * @brief set cluster normal cloud.
     * @param[in] input_normal_cloud input cluster point cloud.
     */
    inline void setNormalCloud(const NormalCloudConstPtr &input_normal_cloud) {
        copyNormalCloud(input_normal_cloud, this->normal_cloud_);
    }

    /**
     * @brief set cluster center normal.
     * @param[in] normal input cluster center normal.
     */
    inline void setNormal(const Normal &normal) {
        this->normal_ = normal;
    }

    /**
     * @brief get cluster center normal.
     * @return output cluster center normal.
     */
    inline Normal getNormal() const {
        return this->normal_;
    }

    /**
     * @brief get most left point in cluster point cloud.
     * @return output point(x,y,z)
     */
    inline Point3D getMostLeftPoint() const {
        return this->most_left_point_;
    }

    /**
     * @brief get most right point in cluster point cloud.
     * @return output point(x,y,z)
     */
    inline Point3D getMostRightPoint() const {
        return this->most_right_point_;
    }

    /**
     * @brief get most up point in cluster point cloud.
     * @return output point(x,y,z)
     */
    inline Point3D getMostUpPoint() const {
        return this->most_up_point_;
    }

    /**
     * @brief get most left point in cluster point cloud.
     * @return output point(x,y,z)
     */
    inline Point3D getMostDownPoint() const {
        return this->most_down_point_;
    }

    /**
     * @brief get cluster width.
     * @return output cluster width.
     */
    inline float getWidth() const {
        return this->width_;
    }

    /**
     * @brief get cluster height.
     * @return output cluster height.
     */
    inline float getHeight() const {
        return this->height_;
    }

    /**
     * @brief get number of cluster point.
     * @return output number of cluster point.
     */
    inline size_t getSize() const {
        return this->point_cloud_->size();
    }

private:

    /**
     * @brief calculate point average-x,y,z.
     */
    void calculatePointAvg();

    /** @brief cluster center-x. */
    float avg_x_;

    /** @brief cluster center-y. */
    float avg_y_;

    /** @brief cluster center-z. */
    float avg_z_;

    /** @brief cluster bounding rectangle. */
    cv::Rect rect_;

    /** @brief cluster width. (Unit: m) */
    float width_;

    /** @brief cluster height. (Unit: m) */
    float height_;

    /** @brief cluster center normal. */
    Normal normal_;

    /** @brief most left point of cluster. */
    Point3D most_left_point_;

    /** @brief most right point of cluster. */
    Point3D most_right_point_;

    /** @brief most up point of cluster. */
    Point3D most_up_point_;

    /** @brief most down point of cluster. */
    Point3D most_down_point_;

    /** @brief point of cluster cloud. */
    PointCloudPtr point_cloud_;

    /** @brief point of cluster cloud. */
    NormalCloudPtr normal_cloud_;
};

using Cluster = perception::Cluster;

} // end of namespace perception
#endif // __CLUSTER_HH__
