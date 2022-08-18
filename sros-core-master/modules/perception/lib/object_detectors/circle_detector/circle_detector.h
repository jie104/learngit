/**
 * @file circle_detector.h
 * @brief Used to detect circular objects
 * 
 * Class for detecting circular targets
 * 
 * @author zhangxu@standard-robots.com
 * @date create date：2020/12/7
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef __CIRCLE_DETECTOR_H__
#define __CIRCLE_DETECTOR_H__

// INCLUDE
#include "circle.hpp"
#include "circle_target.h"
#include "circle_detect_param.hpp"
#include "../base/detector.h"

//CODE
namespace perception {
/**
 * @description : Used to detect circular objects
 * @author      : zhangxu
 * @date        : 2020/12/7 下午4:26
 */
class CircleDetector : public Detector {
public:

    /** @brief constructor. */
    explicit CircleDetector();

    /** @brief destructor. */
    ~CircleDetector() override;

    /** @brief init detect param and build point handler.
     *  @param[in] param detect param.
     *  @param[in] cards detect cards param.
     *  @return if circles.size = 0 return false, otherwise return ture;
     */
    bool init(const CircleDetectParam &param, const std::map<int, Circle> &circles_map);

    /** @brief display param. */
    void showParam() override;

    /** @brief set runtime data as default value. */
    void reset() override;

    /**
     * @brief detect target in frame.
     * @param[in] frame 3D camera frame.
     * @return return detect result.
     */
    DetectResult detect(const O3d3xxFrame &frame) override;

    /**
     * @brief get all of the circle contour image.
     * @param[out] img contour image.
     */
    void getContourImage(cv::Mat &img) const;

    /**
     * @brief get all of the circle cluster image.
     * @param[out] img contour image.
     */
    void getClusterImage(cv::Mat &img) const;

private:

    /**
     * @brief euclidean segmentation.
     * @param[in] normals input normal cloud.
     * @param[in] cloud input point cloud.
     * @param[in] clusters Multiple clusters point indices of cloud.
     * @param[out] clusters_map segmentation pallet set.
     */
    void filterClusters(const NormalCloudConstPtr &normals,
                        const PointCloudConstPtr &cloud,
                        const std::vector<Indices> &clusters_indices,
                        std::map<int, Cluster> &clusters_map);

    /**
     * @brief search target in cluster map, merge all of the cluster to clusters_img_
     *        and all of the contour to contours_img_. select the cluster with the
     *        highest score as the target.
     * @return if found target return true, then return false.
     */
    bool searchTarget() override;

    /**
     * @brief calculate circle detect result.
     * @return if target point cloud fitting plane fail, return false; else return true;
     */
    bool calculateResult() override;


    bool targetIsCircle();

    /**
     * @brief Find out the edge points in the image
     * @param[in] src input image.
     * @param[out] contour_img contour image.
     * @param[out] contour_points all of the contour poinnt.
     */
    void findContourPoint(const cv::Mat_<uint8_t> &src,
                          cv::Mat &contour_img,
                          std::vector<cv::Point> &contour_points) const;

    /**
     * @brief Fitting circle with RANSAC method. The fitting score was calculated according
     *        to the ratio of fitting points to the whole point.
     * @param points point set.
     * @param center circle center.
     * @param radius circle radius.
     * @return return fitting score.
     */
    float findCircle(const std::vector<cv::Point> &points,
                     cv::Point &center,
                     int &radius) const;

    /**
     * @brief Filtering out the points in the circle from the input point cloud image.
     * @param[in] input input point cloud.
     * @param[in] center circle center in image.
     * @param[in] radius circle radius pixel.
     * @param[out] output after filtering point set.
     */
    void filterCircleCloud(const PointCloudConstPtr &input,
                           const cv::Point &center,
                           const int &radius,
                           const PointCloudPtr &output) const;

    /**
     * @brief draw contour to image.
     */
    void drawContour();

    /**
     * @brief draw cluster to image.
     */
    void drawCluster();

    /**
     * @brief draw 'target' text on target image.
     */
    void drawTarget();

    /** @brief detect param. */
    CircleDetectParam param_{};

    /** @brief card type to be detected. */
    std::map<int, Circle> circles_map_;

    /** @brief all of the cluster contour points. */
    std::vector<cv::Point> contours_points_;

    /** @brief all of the contour image  */
    cv::Mat_<uint8_t> contours_img_;

    /** @brief all of the cluster image. */
    cv::Mat_<uint8_t> clusters_img_;

    /** @brief detect target */
    CircleTarget target_;

public:
    typedef std::shared_ptr<CircleDetector> Ptr;
    typedef std::shared_ptr<const CircleDetector> ConstPtr;
};

using CircleDetectorPtr = CircleDetector::Ptr;
using CircleDetectorConstPtr = CircleDetector::ConstPtr;

} // end of namespace perception
#endif //PERCEPTION_SOLUTION_CIRCLE_DETECTOR_H
