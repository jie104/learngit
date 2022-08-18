/**
 * @file handcart_detector.h
 * @brief main functions of card detection
 *
 * Function implementation of card board detection.
 *
 * @author zhangxu@standard-robots.com
 * @date create date：2020/8/31
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef __HANDCART_DETECTOR_HH__
#define __HANDCART_DETECTOR_HH__

// INCLUDE
#include "handcart.hpp"
#include "handcart_target.h"
#include "handcart_detect_param.hpp"
#include "../base/detector.h"
#include "cube.h"

// CODE
namespace perception {
/**
 * @brief detector card class
 */
class HandcartDetector : public Detector {
public:
    using HandcartTarget = perception::HandcartTarget;

    /** @brief constructor. */
    explicit HandcartDetector() = default;

    /** @brief destructor. */
    ~HandcartDetector() override;

    /** @brief display param. */
    void showParam() override;

    /** @brief set runtime data as default value. */
    void reset() override;

    /**
     * @brief detect target in frame.
     * @param[in] cloud point cloud.
     * @return return detect result.
     */
    DetectResult detect(const PointCloudPtr &cloud);

    /**
     * @brief detect target in frame.
     * @param[in] frame 3D camera frame.
     * @return return detect result.
     */
    DetectResult detect(const O3d3xxFrame &frame) override { return result_; };

    /** @brief init detect param and build point handler.
     *  @param[in] param detect param.
     *  @param[in] handcarts detect cards param.
     *  @return if handcarts.size = 0 return false, otherwise return ture;
     */
    bool init(const HandcartDetectParam &param, const std::map<int, Handcart> &handcarts);

private:

    // 过滤孤立点.
    void outlierFilter(const PointCloudConstPtr &cloud,
                       const PointCloudConstPtr &input_cloud,
                       const PointCloudPtr &output_cloud,
                       cv::Mat &mask);

    // 求yoz面内x最小的的点云.
    void getGenerateCloud(const cv::Mat &mask,
                          const PointCloudConstPtr &cloud,
                          const PointCloudPtr &mask_cloud,
                          const PointCloudPtr &front_cloud);

    /**
     * @brief search target in mat map.
     * @return if found target return true, then return false.
     */
    bool searchTarget() override;

    /**
     * @brief correct target width according to center mat width.
     * @param[in] param input detect param.
     * @param[in,out] card output detect card.
     * @param[in,out] clusters_map output mats map.
     * @param[in,out] target output target.
     */
    void correctTargetWidth();

    /**
     * @brief euclidean segmentation.
     * @param[in] normals input normal cloud.
     * @param[in] cloud input point cloud.
     * @param[in] clusters Multiple clusters point indices of cloud.
     * @param[out] clusters_map segmentation mat set.
     */
     void filterClusters(const NormalCloudConstPtr &normals,
                         const PointCloudConstPtr &cloud,
                         const std::vector<Indices> &clusters_indices,
                         std::map<int, Cluster> &clusters_map);

    /**
     * @brief calculate card detect result.if card all mat point cloud fitting plane fail,
     *        return false; else return true;
     * @return if card all mat point cloud fitting plane fail, return false; else return true;
     */
    bool calculateResult() override;

    /** @brief detect param. */
    HandcartDetectParam param_{};

    CubePtr cube_;

    /** @brief card type to be detected. */
    std::map<int, Handcart> handcarts_;

    PointCloudPtr filter_front_cloud_;

    /** @brief detect target */
    HandcartTarget target_;

public:
    typedef std::shared_ptr<HandcartDetector> Ptr;
    typedef std::shared_ptr<const HandcartDetector> ConstPtr;
};

using HandcartDetectorPtr = HandcartDetector::Ptr;
using HandcartDetectorConstPtr = HandcartDetector::ConstPtr;

} // end of namespace perception
#endif // __HANDCART_DETECTOR_HH__