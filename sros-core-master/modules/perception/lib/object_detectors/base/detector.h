/**
 * @file detector.h
 * @brief 简述文件内容
 * 
 * 此处写文件的详细内容，注意需要与上下内容用空格分开。
 * 
 * @author zhangxu@standard-robots.com
 * @date create date：2020/12/17
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef __DETECTOR_H__
#define __DETECTOR_H__

// INCLUDE
#include "cluster.h"
#include "param_base.hpp"
#include "detect_result.hpp"
#include "../../common/algorithm/euclidean_cluster_extraction.h"
#include "../../../common/common_func.hpp"
#include "../../../common/o3d3xxFrame.hpp"

//CODE
namespace perception {
/**
 * @description : Detector detector_base.
 * @note        : All static object recognition must inherit the detector detector_base class.
 * @author      : zhangxu
 * @date        : 2020/12/17 下午4:06
 */
class Detector {
public:
   typedef std::shared_ptr<Detector> Ptr;
   typedef std::shared_ptr<Detector const> ConstPtr ;

   /** @brief constructor. */
    Detector();

    /** @brief copy constructor */
    Detector(const Detector &detector) = delete;

    /** @brief assignment copy constructor */
    Detector& operator=(const Detector &detector) = delete;

    /** @brief destructor. */
    virtual ~Detector() = default;

    /** @brief display param. */
    virtual void showParam() = 0;

    /** @brief set runtime data as default value. */
    virtual void reset() = 0;

    /**
     * @brief detect target in frame.
     * @param[in] frame 3D camera frame.
     * @return return detect result.
     */
    virtual DetectResult detect(const O3d3xxFrame &frame) = 0;

    /**
     * @brief get camera original image.
     * @param[out] img output image.
     */
    void getCameraImage(cv::Mat &img) const;

    /**
    * @brief get an image with a rectangular box with target.
    * @param[out] img output image.
    */
    void getTargetImage(cv::Mat &img) const;

    /**
     * @brief get the point cloud after filter by make.
     * @param[out] filter_by_mask_cloud output point cloud.
     */
    void getMaskCloud(const PointCloudPtr &filter_by_mask_cloud) const;

    /**
     * @brief get the cloud after filter by XYZ-range.
     * @param[out] filter_by_xyz_range_cloud output point cloud.
     */
    void getXYZRangeCloud(const PointCloudPtr &filter_by_xyz_range_cloud) const;

    /**
     * @brief get the cloud after median filter.
     * @param[out] after_median_filter_cloud output point cloud.
     */
    void getMedianFilterCloud(const PointCloudPtr &after_median_filter_cloud) const;

    /**
     * @brief get the cloud after filter by outlier.
     * @param[out] filtered_outlier_cloud output point cloud.
     */
    void getOutlierCloud(const PointCloudPtr &filtered_outlier_cloud) const;

    /**
     * @brief get the cloud after filter by normal angel.
     * @param[out] filter_by_normal_point_cloud output point cloud.
     */
    void getFilterNormalPointCloud(const PointCloudPtr &filter_by_normal_point_cloud) const;

    /**
     * @brief get the normal cloud after filter by normal angel.
     * @param[out] img output point cloud.
     */
    void getNormalFilterNormalCloud(const NormalCloudPtr &filter_by_normal_normal_cloud) const;

    /**
     * @brief get target point cloud.
     * @param[out] img output point cloud.
     */
    void getTargetPointCloud(const PointCloudPtr &cloud) const;

    /**
     * @brief get cluster point cloud.
     * @param[out] cloud output cluster point cloud.
     */
    void getClusterPointCloud(const PointCloudPtr &cloud) const;

    /**
     * @brief get cluster center point list.
     * @param[out] points center point list.
     */
    void getClusterCenter(std::vector<Point3D> &points) const;

    /**
     * @brief get cluster normal cloud.
     * @param[out] centers cluster normal cloud.
     */
    void getClusterNormal(std::vector<Normal> &centers) const;

    /**
     * @brief gets the processing time of the main function.
     * @param[out] consume_time consume time list.
     */
    void getConsumingTime(std::vector<std::string> &consume_time) const;

    /**
    * @brief draw 'mask' text on mask image.
    */
    void drawMask();

    /**
     * @brief draw 'camera image" text on camera image.
     */
    void drawCameraImage();

    /**
     * @brief collection all of cluster center.
     */
    void gatherAllClusterCenter();

protected:
    /**
     * @brief search target in cluster map
     * @return if found target return true, then return false.
     */
    virtual bool searchTarget() = 0;

    /**
     * @brief calculate card detect result.if card all pallet point cloud fitting plane fail,
     *        return false; else return true;
     * @return if card all pallet point cloud fitting plane fail, return false; else return true;
     */
    virtual bool calculateResult() = 0;

    /**
     * @brief generate distance image according to the point(x,y,z).
     * @param[in] cloud point cloud.
     * @param[in,out] distance_img distance image.
     */
    void generateDistanceImage(const PointCloudConstPtr &cloud,
                               cv::Mat_<uint16_t> &distance_img) const;

    /**
     * @brief filter point cloud by mask.
     * @param[in] confidence_img input 3D-camera confidence image.
     * @param[in] amplitude_img input 3D-camera amplitude image.
     * @param[in] input_cloud input 3D-camera point cloud.
     * @param[out] confidence_normalize output confidence normalize image.
     * @param[out] amplitude_normalize output amplitude normalize image.
     * @param[out] filter_mask output mask where pix > mean_pix.
     * @param[out] output_cloud output after filter point cloud.
     */
    static void maskFilter(const cv::Mat &confidence_img,
                           const cv::Mat &amplitude_img,
                           const PointCloudConstPtr &input_cloud,
                           cv::Mat &confidence_normalize,
                           cv::Mat &amplitude_normalize,
                           cv::Mat &filter_mask,
                           PointCloudPtr &output_cloud);

    /**
    * @brief filter point cloud by mask.
    * @param mask_img input mask image.
    * @param input_cloud input 3D-camera point cloud.
    * @param output_cloud output after filter point cloud.
    */
    static void maskFilter(const cv::Mat &mask_img,
                           const PointCloudConstPtr &input_cloud,
                           PointCloudPtr &output_cloud);

    /**
     * @brief filter point cloud by normal.
     * @param[in] normals input normal cloud.
     * @param[in] cloud input point cloud.
     * @param[in] angle_tolerance angle tolerance param.
     * @param[out] output_cloud output filter point cloud.
     * @param[out] output_normal output filter normal cloud.
     */
    static void normalFilter(const NormalCloudConstPtr &normals,
                             const PointCloudConstPtr &cloud,
                             const float &angle_tolerance,
                             PointCloudPtr &output_cloud,
                             NormalCloudPtr &output_normal);

    /**
     * @brief filter point cloud by normal.
     * @param[in] normals input normal cloud.
     * @param[in] cloud input point cloud.
     * @param[in] axle_name the axle of filter ("x" , "y", "z").
     * @param[in] angle_tolerance angle tolerance param.
     * @param[out] output_cloud output filter point cloud.
     * @param[out] output_normal output filter normal cloud.
     */
    static void normalFilter(const NormalCloudConstPtr &normals,
                             const PointCloudConstPtr &cloud,
                             const float &angle_tolerance,
                             const std::string axle_name,
                             PointCloudPtr &output_cloud,
                             NormalCloudPtr &output_normal);

    /**
     * @brief calculate normal estimation.
     * @param[in] cloud input point cloud.
     * @param[out] normals output normal cloud.
     */
    static void calculateNormalEstimation(const PointCloudConstPtr &cloud,
                                   NormalCloud::Ptr &normals);

    /**
     * @brief euclidean segmentation.
     * @param[in] param card detect param.
     * @param[in] normals input normal cloud.
     * @param[in] cloud input point cloud.
     * @param[out] pallets_map segmentation pallet set.
     */
    template<class Param>
    void euclidean(const Param &param,
                   const NormalCloudConstPtr &normals,
                   const PointCloudConstPtr &cloud,
                   std::vector<Indices> &clusters) {
        int64_t start, finish;
        start = common_func::get_timestamp_in_ms();

        // The KD tree object is established to search the nearest point.
        KdTree3d::Ptr kd_tree = std::make_shared<KdTree3d>();
        kd_tree->build(cloud->points);

        // Euclidean Clustering objects.
        EuclideanClusterExtraction clustering;
        // set small values of cluster point size  (small values may cause objects to be divided
        // in several clusters, whereas big values may join objects in a same cluster).
        clustering.setClusterTolerance(param.point_dist_tolerance);
        clustering.setMinClusterSize(param.min_cluster_size);
        clustering.setMaxClusterSize(param.max_cluster_size);
        clustering.setSearchMethod(kd_tree);
        clustering.setInputCloud(cloud);
        clustering.extract(clusters);

        finish = common_func::get_timestamp_in_ms();
        int64_t euclidean_segmentation_time = finish - start;
        LOG(INFO) << "euclidean Segmentation: cloud.size=" << cloud->size() << " -> clusters.size=" << clusters.size()
                  << " time consuming " << euclidean_segmentation_time << "ms";
    }

    /**
 * @brief
 * @param cloud
 * @param min_p
 * @param max_p
 */
    void getMaxMin(const PointCloudConstPtr &cloud, Eigen::Vector4f& min_p, Eigen::Vector4f& max_p);

    /**
     *
     * @param cloud
     * @param min_p
     * @param max_p
     */
    void getMaxMin(const PointCloudConstPtr &cloud, std::vector<float>& min_p, std::vector<float>& max_p);

    /**
     *
     * @param cloud
     * @param output_cloud
     * @param voxel_range
     */
    void voxelGridFilter(const PointCloudConstPtr &cloud,
                         const PointCloudPtr &output_cloud,
                         const Eigen::Vector3f &voxel_range);

    /**
     * @brief filter by range([min_x, max_x],[min_y, max_y],[min_z,max_z]).
     * @param[in] range input detect cube space.
     * @param[in] cloud input point cloud.
     * @param[out] output_cloud output after filter cloud.
     */
    static void passThroughFilter(const Range3D<float> &range,
                                  const PointCloudConstPtr &cloud,
                                  const PointCloudPtr &output_cloud);

    /**
     * @brief Using median filter to smooth the jitter of the point
     * @param cloud[in] source point cloud.
     * @param xyz_cloud[in] cloud Points before filtering.
     * @param output_cloud[out] Points after filtering
     */
    void medianFilter(const PointCloudConstPtr &cloud,
                      const PointCloudConstPtr &xyz_cloud,
                      const PointCloudPtr &output_cloud) const;

    /**
     * @brief Using mean filter to smooth the jitter of the point
     * @param cloud[in] source point cloud.
     * @param xyz_cloud[in] cloud Points before filtering.
     * @param output_cloud[out] Points after filtering
     */
    void meanFilter(const PointCloudConstPtr &cloud,
                    const PointCloudConstPtr &xyz_cloud,
                    const PointCloudPtr &output_cloud) const;

    /**
     * @brief filter outlier point.
     * @param[in] param input detect param.
     * @param[in] cloud input point cloud.
     * @param[out] output_cloud output after filter cloud.
     */
    static void outlierFilter(const PointCloudConstPtr &cloud,
                              const PointCloudPtr &output_cloud);

    /**
     * @brief find max connected component pixel area.
     * @param[in] binImg input bin image(pixel_value=0/1).
     * @param[out] card_rect output card rectangle.
     */
    static void findMaxConnectedComponent(const cv::Mat &binImg,
                                          cv::Rect &card_rect);

    /**
     * @brief random consistency sampling. from a group of high noise points,
     *        find a group of points that conform to the model.
     * @param[in] cloud input point cloud.
     * @param[in] normals input normal cloud.
     * @param[out] inner output inner points.operator=!
     * @param[out] coefficients_plane fitting plane's param.
     */
    static void ransac(const PointCloudConstPtr &cloud,
                       const NormalCloudConstPtr &normals,
                       std::vector<int> &inner,
                       std::vector<float> &coefficients_plane);

    /** @brief height of camera image. */
    int image_height_{};

    /** @brief width of camera image. */
    int image_width_{};

    /** @brief point cloud deep image. */
    cv::Mat_<uint8_t> deep_img_;

    /** @brief store the points after filtering mask point cloud. */
    cv::Mat_<uint8_t> filter_mask_img_;

    /** @brief 3D camera amplitude normalize. */
    cv::Mat_<uint8_t> amplitude_normalize_;

    /** @brief 3D camera confidence normalize. */
    cv::Mat_<uint8_t> confidence_normalize_;

    /** @brief converted 3d image frame. */
    O3d3xxFrame tf_frame_;

    /** @brief Point cloud constructed by amplitude and confidence. */
    cv::Mat mask_img_;

    /** @brief camera original image. */
    cv::Mat camera_img_;

    /** @brief card of target image. */
    cv::Mat target_img_;

    /** @brief mask filtered point cloud. */
    PointCloudPtr filter_mask_cloud_;

    /** @brief mask front filtered point cloud.*/
    PointCloudPtr filter_front_cloud_;

    /** @brief xyz-range filtered points. */
    PointCloudPtr filter_xyz_range_cloud_;

    /** @brief point cloud after median filter. */
    PointCloudPtr filter_median_cloud_;

    /** @brief point cloud after outlier removal. */
    PointCloudPtr filter_outlier_cloud_;

    /** @brief point cloud filtered by normal vector. */
    PointCloudPtr filter_normal_point_cloud_;

    /** @brief filtered normal vector set. */
    NormalCloudPtr filter_normal_normal_cloud_;

    /** @brief point cloud segmentation cluster set. */
    std::map<int, Cluster> clusters_map_;

    /** @brief detect result. */
    DetectResult result_;

    /** @brief  target point cloud. */
    PointCloudPtr target_cloud_;

    /** @brief all cluster point cloud. */
    PointCloudPtr clusters_cloud_;

    /** @brief time consumed by each function. */
    std::vector<std::string> consuming_time_recorder_;
};

using DetectorPtr = Detector::Ptr ;
using DetectorConstPtr = Detector::ConstPtr ;
} // end of namespace perception
#endif //PERCEPTION_SOLUTION_DETECTOR_H
