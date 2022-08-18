/**
 * @file perception_module.h
 * @brief The class responsible for the card awareness module.
 *
 * If the parameter of main.enable_card_detect is set to true, the card sensing module will be
 * turned on and waiting for the collet to be RECEIVED_QUERY_COMMAND and IFM3D_IMG message,
 * once received PALLET_QUERY_COMMAND message, that is enable_img_process_ will be set to
 * true, the next IFM3D_IMG received will be processed by run(), than the card detection
 * program will be executed and the result will be sent to PALLET_INFO topic.
 *
 * @author zhangxu@standard-robots.com
 * @date create date：2020/11/25
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef __SROS_PUT_SPACE_DETECTOR_MODULE_H__
#define __SROS_PUT_SPACE_DETECTOR_MODULE_H__

// INCLUDE
#include "../../lib/object_detectors/put_space_detector/put_space_detect_param.hpp"
#include "../../lib/object_detectors/put_space_detector/put_space_detector.h"
#include "modules/perception/perception-sros/detector_base/detection_base.h"
//
#include "core/msg/point_cloud_msg.hpp"
// CODE
namespace object_detector {
/**
 * @description : The class responsible for the card awareness module.
 * @author      : lijunhong
 * @date        : 2022/01/15 下午4:01
 */
class PutspaceDetectModule : public DetectorModuleBase {
 public:
    typedef sros::core::PerceptionCommandMsg::Command Command;
    typedef std::shared_ptr<PutspaceDetectModule> Ptr;
    typedef std::shared_ptr<const PutspaceDetectModule> ConstPtr;
    typedef std::array<sros::core::Location, 4>  Rectangle;

    /**
     * @brief construction.
     */
    PutspaceDetectModule(const MsgCallback &sendMsg, const TopicCallback &subscribeTopic);

    /**
     * @brief deconstruction.
     */
    ~PutspaceDetectModule() override = default;

    /**
     * @brief Main processing functions.
     */
    virtual bool init() override;

    /**
     * @brief Check whether the function is on
     * @return if detector is enable then return true. otherwise return false.
     */
    static bool isEnable();

    /**
     * @brief Handling IFM3D_IMG command.
     * @param msg pointer of sros::core::ImageMsg.
     */
    void onSensorMsg(const sros::core::base_msg_ptr& msg) override;

    /**
     * @brief Handling LIVOX_MID70 command.
     * @param msg
     */
    void onHeadupMid70SensorMsg(const sros::core::base_msg_ptr& msg);


    void onPutSpaceMid70SensorMsg(const sros::core::base_msg_ptr& msg);

    /**
     * @brief After receiving the command, turn on the detection.
     * @param msg pointer of sros::core::ImageMsg.
     */
    void onDetectCommandMsg(const sros::core::base_msg_ptr &msg) override;

    /**
     * @brief Initialize detection environment.
     */
    void startDetector() override;

    /**
     * @brief Remove residual test data.
     */
    void stopDetector() override;

    /**
     * @brief default enable ifm-o3d3xx camera or Livox-mid70 lidar data publishing.
     */
    void enableSensor() override;

    /**
     * @brief default disable ifm-o3d3xx camera or Livox-mid70 lidar data publishing.
     */
    void disableSensor() override;


    /**
    * @brief Calculate the global coordinates of the detection area.
    * @param[in] cur_pose current pose of agv.
    * @param[in,out] rect detection area.
    */
    void calculateDetectCubeRange(const Eigen::Vector3d &cur_pose, Range3D<float>& cube_range);


    /**
     * @brief
     * @param cube_range
     * @param laser_points
     * @return
     */
    bool isRectangleRegionOccupy(const Range3D<float> &cube_range, const std::vector<Eigen::Vector3d>& laser_points);


    /**
    * @brief convert laser scan point to world.
    * @param[in] world_tf scan to world transform.
    * @param[in] scan_msg laser data massage.
    * @param[in,out] laser_points laser point in glob world.
    */
    void convertScanToWorld(const Eigen::Affine2d &world_tf,
                            const Range3D<float> &range,
                            const PointCloudPtr& cloud,
                            std::vector<Eigen::Vector3d> &laser_points);


    /**
    * @brief load object detect area by object index;
    * @param[in] object_index  = -1 search all of the targets and get the max.
    *                          >= 0 Only the index area of the specified target is detected.
    * @return if object area init fail return false, otherwise return true.
    */
    bool loadObjectArea(const int object_index, const Command &cmd);

    /**
     * @brief
     * @param cloud
     * @return
     */
    bool putSpaceDetect(slam::tf::TransForm& curr_pose, const PointCloudPtr& cloud);


    private:

       /**
        * @brief load card detector parameter;
        */
    void loadDetectParam() override;

    /**
     * @brief Load the target parameters to be detected.
     */
    void loadTargetParam() override;






private:
    /** @brief The parameter of the card detector. */
    PutspaceDetectParam param_{};

    /** @brief The queue of 3d lidar frame*/
    std::deque<PointCloudPtr> frame_queue_;

    /** @brief Objects responsible for detect card. */
    perception::PutspaceDetector put_space_detector_;


    /** @brief Time required for detecting. */
    const int DETECTED_TIME_OUT = 5000;

    /** @brief The number of accumulated laser frames. */
    const int ACCUMULATE_QUEUE_SIZE_COUNT = 3000 ;

    /** The flag of use mid70 laser.*/
    bool is_mid70_laser_ = false;

    /** @brief scan transform to agv world */
    Eigen::Affine2d scan_to_agv_tf_;

    /** @brief detector base parameter. */
    ParamBase param_base_;

    /** @brief max length of object. */
    float object_length_{};

    /** @brief max width of object. */
    float object_width_{};

    /** @brief max width of object. */
    float object_height_{};

    Range3D<float> cube_range_{};

    uint64_t last_frame_time_ = 0;


//    PerceptionCommandMsg::ObjectType put_space_type_{};

};

using PutspaceDetectModulePtr = PutspaceDetectModule::Ptr;
using PutspaceDetectModuleConstPtr = PutspaceDetectModule::ConstPtr;

} // end of object_detector namespace
#endif  // SROS_CARD_DETECTOR_MODULE_H