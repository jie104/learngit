/**
 * @file custom_region_detector_module.h
 * @brief Module for custom area detection.
 *
 * It is used to process whether there are obstacles in the defined detection area, subscribe
 * PerceptionCommandMsg topic messages, and publish detection results to ELECTRODES_DETECT_RESULT
 * topics. The configuration parameters of user-defined area detection are in the perception
 * parameter section. (e.g perception.custom_ region_ get_ good）
 *
 * @author zhangxu@standard-robots.com
 * @date create date：2021/1/12
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef __SROS_CUSTOM_REGION_DETECTOR_MODULE_H__
#define __SROS_CUSTOM_REGION_DETECTOR_MODULE_H__

// INCLUDE
#include <Eigen/Dense>
#include "core/core.h"
#include "core/msg/laser_scan_msg.hpp"
#include "core/msg/perception_command_msg.hpp"
#include "core/tf/TFOperator.h"
#include "modules/navigation/lib/include/geometry.h"
#include "modules/perception/perception-sros/detector_base/detection_base.h"

// CODE
namespace object_detector {
/**
 * @description : Module for custom area detection.
 * @author      : zhangxu
 * @date        : 2021/1/12 下午2:33
 */
class CustomRegionDetectorModule : public DetectorModuleBase  {
 public:
    typedef sros::core::PerceptionCommandMsg::Command PerceptionCommand;
    typedef std::shared_ptr<CustomRegionDetectorModule> Ptr;
    typedef std::shared_ptr<const CustomRegionDetectorModule> ConstPtr;

    /**
     * @brief User defined sector
     * @note Parameter range: begin_angle < end_angle and radius > 0.
     */
    struct SectorRegion{
        float begin_angle = .0f;
        float end_angle = .0f;
        float radius = .0f;
    };

    typedef std::vector<Eigen::Vector2d>  Rectangle;

    /**
     * @brief Constructors.
     */
    CustomRegionDetectorModule(const MsgCallback &sendMsg, const TopicCallback &subscribeTopic);

    /**
     * @brief Destructor.
     */
    ~CustomRegionDetectorModule() override = default;

    /**
     * @brief Delete Assignment constructor.
     */
    CustomRegionDetectorModule& operator=(const CustomRegionDetectorModule&) = delete;

    CustomRegionDetectorModule(const CustomRegionDetectorModule&) = delete;

    static bool isEnable();

    /**
     * @brief Main processing functions.
     */
    virtual bool init() override;

    /**
     * @brief 05LA_SCAN topic message processing function.
     * @param[in] msg sros::core::LaserScanMsg.
     */
    void onSensorMsg(const sros::core::base_msg_ptr& msg) override;

    /**
     * @brief DETECT_COMMAND topic message processing function.
     * @param[in] msg sros::core::PerceptionCommandMsg.
     */
    void onDetectCommandMsg(const sros::core::base_msg_ptr& msg) override;

 private:

    /**
     * @brief load parameter of detect area.
     * @return if parameter legal, return ture, else return false.
     */
    void loadDetectParam() override;

    /**
     * @brief Load the target parameters to be detected.
     */
    void loadTargetParam() override {};

    /**
     * @brief Verify the integrity of the received data
     * @param[in] scan_msg laser scan msg
     * @return If the data is abnormal, return false, otherwise return true.
     */
    bool isScanDataIntegrity(const sros::core::LaserScan_ptr &scan_msg);

    /**
     * @brief Display obstacle avoidance radar data to Matrix.
     * @param[in] cur_pose pose of AGV when start detect.
     * @param[in] scan_msg laser data massage.
     */
    void showLaserScanOnMatrix(const Eigen::Vector3d &cur_pose,
                               const sros::core::LaserScan_ptr &scan_msg);

    /**
     * @brief decode rectangle parameter from information string.
     * @param[in] info_str string parameter was loaded from config.csv.
     * @param[in] rectangle Four points of rectangular area.
     * @return if decode parameter error, then return false, else return true.
     */
    static bool buildRectangleInfo(const std::string& info_str,
                            Rectangle &rectangle);

    /**
     * @brief Judge whether the sector area is occupied or not， in other words,
     *        judge whether there are laser points in the sector area.
     * @param[in] region sector area.
     * @param[in] scan laser scan data.
     * @return if region is occupy return ture, otherwise return false.
     */
    static bool isSectorRegionOccupy(const SectorRegion &region,
                                     const sros::core::LaserScan_ptr &scan);

    /**
     * @brief Judge whether there are laser points in the sector area.
     * @param rectangle rectangle area.
     * @param scan laser scan data.
     * @return if region is occupy return ture, otherwise return false.
     */
    static bool isRectangleRegionOccupy(const Rectangle &rectangle,
                                        const sros::core::LaserScan_ptr &scan);

    /** @brief rectangle detect area. */
    Rectangle rectangle_;

    /** @brief current position of the AGV in world. */
    Eigen::Vector3d cur_pose_;
};

using CustomRegionDetectorModulePtr = CustomRegionDetectorModule::Ptr;
using CustomRegionDetectorModuleConstPtr = CustomRegionDetectorModule::ConstPtr;

} // end of namespace space_detector.
#endif  // SROS_CUSTOM_REGION_DETECTOR_MODULE_H
