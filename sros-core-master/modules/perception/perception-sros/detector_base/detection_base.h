/**
 * @file detector_module_base.h
 * @brief Base class for all static object detection
 *
 * Base class for all static object detection,Subclasses need to override virtual
 * functions after inheriting this base class,
 *
 * @note the run() function need to override by inheriting.
 * @author zhangxu@standard-robots.com
 * @date create date：2021/1/5
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef __SROS_DETECTOR_MODULE_BASE_H__
#define __SROS_DETECTOR_MODULE_BASE_H__
// INCLUDE
#include "core/core.h"
#include "core/tf/TFOperator.h"
#include "core/msg/perception_command_msg.hpp"
#include "core/msg/perception_state_msg.hpp"
#include "../../lib/object_detectors/base/detect_result.hpp"
#include "../../common/o3d3xxFrame.hpp"
#include <fstream>

// CODE
namespace object_detector {
/**
 * @description : Base class for all static object detection
 * @author      : zhangxu
 * @date        : 2021/1/5 上午11:02
 */
class DetectorModuleBase {
 public:
    typedef sros::core::PerceptionCommandMsg::Command PerceptionCommand;
    typedef boost::function<void( sros::core::base_msg_ptr )> MsgCallback;
    typedef boost::function<void( std::string, MsgCallback )> TopicCallback;

    /**
     * @brief Parametric constructor.
     * @param module_name name of create module.
     */
    explicit DetectorModuleBase(const std::string &module_name,const MsgCallback &sendMsg,const TopicCallback &subscribeTopic);

    /**
     * @brief Delete assignment constructor.
     */
    DetectorModuleBase& operator=(const DetectorModuleBase &) = delete;

    /**
     * @brief Delete copy constructor.
     */
    DetectorModuleBase(const DetectorModuleBase&) = delete;

    /**
     * @brief Deconstruction
     */
    virtual ~DetectorModuleBase() = default;

    /**
     * @brief Main processing functions.
     */
    virtual bool init() = 0;

    /**
     * @brief Whether the detection time is out of time.
     * @param msg pointer of sros::core::base_msg_ptr.
     */
    virtual void checkDetectTime(const sros::core::base_msg_ptr &msg);

 protected:

    /**
     * @brief Receive sensor massage processing function for derived class inheritance.
     * @param[in] msg sensor massage.
     */
    virtual void onSensorMsg(const sros::core::base_msg_ptr& msg) = 0;

    /**
     * @brief Load the parameters required by the detection for the derived
     *        class inheritance.
     */
    virtual void loadDetectParam() = 0;

    /**
     * @brief Load the parameters of the detection target for the derived
     *        class inheritance
     */
    virtual void loadTargetParam() = 0;

    /**
     * @brief default enable ifm-o3d3xx camera data publishing.
     */
    virtual void enableSensor();

    /**
     * @brief default disable ifm-o3d3xx camera data publishing.
     */
    virtual void disableSensor();

    /**
     * @brief Initialize detection environment.
     */
    virtual void startDetector();

    /**
     * @brief Remove residual test data.
     */
    virtual void stopDetector();

    /**
     * @brief Handle request detection command. if recive detection command, If the
     *        detection request is received, the camera will be turned on to publish data.
     * @param[in] msg command massage
     */
    virtual void onDetectCommandMsg(const sros::core::base_msg_ptr &msg) = 0;

    /**
     * @brief show the detect result.
     * @param[in] result detect result.
     * @param[in] time The time taken to detect the current frame。
     */
    static void showResult(const DetectResult &result, const int64_t &time);

    /**
     * @brief show the detect result and send out to topic.
     * @param[in] result_msg detect result.
     */
    void sendResultMsg(const sros::core::PerceptionStateMsg::Ptr &result_msg);

    /**
     * @brief The average value of multiple detect results was counted.
     * @param target_poses detect result list.
     * @return The average value of multiple detect results.
     */
    static Eigen::Vector3f computeTargetMeanPose(const std::vector<Eigen::Vector3f> &Target_poses);

    /**
     * @brief The average value of multiple detect results was counted.
     * @param target_poses detect result list.
     * @return The average value of multiple detect results.
     */
    static Eigen::Vector4f computeTargetMeanPose(const std::vector<Eigen::Vector4f> &pallet_poses);

    /**
     * @brief record sensor data..
     * @param[in] dir Directory of records.
     * @param[in] frame o3d3xx frame to be recorded.
     */
    void recordSensorData(const std::string &dir,
                          const O3d3xxFrame &frame);
                          
    void recordSensorData(const std::string &dir,
                          const std::string &type,
                          const O3d3xxFrame &frame);

    /**
     * @brief record sensor data..
     * @param[in] dir Directory of records.
     * @param[in] cloud point cloud to be recorded.
     */
    void recordSensorData(const std::string &dir,
                          const PointCloudPtr &cloud);

    void recordSensorData(const std::string &dir,
                          const std::string &type,
                          const PointCloudPtr &cloud);

    /**
     * @brief read sensor data and init frame.
     * @param file_name  file name of sensor data.
     * @param frame o3d3xx frame.
     * @return if read frame successful then return ture, otherwise return false.
     */
    bool readSensorData(const std::string &file_name,
                        O3d3xxFrame &frame);
    bool readSensorData(const std::string &file_name,
                        const PointCloudPtr &frame);

 protected:

    /** @brief whether record the sensor data, default false. */
    bool is_record_;

    /** @brief whether record the sensor data, default false. */
    bool is_init_detector_;

    /** @brief The location of the log record when the detection fails. */
    std::string sensor_data_file_path_;

    /** @brief The location of the log record when the detection fails. */
    std::ofstream out_put_;

    /** @brief Write data log file descriptor. */
    std::ifstream in_put_;

    /** @brief Read data log file descriptor. */
    uint64_t receive_cmd_time_;

    /** @brief perception command type. */
    PerceptionCommand command_;

    /** @brief Conversion from camera coordinate system to world coordinate system. */
    std::shared_ptr<slam::tf::TFOperator> tf_base_to_world_;

    /** @brief Current number of detect results. */
    int detected_count_ = 0;

    /** @brief detect object index. */
    int goal_id_{};

    /** @brief Data processing switch. */
    bool enable_process_;

    /** @brief It is used to judge whether the detection command is received or not. */
    bool is_receive_cmd_;

    /** @brief Collection of test results. */
    std::vector<Eigen::Vector3f> detected_poses_;

    /** @brief Collection of pose (x,y,z,yaw) results. */
    std::vector<Eigen::Vector4f> detected_xyzyaw_;

    /** @brief Camera position relative to AGV. */
    Eigen::Vector3f camera_to_base_pose_;

    float laser_install_fork_z_offset_;

    float laser_install_pitch_;

    /** @brief current position of the AGV in world. */
    Eigen::Vector3d cur_pose_;

    /** @brief The theoretical position of the target. */
    Eigen::Vector3d theory_pose_;

    /** @brief Count the times of detect results, (and calculate the test
     *         results when the times exceed the set value). */
    const int DETECTED_COUNT_THRESH = 4;

    /** @brief Time required for detecting. */
    const int DETECTED_TIME_OUT = 5000;

    /** @brief send massage call back function pointer. */
    MsgCallback sendMsg_;

    /** @brief subscribe topic call back function pointer. 13631258369*/
    TopicCallback subscribeTopic_;
};

}   // end of object detector namespace
#endif  // SROS_DETECTOR_MODULE_BASE_H
