/**
 * @file QRcode_detection.h
 * @brief 简述文件内容
 *
 * 此处写文件的详细内容，注意需要与上下内容用空格分开。
 *
 * @author zhangxu@standard-robots.com
 * @date create date：2021/7/1
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef __SROS_QRCODE_DETECTION_H__
#define __SROS_QRCODE_DETECTION_H__

// INCLUDE
#include "core/core.h"
#include "core/tf/TFOperator.h"
#include "core/msg/perception_state_msg.hpp"
#include "core/msg/perception_command_msg.hpp"
#include <memory>
#include "core/tf/transform_3d.hpp"

// CODE
namespace object_detector {
/**
 * @description : The class responsible for the QRcode awareness module.
 * @author      : zhangxu
 * @date        : 2021/7/1 下午4:41
 */
struct QRCodeWidthOdoPair{
    slam::tf::Transform3D odo_pose;
    slam::tf::Transform3D code_pose;
};

class QRcodeDetection {
 public:
    using PerceptionCommand = sros::core::PerceptionCommandMsg::Command;
    using PerceptionCommandMsg = sros::core::PerceptionCommandMsg;
    using PerceptionCommandMsgPtr = sros::core::PerceptionCommandMsgPtr;
    using PerceptionStateMsg = sros::core::PerceptionStateMsg;
    using PerceptionStateMsgPtr = sros::core::PerceptionStateMsgPtr;

    typedef std::shared_ptr<QRcodeDetection> Ptr;
    typedef std::shared_ptr<const QRcodeDetection> ConstPtr;
    typedef boost::function<void( sros::core::base_msg_ptr )> MsgCallback;
    typedef boost::function<void( std::string, MsgCallback )> TopicCallback;

    /**
     * @brief detection constructor.
     * @param sendMsg send message callback.
     * @param subscribeTopic subscribe topic callback.
     */
    QRcodeDetection(const MsgCallback &sendMsg, const TopicCallback &subscribeTopic);

    /**
     * @brief After receiving the command, turn on the detection.
     * @param msg pointer of sros::core::ImageMsg.
     */
    ~QRcodeDetection() = default;

    /**
     * @brief Delete assignment constructor.
     */
    QRcodeDetection& operator=(const QRcodeDetection &) = delete;

    /**
     * @brief Delete copy constructor.
     */
    QRcodeDetection(const QRcodeDetection&) = delete;

    /**
     * @brief Initialize detector and return true if successful, otherwise return false.
     */
    bool init();

    /**
     * @brief checking the detection function is open. returns true if successful, otherwise returns false.
     */
    static bool isEnable();

    /**
     * @brief Enable the sensor to work.
     */
    void enableSensor();

    /**
     * @brief Enable the sensor to stop working.
     */
    void disableSensor();

    /**
     * @brief The detector is working.
     */
    void startDetector();

    /**
     * @brief The detector stops working.
     */
    void stopDetector();

    /**
     * @brief Whether the detection time is out of time.
     * @param msg pointer of sros::core::base_msg_ptr.
     */
    virtual void checkDetectTime(const sros::core::base_msg_ptr &msg);

    /**
     * @brief Odometer data processing function is responsible for fusing QR code and odometer information,
     *        and publishing the integration results to DETECT_RESULT topic.
     * @param msg pointer of odometer message.
     */
    void onOdomMsg(const sros::core::base_msg_ptr& msg);

    /**
     * @brief The QR code processing function is responsible for recording the latest QR code position
     *       and attitude and updating the latest odometer TF.
     * @param msg pointer of sros::core::ImageMsg.
     */
    void onQRcodeMsg(const sros::core::base_msg_ptr& msg);

    /**
     * @brief Responsible for opening and closing the QR code detector.
     * @param msg pointer of sros::core::PerceptionCommandMsg.
     */
    void onDetectCommandMsg(const sros::core::base_msg_ptr &msg);

    bool checkDetectAngleThresh(const slam::tf::Transform3D& camera_in_code,double yaw_thresh = 0.30);

    slam::tf::Transform3D getBoltInHoleTF(const slam::tf::Transform3D& code_pose,const slam::tf::Transform3D& odo_pose,const slam::tf::Transform3D& real_time_odo);

 private:
    /** @brief send massage call back function pointer. */
    MsgCallback sendMsg_;

    /** @brief subscribe topic call back function pointer. 13631258369*/
    TopicCallback subscribeTopic_;

    /** @brief Used to mark whether the current detection is successfully initialized. */
    bool is_init_detector_;

    /** @brief Read data log file descriptor. */
    uint64_t receive_cmd_time_;

    /** @brief The last time the QR code was detected. */
    uint64_t last_found_QRcode_time_;

    /** @brief ID number of QR code. */
    int QRcode_id_;

    /** @brief perception command type. */
    PerceptionCommand command_;

    /** @brief It is used to judge whether the detection command is received or not. */
    bool is_receive_cmd_;

    /** @brief Used to record whether the QR code is recognized */
    bool is_find_QRcode_;


    /** @brief Get the transformation tree of Odom coordinate system. */
    std::shared_ptr<slam::tf::TFOperator> tf_base_to_odo_;

    /** @brief The method of transforming the pose to the coordinate system based on the current odometer pose. */
    slam::tf::Transform3D odom_base_tf_;

    /** @brief camera in camera world pose. */
    slam::tf::Transform3D camera_in_QRcode_pose_;

    /** @@brief bolt to camera coordinate system conversion.*/
    slam::tf::Transform3D bolt_in_camera_tf_;

    slam::tf::Transform3D camera_in_bolt_tf_;

    slam::tf::Transform3D camere_in_center_tf_;

    slam::tf::Transform3D center_in_camera_tf_;

    /** @@brief hole to QRcode coordinate system conversion.*/
    slam::tf::Transform3D hole_in_QRcode_tf_;

    std::deque<QRCodeWidthOdoPair> code_with_odo_pairs_;
    /** @brief process switch. */
    bool enable_process_;

    /** @brief Time required for detecting. */
    const int DETECTED_TIME_OUT;
};

using QRcodeDetectionPtr = QRcodeDetection::Ptr;
using QRcodeDetectionConstPtr = QRcodeDetection::ConstPtr;
}
#endif  // SROS_QRCODE_DETECTION_H
