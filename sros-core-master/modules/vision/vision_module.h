/*
 * vision_module.h
 *
 *  Created on: 2018/05/31
 *      Author: lhx
 */

#ifndef SROS_VISION_MODULE_H_
#define SROS_VISION_MODULE_H_

#include <opencv2/core/core.hpp>

#include <core/msg/image_msg.hpp>
#include <eigen3/Eigen/Dense>

#include "core/core.h"
#include "core/msg/str_msg.hpp"
#include "core/pose.h"

#include "include/dm_detector.h"
#include "include/fm_detector.h"
#include "include/obstacle_finder.h"
#include "msg_process_manager.hpp"
#include "core/tf/TFOperator.h"

namespace vision {

struct DmDetectorInfo {
    sros::device::Device_ptr device;
    DataMatrixDetector dm_detector;
    FractalMarkerDetector fm_detector;
    std::string suffix;
    std::string camera_name;
    std::string code_type;
    double dm_code_direction;
    double camera_install_diretion = 0;
    bool is_right_handed = false;
    uint16_t start_addr = 0x1240;
    std::shared_ptr<msg_process::MsgProcessManager<sros::core::image_msg_ptr>> process_manager;
    Eigen::Vector3d camera_2_robot_center = Eigen::Vector3d::Zero();  //相机坐标系到运动中心的坐标变换
    Eigen::Vector3d code_2_rack_center = Eigen::Vector3d::Zero();     //二维码到货架中心的变换
    bool last_dm_detect_ok_;
    bool dm_detector_watch_dog_;
    Eigen::Quaterniond rotate_matrix;
    std::deque<cv::Mat> images;
    bool dmcode_runstate = false;
};
typedef std::shared_ptr<DmDetectorInfo> DmDetectorInfoPtr;

class VisionModule : public sros::core::Module {
 public:
    VisionModule();

    virtual ~VisionModule() = default;

    virtual void run();

    int getVersion();

    std::string getVersionStr();

    template <class T>
    void normalizeAngle(T &angle) {
        const T &max_angle = (T)(2 * M_PI);

        angle = fmod(angle, (max_angle));
        if (angle >= (T)(M_PI)) {
            angle -= max_angle;
        } else if (angle < -(T)(M_PI)) {
            angle += max_angle;
        }
    }

 private:
    void initializeDmDetectorInfo(DmDetectorInfoPtr &dm_detector_info,const std::string svc100_prefix);

    void onColorImageMsg(sros::core::base_msg_ptr m);

    void onRotateJackUpEndMsg(sros::core::base_msg_ptr m);

    void onRotateJackDownEndMsg(sros::core::base_msg_ptr m);

    void processDetect(DmDetectorInfoPtr &dm_detector_info, sros::core::image_msg_ptr m);

    void processDmDetect(DmDetectorInfoPtr &dm_detector_info, sros::core::image_msg_ptr m,bool enable_svc100_debug_output);

    void processFmDetect(DmDetectorInfoPtr &detector_info, sros::core::image_msg_ptr m,bool enable_svc100_debug_output);

    void updateCalibrationParaOnline(DmDetectorInfoPtr &dm_detector_info);

    void onTimer_1s(sros::core::base_msg_ptr m);

    void onSaveImgCmd(sros::core::base_msg_ptr m);
    
    void onSaveFailureImgCmd(sros::core::base_msg_ptr m);

    void calibrateInstallPose(Eigen::Vector3d &forwad_pose, Eigen::Vector3d &back_pose,
                              const double camera_initial_direction, Eigen::Vector3d &camera_2_robot_center,
                              Eigen::Vector3d &code_2_rack_center);

    void convertToRobotCenterPose(const Eigen::Vector3d &camera_2_robot_center,
                                  const Eigen::Vector3d &code_2_rack_center, const double dm_code_direction,bool use_right_hand,
                                  DMResult &result);

    std::vector<double> splitStrsToDoubles(const std::string &s, const char seperator);

    std::deque<cv::Mat> save_failure_img_;
    DmDetectorInfoPtr dm_up_detector_info;
    DmDetectorInfoPtr dm_down_detector_info;
    DmDetectorInfoPtr dm_svc200_up_detector_info;
    DmDetectorInfoPtr dm_svc200_down_detector_info;
    DmDetectorInfoPtr fm_behind_detector_info;
    DmDetectorInfoPtr fm_d435_1_detector_info;
    DmDetectorInfoPtr fm_d435_2_detector_info;
    DmDetectorInfoPtr fm_d435_3_detector_info;
    DmDetectorInfoPtr fm_d435_4_detector_info;
    DmDetectorInfoPtr fm_svc200_detector_info;
    DmDetectorInfoPtr fm_mvs_detector_info;
    std::shared_ptr<slam::tf::TFOperator> tf_base_to_odo;
    DMResult lastsult;
    //    double dm_code_direction;
};

}  // namespace vision

#endif  // SROS_VISION_MODULE_H_
