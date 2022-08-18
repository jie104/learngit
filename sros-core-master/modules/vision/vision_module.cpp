/*
 * vision_module.cpp
 *
 *  Created on: 2016/12/01
 *      Author: lhx
 */

#include "vision_module.h"
#include <core/msg/data_matrix_code_msg.hpp>
#include <ctime>
#include <iomanip>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <core/msg/common_command_msg.hpp>
#include <core/msg/common_msg.hpp>
#include "core/device/device_manager.h"
#include "core/msg/ObstacleMsg.hpp"
#include "core/msg/data_matrix_code_msg.hpp"
#include "core/msg/image_msg.hpp"
#include "core/settings.h"
#include "core/src.h"
#include <core/util/record_file_manager.hpp>

namespace vision {

using namespace std;
using namespace cv;
//#define DEBUG_ON_PC

template <class Quaternion>
static void QuaternionToEuler(const Quaternion& quaternion,double &yaw, double &roll, double &pitch) {
    auto w = quaternion.w();
    auto x = quaternion.x();
    auto y = quaternion.y();
    auto z = quaternion.z();
//    yaw = atan2(2*(w * z + x * y), 1 - 2 * (z * z + x * x));
//    roll = asin(2*(w * x - y * z));
//    pitch = atan2(w * y + z * x, 1 - 2 * (x * x + y * y));
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (w * y - z * x);
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

template <class Quaternion>
void EulerToQuaternion(double yaw, double roll, double pitch,
                       Quaternion &quaternion) {
    double cy = cos(yaw / 2.0);
    double sy = sin(yaw / 2.0);
    double cr = cos(roll / 2.0);
    double sr = sin(roll / 2.0);
    double cp = cos(pitch / 2.0);
    double sp = sin(pitch / 2.0);
    quaternion.w() = cr * cp * cy + sr * sp * sy;
    quaternion.x() = sr * cp * cy - cr * sp * sy;
    quaternion.y() = cr * sp * cy + sr * cp * sy;
    quaternion.z() = cr * cp * sy - sr * sp * cy;
}

VisionModule::VisionModule() : Module("VisionModule") {}

void VisionModule::run() {
    //    waitForStartCommand();
    //
    slam::tf::FrameToFrame base_to_odo_frame;
    base_to_odo_frame.parent_frame = "odom";
    base_to_odo_frame.child_frame = "base_link";
    tf_base_to_odo.reset(new slam::tf::TFOperator(base_to_odo_frame));

    auto &s = sros::core::Settings::getInstance();

    auto enable_vision_module = (s.getValue<std::string>("main.enable_vision_module", "False") == "True");
    if (!enable_vision_module) {
        LOG(INFO) << "vision module not running!";
    }
    LOG(INFO) << "VisionModule module start running";

    auto enable_svc100_camera = (s.getValue<std::string>("camera.enable_svc100_camera", "False") == "True");
    if (enable_svc100_camera) {
        dm_up_detector_info.reset(new DmDetectorInfo);
        dm_up_detector_info->suffix = "";
        dm_up_detector_info->camera_name = sros::device::DEVICE_SVC100_UP;
        dm_up_detector_info->start_addr = 0x1240;
        EulerToQuaternion(0, 0, 0, dm_up_detector_info->rotate_matrix);//初始化，需要在initialize之前赋值
        initializeDmDetectorInfo(dm_up_detector_info,"svc100_");
        dm_up_detector_info->process_manager.reset(new msg_process::MsgProcessManager<sros::core::image_msg_ptr>(
            boost::bind(&VisionModule::processDetect, this, dm_up_detector_info, _1), 0));
        dm_up_detector_info->process_manager->creatMatchThread();
    }
    auto enable_svc100_down_camera = (s.getValue<std::string>("camera.enable_svc100_down_camera", "False") == "True");
    if (enable_svc100_down_camera) {
        dm_down_detector_info.reset(new DmDetectorInfo);
        dm_down_detector_info->suffix = "down_";
        dm_down_detector_info->start_addr = 0x1250;
        dm_down_detector_info->camera_name = sros::device::DEVICE_SVC100_DOWN;
        EulerToQuaternion(0, M_PI, 0, dm_down_detector_info->rotate_matrix);//初始化，需要在initialize之前赋值
        initializeDmDetectorInfo(dm_down_detector_info,"svc100_");
        dm_down_detector_info->process_manager.reset(new msg_process::MsgProcessManager<sros::core::image_msg_ptr>(
            boost::bind(&VisionModule::processDetect, this, dm_down_detector_info, _1), 3));
        dm_down_detector_info->process_manager->creatMatchThread();
    }

    auto enable_svc200_up_camera = (s.getValue<std::string>("camera.enable_up_camera", "False") == "True");
    if (enable_svc200_up_camera) {
        dm_svc200_up_detector_info.reset(new DmDetectorInfo);
        dm_svc200_up_detector_info->suffix = "up_";
        dm_svc200_up_detector_info->camera_name = sros::device::DEVICE_SVC200_UP;
        dm_svc200_up_detector_info->start_addr = 0x1240;
        EulerToQuaternion(0, 0, 0, dm_svc200_up_detector_info->rotate_matrix);//初始化，需要在initialize之前赋值
        initializeDmDetectorInfo(dm_svc200_up_detector_info,"");
        dm_svc200_up_detector_info->process_manager.reset(new msg_process::MsgProcessManager<sros::core::image_msg_ptr>(
            boost::bind(&VisionModule::processDetect, this, dm_svc200_up_detector_info, _1), 0));
        dm_svc200_up_detector_info->process_manager->creatMatchThread();
    }

    auto enable_svc200_down_camera = (s.getValue<std::string>("camera.enable_down_camera", "False") == "True");
    if (enable_svc200_down_camera) {
        dm_svc200_down_detector_info.reset(new DmDetectorInfo);
        dm_svc200_down_detector_info->suffix = "down_";
        dm_svc200_down_detector_info->start_addr = 0x1250;
        dm_svc200_down_detector_info->camera_name = sros::device::DEVICE_SVC200_DOWN;
        EulerToQuaternion(0, M_PI, 0, dm_svc200_down_detector_info->rotate_matrix);//初始化，需要在initialize之前赋值
        initializeDmDetectorInfo(dm_svc200_down_detector_info,"");
        dm_svc200_down_detector_info->process_manager.reset(new msg_process::MsgProcessManager<sros::core::image_msg_ptr>(
            boost::bind(&VisionModule::processDetect, this, dm_svc200_down_detector_info, _1), 3));
        dm_svc200_down_detector_info->process_manager->creatMatchThread();
    }


    auto enable_behind_camera = (s.getValue<std::string>("camera.enable_azure_camera", "True") == "True");
    if (enable_behind_camera) {
        fm_behind_detector_info.reset(new DmDetectorInfo);
        fm_behind_detector_info->suffix = "azure_";
        fm_behind_detector_info->start_addr = 0x1260;
        fm_behind_detector_info->camera_name = sros::device::DEVICE_CAMERA_BACKWARD;
        initializeDmDetectorInfo(fm_behind_detector_info, "");
        fm_behind_detector_info->process_manager.reset(new msg_process::MsgProcessManager<sros::core::image_msg_ptr>(
            boost::bind(&VisionModule::processDetect, this, fm_behind_detector_info, _1), 3));
        fm_behind_detector_info->process_manager->creatMatchThread();
    }

    auto enable_d435_1_camera = (s.getValue<std::string>("camera.enable_d435_camera", "True") == "True");
    auto enable_d435_2_camera = (s.getValue<std::string>("camera.enable_d435_2_camera", "True") == "True");
    auto enable_d435_3_camera = (s.getValue<std::string>("camera.enable_d435_3_camera", "True") == "True");
    auto enable_d435_4_camera = (s.getValue<std::string>("camera.enable_d435_4_camera", "True") == "True");
    auto enable_d435_1_using_avoidance = (s.getValue<std::string>("camera.d435_1_using_avoidance", "True") == "True");
    auto enable_d435_2_using_avoidance = (s.getValue<std::string>("camera.d435_2_using_avoidance", "True") == "True");
    auto enable_d435_3_using_avoidance = (s.getValue<std::string>("camera.d435_3_using_avoidance", "True") == "True");
    auto enable_d435_4_using_avoidance = (s.getValue<std::string>("camera.d435_4_using_avoidance", "True") == "True");

    if(enable_d435_1_using_avoidance == enable_d435_2_using_avoidance){
        LOG(ERROR)<<"please check enable d435 using avoidance params";
    }

    if (enable_d435_1_camera && !enable_d435_1_using_avoidance){
        fm_d435_1_detector_info.reset(new DmDetectorInfo);
        fm_d435_1_detector_info->suffix = "d435_";
        fm_d435_1_detector_info->start_addr = 0x1270;
        fm_d435_1_detector_info->camera_name = sros::device::DEVICE_CAMERA_D435;

        LOG(INFO)<<fm_d435_1_detector_info->camera_name;
        initializeDmDetectorInfo(fm_d435_1_detector_info, "");
        LOG(INFO)<<"init d435_1";
        fm_d435_1_detector_info->process_manager.reset(new msg_process::MsgProcessManager<sros::core::image_msg_ptr>(
            boost::bind(&VisionModule::processDetect, this, fm_d435_1_detector_info, _1), 3));
        fm_d435_1_detector_info->process_manager->creatMatchThread();
    }

    if (enable_d435_2_camera && !enable_d435_2_using_avoidance){
        fm_d435_2_detector_info.reset(new DmDetectorInfo);
        fm_d435_2_detector_info->suffix = "d435_";
        fm_d435_2_detector_info->start_addr = 0x1280;
        fm_d435_2_detector_info->camera_name = sros::device::DEVICE_CAMERA_D435_2;

        LOG(INFO)<<fm_d435_2_detector_info->camera_name;
        initializeDmDetectorInfo(fm_d435_2_detector_info, "");
        LOG(INFO)<<"init d435_2";
        fm_d435_2_detector_info->process_manager.reset(new msg_process::MsgProcessManager<sros::core::image_msg_ptr>(
            boost::bind(&VisionModule::processDetect, this, fm_d435_2_detector_info, _1), 3));
        fm_d435_2_detector_info->process_manager->creatMatchThread();
    }

    auto enable_mvs_camera = (s.getValue<std::string>("camera.enable_mvs_camera", "True") == "True");
    if(enable_mvs_camera){
        fm_mvs_detector_info.reset(new DmDetectorInfo);
        fm_mvs_detector_info->suffix = "mvs_";
        fm_mvs_detector_info->start_addr = 0x1290;
        fm_mvs_detector_info->camera_name = sros::device::DEVICE_CAMERA_MV_CE013;

        LOG(INFO)<<fm_mvs_detector_info->camera_name;
        initializeDmDetectorInfo(fm_mvs_detector_info, "");
        LOG(INFO)<<"init mvs camera";
        fm_mvs_detector_info->process_manager.reset(new msg_process::MsgProcessManager<sros::core::image_msg_ptr>(
                boost::bind(&VisionModule::processDetect, this, fm_mvs_detector_info, _1), 3));
        fm_mvs_detector_info->process_manager->creatMatchThread();
    }

    if (enable_d435_3_camera && !enable_d435_3_using_avoidance){
        fm_d435_3_detector_info.reset(new DmDetectorInfo);
        fm_d435_3_detector_info->suffix = "d435_";
        fm_d435_3_detector_info->start_addr = 0x1280;
        fm_d435_3_detector_info->camera_name = sros::device::DEVICE_CAMERA_D435_3;

        LOG(INFO)<<fm_d435_3_detector_info->camera_name;
        initializeDmDetectorInfo(fm_d435_3_detector_info, "");
        LOG(INFO)<<"init d435_3";
        fm_d435_3_detector_info->process_manager.reset(new msg_process::MsgProcessManager<sros::core::image_msg_ptr>(
                boost::bind(&VisionModule::processDetect, this, fm_d435_3_detector_info, _1), 3));
        fm_d435_3_detector_info->process_manager->creatMatchThread();
    }

    if (enable_d435_4_camera && !enable_d435_4_using_avoidance){
        fm_d435_4_detector_info.reset(new DmDetectorInfo);
        fm_d435_4_detector_info->suffix = "d435_";
        fm_d435_4_detector_info->start_addr = 0x1280;
        fm_d435_4_detector_info->camera_name = sros::device::DEVICE_CAMERA_D435_4;

        LOG(INFO)<<fm_d435_4_detector_info->camera_name;
        initializeDmDetectorInfo(fm_d435_4_detector_info, "");
        LOG(INFO)<<"init d435_4";
        fm_d435_4_detector_info->process_manager.reset(new msg_process::MsgProcessManager<sros::core::image_msg_ptr>(
                boost::bind(&VisionModule::processDetect, this, fm_d435_4_detector_info, _1), 3));
        fm_d435_4_detector_info->process_manager->creatMatchThread();
    }
    auto enable_svc200_back_cam = (s.getValue<std::string>("camera.enable_back_camera", "True") == "True");
    if(enable_svc200_back_cam){
        fm_svc200_detector_info.reset(new DmDetectorInfo);
        fm_svc200_detector_info->suffix = "back_";
        fm_svc200_detector_info->start_addr = 0x1260;
        fm_svc200_detector_info->camera_name = sros::device::DEVICE_CAMERA_BACKWARD;
        LOG(INFO)<<fm_svc200_detector_info->camera_name;
        initializeDmDetectorInfo(fm_svc200_detector_info, "");

        LOG(INFO)<<"init svc200_back_camera";
        fm_svc200_detector_info->process_manager.reset(new msg_process::MsgProcessManager<sros::core::image_msg_ptr>(
            boost::bind(&VisionModule::processDetect, this, fm_svc200_detector_info, _1), 3));
        fm_svc200_detector_info->process_manager->creatMatchThread();
    }
    save_failure_img_ = std::deque<cv::Mat>(5, cv::Mat());

    int dml_version;
    std::string dml_commit;
    dml_commit = getVersionStr();
    dml_version = getVersion();
    LOG(INFO) << "dml version:" << dml_version << "," << dml_commit;
    LOG(INFO) << "fm version:" << FractalMarkerDetector::getVersionStr();
    //    subscribeTopic("TOPIC_DEPTH", CALLBACK(&VisionModule::onDepthImageMsg));
    subscribeTopic("TOPIC_COLOR", CALLBACK(&VisionModule::onColorImageMsg));
    subscribeTopic("TIMER_1S", CALLBACK(&VisionModule::onTimer_1s));
//    subscribeTopic("SAVE_IMG", CALLBACK(&VisionModule::onSaveImgCmd));
    subscribeTopic("SAVE_IMG", CALLBACK(&VisionModule::onSaveImgCmd));
    subscribeTopic("SAVE_FAILURE_IMG", CALLBACK(&VisionModule::onSaveFailureImgCmd));
    subscribeTopic("TOPIC_ROTARY_JACK_UP_END", CALLBACK(&VisionModule::onRotateJackUpEndMsg));
    subscribeTopic("TOPIC_ROTARY_JACK_DOWN_END", CALLBACK(&VisionModule::onRotateJackDownEndMsg));

    dispatch();
}

void VisionModule::onColorImageMsg(sros::core::base_msg_ptr m) {
    auto msg = std::dynamic_pointer_cast<sros::core::ImageMsg>(m);
    if (dm_up_detector_info && msg->getCameraName() == dm_up_detector_info->camera_name) {
        if (dm_up_detector_info->process_manager) {
            dm_up_detector_info->process_manager->pushToQueue(msg);
        }
    } else if (dm_down_detector_info && msg->getCameraName() == dm_down_detector_info->camera_name) {
        if (dm_down_detector_info->process_manager) {
            dm_down_detector_info->process_manager->pushToQueue(msg);
        }
    } else if(dm_svc200_up_detector_info && msg->getCameraName() == dm_svc200_up_detector_info->camera_name){
        if(dm_svc200_up_detector_info->process_manager){
            dm_svc200_up_detector_info->process_manager->pushToQueue(msg);
        }
    }else if(dm_svc200_down_detector_info && msg->getCameraName() == dm_svc200_down_detector_info->camera_name){
        if(dm_svc200_down_detector_info->process_manager){
            dm_svc200_down_detector_info->process_manager->pushToQueue(msg);
        }
    }else if (fm_behind_detector_info && msg->getCameraName() == fm_behind_detector_info->camera_name) {
        if (fm_behind_detector_info->process_manager) {
            fm_behind_detector_info->process_manager->pushToQueue(msg);
        }
    }else if (fm_d435_1_detector_info && msg->getCameraName() == fm_d435_1_detector_info->camera_name) {
        if (fm_d435_1_detector_info->process_manager) {
            fm_d435_1_detector_info->process_manager->pushToQueue(msg);
        }
    } else if (fm_d435_2_detector_info && msg->getCameraName() == fm_d435_2_detector_info->camera_name) {
        if (fm_d435_2_detector_info->process_manager) {
            fm_d435_2_detector_info->process_manager->pushToQueue(msg);
        }
    }else if(fm_svc200_detector_info && msg->getCameraName() == fm_svc200_detector_info->camera_name){
        if(fm_svc200_detector_info->process_manager){
            fm_svc200_detector_info->process_manager->pushToQueue(msg);
        }
    } else if(fm_mvs_detector_info && msg->getCameraName() == fm_mvs_detector_info->camera_name){
        if(fm_mvs_detector_info->process_manager){
            fm_mvs_detector_info->process_manager->pushToQueue(msg);
        }
    }else {
        LOG(INFO) << "err to find camera:" << msg->getCameraName();
    }
}

void VisionModule::onRotateJackDownEndMsg(sros::core::base_msg_ptr m){
    auto msg = std::dynamic_pointer_cast<sros::core::CommonMsg>(m);
    auto &dm_detector = dm_up_detector_info->dm_detector;
    if(dm_up_detector_info){
       if(!msg->flag){
           dm_detector.setRunCodeState(false);
       }
    }
}

void VisionModule::onRotateJackUpEndMsg(sros::core::base_msg_ptr m){
    auto msg = std::dynamic_pointer_cast<sros::core::CommonMsg>(m);
    auto &dm_detector = dm_up_detector_info->dm_detector;
    if(dm_up_detector_info){
       if(msg->flag){
           dm_detector.setRunCodeState(true);
       }
    }
}

void VisionModule::onTimer_1s(sros::core::base_msg_ptr m) {
    if (dm_up_detector_info) {
        auto &dm_detector_watch_dog = dm_up_detector_info->dm_detector_watch_dog_;
        auto &last_dm_detect_ok = dm_up_detector_info->last_dm_detect_ok_;
        if (dm_detector_watch_dog && !last_dm_detect_ok) {
            // 看门狗超时，设置dm_detector结果为无效
            // 仅当上次扫码结果ok时，才发送一次failed信息
            src_sdk->setUpCameraDetectInfo(0, 0, 0, 0, 0);
            auto device = dm_up_detector_info->device;
            if (device) {
                device->setInfo("");
            }
        }
        last_dm_detect_ok = false;
        // 设置看门狗标志位，如果200ms内没有被设置为false的话，下次执行超时操作
        dm_detector_watch_dog = true;
    }
    if (dm_down_detector_info) {
        auto &dm_detector_watch_dog = dm_down_detector_info->dm_detector_watch_dog_;
        auto &last_dm_detect_ok = dm_down_detector_info->last_dm_detect_ok_;
        if (dm_detector_watch_dog && !last_dm_detect_ok) {
            // 看门狗超时，设置dm_detector结果为无效
            // 仅当上次扫码结果ok时，才发送一次failed信息
            src_sdk->setDownCameraDetectInfo(0, 0, 0, 0, 0);
            auto device = dm_down_detector_info->device;
            if (device) {
                device->setInfo("");
            }
        }
        last_dm_detect_ok = false;
        // 设置看门狗标志位，如果200ms内没有被设置为false的话，下次执行超时操作
        dm_detector_watch_dog = true;
    }
    if (dm_svc200_up_detector_info) {
        auto &dm_detector_watch_dog = dm_svc200_up_detector_info->dm_detector_watch_dog_;
        auto &last_dm_detect_ok = dm_svc200_up_detector_info->last_dm_detect_ok_;
        if (dm_detector_watch_dog && !last_dm_detect_ok) {
            // 看门狗超时，设置dm_detector结果为无效
            // 仅当上次扫码结果ok时，才发送一次failed信息
            src_sdk->setUpCameraDetectInfo(0, 0, 0, 0, 0);
            auto device = dm_svc200_up_detector_info->device;
            if (device) {
                device->setInfo("");
            }
        }
        last_dm_detect_ok = false;
        // 设置看门狗标志位，如果200ms内没有被设置为false的话，下次执行超时操作
        dm_detector_watch_dog = true;
    }
    if (dm_svc200_down_detector_info) {
        auto &dm_detector_watch_dog = dm_svc200_down_detector_info->dm_detector_watch_dog_;
        auto &last_dm_detect_ok = dm_svc200_down_detector_info->last_dm_detect_ok_;
        if (dm_detector_watch_dog && !last_dm_detect_ok) {
            // 看门狗超时，设置dm_detector结果为无效
            // 仅当上次扫码结果ok时，才发送一次failed信息
            src_sdk->setDownCameraDetectInfo(0, 0, 0, 0, 0);
            auto device = dm_svc200_down_detector_info->device;
            if (device) {
                device->setInfo("");
            }
        }
        last_dm_detect_ok = false;
        // 设置看门狗标志位，如果200ms内没有被设置为false的话，下次执行超时操作
        dm_detector_watch_dog = true;
    }
    if (fm_behind_detector_info) {
        auto &fm_detector_watch_dog = fm_behind_detector_info->dm_detector_watch_dog_;
        auto &last_fm_detect_ok = fm_behind_detector_info->last_dm_detect_ok_;
        if (fm_detector_watch_dog && !last_fm_detect_ok) {
            // 看门狗超时，设置fm_detector结果为无效
            // 仅当上次扫码结果ok时，才发送一次failed信息
            auto device = fm_behind_detector_info->device;
            if (device) {
                device->setInfo("");
            }
        }
        last_fm_detect_ok = false;
        // 设置看门狗标志位，如果200ms内没有被设置为false的话，下次执行超时操作
        fm_detector_watch_dog = true;
    }
    if(fm_svc200_detector_info){
        auto &fm_detector_watch_dog = fm_svc200_detector_info->dm_detector_watch_dog_;
        auto &last_fm_detect_ok = fm_svc200_detector_info->last_dm_detect_ok_;
        if (fm_detector_watch_dog && !last_fm_detect_ok) {
            // 看门狗超时，设置fm_detector结果为无效
            // 仅当上次扫码结果ok时，才发送一次failed信息
            auto device = fm_svc200_detector_info->device;
            if (device) {
                device->setInfo("");
            }
        }
        last_fm_detect_ok = false;
        // 设置看门狗标志位，如果200ms内没有被设置为false的话，下次执行超时操作
        fm_detector_watch_dog = true;
    }
    if (fm_d435_1_detector_info) {
        auto &fm_detector_watch_dog = fm_d435_1_detector_info->dm_detector_watch_dog_;
        auto &last_fm_detect_ok = fm_d435_1_detector_info->last_dm_detect_ok_;
        if (fm_detector_watch_dog && !last_fm_detect_ok) {
            // 看门狗超时，设置fm_detector结果为无效
            // 仅当上次扫码结果ok时，才发送一次failed信息
            auto device = fm_d435_1_detector_info->device;
            if (device) {
                device->setInfo("");
            }
        }
        last_fm_detect_ok = false;
        // 设置看门狗标志位，如果200ms内没有被设置为false的话，下次执行超时操作
        fm_detector_watch_dog = true;
    }
    if (fm_d435_2_detector_info) {
        auto &fm_detector_watch_dog = fm_d435_2_detector_info->dm_detector_watch_dog_;
        auto &last_fm_detect_ok = fm_d435_2_detector_info->last_dm_detect_ok_;
        if (fm_detector_watch_dog && !last_fm_detect_ok) {
            // 看门狗超时，设置fm_detector结果为无效
            // 仅当上次扫码结果ok时，才发送一次failed信息
            auto device = fm_d435_2_detector_info->device;
            if (device) {
                device->setInfo("");
            }
        }
        last_fm_detect_ok = false;
        // 设置看门狗标志位，如果200ms内没有被设置为false的话，下次执行超时操作
        fm_detector_watch_dog = true;
    }

    if(fm_mvs_detector_info){
        auto &fm_detector_watch_dog = fm_mvs_detector_info->dm_detector_watch_dog_;
        auto &last_fm_detect_ok = fm_mvs_detector_info->last_dm_detect_ok_;
        if (fm_detector_watch_dog && !last_fm_detect_ok) {
            // 看门狗超时，设置fm_detector结果为无效
            // 仅当上次扫码结果ok时，才发送一次failed信息
            auto device = fm_mvs_detector_info->device;
            if (device) {
                device->setInfo("");
            }
        }
        last_fm_detect_ok = false;
        // 设置看门狗标志位，如果200ms内没有被设置为false的话，下次执行超时操作
        fm_detector_watch_dog = true;
    }
}

void VisionModule::processDetect(DmDetectorInfoPtr &dm_detector_info, sros::core::image_msg_ptr m) {
    auto &dm_detector = dm_detector_info->dm_detector;
    auto &dm_code_direction = dm_detector_info->dm_code_direction;
    Mat image = (m->getMat());
    if (image.empty()) {
        LOG(INFO) << "img empty!";
        return;
    }

    static int output_every_20_increment = 0;
    static bool enable_svc100_debug_output = false;
    static bool enable_output_debug_img = false;
    if (output_every_20_increment++ % 20 == 0) {
        enable_output_debug_img =
                (sros::core::Settings::getInstance().getValue<std::string>("vision.output_debug_img", "False") == "True");
        const auto online_calibrate_install_position =
                (sros::core::Settings::getInstance().getValue<std::string>("vision.online_calibrate_install_position",
                                                                            "False") == "True");
        if (online_calibrate_install_position) {
            updateCalibrationParaOnline(dm_detector_info);
        }
        enable_svc100_debug_output = (sros::core::Settings::getInstance().getValue<std::string>(
            "debug.enable_svc100_debug_output", "False") == "True");
    }
    if (enable_output_debug_img) {
        std::string saveImgName = "no_camera_find";
        if(dm_detector_info->suffix == ""){
            saveImgName = "CAMERA_UP_1";
        }
        else if(dm_detector_info->suffix == "down_"){
            saveImgName = "CAMERA_DOWN_1";
        }
        else{
            saveImgName = dm_detector_info->suffix;
        }

        LOG(INFO) << "save: " << saveImgName << ".png";
        std::string curr_name = record::RecordFileManager::creatImgName(record::RecordFileManager::getCurrSaveTime());
        cv::putText(image, curr_name, cvPoint(10, 10), FONT_HERSHEY_COMPLEX_SMALL, 0.6, cvScalar(128, 128, 128));

        dm_detector.savePngImg("/sros/debug_data/" + saveImgName + ".png", image);
        dm_detector.savePngImg("/tmp/" + saveImgName + ".png", image);
        sleep(1);
    }
    if(dm_detector_info->code_type == "FRACTAL_MARKER_3L" ||
        dm_detector_info->code_type == "FRACTAL_MARKER_2L" ||
        dm_detector_info->code_type == "FRACTAL_MARKER_1L"){
        processFmDetect(dm_detector_info, m, enable_svc100_debug_output);
    }else{
        processDmDetect(dm_detector_info, m, enable_svc100_debug_output);
    }

}  // namespace vision

void VisionModule::calibrateInstallPose(Eigen::Vector3d &forwad_pose, Eigen::Vector3d &back_pose,
                                        const double camera_initial_direction, Eigen::Vector3d &camera_2_robot_center,
                                        Eigen::Vector3d &code_2_rack_center) {
    if (forwad_pose[2] == back_pose[2]) {
        LOG(INFO) << "back pose set is wrong!";
        back_pose[2] = forwad_pose[2] + M_PI;
    }

    camera_2_robot_center[2] = camera_initial_direction;

    code_2_rack_center[2] = 0.0;
    normalizeAngle(code_2_rack_center[2]);

    auto cos_code_2_rack = cos(code_2_rack_center[2]);
    auto sin_code_2_rack = sin(code_2_rack_center[2]);
    auto cos_theta_sum_1 = cos(code_2_rack_center[2] + forwad_pose[2]);
    auto sin_theta_sum_1 = sin(code_2_rack_center[2] + forwad_pose[2]);

    auto cos_theta_sum_2 = cos(code_2_rack_center[2] + back_pose[2]);
    auto sin_theta_sum_2 = sin(code_2_rack_center[2] + back_pose[2]);

    Eigen::Vector2d k_1, k_2, k_5;
    k_1[0] = cos_code_2_rack * forwad_pose[0] - sin_code_2_rack * forwad_pose[1];
    k_1[1] = sin_code_2_rack * forwad_pose[0] + cos_code_2_rack * forwad_pose[1];

    k_2[0] = cos_code_2_rack * back_pose[0] - sin_code_2_rack * back_pose[1];
    k_2[1] = sin_code_2_rack * back_pose[0] + cos_code_2_rack * back_pose[1];

    double delta_k_1 = (cos_theta_sum_1 - cos_theta_sum_2);
    double delta_k_2 = (sin_theta_sum_1 - sin_theta_sum_2);

    double delta_k_3 = (k_2[0] - k_1[0]);
    double delta_k_4 = k_2[1] - k_1[1];

    camera_2_robot_center[0] =
            (delta_k_3 * delta_k_1 + delta_k_4 * delta_k_2) / (delta_k_1 * delta_k_1 + delta_k_2 * delta_k_2);
    camera_2_robot_center[1] =
            (delta_k_4 * delta_k_1 - delta_k_3 * delta_k_2) / (delta_k_1 * delta_k_1 + delta_k_2 * delta_k_2);

    k_5[0] = cos_theta_sum_1 * camera_2_robot_center[0] - sin_theta_sum_1 * camera_2_robot_center[1];
    k_5[1] = sin_theta_sum_1 * camera_2_robot_center[0] + cos_theta_sum_1 * camera_2_robot_center[1];

    code_2_rack_center[0] = -(k_5[0] + k_1[0]);
    code_2_rack_center[1] = -(k_5[1] + k_1[1]);

    LOG(INFO) << "camera_2_robot_center:" << camera_2_robot_center[0] << "," << camera_2_robot_center[1] << ","
                << camera_2_robot_center[2];
    LOG(INFO) << "code_2_rack_center:" << code_2_rack_center[0] << "," << code_2_rack_center[1] << ","
                << code_2_rack_center[2];
}

void VisionModule::convertToRobotCenterPose(const Eigen::Vector3d &camera_2_robot_center,
                                            const Eigen::Vector3d &code_2_rack_center, const double dm_code_direction,
                                            bool use_right_hand, DMResult &result) {
    Eigen::Affine2d code_2_rack_center_tf =
            Eigen::Translation2d(code_2_rack_center[0], code_2_rack_center[1]) * Eigen::Rotation2Dd(code_2_rack_center[2]);

    Eigen::Vector2d curr_pose(result.x_offset / 1000.0, result.y_offset / 1000.0);
    double angle_offset = result.angle_offset;

    Eigen::Affine2d camera_pose_tf = Eigen::Translation2d(curr_pose) * Eigen::Rotation2Dd(angle_offset);
    auto real_pose = code_2_rack_center_tf * camera_pose_tf * camera_2_robot_center.head<2>();

    angle_offset = angle_offset + camera_2_robot_center[2] + code_2_rack_center[2];
    normalizeAngle(angle_offset);
    Eigen::Rotation2Dd dm_code_rotate_tf(dm_code_direction);
    real_pose = dm_code_rotate_tf * real_pose;
    angle_offset += dm_code_direction;
    normalizeAngle(angle_offset);
    if (use_right_hand) {
        result.x_offset = -real_pose[1] * 1000.0;
        result.y_offset = real_pose[0] * 1000.0;  //以二维码正方向为Y的右手系
        result.angle_offset = angle_offset + M_PI_2;
    } else {
        result.x_offset = real_pose[0] * 1000.0;
        result.y_offset = -real_pose[1] * 1000.0;  //从右手系转换成左手系
        result.angle_offset = -angle_offset;
    }
    normalizeAngle(result.angle_offset);
}

std::vector<double> VisionModule::splitStrsToDoubles(const std::string &s, const char seperator) {
    vector<double> result;
    typedef string::size_type string_size;

    string_size i = 0;
    string_size j = 0;
    char c = s[0];
    if (c == '"')  // chip 设置的话一般带“”
        j = 1;
    // LOG(INFO) << "the s is:" << s;
    while (i < s.size()) {
        if (s[i] == seperator || i == s.size() - 1) {
            if (j != i || i == s.size() - 1) {
                auto len = (s[i] != seperator && i == s.size() - 1) ? (s.size() - j) : i - j;
                string item_s = s.substr(j, len);
                if (item_s == "\"") break;
                try {
                    double item = stof(item_s);
                    result.push_back(item);
                } catch (std::exception &e) {
                    LOG(INFO) << "throw error:" << e.what() << item_s;
                }
            }
            j = i + 1;
        }
        i++;
    }

    return result;
}

int VisionModule::getVersion() {
    if (dm_up_detector_info) {
        return dm_up_detector_info->dm_detector.getVersion();
    } else if (dm_down_detector_info) {
        return dm_down_detector_info->dm_detector.getVersion();
    }
    LOG(INFO) << "no vision processor running!";
    return 0;
}

std::string VisionModule::getVersionStr() {
    if (dm_up_detector_info) {
        return dm_up_detector_info->dm_detector.getVersionStr();
    } else if (dm_down_detector_info) {
        return dm_down_detector_info->dm_detector.getVersionStr();
    }
    LOG(INFO) << "no vision processor running!";
    return "";
}

void VisionModule::initializeDmDetectorInfo(DmDetectorInfoPtr &dm_detector_info,const std::string svc100_prefix) {
    auto &s = sros::core::Settings::getInstance();
    std::string camera_prefix = "camera.";
    std::string vision_prefix = "vision.";

    std::string install_direction_para = camera_prefix + svc100_prefix + dm_detector_info->suffix + "install_direction";
    dm_detector_info->camera_install_diretion = s.getValue<double>(install_direction_para, 180.0) / 180.0 * M_PI;

    Eigen::Quaterniond rotate_q;
    EulerToQuaternion(dm_detector_info->camera_install_diretion, 0, 0, rotate_q);
    auto real_q = rotate_q * dm_detector_info->rotate_matrix;
    dm_detector_info->rotate_matrix = real_q;//用来做里程计补偿，需要将里程计转换到相机坐标系。

    std::string inner_param_para = camera_prefix + svc100_prefix + dm_detector_info->suffix + "inner_param";
    std::string disort_param_para = camera_prefix + svc100_prefix + dm_detector_info->suffix + "disort_param";
    std::string vanish_point_para = camera_prefix + svc100_prefix + dm_detector_info->suffix + "vanish_point";
    auto camera_params =
            s.getValue<std::string>(inner_param_para, "304.687964;0;330.872456;0;304.687963;221.071245;0;0;1.0");
    auto disort_params =
            s.getValue<std::string>(disort_param_para, "0.0228901;-0.0637674;0.00244537;0.00234416;0.0197826");
    auto vanish_params = s.getValue<std::string>(vanish_point_para, "0;0");

    std::string code_type_para = vision_prefix + dm_detector_info->suffix + "code_type";
    std::string recog_mothod_para = vision_prefix + dm_detector_info->suffix + "code_recog_method";
    std::string dm_code_direction_para = vision_prefix + dm_detector_info->suffix + "dm_code_direction";
    std::string dm_code_right_handed_para = vision_prefix + dm_detector_info->suffix + "dm_code_right_handed_system";

    dm_detector_info->dm_code_direction = s.getValue<double>(dm_code_direction_para, 0) / 180.0 * M_PI;
    dm_detector_info->code_type = s.getValue<std::string>(code_type_para, "HUAWEI_SINGLE_LARGE_16");
    auto recog_method = s.getValue<std::string>(recog_mothod_para, "TIME_FIRST");

    dm_detector_info->is_right_handed = (s.getValue<std::string>(dm_code_right_handed_para, "True") == "True");

    std::string fm_marker_size = vision_prefix + dm_detector_info->suffix + "fm_marker_size";
    float fm_ms = s.getValue<float>(fm_marker_size, 0.139);

    //以上两个参数是不能通过别人配置的,需要由标定模块自动生成.
    auto camera_param = splitStrsToDoubles(camera_params, ';');  //相机参数
    auto disort_param = splitStrsToDoubles(disort_params, ';');  //畸变修正参数
    auto vanish_param = splitStrsToDoubles(vanish_params, ';');
    dm_detector_info->dmcode_runstate = (s.getValue<std::string>("inspection.enable_rotary_jack_keep_scan", "False") == "True");
    auto mincirring = s.getValue<int>("vision.dm_min_cirring",50);
    auto maxcirring = s.getValue<int>("vision.dm_max_cirring",110);
    LOG(INFO) << "camera para:" << camera_params << "," << camera_param.size();
    // for (auto &para : camera_param) {
    //     LOG(INFO) << "para:" << para;
    // }
    LOG(INFO) << "disort para:" << disort_params << "," << disort_param.size();
    // for (auto &para : disort_param) {
    //     LOG(INFO) << "para:" << para;
    // }

    auto &dm_detector = dm_detector_info->dm_detector;
    auto &fm_detector = dm_detector_info->fm_detector;
    dm_detector.setCameraParam(camera_param, disort_param);
    dm_detector.setVanishPoint(vanish_param);
    dm_detector.setCodeType(dm_detector_info->code_type);
    dm_detector.setRoughLocationMethod(recog_method);
    dm_detector.setDMCodeRunflag(dm_detector_info->dmcode_runstate);
    dm_detector.setcirringvale(mincirring,maxcirring);
    dm_detector.setRunCodeState(false);

    fm_detector.setCodeType(dm_detector_info->code_type);
    fm_detector.setCameraParam(camera_param, disort_param);

    fm_detector.setCodeParam(fm_ms);

    if (dm_detector_info->camera_name == sros::device::DEVICE_SVC100_UP || dm_detector_info->camera_name == sros::device::DEVICE_CAMERA_UP) {
        dm_detector.setCameraInstallType("UP");
        LOG(INFO) << "will set camera to up!";
    }else if(dm_detector_info->camera_name == sros::device::DEVICE_SVC100_DOWN || dm_detector_info->camera_name == sros::device::DEVICE_CAMERA_DOWN){
        LOG(INFO) << "will set camera to down!";
        dm_detector.setCameraInstallType("DOWN");
    }
    int camera_size = camera_param.size();
    if (camera_size != 9) {
        LOG(INFO) << "camera param size is err! cannot calibrate the img! real param size is 9!";
    }
    int disort_para_size = disort_param.size();
    if (disort_para_size != 5) {
        LOG(INFO) << "disort param size is err! cannot calibrate the img! real param size is 5";
    }

    Eigen::Vector3d forward_direction = Eigen::Vector3d::Zero();
    Eigen::Vector3d back_direction = Eigen::Vector3d::Zero();

    calibrateInstallPose(forward_direction, back_direction, dm_detector_info->camera_install_diretion,
                            dm_detector_info->camera_2_robot_center, dm_detector_info->code_2_rack_center);

    LOG(INFO) << "all para name:" << install_direction_para << "," << inner_param_para << "," << disort_param_para
                << "," << vanish_point_para << "," << code_type_para << "," << recog_mothod_para << ","
                << dm_code_direction_para;
}

void VisionModule::updateCalibrationParaOnline(DmDetectorInfoPtr &dm_detector_info) {
    auto &s = sros::core::Settings::getInstance();
    Eigen::Vector3d forward_direction;
    Eigen::Vector3d back_direction;
    forward_direction[0] = s.getValue<double>("vision.lift_rack_forward_pose_x", 0.0) / 10000.0;
    forward_direction[1] = -s.getValue<double>("vision.lift_rack_forward_pose_y", 0.0) / 10000.0;
    forward_direction[2] = -s.getValue<double>("vision.lift_rack_forward_pose_yaw", 0.0) / 180.0 * M_PI;

    back_direction[0] = s.getValue<double>("vision.lift_rack_backward_pose_x", 0.0) / 10000.0;
    back_direction[1] = -s.getValue<double>("vision.lift_rack_backward_pose_y", 0.0) / 10000.0;
    back_direction[2] = -s.getValue<double>("vision.lift_rack_backward_pose_yaw", 0.0) / 180.0 * M_PI;
    std::string install_direction_para = "camera.svc100_" + dm_detector_info->suffix + "install_direction";
    dm_detector_info->camera_install_diretion =
            s.getValue<double>(install_direction_para, 180.0) / 180.0 * M_PI;
    calibrateInstallPose(forward_direction, back_direction, dm_detector_info->camera_install_diretion,
                            dm_detector_info->camera_2_robot_center, dm_detector_info->code_2_rack_center);

    std::string vision_prefix = "vision.";
    std::string code_type_para = vision_prefix + dm_detector_info->suffix + "code_type";
    std::string recog_mothod_para = vision_prefix + dm_detector_info->suffix + "code_recog_method";
    std::string dm_code_direction_para = vision_prefix + dm_detector_info->suffix + "dm_code_direction";
    dm_detector_info->dm_code_direction = s.getValue<double>(dm_code_direction_para, 0) / 180.0 * M_PI;
    dm_detector_info->code_type = s.getValue<std::string>(code_type_para, "HUAWEI_SINGLE_LARGE_16");
    dm_detector_info->fm_detector.setCodeType(dm_detector_info->code_type);
    auto recog_method = s.getValue<std::string>(recog_mothod_para, "TIME_FIRST");
    dm_detector_info->dm_detector.setCodeType(dm_detector_info->code_type);
    dm_detector_info->dm_detector.setRoughLocationMethod(recog_method);
}

void VisionModule::processDmDetect(DmDetectorInfoPtr &dm_detector_info, sros::core::image_msg_ptr m,bool enable_svc100_debug_output) {
    auto &dm_detector = dm_detector_info->dm_detector;
    auto &dm_code_direction = dm_detector_info->dm_code_direction;

    Mat image = (m->getMat());
    if (image.empty()) {
        LOG(INFO) << "img empty!";
        return;
    }
    
    auto set_time = sros::core::util::get_time_in_us();
    auto result = dm_detector.detect(image);
    sros::core::DataMatrixCodeMsg_ptr dm_code_info(new sros::core::DataMatrixCodeMsg("DM_CODE_INFO"));
    dm_code_info->setCameraName(m->getCameraName());
    dm_code_info->setTimestamp(m->getTimestamp());
    switch (result.detect_state) {
        case DmDetectState::TYPE_DM_NONE: {
        } break;
        case DmDetectState::TYPE_DM_RECOGNIZED: {
            try {
                //目前从相机获取到的二维码位姿是右手系，mm为单位，在外部使用时，考虑到外部使用的坐标系不同需要做各种适配。
                //基于相机安装位置定义的不同，上视相机的旋转矩阵的欧拉角全部是0°，下视相机，因为z轴方向相反，可认为是相机沿着x轴旋转了180°得到。
                //使用里程计补偿的算法思想是：
                //delta_pose(last_to_curr_incamera) =（T(odo_last)*T(camera_to_center)）^-1*(T(odo_curr)*T(camera_to_center))
                //                                  = T(camera_to_center)^-1(*T(odo_last)^-1*T(odo_curr))*T(camera_to_center)
                if(dm_detector_info->dmcode_runstate && !result.code_str.empty()){
                    lastsult = result;
                }
                if(dm_detector_info->dmcode_runstate && !lastsult.code_str.empty()){
                    result = lastsult;
                }
                int64_t curr_time = sros::core::util::get_time_in_us();
                auto& img_time = m->time_;
                slam::tf::TransForm curr_tf,img_tf;
                Eigen::Vector3d delta_point_in_camera = Eigen::Vector3d::Zero();
                Eigen::Quaterniond delta_q_in_camera,delta_q;
                EulerToQuaternion(0, 0, 0, delta_q_in_camera);
                if(tf_base_to_odo->lookUpTransForm(curr_time,curr_tf,60000)&&tf_base_to_odo->lookUpTransForm(img_time,img_tf,60000)){
                    Eigen::Affine2d img_affine(Eigen::Translation2d(img_tf.position.x(), img_tf.position.y()) *
                                                Eigen::Rotation2Dd(img_tf.rotation.yaw()));
                    Eigen::Vector2d delta_point,curr_point;
                    curr_point = Eigen::Vector2d(curr_tf.position.x(), curr_tf.position.y());
                    delta_point = img_affine.inverse() * curr_point;
                    Eigen::Vector3d delta_point_3d(delta_point[0], delta_point[1], 0);
                    delta_point_in_camera = dm_detector_info->rotate_matrix.inverse() * delta_point_3d;
                    double delta_yaw = curr_tf.rotation.yaw() - img_tf.rotation.yaw();
                    EulerToQuaternion(delta_yaw, 0, 0, delta_q);
                    delta_q_in_camera = dm_detector_info->rotate_matrix.inverse() * delta_q * dm_detector_info->rotate_matrix;
                }

                Eigen::Vector3d code_in_camera_point(result.x_offset / 1000.0, result.y_offset / 1000.0, 0);//mm转化成m
                Eigen::Quaterniond code_in_camera_q;
                EulerToQuaternion(result.angle_offset, 0, 0, code_in_camera_q);
                Eigen::Vector3d point_in_realtime_camera = delta_q_in_camera.inverse()*(code_in_camera_point - delta_point_in_camera);
                Eigen::Quaterniond q_in_realtime_camera = delta_q_in_camera.inverse() * code_in_camera_q;
                double code_in_realtime_yaw,roll,pitch;
                QuaternionToEuler(q_in_realtime_camera, code_in_realtime_yaw, roll, pitch);
                auto delta_yaw = code_in_realtime_yaw - result.angle_offset;
                normalizeAngle(delta_yaw);
                if (fabs(delta_yaw >= 0.17)) {//说明转换错误。
                    LOG(INFO) << "delta yaw is wrong!" << delta_yaw << ",origin angle:" << result.angle_offset
                                << ",curr angle:" << code_in_realtime_yaw << "," << roll << "," << pitch;
                    code_in_realtime_yaw = result.angle_offset;
                }
                DMResult send_to_src_result = result;//只有发给下位机的数据才需要同步，sros自身的数据不需要里程计同步，因为有时间戳存在。
                send_to_src_result.x_offset = point_in_realtime_camera[0] * 1000.0;
                send_to_src_result.y_offset = point_in_realtime_camera[1] * 1000.0;
                send_to_src_result.angle_offset = code_in_realtime_yaw;

                convertToRobotCenterPose(dm_detector_info->camera_2_robot_center, dm_detector_info->code_2_rack_center,
                                            dm_code_direction, dm_detector_info->is_right_handed, result);

                convertToRobotCenterPose(dm_detector_info->camera_2_robot_center, dm_detector_info->code_2_rack_center,
                                            dm_code_direction, dm_detector_info->is_right_handed, send_to_src_result);//发给下位机的数据也进行一次转换
                auto x = static_cast<int>(result.x_offset * 10 + 0.5);      // 下发到下位机的数据，单位0.1mm
                auto y = static_cast<int>(result.y_offset * 10 + 0.5);      // 下发到下位机的数据，单位0.1mm
                auto angle = static_cast<int>(result.angle_offset * 1000);  // 下发到下位机的数据，单位0.001rad

                auto send_to_src_x = static_cast<int>(send_to_src_result.x_offset * 10 + 0.5);      // 下发到下位机的数据，单位0.1mm
                auto send_to_src_y = static_cast<int>(send_to_src_result.y_offset * 10 + 0.5);      // 下发到下位机的数据，单位0.1mm
                auto send_to_src_angle = static_cast<int>(send_to_src_result.angle_offset * 1000);  // 下发到下位机的数据，单位0.001rad

                int64_t code_int_64 = 0;
                for (auto &char_s : result.code_str) {
                    if (char_s >= '0' && char_s <= '9') {
                        code_int_64 = code_int_64 * 10 + char_s - 48;
                    }
                }
                int64_t rest_int = 1e9;
                int code_int = code_int_64 % rest_int;
                //                LOG(INFO) << "code int:" << code_int_64 << "," << code_int;
                auto coord = std::string("(") + to_string(x) + ", " + to_string(y) + ", " +
                                to_string(double((int)(((angle * 180.0 / M_PI) / 100 + 0.5))) / 10.0) + ")";

                auto match_time = sros::core::util::get_time_in_us();

                if (dm_detector_info == dm_up_detector_info) {
                    dm_code_info->setCodeInfo(DM_CODE_DETECTED, x, y, angle, code_int);
                    dm_code_info->setCodeStr(result.code_str);
                    src_sdk->setUpCameraDetectInfo(1, send_to_src_x, send_to_src_y, send_to_src_angle * 10, code_int);
                } else if (dm_detector_info == dm_down_detector_info || dm_detector_info == dm_svc200_down_detector_info) {
                    dm_code_info->setCodeInfo(DM_CODE_DETECTED, x, y, angle, code_int);
                    dm_code_info->setCodeStr(result.code_str);
                    //                    LOG(INFO) << "1(" << x << ", " << y << ", " << angle * 10 << ") " << code_int;
                   src_sdk->setDownCameraDetectInfo(1, send_to_src_x, send_to_src_y, send_to_src_angle * 10,
                                                    code_int);  // 发给src的单位是万分之一米万分之一弧度
                }
                auto send_time = sros::core::util::get_time_in_us();
                auto device = dm_detector_info->device;
                if (!device) {
                    device = sros::device::DeviceManager::getInstance()->getDeviceByName(dm_detector_info->camera_name);
                    dm_detector_info->device = device;
                }
                if (device) {
                    std::string info = result.code_str + coord;
                    device->setInfo(info);
                    //                    device->setModelNo(coord);
                }
                auto device_time = sros::core::util::get_time_in_us();

                double inv_step = 1e-6;
                if ((device_time - set_time) * inv_step > 0.5) {
                    LOG(ERROR) << "delta dml match time is large:" << (device_time - set_time) * inv_step << ","
                                << (send_time - set_time) * inv_step << "," << (match_time - set_time) * inv_step << ","
                                << (set_time - set_time) * inv_step;
                }

//                if (enable_svc100_debug_output) {
//                    LOG(INFO) << "code info:" << result.code_str << "," << x << "," << y << "," << angle;
//                }
                //                LOG(INFO) << "end detect!" << x << "," << y << "," << angle <<"," <<  result.code_str;
                // 扫码成功，清除看门狗标志

                dm_detector_info->dm_detector_watch_dog_ = false;
                dm_detector_info->last_dm_detect_ok_ = true;
            } catch (std::invalid_argument &e) {
                LOG(INFO) << "the err is:" << e.what();
                LOG(INFO) << "the str is:" << result.code_str;
            }
        } break;
        default:
            break;
    }
    if (dm_code_info) {
        LOG(INFO) << "code info:" << result.code_str << "," << result.x_offset << "," << result.y_offset << "," << result.angle_offset << "," << result.detect_state;
//        if (enable_svc100_debug_output) {
//            LOG(INFO) << "code info:" << result.code_str << "," << result.x_offset << "," << result.y_offset << "," << result.angle_offset << "," << result.detect_state;
//        }
       sendMsg(dm_code_info);
       if (dm_code_info->code_str_.empty()) {
        //    LOG(INFO) << "cannot detect!";
           dm_detector_info->images.push_back(image);
           if (dm_detector_info->images.size() > 10) {
               dm_detector_info->images.pop_front();
           }
       }
    }
}

void VisionModule::processFmDetect(DmDetectorInfoPtr &detector_info, sros::core::image_msg_ptr m,bool enable_svc100_debug_output) {
    auto &fm_detector = detector_info->fm_detector;
    auto &dm_code_direction = detector_info->dm_code_direction;
    Mat image = (m->getMat());
    if (image.empty()) {
        LOG(INFO) << "img empty!";
        return;
    }
    // TODO 需要修改吗?
    sros::core::DataMatrixCodeMsg_ptr fm_code_info(new sros::core::DataMatrixCodeMsg("DM_CODE_INFO"));
    fm_code_info->setCameraName(m->getCameraName());
    fm_code_info->setTimestamp(m->getTimestamp());
    auto set_time = sros::core::util::get_time_in_us();
//    LOG(INFO) << "DM START!";
    auto result = fm_detector.detect(image);
    if (enable_svc100_debug_output) {
        auto end_time = sros::core::util::get_time_in_us();
        double delta_time = (end_time - set_time) / 1.0e6;
        LOG(INFO) << "dml time is:" << delta_time << ",freq is:" << 1 / delta_time;
        if (result.detect_state == DmDetectState::TYPE_DM_NONE) {
            LOG(INFO) << "cannot detect!";
        }
    }
    switch (result.detect_state) {
        case FmDetectState::TYPE_FM_NONE: {
            save_failure_img_.push_back(image);
            save_failure_img_.pop_front();
        } break;
        case FmDetectState::TYPE_FM_RECOGNIZED: {
            try {
                auto x = static_cast<int>(result.x_offset * 10 + 0.5);            // 单位0.1mm
                auto y = static_cast<int>(result.y_offset * 10 + 0.5);            // 单位0.1mm
                auto z = static_cast<int>(result.z_offset * 10 + 0.5);            // 单位0.1mm
                auto angle = static_cast<int>(result.angle_yaw_offset * 1000);    // 单位0.001rad
                auto roll = static_cast<int>(result.angle_roll_offset * 1000);    // 单位0.001rad
                auto pitch = static_cast<int>(result.angle_pitch_offset * 1000);  // 单位0.001rad

                // 从二维码code str 中提取除的数字信息,单fm并没有该选项
                int64_t code_int_64 = 0;
                for (auto &char_s : result.code_str) {
                    if (char_s >= '0' && char_s <= '9') {
                        code_int_64 = code_int_64 * 10 + char_s - 48;
                    }
                }
                int64_t rest_int = 1e9;
                int code_int = code_int_64 % rest_int;
                // LOG(INFO) << "code int:" << code_int_64 << "," << code_int;

                auto coord = std::string("(") + to_string(x) + ", " + to_string(y) + ", " +
                             to_string(double((int)(((angle * 180.0 / M_PI) / 100 + 0.5))) / 10.0) + ")";

                auto match_time = sros::core::util::get_time_in_us();

                if (detector_info == fm_behind_detector_info ||
                    detector_info == fm_d435_1_detector_info ||
                    detector_info == fm_d435_2_detector_info ||
                    detector_info == fm_svc200_detector_info ||
                    detector_info == fm_mvs_detector_info) {
                    fm_code_info->setCodeInfo(DM_CODE_DETECTED, x, y, z, angle, roll, pitch, code_int);
                    fm_code_info->setCodeStr(result.code_str);
                    fm_code_info->setCodeType(TYPE_FM_CODE);
                    // src_sdk->setParameters(0x1280, {1, x, y, z, angle, roll, pitch, code_int});
                }
                auto device = detector_info->device;
                if (!device) {
                    device =
                        sros::device::DeviceManager::getInstance()->getDeviceByName(detector_info->camera_name);
                    detector_info->device = device;
                }
                std::string info = result.code_str + coord;
                if (device) {
                    device->setInfo(info);
                }
                // device->setModelNo(coord);
                if (enable_svc100_debug_output) {
                    LOG(INFO) << "Fm code info:[code_str, (0.1mm)x, y, z, ()angle, roll, pitch]:" << result.code_str << "," << x << "," 
                        << y << "," << z << ","<< roll << ","<< pitch << ","<< angle;
                }
                // 扫码成功，清除看门狗标志
                detector_info->dm_detector_watch_dog_ = false;
                detector_info->last_dm_detect_ok_ = true;
            } catch (std::invalid_argument &e) {
                LOG(INFO) << "the err is:" << e.what();
                LOG(INFO) << "the str is:" << result.code_str;
            }
        } break;
        default:
            break;
    }
    if (fm_code_info) {
        sendMsg(fm_code_info);
    }
}

void VisionModule::onSaveImgCmd(sros::core::base_msg_ptr m) {
    int64_t curr_time = sros::core::util::get_time_in_us();
    static int64_t last_save_time = 0;
    if(abs(curr_time-last_save_time)<60e6){//1分钟存一次
        LOG(INFO) << "have not reach save cmd!";
        return;
    }
    last_save_time = curr_time;
    auto msg_cmd = std::dynamic_pointer_cast<sros::core::CommonCommandMsg<std::string>>(m);
    LOG(INFO)<<"save msg info:"<<msg_cmd->command<<","<<msg_cmd->str0<<",seq:"<<msg_cmd->seq;
    std::string path = "/sros/debug_data/";
    std::string png_type = ".png";
    record::RecordFileManager::manageRecordFile(path, png_type, 3);
    if (msg_cmd->str0 == sros::device::DEVICE_SVC100_UP && dm_up_detector_info) {
        auto& dm_detector = dm_up_detector_info->dm_detector;
        std::deque<cv::Mat> imgs = dm_up_detector_info->images;
        if (imgs.empty()) {
            LOG(INFO) << "cannot save img!";
            return;
        }
        static int count = 0;
        for (auto img : imgs) {
            std::string curr_name =
                    record::RecordFileManager::creatImgName(record::RecordFileManager::getCurrSaveTime());
            curr_name += "_seq_" + std::to_string(msg_cmd->seq) + "_" + std::to_string(count++);
            LOG(INFO) << "save: " << curr_name << ".png";
            std::string curr_day_time;
            std::stringstream sstream;
            sstream << curr_name;
            std::getline(sstream, curr_day_time, '.');
            cv::putText(img, curr_day_time, cvPoint(10, 10), FONT_HERSHEY_COMPLEX_SMALL, 0.6, cvScalar(128, 128, 128));
            dm_detector.savePngImg(path + curr_name + png_type, img);
        }
    } else {
        LOG(INFO) << "director is not exist!" << dm_up_detector_info;
    }
}

void VisionModule::onSaveFailureImgCmd(sros::core::base_msg_ptr m){
    auto msg = std::dynamic_pointer_cast<sros::core::CommonMsg>(m);
    LOG(INFO)<<"Enter onSaveFailureImgCmd";
    //todo conditions
    LOG(INFO)<<"msg->flag"<<msg->flag;
    if(fm_mvs_detector_info && msg->flag){
        // save failure imgs
        for (auto i = 0; i < save_failure_img_.size(); i++) {
            if(!save_failure_img_[i].empty()){
                LOG(INFO)<<"save failure image "<< i;
                std::string curr_name = record::RecordFileManager::creatImgName(record::RecordFileManager::getCurrSaveTime());
                cv::putText(save_failure_img_[i], curr_name, cvPoint(10, 10), FONT_HERSHEY_COMPLEX_SMALL, 0.6, cvScalar(128, 128, 128));
                fm_mvs_detector_info->fm_detector.savePngImg("/sros/debug_data/Fm_mvs_"+std::to_string(i)+".jpg", save_failure_img_[i]);   
            }
        }
    }

    save_failure_img_ = std::deque<cv::Mat>(5, cv::Mat());
}

}  // namespace vision
