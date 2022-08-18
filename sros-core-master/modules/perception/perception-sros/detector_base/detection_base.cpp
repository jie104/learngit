/**
 * @file detector_module_base.cpp
 * @brief Base class for all static object detection
 *
 * Base class for all static object detection,Subclasses need to override virtual
 * functions after inheriting this base class,
 *
 * @author zhangxu@standard-robots.com
 * @date create date：2021/1/5
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

// INCLUDE
#include "detection_base.h"
#include "../../common/common_func.hpp"
#include "core/logger.h"
#include "core/msg/common_msg.hpp"

#include <iomanip>
#include <utility>

using PerceptionCommandMsg = sros::core::PerceptionCommandMsg;
using PerceptionStateMsg = sros::core::PerceptionStateMsg;
using Command = sros::core::PerceptionCommandMsg::Command;
using DetectStage = sros::core::PerceptionCommandMsg::DetectStage;
using ObjectType = sros::core::PerceptionCommandMsg::ObjectType;

// CODE
namespace object_detector {
DetectorModuleBase::DetectorModuleBase(const std::string &module_name, const MsgCallback &sendMsg,
                                       const TopicCallback &subscribeTopic)
    : out_put_(nullptr)
    , in_put_(nullptr)
    , is_record_(false)
    , is_init_detector_(false)
    , enable_process_(false)
    , is_receive_cmd_(false)
    , sendMsg_(sendMsg)
    , subscribeTopic_(subscribeTopic)
    , sensor_data_file_path_(""){
    slam::tf::FrameToFrame base_to_world_frame;
    base_to_world_frame.parent_frame = "world";
    base_to_world_frame.child_frame = "base_link";
    tf_base_to_world_.reset(new slam::tf::TFOperator(base_to_world_frame));

}

void
DetectorModuleBase::checkDetectTime(const sros::core::base_msg_ptr &msg) {
    if (is_receive_cmd_) {
        auto cur_time = sros::core::util::get_time_in_ms();
        if (cur_time > DETECTED_TIME_OUT + receive_cmd_time_) {
            auto result_msg = std::make_shared<PerceptionStateMsg>("DETECT_RESULT");
            result_msg->command = this->command_;
            result_msg->goal_in_global_pose.x() = .0f;
            result_msg->goal_in_global_pose.y() = .0f;
            result_msg->goal_in_global_pose.yaw() = .0f;
            result_msg->detect_result = PerceptionStateMsg::DETECT_RESULT_FAIL;
            result_msg->error_code = PerceptionStateMsg::ERROR_CODE_TIMEOUT;
            LOGGER(INFO, SROS) << "detect time out! " << cur_time - receive_cmd_time_ << ">" << DETECTED_TIME_OUT;
            is_receive_cmd_ = false;
            stopDetector();
            sendMsg_(result_msg);
        }
    }
}

void
DetectorModuleBase::enableSensor() {
    LOG(INFO)  << "send TOPIC_SVC100_ENABLE_PUBLISH msg";
    std::shared_ptr<sros::core::CommonMsg> cmd_msg(new sros::core::CommonMsg("TOPIC_SVC100_ENABLE_PUBLISH"));
    cmd_msg->str_0_ = sros::device::DEVICE_CAMERA_O3D303;
    cmd_msg->flag = true;
    sendMsg_(cmd_msg);
}

void
DetectorModuleBase::disableSensor() {
    LOG(INFO)  << "send TOPIC_SVC100_ENABLE_PUBLISH msg";
    std::shared_ptr<sros::core::CommonMsg> cmd_msg(new sros::core::CommonMsg("TOPIC_SVC100_ENABLE_PUBLISH"));
    cmd_msg->str_0_ = sros::device::DEVICE_CAMERA_O3D303;
    cmd_msg->flag = false;
    sendMsg_(cmd_msg);
}

void
DetectorModuleBase::startDetector(){
    detected_poses_.clear();
    detected_xyzyaw_.clear();
    enable_process_ = true;
    detected_count_ = 0;
    enableSensor();
}

void
DetectorModuleBase::stopDetector(){
    detected_poses_.clear();
    detected_xyzyaw_.clear();
    enable_process_ = false;
    detected_count_ = 0;
    disableSensor();
}

Eigen::Vector3f
DetectorModuleBase::computeTargetMeanPose(const std::vector<Eigen::Vector3f> &pallet_poses) {
    if (pallet_poses.empty()) {
        LOG(INFO) << "circle pose is wrong!";
        return Eigen::Vector3f{0, 0, 0};
    }
    std::vector<float> xs, ys;
    std::vector<std::pair<float, float>> cos_yaws;
    for (const auto &pose : pallet_poses) {
        xs.push_back(pose[0]);
        ys.push_back(pose[1]);
        cos_yaws.emplace_back(std::cos(pose[2]), pose[2]);
    }
    std::nth_element(xs.begin(), xs.begin() + xs.size() / 2, xs.end());
    std::nth_element(ys.begin(), ys.begin() + ys.size() / 2, ys.end());
    std::nth_element(cos_yaws.begin(), cos_yaws.begin() + cos_yaws.size() / 2, cos_yaws.end());
    return Eigen::Vector3f{xs[xs.size() / 2.0f], ys[ys.size() / 2.0f], cos_yaws[cos_yaws.size() / 2.0f].second};
}

Eigen::Vector4f
DetectorModuleBase::computeTargetMeanPose(const std::vector<Eigen::Vector4f> &pallet_poses) {
    if (pallet_poses.empty()) {
        LOGGER(INFO, ACTION_TASK)  << "#Decards Error: pallet poses is wrong!";
        return Eigen::Vector4f{0, 0, 0, 0};
    }
    std::vector<float> xs, ys, zs;
    std::vector<std::pair<float, float>> cos_yaws;
    for (const auto &pose : pallet_poses) {
        xs.push_back(pose[0]);
        ys.push_back(pose[1]);
        zs.push_back(pose[2]);
        cos_yaws.emplace_back(std::cos(pose[3]), pose[3]);
        LOGGER(INFO, ACTION_TASK)  << "#Decards Info: z mean1-4: " << pose[2];
    }
    std::nth_element(xs.begin(), xs.begin() + xs.size() / 2, xs.end());
    std::nth_element(ys.begin(), ys.begin() + ys.size() / 2, ys.end());
    std::nth_element(zs.begin(), zs.begin() + zs.size() / 2, zs.end());
    std::nth_element(cos_yaws.begin(), cos_yaws.begin() + cos_yaws.size() / 2, cos_yaws.end());
    LOGGER(INFO, ACTION_TASK)  << "#Decards Info: z mean final: " << zs[zs.size() / 2.0f];
    return Eigen::Vector4f{xs[xs.size() / 2.0f], ys[ys.size() / 2.0f], zs[zs.size() / 2.0f],  cos_yaws[cos_yaws.size() / 2.0f].second};
}

void
DetectorModuleBase::showResult(const DetectResult &result, const int64_t &time) {
    LOG(INFO) << "id=" << result.id
                 << " goal: " << std::setfill(' ') << std::setiosflags(std::ios::right) << std::setw(5)
                 << static_cast<int>(result.x * 1000) << " " << std::setfill(' ') << std::setiosflags(std::ios::right)
                 << std::setw(4) << static_cast<int> (result.y * 1000) << " " << std::setfill(' ')
                 << std::setiosflags(std::ios::right) << std::setw(5) << static_cast<int> (result.z * 1000)
                 << " " << std::setfill(' ') << std::setiosflags(std::ios::right) << std::setw(7)
                 << std::setprecision(2) << std::setiosflags(std::ios::fixed) << std::fmod(result.angle/M_PI*180,180)
                 << " width: " << static_cast<int> (result.width * 1000)
                 << " height: " << static_cast<int> (result.height * 1000)
                 << " time:" << time << "ms";
    LOGGER(INFO, ACTION_TASK) << "id," << result.id << " goal, " << static_cast<int>(result.x * 1000)
                              << ", " << static_cast<int>(result.y * 1000)
                              << ", " << static_cast<int>(result.z * 1000)
                              << ", " << std::fmod(result.angle/M_PI*180, 180) << ", " << time;
    LOG(INFO) << "--------------------------------------------------------------------------------------------";
}

void
DetectorModuleBase::sendResultMsg(const PerceptionStateMsg::Ptr &result_msg) {
    if (result_msg->command.detect_stage == DetectStage::DETECT_STAGE_LOAD &&
        result_msg->detect_result == PerceptionStateMsg::DetectResult::DETECT_RESULT_SUCCESS) {
        LOGGER(INFO, ACTION_TASK) << "sendResultMsg cmd:"
                                  << result_msg->command.detect_stage * 10 + result_msg->command.object_type
                                  << " result:" << result_msg->detect_result
                                  << " Pose(x:" << result_msg->goal_in_global_pose.x()
                                  << ",y:" << result_msg->goal_in_global_pose.y()
                                  << ",yaw:" << result_msg->goal_in_global_pose.yaw() << ")";
    } else {
        LOGGER(INFO, ACTION_TASK) << "sendResultMsg cmd:"
                                  << result_msg->command.detect_stage * 10 + result_msg->command.object_type
                                  << " result:" << result_msg->detect_result;
    }
    this->is_receive_cmd_ = false;
    this->enable_process_ = false;

    sendMsg_(result_msg);
}


void
DetectorModuleBase::recordSensorData(const std::string &dir,
                                     const O3d3xxFrame &frame) {
    if (!out_put_.is_open()) {
        std::string time_str = common_func::getDateTimeStr();
        //std::string logfile_name = "sensor_data_" + time_str + ".log";
        std::string logfile_name = "sensor_data_ifm_camera.log";
        this->sensor_data_file_path_ = dir + logfile_name;

        out_put_.open(this->sensor_data_file_path_);
        if (!out_put_.is_open()) {
            LOG(INFO) << "record frame fail!" << this->sensor_data_file_path_;
            return;
        } else {
            LOG(INFO) << "create sensor data file: " << this->sensor_data_file_path_;
        }
    }

    const size_t img_height = frame.xyz_img.rows;
    const size_t img_width = frame.xyz_img.cols;

    if (0 == img_width || 0 == img_height){
        LOG(INFO) << "0 == img_width || 0 == img_height";
        return;
    }

    auto xyz_ptr = (short *) (frame.xyz_img.data);
    auto ampl_ptr = (ushort *) (frame.amplitude_img.data);
    auto conf_ptr = (uchar *) (frame.confidence_img.data);
    short x, y, z;
    float dist;
    ushort amp_pixel;
    uchar conf_pixel;
    std::stringstream temp_str;
    size_t effective_number = 0;
    for (size_t r = 0; r < img_height; ++r) {
        for (size_t c = 0; c < img_width; ++c) {
            const size_t idx = r * img_width + c;
            x = static_cast<short>(xyz_ptr[idx * 3 + 0]);
            y = static_cast<short>(xyz_ptr[idx * 3 + 1]);
            z = static_cast<short>(xyz_ptr[idx * 3 + 2]);
            dist = sqrt(x*x + y*y + z*z);
            if (dist < std::numeric_limits<float>::epsilon()){
                continue;
            }
            amp_pixel = ampl_ptr[idx];
            conf_pixel = conf_ptr[idx];
            temp_str << idx << " " <<  x << " " << y << " " << z << " " << dist << " "
                     << amp_pixel << " " << conf_pixel << std::endl;
            effective_number++;
        }
    }
    out_put_ << img_height << " " << img_width << " " << effective_number << std::endl;
    out_put_ << temp_str.str();
    out_put_.close();
    LOG(INFO) << "close sensor data file: " << this->sensor_data_file_path_;
}

void
DetectorModuleBase::recordSensorData(const std::string &dir,
                                     const std::string &type,
                                     const O3d3xxFrame &frame) {
    if (!out_put_.is_open()) {
        std::string logfile_name = "sensor_data_" + type + ".log";
        this->sensor_data_file_path_ = dir + logfile_name;

        out_put_.open(this->sensor_data_file_path_);
        if (!out_put_.is_open()) {
            LOG(INFO) << "record frame fail!" << this->sensor_data_file_path_;
            return;
        } else {
            LOG(INFO) << "create sensor data file: " << this->sensor_data_file_path_;
        }
    }

    const size_t img_height = frame.xyz_img.rows;
    const size_t img_width = frame.xyz_img.cols;

    if (0 == img_width || 0 == img_height){
        LOG(INFO) << "0 == img_width || 0 == img_height";
        return;
    }

    auto xyz_ptr = (short *) (frame.xyz_img.data);
    auto ampl_ptr = (ushort *) (frame.amplitude_img.data);
    auto conf_ptr = (uchar *) (frame.confidence_img.data);
    short x, y, z;
    float dist;
    ushort amp_pixel;
    uchar conf_pixel;
    std::stringstream temp_str;
    size_t effective_number = 0;
    for (size_t r = 0; r < img_height; ++r) {
        for (size_t c = 0; c < img_width; ++c) {
            const size_t idx = r * img_width + c;
            x = static_cast<short>(xyz_ptr[idx * 3 + 0]);
            y = static_cast<short>(xyz_ptr[idx * 3 + 1]);
            z = static_cast<short>(xyz_ptr[idx * 3 + 2]);
            dist = sqrt(x*x + y*y + z*z);
            if (dist < std::numeric_limits<float>::epsilon()){
                continue;
            }
            amp_pixel = ampl_ptr[idx];
            conf_pixel = conf_ptr[idx];
            temp_str << idx << " " <<  x << " " << y << " " << z << " " << dist << " "
                     << amp_pixel << " " << conf_pixel << std::endl;
            effective_number++;
        }
    }
    out_put_ << img_height << " " << img_width << " " << effective_number << std::endl;
    out_put_ << temp_str.str();
    out_put_.close();
    LOG(INFO) << "close sensor data file: " << this->sensor_data_file_path_;
}

void
DetectorModuleBase::recordSensorData(const std::string &dir,
                                     const PointCloudPtr &cloud) {
    if (!out_put_.is_open()) {
        std::string time_str = common_func::getDateTimeStr();
        std::string logfile_name = "sensor_data_" + time_str + ".log";
        this->sensor_data_file_path_ = dir + logfile_name;

        out_put_.open(this->sensor_data_file_path_);
        if (!out_put_.is_open()) {
            LOG(INFO) << "record frame fail!" << this->sensor_data_file_path_;
            return;
        } else {
            LOG(INFO) << "create sensor data file: " << this->sensor_data_file_path_;
        }
    }

    std::stringstream temp_str;
    float dist;
    size_t effective_number = 0;
    for (auto const &p : cloud->points) {
        dist = sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
        if (dist < std::numeric_limits<float>::epsilon()){
            continue;
        }
        temp_str << " " << p.x << " " << p.y << " " << p.z << std::endl;
        ++effective_number;
    }

    if (effective_number > 0) {
        out_put_ << cloud->time << " " << effective_number << std::endl;
        out_put_ << temp_str.str();
    }
    
    out_put_.close();
    LOG(INFO) << "close sensor data file: " << this->sensor_data_file_path_;
}

void
DetectorModuleBase::recordSensorData(const std::string &dir,
                                     const std::string &type,
                                     const PointCloudPtr &cloud) {
    if (!out_put_.is_open()) {
        std::string time_str = common_func::getDateTimeStr();
        // for test comment
        std::string logfile_name = "sensor_data_" + type + ".log";
        //std::string logfile_name = "sensor_data_" + time_str + ".log";
        this->sensor_data_file_path_ = dir + logfile_name;

        out_put_.open(this->sensor_data_file_path_);
        if (!out_put_.is_open()) {
            LOG(INFO) << "record frame fail!" << this->sensor_data_file_path_;
            return;
        } else {
            LOG(INFO) << "create sensor data file: " << this->sensor_data_file_path_;
        }
    }

    std::stringstream temp_str;
    float dist;
    size_t effective_number = 0;
    for (auto const &p : cloud->points) {
        dist = sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
        if (dist < std::numeric_limits<float>::epsilon()){
            continue;
        }
        temp_str << " " << p.x << " " << p.y << " " << p.z << std::endl;
        ++effective_number;
    }

    if (effective_number > 0) {
        out_put_ << cloud->time << " " << effective_number << std::endl;
        out_put_ << temp_str.str();
    }
    out_put_.close();
    LOG(INFO) << "close sensor data file: " << this->sensor_data_file_path_;
}

bool
DetectorModuleBase::readSensorData(const std::string &file_name,
                                   O3d3xxFrame &frame) {
    if (!in_put_.is_open()){
        in_put_.open(file_name);
    }

    if (!in_put_.is_open()){
        LOG(INFO) << "read frame fail! " << file_name;
        return false;
    }

    size_t img_height , img_width, effective_number;
    in_put_ >> img_height >> img_width >> effective_number;

    if (in_put_.eof()){
        return false;
    }

    frame.xyz_img = cv::Mat::zeros(img_height, img_width, CV_16SC3);
    frame.amplitude_img = cv::Mat::zeros(img_height, img_width, CV_16UC1);
    frame.confidence_img = cv::Mat::zeros(img_height, img_width, CV_8UC1);
    auto *xyz_ptr = (short *)(frame.xyz_img.data);
    auto *ampl_ptr = (ushort *)(frame.amplitude_img.data);
    auto *conf_ptr = (uchar*)(frame.confidence_img.data);

    float x, y, z, dist;
    ushort amp_pixel;
    uchar conf_pixel;
    size_t idx;
    size_t line_number = 0;
    while (!in_put_.eof() && line_number < effective_number) {
        in_put_ >> idx >> x >> y >> z >> dist >> amp_pixel >> conf_pixel;
        xyz_ptr[idx * 3 + 0] = x;
        xyz_ptr[idx * 3 + 1] = y;
        xyz_ptr[idx * 3 + 2] = z;
        ampl_ptr[idx] = amp_pixel;
        conf_ptr[idx] = conf_pixel;
        line_number++;
    }
    in_put_.close();
    return true;
}

bool
DetectorModuleBase::readSensorData(const std::string &file_name,
                                   const PointCloudPtr &frame) {
    if (!in_put_.is_open()){
        in_put_.open(file_name);
    }

    if (!in_put_.is_open()){
        LOG(INFO) << "read frame fail! " << file_name;
        return false;
    }

    size_t size=0;
    in_put_ >> frame->time >> size;

    if (in_put_.eof()){
        return false;
    }

    frame->resize(size);
    size_t i = 0;

    while (!in_put_.eof() && i < size) {
        in_put_ >> frame->points[i].x  >> frame->points[i].y >> frame->points[i].z;
        frame->image_indices[i] = i;
        ++i;
    }

    return true;
}
} // end of object detector namespace
