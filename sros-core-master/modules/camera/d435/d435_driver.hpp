//
// Created by lfc on 19-5-5.
//

#ifndef SROS_D435_DRIVER_HPP
#define SROS_D435_DRIVER_HPP

#include <glog/logging.h>
#include <math.h>
#include <iostream>
#include <core/circle_optimizer_set.hpp>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <sstream>
#include "../stereo_point.h"
#include "core/state.h"
#include "obstacle_detecter.hpp"
#include <opencv2/opencv.hpp>

namespace d435 {
class D435Driver {
 public:
    D435Driver(std::string driver_address, const std::string& stereo_camera_type, bool using_ir_image = false)
        : serial_number_(driver_address),
          camera_type_(stereo_camera_type),
          using_ir_image_(using_ir_image),
          decimation_filter(2.0f),
          threshold_filter(0.15f, 4.f),
          temporal_filter(0.4f, 20.f, 4) {
    }

    static const std::vector<std::string> getAllDevices() {
        std::vector<std::string> serial_numbers;
        try {
            auto query_devices = ctx_.query_devices();
            LOG(INFO) << "found devices numbers:" << query_devices.size();
            for (auto &&dev : query_devices) {
                serial_numbers.emplace_back();
                LOG(INFO) << "found serial info:" << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
                serial_numbers.back() = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            }
        } catch (std::exception &e) {
            LOG(ERROR) << e.what();
        }
        return serial_numbers;
    }

    static void waitForInitialized() {
        uint sleep_in_50_ms = 5e4;
        while (initialized_state_) {
            usleep(sleep_in_50_ms);
        }
    }

    void closeDevice() {
        if(pipe) {
            pipe->stop();
        }
        opened_ = false;
    }

    bool isOpened() { return opened_; }

    bool init(bool set_option = true) {
        if(!serial_number_.size()) {
            LOG(ERROR) << "SN null, init error!!!";
            return false;
        }
        try {
            waitForInitialized();
            initialized_state_ = true;

            LOG(INFO) << "try init serial:" << serial_number_;
            if (findSerialNumber(serial_number_)) {
                cfg.reset(new rs2::config);
                cfg->enable_device(serial_number_);
                if(using_ir_image_)
                    ///在只使用ir相机进行数据采集时
                    initIrCamera();
                else{
                    initDepthCamera(set_option);
                }
                opened_ = true;
                initialized_state_ = false;
                return true;
            } else {
                LOG(ERROR) << "cannot find the device:" << serial_number_;
                initialized_state_ = false;

                // 硬件重启d435。暂时关闭，发现有一直重连不上的现象20211125
                // auto query_devices = ctx_.query_devices();
                // for (auto &&dev : query_devices) {
                //     dev.hardware_reset();
                //     LOG(INFO) << "d435 hardware reset!";
                // }
                // sleep(10);
                return false;
            }
        } catch (std::exception &e) {
            opened_ = false;
            LOG(ERROR) << "cannot open d435!" << e.what() << "," << serial_number_;
            initialized_state_ = false;
            enable_set_option_ = false;
            return false;
        } catch (...) {
            opened_ = false;
            LOG(ERROR) << "cannot open d435:" << serial_number_;
            initialized_state_ = false;
            enable_set_option_ = false;
            return false;
        }
    }

    void initDepthCamera(bool set_option){
        if (camera_type_ == "D435") {
            cfg->enable_stream(rs2_stream::RS2_STREAM_DEPTH, 848, 480, rs2_format::RS2_FORMAT_Z16, 15);
            cfg->enable_stream(rs2_stream::RS2_STREAM_COLOR, 848, 480, rs2_format::RS2_FORMAT_RGB8, 15);
        } else {  //D430
            cfg->enable_stream(rs2_stream::RS2_STREAM_DEPTH, 848, 480, rs2_format::RS2_FORMAT_Z16, 15);
        }
        pipe.reset(new rs2::pipeline(ctx_));
        auto profile = pipe->start(*cfg);
        sensor_.reset(new rs2::depth_sensor(profile.get_device().first<rs2::depth_sensor>()));
        if (enable_set_option_&&set_option) {
            if (sensor_->supports(RS2_OPTION_VISUAL_PRESET)) {
                auto reset_accuracy = sensor_->get_option(RS2_OPTION_VISUAL_PRESET);
                LOG(INFO) << "reset accuracy:" << reset_accuracy;
                if (sensor_->get_option(RS2_OPTION_VISUAL_PRESET) != RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY) {
                    LOG(INFO) << "set option:" << RS2_OPTION_VISUAL_PRESET;
                    sensor_->set_option(RS2_OPTION_VISUAL_PRESET, RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY);
                }
            }
            if (sensor_->supports(RS2_OPTION_LASER_POWER)) {
                //                    LOG(INFO) << "supports!";
                auto range = sensor_->get_option_range(RS2_OPTION_LASER_POWER);
                auto range_option = sensor_->get_option(RS2_OPTION_LASER_POWER);
                LOG(INFO) << "option:" << range_option;
                if (sensor_->get_option(RS2_OPTION_LASER_POWER) != range.max) {
                    LOG(INFO) << "set option:" << RS2_OPTION_LASER_POWER << "," << range.max;
                    sensor_->set_option(RS2_OPTION_LASER_POWER, range.max);
                }
            }
        } else {
            auto reset_accuracy = sensor_->get_option(RS2_OPTION_VISUAL_PRESET);
            auto range_option = sensor_->get_option(RS2_OPTION_LASER_POWER);
            auto range = sensor_->get_option_range(RS2_OPTION_LASER_POWER);
            LOG(INFO) << "reset accuracy:" << reset_accuracy;
            if (!set_option) {
                LOG(INFO) << "will set option!";
                sensor_->set_option(RS2_OPTION_VISUAL_PRESET, RS2_RS400_VISUAL_PRESET_DEFAULT);
                sensor_->set_option(RS2_OPTION_LASER_POWER, range.def);
                usleep(2e5);
            }
            LOG(INFO) << "range :" << range.def << "," << range.min << "," << range.max;
            LOG(INFO) << "option:" << range_option;
            LOG(INFO) << "cannot set option!";
        }
    }

    void initIrCamera(){
        const int width = 640;
        const int height = 480;
        const int fps = 30;
//        cfg->enable_stream(rs2_stream::RS2_STREAM_INFRARED, 1, width, height, rs2_format::RS2_FORMAT_Y8, fps);
        cfg->enable_stream(rs2_stream::RS2_STREAM_COLOR, width, height, rs2_format::RS2_FORMAT_RGB8, fps);
        pipe.reset(new rs2::pipeline(ctx_));
        auto profile = pipe->start(*cfg);
        rs2::sensor sensor = rs2::depth_sensor(profile.get_device().first<rs2::depth_sensor>());
        if(sensor.supports(RS2_OPTION_EMITTER_ENABLED)){
            sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0);
            LOG(INFO) << serial_number_ << " ir projector is disable";
        }
    }

    bool setNormalMode(){
        if (isOpened()) {
            auto reset_accuracy = sensor_->get_option(RS2_OPTION_VISUAL_PRESET);
            auto range_option = sensor_->get_option(RS2_OPTION_LASER_POWER);
            auto range = sensor_->get_option_range(RS2_OPTION_LASER_POWER);
            LOG(INFO) << "reset accuracy:" << reset_accuracy;
            LOG(INFO) << "will set option!";
            sensor_->set_option(RS2_OPTION_VISUAL_PRESET, RS2_RS400_VISUAL_PRESET_DEFAULT);
            sensor_->set_option(RS2_OPTION_LASER_POWER, range.def);
            usleep(2e5);
            LOG(INFO) << "range :" << range.def << "," << range.min << "," << range.max;
            LOG(INFO) << "option:" << range_option;
            return true;
        }
        return false;
    }

    bool setHighResolutionMode(){
         if (isOpened()) {
            try{
                if (sensor_->supports(RS2_OPTION_VISUAL_PRESET)) {
                    auto reset_accuracy = sensor_->get_option(RS2_OPTION_VISUAL_PRESET);
                    LOG(INFO) << "reset accuracy:" << reset_accuracy;
                    if (sensor_->get_option(RS2_OPTION_VISUAL_PRESET) != RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY) {
                        LOG(INFO) << "set option:" << RS2_OPTION_VISUAL_PRESET;
                        sensor_->set_option(RS2_OPTION_VISUAL_PRESET, RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY);
                    }
                }
                if (sensor_->supports(RS2_OPTION_LASER_POWER)) {
                    //                    LOG(INFO) << "supports!";
                    auto range = sensor_->get_option_range(RS2_OPTION_LASER_POWER);
                    auto range_option = sensor_->get_option(RS2_OPTION_LASER_POWER);
                    LOG(INFO) << "option:" << range_option;
                    if (sensor_->get_option(RS2_OPTION_LASER_POWER) != range.max) {
                        LOG(INFO) << "set option:" << RS2_OPTION_LASER_POWER << "," << range.max;
                        sensor_->set_option(RS2_OPTION_LASER_POWER, range.max);
                    }
                }
                return true;
            } catch (...) {
                LOG(ERROR) << "cannot set option!";
                return false;
            }
        }
        return false;
    }

    bool findSerialNumber(const std::string &number) {
        auto &sns = getAllDevices();
        for (auto &sn : sns) {
            if (sn == number) {
                return true;
            }
        }
        LOG(ERROR) << "cannot find camera SN " << number;
        return false;
    }

    bool getStereoPoints(rs2::points& stereo_points,int &length,int &width){
        waitForInitialized();
        if (!opened_) {
            LOG(ERROR) << "camera not opened!";
            return false;
        }
        try {
            auto frames = pipe->wait_for_frames();
            if (camera_type_ == "D435") {
                auto color = frames.get_color_frame();
                pointcloud.map_to(color);
            }
            auto depth = frames.get_depth_frame();
            rs2::frame filtered = depth;
            filtered = decimation_filter.process(filtered);
            filtered = threshold_filter.process(filtered);
            filtered = temporal_filter.process(filtered);
            stereo_points = pointcloud.calculate(filtered);
            length = depth.get_width() / 2;
            width = depth.get_height() / 2;
            return true;
        }catch (...) {
            LOG(ERROR) << "get data error! will reopen d435!";
            closeDevice();
            return false;
        }
    }

    bool getIRImage(cv::Mat &ir_image,int64_t &frame_time){
        waitForInitialized();
        if (!opened_) {
            LOG(ERROR) << "have not opened!";
            return false;
        }

        try {
            auto frames = pipe->wait_for_frames();
//            rs2::video_frame ir_frame = frames.get_infrared_frame();
            rs2::video_frame ir_frame = frames.get_color_frame();
            int64_t curr_frame_time = ir_frame.get_timestamp()*1e3;//us
            frame_time = getFrameTime(curr_frame_time, sros::core::util::get_time_in_us());
            convertFrameToMat(ir_frame, ir_image);
        } catch (...) {
            LOG(ERROR) << "get IR data error! will reopen d435!";
            closeDevice();
            return false;
        }

        return true;
    }

    int64_t getFrameTime(int64_t origin_frame_time,int64_t system_time){
        if (delta_time_stamp_.empty()) {
            delta_time_stamp_.resize(100);
        }
        int64_t delta_time_stamp = system_time - origin_frame_time;
        delta_time_stamp_.push_back(delta_time_stamp);
        auto min_delta_stamp = delta_time_stamp_.getMinValue();
        int64_t curr_time = min_delta_stamp + origin_frame_time;
        if ((system_time - curr_time) > 1e5) {
            LOG(INFO) << "find large delta time!" << system_time - curr_time;
            curr_time = system_time;
            delta_time_stamp_.resize(100);
        }
        return curr_time;
    }

    void getIrIntrinsic(std::vector<double> &intrinsic, std::vector<double> &distort){
        if(opened_&&pipe){
            auto frames = pipe->wait_for_frames();
    //        rs2::video_frame ir_frame = frames.get_infrared_frame();
            rs2::video_frame ir_frame = frames.get_color_frame();
            getIntrinsic(ir_frame, intrinsic, distort);
        }else{
            LOG(ERROR) << "cannot get intrinsic!";
        }
    }

 private:
    void save() {
        LOG(INFO) << "Saving data";
        auto now = std::chrono::system_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        std::ostringstream oss;
        oss << std::put_time(std::localtime(&in_time_t), "%d-%X");
        oss << "-" << std::setfill('0') << std::setw(3) << ms.count();
        const std::string prefix = oss.str();
        for (int i = 0; i < 10; i++) {
            std::ofstream ofstream(std::string("/sros/") + prefix + std::to_string(i), std::ios::out);
            boost::archive::text_oarchive text_oarchive(ofstream);
            int points_size = point_clouds[i].points.size();
            text_oarchive << points_size;
            for (int j = 0; j < points_size; j++) {
                text_oarchive << point_clouds[i].points[j];
            }
            ofstream.close();
        }
    }

    void convertFrameToMat(const rs2::video_frame &frame, cv::Mat &image){

        int witdh = frame.get_width();
        int height = frame.get_height();
        cv::Size size(witdh, height);

        if(frame.get_profile().stream_type() == RS2_STREAM_COLOR){
            cv::Mat rgb_image = cv::Mat(size, CV_8UC3, (void*)frame.get_data());
            cv::cvtColor(rgb_image, image, cv::COLOR_RGB2BGR);
        }else if(frame.get_profile().stream_type() == RS2_STREAM_DEPTH){
            image = cv::Mat(size, CV_16UC1, (void*)frame.get_data());
        }else if(frame.get_profile().stream_type() == RS2_STREAM_INFRARED){
            image = cv::Mat(size, CV_8UC1, (void*)frame.get_data(), cv::Mat::AUTO_STEP);
        }
    }

    void getIntrinsic(rs2::frame &frame, std::vector<double> &intrinsic, std::vector<double> &distort){
        rs2::stream_profile profile = frame.get_profile();
        rs2::video_stream_profile cvs_profile(profile);
        rs2_intrinsics rs_intrinsic = cvs_profile.get_intrinsics();

        //内参
        intrinsic.emplace_back(rs_intrinsic.fx);
        intrinsic.emplace_back(0);
        intrinsic.emplace_back(rs_intrinsic.ppx);
        intrinsic.emplace_back(0);
        intrinsic.emplace_back(rs_intrinsic.fy);
        intrinsic.emplace_back(rs_intrinsic.ppy);
        intrinsic.emplace_back(0);
        intrinsic.emplace_back(0);
        intrinsic.emplace_back(1.0);

        //畸变系数
        distort.emplace_back(rs_intrinsic.coeffs[0]);
        distort.emplace_back(rs_intrinsic.coeffs[1]);
        distort.emplace_back(rs_intrinsic.coeffs[2]);
        distort.emplace_back(rs_intrinsic.coeffs[3]);
        distort.emplace_back(rs_intrinsic.coeffs[4]);
    }

 private:
    std::shared_ptr<rs2::config> cfg;
    std::string serial_number_;
    circle::CircleOptimizerArray<int64_t> delta_time_stamp_;
    std::shared_ptr<rs2::depth_sensor> sensor_;
    rs2::pointcloud pointcloud;
    std::shared_ptr<rs2::pipeline> pipe;
    rs2::decimation_filter decimation_filter;
    rs2::threshold_filter threshold_filter;
    rs2::temporal_filter temporal_filter;
    StereoPoints point_clouds[10];
    static rs2::context ctx_;
    static bool initialized_state_;
    bool opened_ = false;
    bool enable_set_option_ = true;
    int i = 0;

    bool using_ir_image_ = false;
    std::map<std::string, std::vector<rs2::sensor>> map_sensors_;
    std::string camera_type_;

};
}  // namespace d435

#endif  // SROS_D435_DRIVER_HPP
