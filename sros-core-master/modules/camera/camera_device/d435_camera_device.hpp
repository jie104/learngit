//
// Created by lfc on 2019/11/26.
//

#ifndef SROS_D435_CAMERA_DEVICE_HPP
#define SROS_D435_CAMERA_DEVICE_HPP
#include <boost/thread.hpp>
#include <mutex>
#include <condition_variable>
#include "../d435/d435_driver.hpp"
#include "../d435/d435_obstacle_detector.hpp"
#include "../stereo_module_para.hpp"
#include "core/msg/image_msg.hpp"
#include "base_camera_device.hpp"

namespace camera {
class D435CameraDevice : public BaseCameraDevice {
 public:
    D435CameraDevice(const std::shared_ptr<stereo::StereoModulePara> &para, std::string device_sn,
                     ImgMsgCallback callback, const std::string &name, sros::device::DeviceID device_id, const std::string& stereo_camera_type)
        : device_sn_(device_sn),
          para_(para),
          stereo_camera_type_(stereo_camera_type),
          BaseCameraDevice(callback, name, device_id, sros::device::DEVICE_COMM_INTERFACE_TYPE_USB_0,
                           sros::device::DEVICE_MOUNT_SROS) {
        d435_driver_.reset(new d435::D435Driver(device_sn_, stereo_camera_type));
        setSerialNo(device_sn);
    }


    bool ok() { return ok_; }

    virtual bool isOpened() { return d435_driver_->isOpened(); }

    virtual bool isEnabled() { return enable_state_; }

    bool calibrateCameraInstallErr(Eigen::Quaternionf initial_q, const double calibration_height, double &install_height,
                                   Eigen::Matrix3d &calibration_matrix) {
        rs2::points stereo_points;
        int length,width;
        int count = 0;
        int max_wait_count = 10;
        while (!d435_driver_->isOpened()) {
            if (count++ > max_wait_count) {
                LOGGER(INFO, SROS) << "cannot open D435! will return false!";
                return false;
            }
        }
        d435_driver_->setNormalMode();
        if (d435_driver_->getStereoPoints(stereo_points,length,width)) {
            int try_count = 0;
            int max_try_count = 10;
            while (d435_driver_->getStereoPoints(stereo_points, length, width)) {
                if(d435_detector_.getEnoughtPlane(stereo_points,length,width)) {
                    d435_driver_->setHighResolutionMode();
                    Eigen::Matrix3f calibration_matrix_f = initial_q.matrix();
                    float calibration_height_f = calibration_height,install_height_f = install_height;
                    if(d435_detector_.calibrateCamera(stereo_points, length, width, calibration_matrix_f, install_height_f,
                                                      calibration_height_f)){
                        install_height = install_height_f;
                        calibration_matrix = calibration_matrix_f.cast<double>();
                        return true;
                    }
                }
                if (try_count++ > max_try_count) {
                    LOGGER(INFO, SROS) << "d435 calibration try enough time! will break!";
                    break;
                }
            }
            LOGGER(INFO, SROS) << "d435 calibration try count:" << try_count << ",max count:" << max_try_count;
        }
        LOGGER(INFO, SROS) << "cannot get d435 calibration!";
        return false;
    }

    void updateCameraInstallErr(const Eigen::Matrix3d &install_matrix, double install_height) {
        auto install_matrix_f = install_matrix.cast<float>();
        d435_detector_.setCalibrationInfo(install_matrix_f, install_height);
        LOG(INFO) << "UPDATA install_matrix_f :" << install_matrix_f << ",install_height :" << install_height;
        //        d435_driver_->updateCameraInstallErr(install_matrix, install_height);
    }

    void setHeightThresh(const double min_height,const double max_height){
        d435_detector_.setHeightThresh(min_height, max_height);
    }

    void setDetectDistThresh(const double max_dist){
        d435_detector_.setMaxDistThresh(max_dist);
    }

    virtual bool open() {
        ok_ = true;
        setStateInitialization();
        const int try_times_thresh = 1;
        int curr_time = 0;
        while (curr_time++ < try_times_thresh) {
            if (d435_driver_->init()) {
                LOG(INFO) << "succesfully to open D435 camera!";
                setStateOK();
                break;
            } else if(curr_time < try_times_thresh) {
                sleep(10);
                LOG(ERROR) << "cannot open D435! sleep 10s and retry: " << curr_time << " (" << try_times_thresh << ")";
            }
        }
        if (!thread_running_) {
            creatHandleCameraThread();
        }
        if (d435_driver_->isOpened()) {
            return true;
        } else {
            LOG(ERROR) << "cannot open D435 camera!";
            setStateOpenFailed();
            return false;
        }
    }

    virtual void close() {
        ok_ = false;
        enable_state_ = false;
        d435_driver_.reset(new d435::D435Driver(device_sn_, stereo_camera_type_));
    }

    virtual bool enableCamera() {
        LOG(INFO) << "will enable D435!";
        return wakeUpThread();
    }

    bool wakeUpThread() {
        enable_state_ = true;
        int wait_count = 0;
        while (idle_wait_state_) {
            condition_.notify_one();
            if (idle_wait_state_) {
                sleepFor10ms();
                if (wait_count++ > 50) {
                    LOG(ERROR) << "cannot wake up thread! will return false!";
                    return false;
                }
            }
        }
        return true;
    }

    virtual bool disableCamera() {
        LOG(INFO) << "will disable camera!";
        enable_state_ = false;
		return true;
    }

    // 在设备重连后执行,更新数据库的配置参数SN的取值范围
    void updateUserSettingsValueRange(const std::string &new_sn) {
        // read user settings and set new value range
        auto &s = sros::core::Settings::getInstance();
        bool enable_stereo_camera_1 = (s.getValue<std::string>("camera.enable_d435_camera", "True") == "True");
        bool enable_stereo_camera_2 = (s.getValue<std::string>("camera.enable_d435_2_camera", "True") == "True");
        bool enable_stereo_camera_3 = (s.getValue<std::string>("camera.enable_d435_3_camera", "True") == "True");
        bool enable_stereo_camera_4 = (s.getValue<std::string>("camera.enable_d435_4_camera", "True") == "True");
        auto stereo_camera_1_info = s.getItemInfo("camera.d435_1_serial_number");
        auto stereo_camera_2_info = s.getItemInfo("camera.d435_2_serial_number");
        auto stereo_camera_3_info = s.getItemInfo("camera.d435_3_serial_number");
        auto stereo_camera_4_info = s.getItemInfo("camera.d435_4_serial_number");
        LOG(INFO) << "user set SN old value_range: " << stereo_camera_1_info.value_range;
        
        // handle the new sn from user settings
        std::vector<std::string> sns;
        std::string value_range_old = stereo_camera_1_info.value_range;
        if (value_range_old.empty() || (value_range_old.find(new_sn) == std::string::npos)) {
            LOG(WARNING) << "can not find new_sn in user settings, add it";
            sns.emplace_back(new_sn);
        } else {
            LOG(WARNING) << "sns value_range no update";
            return;
        }

        // 把实际存在的相机序列号 sn 放进数据库的value_range
        std::string value_range = stereo_camera_1_info.value_range;
        for (auto& stereo_camera_sn : sns) {
            value_range += stereo_camera_sn;
            value_range += ';';
        }
        LOG(INFO) << "the online SN new value_range:" << value_range;
        stereo_camera_4_info.value_range = value_range;
        stereo_camera_3_info.value_range = value_range;
        stereo_camera_2_info.value_range = value_range;
        stereo_camera_1_info.value_range = value_range;
        s.setItemInfo(stereo_camera_4_info);
        s.setItemInfo(stereo_camera_3_info);
        s.setItemInfo(stereo_camera_2_info);
        s.setItemInfo(stereo_camera_1_info);
    }

 private:
    void creatHandleCameraThread() { boost::thread(boost::bind(&D435CameraDevice::handleD435Camera, this)); }

    void handleD435Camera() {
        thread_running_ = true;

        int64_t last_publish_time = 0;
        int64_t sleep_in_40_ms = 4e4;
        while (ok()) {
            if (!enable_state_&&!isRecording()) {  //在未启用摄像头的状态下,需要每隔一段时间读取一次摄像头数据,确定是否失效；只有当前不是录制状态，才需要做睡眠
                std::unique_lock<std::mutex> lock(condition_mutex_);
                idle_wait_state_ = true;
//                if (!enable_state_) {
//                    auto state = condition_.wait_for(lock, std::chrono::seconds(1));
//                }
            }

            idle_wait_state_ = false;
            if (d435_driver_) {
                if (!d435_driver_->isOpened()) {
                    LOG(WARNING) << "d435 not opened! will close and open!";
                    if (d435_driver_->init()) {
                        setStateOK();
                        updateUserSettingsValueRange(device_sn_);
                        LOG(INFO) << "successfully to open D435!";
                    } else {
                        LOG(ERROR) << "cannot open D435! sleep 10s";
                        sleep(10);
                    }
                }
                if (d435_driver_->isOpened()) {
                    int64_t frame_stamp = sros::core::util::get_time_in_us();
                    std::shared_ptr<StereoPoints> filter_points(new StereoPoints);
                    rs2::points stereo_points;
                    int length,width;
                    if (d435_driver_->getStereoPoints(stereo_points,length,width)) {
                        if (enable_state_) {
                            d435_detector_.computeObstaclePoints(stereo_points, length, width, *filter_points,obszie_);
                            ImgWithStampInfoPtr img_with_stamp(new ImgWithStampInfo);
                            img_with_stamp->use_points_info = true;
                            img_with_stamp->points.reset(new sros::core::ObstacleMsg("IFM_ORIGIN_POINTS"));
                            img_with_stamp->points->oba_name = getName();
                            img_with_stamp->points->time_ = frame_stamp;
                            convertToObaPoints(filter_points, img_with_stamp->points);
                            img_with_stamp->points_topic_name = "IFM_ORIGIN_POINTS";
                            img_with_stamp->camera_name = getName();
                            img_with_stamp->stamp = frame_stamp;
                            if (sendMsg) {
                                sendMsg(img_with_stamp);
                            }
                        }
                        if (isRecording()) {
                            serialization::PointImage image(length, width);
                            convertToRecordData(stereo_points, length, width, image);
                            recordData(image);
                        }
                        keepAlive();
                    } else {
                        LOG(ERROR) << "D435 get data error! setStateOpenFailed";
                        setStateOpenFailed();
                    }

//                    std::shared_ptr<StereoPoints> filter_points(new StereoPoints);
//                    if (d435_driver_->getFilterPointCloud(filter_points)) {
//                        if (enable_state_) {
//                            ImgWithStampInfoPtr img_with_stamp(new ImgWithStampInfo);
//                            img_with_stamp->use_points_info = true;
//                            img_with_stamp->points.reset(new sros::core::ObstacleMsg("IFM_ORIGIN_POINTS"));
//                            img_with_stamp->points->oba_name = getName();
//                            img_with_stamp->points->time_ = frame_stamp;
//                            convertToObaPoints(filter_points, img_with_stamp->points);
//                            img_with_stamp->points_topic_name = "IFM_ORIGIN_POINTS";
//                            img_with_stamp->camera_name = getName();
//                            img_with_stamp->stamp = frame_stamp;
//                            if (sendMsg) {
//                                sendMsg(img_with_stamp);
//                            }
//                        }
//                        keepAlive();
//                    } else {
//                        LOG(INFO) << "D435 has closed! will reopen it!";
//                        setStateOpenFailed();
//                    }
                }
            } else {
                LOG(ERROR) << "d435 is not opened!";
            }
            usleep(sleep_in_40_ms);
        }
        thread_running_ = false;
    }

 private:
    void convertToRecordData(const rs2::points& stereo_points,int length,int width,serialization::PointImage& point_image){
        auto vertices = stereo_points.get_vertices();
        for (int i = 0; i < width; ++i) {
            for (int j = 0; j < length; ++j) {
                auto &vertice = vertices[i * length + j];
                auto &point = point_image(j, i);
                point.coord_x = j, point.coord_y = i;
                if (!std::isfinite(vertice.x) || !std::isfinite(vertice.y) || !std::isfinite(vertice.z)) {
                    point.valid = false;
                } else if (math::isZero(vertice.x) && math::isZero(vertice.y) && math::isZero(vertice.z)) {
                    point.valid = false;
                } else {
                    point.point = Eigen::Vector3f(vertice.z, -vertice.x, -vertice.y);
                    point.valid = true;
                }
            }
        }
    }

    void computeRackLegs(const double rack_length, const double rack_width, std::vector<Eigen::Vector2d> &rack_legs) {
        Eigen::Vector2d first_rack(rack_length / 2.0, rack_width / 2.0);
        Eigen::Vector2d second_rack(-rack_length / 2.0, rack_width / 2.0);
        Eigen::Vector2d third_rack(-rack_length / 2.0, -rack_width / 2.0);
        Eigen::Vector2d forth_rack(rack_length / 2.0, -rack_width / 2.0);
        rack_legs.clear();
        rack_legs.push_back(first_rack);
        rack_legs.push_back(second_rack);
        rack_legs.push_back(third_rack);
        rack_legs.push_back(forth_rack);
    }

    bool near(const Eigen::Vector2d &curr_point, const Eigen::Vector2d &other_point, double dist_thresh) const {
        return (curr_point - other_point).norm() < dist_thresh;
    }

    void convertToObaPoints(const std::shared_ptr<StereoPoints> &origin_points,
                            sros::core::ObstacleMsg_ptr &filtered_points) {
        filtered_points->point_cloud.reserve(origin_points->points.size());
        bool do_filter_legs = false;
        double rotate_value = (double)g_state.rotate_value / 1000.0;  //货架相对于小车的旋转角度,逆时针为正
        if (para_->enable_remove_rack_leg) {
            do_filter_legs = true;
        }

        Eigen::Affine2d rotate_tf(Eigen::Translation2d(0, 0) * Eigen::Rotation2Dd(rotate_value));
        auto install_tf = (Eigen::Translation2d(para_->d435_install_x, para_->d435_install_y) *
                           Eigen::Rotation2Dd(para_->d435_install_yaw));
        auto real_tf = install_tf.inverse() * rotate_tf;
        std::vector<Eigen::Vector2d> rack_legs;
        computeRackLegs(para_->rack_leg_wheel_center_length, para_->rack_leg_wheel_center_width, rack_legs);
        std::vector<Eigen::Vector2d> legs_in_d435;
        for (auto &leg : rack_legs) {
            legs_in_d435.push_back(real_tf * leg);
        }
        const double rack_wheel_rotate_radius = para_->rack_wheel_rotate_radius;
        for (auto &point : origin_points->points) {
            if (do_filter_legs) {
                bool find_legs = false;
                for (auto &leg : legs_in_d435) {
                    if (near(leg, Eigen::Vector2d(point.x, point.y), rack_wheel_rotate_radius)) {
                        find_legs = true;
                        break;
                    }
                }
                if (find_legs) {
                    continue;
                }
            }
            filtered_points->point_cloud.emplace_back();
            filtered_points->point_cloud.back().x() = point.x;
            filtered_points->point_cloud.back().y() = point.y;
            filtered_points->point_cloud.back().z() = point.z;
        }

    }

    std::string device_sn_;
    std::shared_ptr<d435::D435Driver> d435_driver_;
    std::shared_ptr<stereo::StereoModulePara> para_;
    Eigen::Translation2d install_tf_;
    bool ok_ = false;
    bool enable_state_ = false;
    bool idle_wait_state_ = true;
    bool thread_running_ = false;

    d435::D435ObstacleDetector d435_detector_;

    std::mutex condition_mutex_;
    std::condition_variable_any condition_;
    std::string stereo_camera_type_;
};

}  // namespace camera

#endif  // SROS_D435_CAMERA_DEVICE_HPP
