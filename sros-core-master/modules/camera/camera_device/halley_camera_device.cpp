//
// Created by ehl on 2022/07/07.
//

#include "halley_camera_device.h"
#include "../d435/math.hpp"

using namespace camera;

DabaiCameraDevice::DabaiCameraDevice(const std::shared_ptr<stereo::StereoModulePara> &para, std::string device_sn,std::string camera_sn,
                      ImgMsgCallback callback, const std::string &name, sros::device::DeviceID device_id)
        : camera_sn_(camera_sn), device_uri_(device_sn), para_(para),
          BaseCameraDevice(callback, name, device_id, sros::device::DEVICE_COMM_INTERFACE_TYPE_USB_0, sros::device::DEVICE_MOUNT_SROS)
{
        LOG(INFO) <<  "DabaiCameraDevice() Uri: " << device_uri_ << " camera_sn_: " << camera_sn_;
        dabai_driver_.reset(new astra_dev::AstraDriver(device_uri_));
        setSerialNo(camera_sn_);
}

DabaiCameraDevice::~DabaiCameraDevice() {

}

bool DabaiCameraDevice::isOpened() {
    return dabai_driver_->isOpened();
}

bool DabaiCameraDevice::isEnabled() {
    return enable_state_;
}

bool DabaiCameraDevice::calibrateCameraInstallErr(Eigen::Quaternionf initial_q, const double calibration_height, double &install_height,
                                Eigen::Matrix3d &calibration_matrix) {
    //rs2::points stereo_points;
    StereoPoints stereo_points;
    int length,width;
    int count = 0;
    int max_wait_count = 10;

    while (!dabai_driver_->isOpened()) {
        if (count++ > max_wait_count){
            LOGGER(INFO, SROS) << "cannot open DabaiProk! will return false!";
            return false;
        }
    }
    LOG(INFO) << "halley get Stereo Data";
    if (dabai_driver_->getStereoData(length,  width, stereo_points)) {
        int try_count = 0;
        int max_try_count = 10;
        while (dabai_driver_->getStereoData(length,  width, stereo_points)) {
            if(dabaiPro_detector_.getEnoughtPlane(stereo_points,length,width)) {

                Eigen::Matrix3f calibration_matrix_f = initial_q.matrix();
                float calibration_height_f = calibration_height,install_height_f = install_height;
                if(dabaiPro_detector_.calibrateCamera(stereo_points, length, width, calibration_matrix_f, install_height_f,
                                                        calibration_height_f)){
                    install_height = install_height_f;
                    calibration_matrix = calibration_matrix_f.cast<double>();
                    return true;
                }
            }
            if (try_count++ > max_try_count) {
                LOGGER(INFO, SROS) << "dabaipro calibration try enough time! will break!";
                break;
            }
        }
        LOGGER(INFO, SROS) << "dabaipro calibration try count:" << try_count << ",max count:" << max_try_count;
    }
    LOGGER(INFO, SROS) << "cannot get dabaipro calibration!";
    return false;
}


void DabaiCameraDevice::updateCameraInstallErr(const Eigen::Matrix3d &install_matrix, double install_height) {
    auto install_matrix_f = install_matrix.cast<float>();
    dabaiPro_detector_.setCalibrationInfo(install_matrix_f, install_height);
}

/*设置检测高度阈值*/
void DabaiCameraDevice::setHeightThresh(const double min_height,const double max_height) {
    dabaiPro_detector_.setHeightThresh(min_height, max_height);
}

/* 设置检测距离阈值 */
void DabaiCameraDevice::setDetectDistThresh(const double max_dist) {
    dabaiPro_detector_.setMaxDistThresh(max_dist);
}

bool DabaiCameraDevice::open() {
    if(camera_sn_.empty()) {
        LOG(ERROR) << "camera_sn_ empty error!!!";
        return false;
    }
    setStateInitialization();
    const int try_times_thresh = 1;
    int curr_time = 0;
    while (curr_time++ < try_times_thresh) {
        if (dabai_driver_->init()) {
            LOG(INFO) << "succesfully to open Dabai camera SN: " << camera_sn_;
            setStateOK();
            break;
        } else {
            sleep(3);
            LOG(ERROR) << "cannot open Dabai! sleep 3s and retry, curr time is:" << curr_time;
            setStateOpenFailed();
        }
    }

    // 对OBDevice, 不开线程, 采用回调的方式拿数据
    // if (!thread_running_) {
    //     LOG(INFO) << "creat handle camera thread.";
    //     creatHandleCameraThread();
    //     return true;
    // }
    // else {
    //     LOG(ERROR) << "cant not creat handle camera thread!!!";
    // }

    if (!dabai_driver_->openStream())
    {
        LOG(ERROR) << "open or start camera stream error!!!";
        setStateOpenFailed();
        return false;
    }
    return true;
}

void DabaiCameraDevice::close() {
    thread_running_ = false;
    enable_state_ = false;
    if(!dabai_driver_) {
        LOG(INFO) << "dabai_driver_ null";
    } else {
        LOG(INFO) << "dabai_driver_ close";
    }
    dabai_driver_->close();
    // dabai_driver_.reset(new astra_dev::AstraDriver(device_uri_)); // 实际执行顺序: 先new,再reset, 再调用析构函数 -- 有误
}

bool DabaiCameraDevice::enableCamera() {
    LOG(INFO) << "will enable Dabai!";
    enable_state_ = true;
    return true;
    // return wakeUpThread();
}

bool DabaiCameraDevice::wakeUpThread() {
    enable_state_ = true;
    int wait_count = 0;
    while (idle_wait_state_) {
        condition_.notify_one();
        if (idle_wait_state_) {
            sleepFor10ms();
            if (wait_count++ > 50) {
                LOG(INFO) << "!!!!! cannot wake up thread! will return false!";
                return false;
            }
        }
    }
    return true;
}

bool DabaiCameraDevice::disableCamera() {
    LOG(INFO) << "will disable camera points!";
    enable_state_ = false;
    return true;
}

void DabaiCameraDevice::creatHandleCameraThread()
{
    boost::thread(boost::bind(&DabaiCameraDevice::handleDabaiProCamera, this));
}

void DabaiCameraDevice::handleDabaiProCamera()
{
    if (!dabai_driver_->openStream())
    {
        return;
    }
    
    thread_running_ = true;
    int64_t sleep_in_40_ms = 4e4;
    int frame_error_count = 0;

    while (thread_running_) {
        if (!enable_state_&&!isRecording()) {  //在未启用摄像头的状态下,需要每隔一段时间读取一次摄像头数据,确定是否失效；只有当前不是录制状态，才需要做睡眠
            std::unique_lock<std::mutex> lock(condition_mutex_);
            idle_wait_state_ = true;
            if (!enable_state_) {
                auto state = condition_.wait_for(lock, std::chrono::seconds(1));
            }
        }

        idle_wait_state_ = false;
        if (dabai_driver_ && dabai_driver_->isOpened() && thread_running_) {
            int64_t frame_stamp = sros::core::util::get_time_in_us(); //  // 数据开始读取前的时间戳
            std::shared_ptr<StereoPoints> filter_points(new StereoPoints);
            StereoPoints stereo_points;
            int length,width;

            if(!thread_running_){
                break;
            }

            if (dabai_driver_->getStereoData(length,  width, stereo_points)){
                if (enable_state_) {
                    dabaiPro_detector_.computeObstaclePoints(stereo_points, length, width, *filter_points);
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
                frame_error_count = 0;
            } else {
                LOG(INFO) << "Dabaipro get frame data failed";
                if(frame_error_count++ > 3) {
                    setStateOpenFailed();
                }
            }
        } else {
            LOG(INFO) << "dabaipro is not opened!";
        }
        usleep(sleep_in_40_ms);
    }
    thread_running_ = false;
    LOG(INFO) << "thread exit";
}

void DabaiCameraDevice::convertToRecordData(const StereoPoints& stereo_points,int length,int width,serialization::PointImage& point_image) {
    auto vertices = stereo_points.points;
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


void DabaiCameraDevice::computeRackLegs(const double rack_length, const double rack_width, std::vector<Eigen::Vector2d> &rack_legs) {
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

bool DabaiCameraDevice::near(const Eigen::Vector2d &curr_point, const Eigen::Vector2d &other_point, double dist_thresh) {
    return (curr_point - other_point).norm() < dist_thresh;
}


void DabaiCameraDevice::convertToObaPoints(const std::shared_ptr<StereoPoints> &origin_points, sros::core::ObstacleMsg_ptr &filtered_points)
{
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
    std::vector<Eigen::Vector2d> legs_in_dabaipro;
    for (auto &leg : rack_legs) {
        legs_in_dabaipro.push_back(real_tf * leg);
    }
    const double rack_wheel_rotate_radius = para_->rack_wheel_rotate_radius;
    for (auto &point : origin_points->points) {
        if (do_filter_legs) {
            bool find_legs = false;
            for (auto &leg : legs_in_dabaipro) {
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

void DabaiCameraDevice::setCameraSN(const std::string &sn) {
    camera_sn_ = sn;
}

void DabaiCameraDevice::setCameraUri(const std::string &uri) {
    device_uri_ = uri;
    if(dabai_driver_) {
        dabai_driver_->setOBDeviceUri(uri);
    } else {
        dabai_driver_.reset(new astra_dev::AstraDriver(uri));
    }
}

bool DabaiCameraDevice::registerFrameDataCallback() {
    if (thread_running_) {
        LOG(WARNING) << "handle camera thread already running! no need to register callback";
        return false;
    }
    LOG(INFO) << "init & start Frame callback";
    if (!dabai_driver_->initFrameCallback()) {
        LOG(ERROR) << "init Callback error!!!";
        return false;
    }
    // set callback function
    dabai_driver_->setFrameDataCallback(std::bind(&DabaiCameraDevice::handleCameraFrame, this, std::placeholders::_1, \
                                      std::placeholders::_2, std::placeholders::_3));
    frame_count_ = 0;
    return true;
}

void DabaiCameraDevice::handleCameraFrame(int frameWidth,  int frameHeight, StereoPoints stereo_points)
{
    idle_wait_state_ = false;
    keepAlive();

    int64_t frame_stamp = sros::core::util::get_time_in_us(); // 数据获取完成后的时间戳
    std::shared_ptr<StereoPoints> filter_points(new StereoPoints);
    int length = frameWidth;
    int width = frameHeight;
    // 能进此函数表示设备的 stream 已经被打开
    if (length && width) { // 判断数据是否有效
            if (enable_state_) {
                dabaiPro_detector_.computeObstaclePoints(stereo_points, length, width, *filter_points);
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
                    if ((frame_count_ % 30) == 0) {
                        LOG(INFO) << camera_sn_ << " Dabaipro sendMsg point_cloud size"  << img_with_stamp->points->point_cloud.size();
                    }
                }
            } else {
                if ((frame_count_ % 30) == 0) {
                    LOG(INFO) << camera_sn_ << " Dabaipro camera StereoPoints not enabled";
                }
            }
            if (isRecording()) {
                serialization::PointImage image(length, width);
                convertToRecordData(stereo_points, length, width, image);
                recordData(image);
            }
    } else {
        if ((frame_count_ % 30) == 0) {
            LOG(INFO) << "Dabaipro camera 30 frames ignored. Uri: " << device_uri_;
        }
    }
    frame_count_ ++;
}
