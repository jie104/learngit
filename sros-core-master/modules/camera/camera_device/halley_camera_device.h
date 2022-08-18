//
// Created by ehl on 2022/07/07.
//
#ifndef SROS_DABAI_CAMERA_DEVICE_HPP
#define SROS_DABAI_CAMERA_DEVICE_HPP

#include <math.h>
#include <boost/thread.hpp>
#include "../Halley/halley_driver.h"
#include "../d435/camera_obstacle_detector.hpp"
#include "../stereo_module_para.hpp"
#include "base_camera_device.hpp"

namespace camera {

class DabaiCameraDevice : public BaseCameraDevice {
 public:
    DabaiCameraDevice(const std::shared_ptr<stereo::StereoModulePara> &para, std::string device_sn,std::string camera_sn,
                      ImgMsgCallback callback, const std::string &name, sros::device::DeviceID device_id);
    
    ~DabaiCameraDevice();

    bool ok();
    virtual bool isOpened();
    virtual bool isEnabled();

    bool calibrateCameraInstallErr(Eigen::Quaternionf initial_q, const double calibration_height, double &install_height,
                                   Eigen::Matrix3d &calibration_matrix) ;

    void updateCameraInstallErr(const Eigen::Matrix3d &install_matrix, double install_height);

    /*设置检测高度阈值*/
    void setHeightThresh(const double min_height,const double max_height);

    /* 设置检测距离阈值 */
    void setDetectDistThresh(const double max_dist);

    // 以设备唯一的序列号打开设备, 初始化参数, 并打开数据流
    virtual bool open();

    // 停止数据流, 关闭设备
    virtual void close();

    // 使能 获取相机的数据并处理
    virtual bool enableCamera();

    bool wakeUpThread();

    // 停止 获取相机的数据
    virtual bool disableCamera();

    std::string getCameraSN(void) {
        return camera_sn_;
    }
    
    std::string getCameraUri(void) {
        return device_uri_;
    }
    
    void setCameraSN(const std::string &sn);
    void setCameraUri(const std::string &uri);

    // 初始化相机数据回调机制
    bool registerFrameDataCallback();

    // 处理相机返回的数据
    void handleCameraFrame(int width_DepthData,  int height_DepthData, StereoPoints stereo_points);

 private:
    void creatHandleCameraThread();

    void handleDabaiProCamera();

    void convertToRecordData(const StereoPoints& stereo_points,int length,int width,serialization::PointImage& point_image);


    void computeRackLegs(const double rack_length, const double rack_width, std::vector<Eigen::Vector2d> &rack_legs);

    bool near(const Eigen::Vector2d &curr_point, const Eigen::Vector2d &other_point, double dist_thresh);

    void convertToObaPoints(const std::shared_ptr<StereoPoints> &origin_points, sros::core::ObstacleMsg_ptr &filtered_points);

 private:
    std::string device_uri_; // OBDevice Uri
    std::string camera_sn_; // OBDevice SN
    std::shared_ptr<astra_dev::AstraDriver> dabai_driver_;
    std::shared_ptr<stereo::StereoModulePara> para_;
    
    bool enable_state_ = false; // enable frame data process
    bool idle_wait_state_ = false;
    bool thread_running_ = false;

    std::mutex condition_mutex_;
    std::condition_variable_any condition_;

    uint32_t  m_devNum;
    openni::Array<openni::DeviceInfo> m_devBindList;

    threedim_camera::CameraObstacleDetector dabaiPro_detector_;

    uint32_t frame_count_;
};

}  // namespace camera

#endif
