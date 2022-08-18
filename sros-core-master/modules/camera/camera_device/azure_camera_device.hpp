//
// Created by lfc on 2020/11/16.
//

#ifndef SROS_AZURE_CAMERA_DEVICE_HPP
#define SROS_AZURE_CAMERA_DEVICE_HPP
#include "base_camera_device.hpp"
#include "../azure_camera.h"

namespace camera{
class AzureCameraDevice: public BaseCameraDevice {
 public:
    AzureCameraDevice(std::string device_address, ImgMsgCallback callback, const std::string &name,
        sros::device::DeviceID device_id)
    : camera_name_(device_address),BaseCameraDevice(callback, name, device_id, sros::device::DEVICE_COMM_INTERFACE_TYPE_USB_0,
                     sros::device::DEVICE_MOUNT_SROS) {
        disableCamera();
        creatHandleCameraThread();
    }

    virtual bool open() {
        disableCamera();
        opened_ = true;
        setStateInitialization();
        const int try_times_thresh = 1;
        int curr_time = 0;
        while (curr_time++ < try_times_thresh) {
            if (initCamera()) {
                LOG(INFO) << "succesfully to open AZURE camera:" << getName();
                setStateOK();
                break;
            } else {
                LOG(INFO) << "cannot open ! try again! curr time is:" << camera_name_;
            }
        }
        if (!thread_running_) {
            creatHandleCameraThread();
        }
        if (isOpened()) {
            return true;
        } else {
            LOG(INFO) << "cannot open:" << getName();
            setStateOpenFailed();
            return false;
        }
    }

    virtual bool isOpened() { return camera_azure_.isOpened(); }

    virtual void close() {
        opened_ = false;
        enable_state_ = false;
        camera_azure_.close_device();
    }

    virtual bool enableCamera() {
        LOG(INFO) << "will enable camera!";
        if(!camera_azure_.setSaturation(saturation_)){
            LOG(INFO) << "cannot set saturation! will close camera and reopen!";
            camera_azure_.close_device();
            if (initCamera()) {
                if(!camera_azure_.setSaturation(saturation_)){
                    LOG(INFO) << "second time set saturation failed!";
                }
            }else{
                LOG(INFO) << "cannot initial camera!";
            }
        }
        return wakeUpThread();
    }

    virtual bool disableCamera() {
        LOG(INFO) << "will disable camera!";
        enable_state_ = false;
        return true;
    }

    bool ok() { return opened_; }

    virtual bool isEnabled() { return enable_state_; }

 private:
    void creatHandleCameraThread() { boost::thread(boost::bind(&AzureCameraDevice::handleAZURECamera, this)); }

    void handleAZURECamera() {
        thread_running_ = true;
        fps_ = 40;
        const int64_t fps_inverse_in_ms = (1000) / fps_;  // ms
        int64_t last_publish_time = 0;
        int64_t check_gpio_state_incre = 0;
        while (ok()) {
            if (!enable_state_) {  //在未启用摄像头的状态下,AZURE需要每隔1s,获取一次摄像头数据,如果读不到,则对外报打开失败提醒;这里通过超时等待实现
                std::unique_lock<std::mutex> lock(condition_mutex_);
                idle_wait_state_ = true;
                if (!enable_state_) {
                    auto state = condition_.wait_for(lock, std::chrono::seconds(1));
                }
            }

            idle_wait_state_ = false;
            if (!camera_azure_.isOpened()) {  //一旦出现掉线,立即重连
                if (!initCamera()) {
                    LOG(INFO) << "err to open camera " << getName() << "will sleep 1s and reopen!";
                    sleep(1);
                    continue;  //重新打开一遍
                }
            }
            auto get_frame_beg = sros::core::util::get_time_in_ms();
            cv::Mat frame;
            int64_t frame_stamp = 0;
            camera_azure_.readFrame(frame, frame_stamp);

            auto curr_time = sros::core::util::get_time_in_ms();
            if ((curr_time - get_frame_beg) > time_out_500ms_) {
                LOG(WARNING) << "camera " << getName() << "get frame timeout!";
            }
            if (!frame.empty()) {
                keepAlive();
                if (enable_state_) {  //如果当前状态为disable,则不外发数据
                    auto delta_time = (curr_time - last_publish_time);
                    if (delta_time > fps_inverse_in_ms) {
                        last_publish_time = curr_time;
                        camera_azure_.decode(frame);
                        ImgWithStampInfoPtr img_with_stamp(new ImgWithStampInfo);
                        img_with_stamp->camera_name = getName();
                        img_with_stamp->topic_name = "TOPIC_COLOR";
                        img_with_stamp->stamp = frame_stamp;
                        img_with_stamp->img = frame;
                        if (sendMsg) {
                            sendMsg(img_with_stamp);
                        }
                    } else if (delta_time < 0) {
                        LOG(WARNING) << "detect time flow back! will not publish img!";
                    }
                }
            } else {
                LOG(WARNING) << "frame empty! will reopen!";
                camera_azure_.close_device();
                setStateOpenFailed();
            }
        }
        thread_running_ = false;
    }

    bool initCamera() {
        bool is_opened = false;
        is_opened = camera_azure_.open_device(camera_name_);
        if (!is_opened) {  //打开相机
            setStateOpenFailed();
            LOG(WARNING) << "CAMERA_AZURE open failed!" << camera_name_;
            return false;
        } else {
            if (camera_azure_.initCamera()) {  //初始化相机
                setStateOK();
            } else {
                camera_azure_.close_device();
                setStateInitialFailed();
                LOG(WARNING) << "CAMERA_AZURE init failed!" << camera_name_;
                return false;
            }
        }

        LOG(INFO) << "CV_CAP_PROP_SATURATION " << camera_azure_.getSaturation();
        camera_azure_.setSaturation(saturation_);
        LOG(INFO) << "CV_CAP_PROP_SATURATION " << camera_azure_.getSaturation();

        if (!camera_azure_.initCapture()) {  //初始化采集失败,直接关闭相机
            camera_azure_.close_device();
            setStateInitialFailed();
            LOG(INFO) << "CAMERA_AZURE init capture failed!";
            return false;
        }
        return true;
    }

    bool wakeUpThread() {
        enable_state_ = true;
        int wait_count = 0;
        while (idle_wait_state_) {
            condition_.notify_one();
            if (idle_wait_state_) {
                sleepFor10ms();
                if (wait_count++ > 50) {
                    LOG(INFO) << "cannot wake up thread! will return false!";
                    return false;
                }
            }
        }
        return true;
    }

    virtual void setParameter(const std::string &name, const double value) {}

    virtual void setParameter(const std::string &name, const std::string value) {}

 private:
    AzureCamera camera_azure_;
    std::string camera_name_;

    bool opened_ = true;
    bool enable_state_ = false;
    bool idle_wait_state_ = true;
    bool thread_running_ = false;

    std::mutex condition_mutex_;
    std::condition_variable_any condition_;

 private:
    const double saturation_ = 30;
    const int time_out_500ms_ = 500;
    const int min_fps_ = 40;

};

}

#endif  // SROS_AZURE_CAMERA_DEVICE_HPP
