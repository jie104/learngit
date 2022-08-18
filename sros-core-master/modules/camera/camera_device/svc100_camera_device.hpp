//
// Created by lfc on 19-10-26.
//

#ifndef SROS_SVC100_CAMERA_DEVICE_HPP
#define SROS_SVC100_CAMERA_DEVICE_HPP

#include <core/src.h>
#include <core/state.h>
#include <glog/logging.h>
#include <boost/thread.hpp>
#include "../camera.h"
#include "base_camera_device.hpp"

namespace camera {
class SVC100CameraDevice : public BaseCameraDevice {
 public:
    SVC100CameraDevice(std::string device_address, bool use_sn_to_open, ImgMsgCallback callback, const std::string &name,
                       sros::device::DeviceID device_id)
        : camera_name_(device_address),use_sn_to_open_(use_sn_to_open),
          BaseCameraDevice(callback, name, device_id, sros::device::DEVICE_COMM_INTERFACE_TYPE_USB_0,
                           sros::device::DEVICE_MOUNT_SROS) {
        if (use_sn_to_open) {
            setSerialNo(device_address);
        }
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
                LOG(INFO) << "succesfully to open SVC100 camera:" << getName();
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

    virtual bool isOpened() { return camera_v4l2_.isOpened(); }

    virtual void close() {
        opened_ = false;
        enable_state_ = false;
        camera_v4l2_.close_device();
    }

    virtual bool enableCamera() {
        LOG(INFO) << "will enable camera!";
        toggleLightState(true);
        if(!camera_v4l2_.setSaturation(saturation_)){
            LOG(INFO) << "cannot set saturation! will close camera and reopen!";
            camera_v4l2_.close_device();
            if (initCamera()) {
                if(!camera_v4l2_.setSaturation(saturation_)){
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
        toggleLightState(false);
        enable_state_ = false;
        return true;
    }

    bool ok() { return opened_; }

    virtual bool isEnabled() { return enable_state_; }

 private:
    void creatHandleCameraThread() { boost::thread(boost::bind(&SVC100CameraDevice::handleSVC100Camera, this)); }

    void handleSVC100Camera() {
        thread_running_ = true;
        if (fps_ < 40) {
            LOG(INFO) << "fps is" << fps_ << " lower than 40! will reset to" << min_fps_;
            fps_ = min_fps_;
        }
        const int64_t fps_inverse_in_ms = (1000) / fps_;  // ms
        int64_t last_publish_time = 0;
        int64_t check_gpio_state_incre = 0;
        while (ok()) {
            if (!enable_state_) {  //在未启用摄像头的状态下,SVC100需要每隔1s,获取一次摄像头数据,如果读不到,则对外报打开失败提醒;这里通过超时等待实现
                std::unique_lock<std::mutex> lock(condition_mutex_);
                idle_wait_state_ = true;
                if (!enable_state_) {
                    auto state = condition_.wait_for(lock, std::chrono::seconds(1));
                    toggleLightState(enable_state_);
                }
            }else{
                if (check_gpio_state_incre++ % 50 == 0) {
                    toggleLightState(enable_state_);//防止出现快速控制io,导致io没有及时更新
                }
            }

            idle_wait_state_ = false;
            if (!camera_v4l2_.isOpened()) {  //一旦出现掉线,立即重连
                if (!initCamera()) {
                    LOG(INFO) << "err to open camera " << getName() << "will sleep 1s and reopen!";
                    sleep(1);
                    continue;  //重新打开一遍
                }
            }
            auto get_frame_beg = sros::core::util::get_time_in_ms();
            cv::Mat frame;
            int64_t frame_stamp = 0;
            camera_v4l2_.readFrame(frame, frame_stamp, false);

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
                        camera_v4l2_.decode(frame);
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
                camera_v4l2_.close_device();
                setStateOpenFailed();
            }
        }

        thread_running_ = false;
    }

    bool initCamera() {
        bool is_opened = false;
        if (use_sn_to_open_) {
            is_opened = camera_v4l2_.openParticularSVC100(camera_name_);
        }else{
            is_opened = camera_v4l2_.open_device(camera_name_);
        }
        if (!is_opened) {  //打开相机
            setStateOpenFailed();
            LOG(WARNING) << "CAMERA_SVC100 open failed!" << camera_name_;
            return false;
        } else {
            if (camera_v4l2_.initCamera()) {  //初始化相机
                setStateOK();
            } else {
                camera_v4l2_.close_device();
                setStateInitialFailed();
                LOG(WARNING) << "CAMERA_SVC100 init failed!" << camera_name_;
                return false;
            }
        }

        LOG(INFO) << "CV_CAP_PROP_SATURATION " << camera_v4l2_.getSaturation();
        camera_v4l2_.setSaturation(saturation_);
        LOG(INFO) << "CV_CAP_PROP_SATURATION " << camera_v4l2_.getSaturation();

        if (!camera_v4l2_.initCapture()) {  //初始化采集失败,直接关闭相机
            camera_v4l2_.close_device();
            setStateInitialFailed();
            LOG(INFO) << "CAMERA_SVC100 init capture failed!";
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

    void toggleLightState(bool enable) {
        int led_value = svc100_led_on_value_;
        if (!enable) {
            led_value = !led_value;
        }

        int svc100_led_bits = control_port_;
        // auto bits_mask = (uint8_t) ~(0x01 << svc100_led_bits);
        // int cur_led_value = led_value;                   // 0表示打开补光灯
        // uint8_t ori_output_value = g_state.gpio_output;  // 保存当前gpio output值
        // auto new_output_value = (uint8_t)((ori_output_value & bits_mask) | (cur_led_value << svc100_led_bits));
        // if (new_output_value != ori_output_value) {
        //     LOG(INFO) << "ori_output_value = " << (int)ori_output_value;
        //     LOG(INFO) << "bits_mask = " << (int)bits_mask;
        //     LOG(INFO) << "new_output_value = " << (int)new_output_value;
        //     src_sdk->setGPIOOuput(new_output_value);
        // }

        uint8_t value = 1 << svc100_led_bits;
        if(led_value == 0) {
            src_sdk->setGPIOOuputBits(value, 0);
        } else {
            src_sdk->setGPIOOuputBits(0, value);
        }
    }

    virtual void setParameter(const std::string &name, const double value) {
        if (name == "saturation") {
            if(saturation_!=value){
                LOG(INFO) << "name :" << name << "," << value;
                saturation_ = value;
            }
        }else{
            LOG(INFO) << "name :" << name << "," << value;
        }
    }

    virtual void setParameter(const std::string &name, const std::string value) {}

    virtual void setParameter(const std::string &name, const int value) {
        LOG(INFO) << "name :" << name << "," << value;
        if (name == "svc100_led_on_value") {
            svc100_led_on_value_ = value;
        } else if (name == "svc100_led_bits") {
            control_port_ = value;
        }
    }

 private:
    V4L2Camera camera_v4l2_;
    std::string camera_name_;

    bool opened_ = true;
    bool enable_state_ = false;
    bool idle_wait_state_ = true;
    bool thread_running_ = false;

    std::mutex condition_mutex_;
    std::condition_variable_any condition_;

    int svc100_led_on_value_ = 0;
    int control_port_ = 0;
    bool use_sn_to_open_ = true;
    double saturation_ = 40;

 private:
    const int time_out_500ms_ = 500;
    const int min_fps_ = 40;
};

}  // namespace camera

#endif  // SROS_SVC100_CAMERA_DEVICE_HPP
