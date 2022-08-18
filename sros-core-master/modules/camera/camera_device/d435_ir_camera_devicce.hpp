//
// Created by duan on 2021/6/24.
//

#ifndef SROS_D435_IR_CAMERA_DEVICCE_H
#define SROS_D435_IR_CAMERA_DEVICCE_H

#include <core/src.h>
#include <core/state.h>
#include <boost/thread.hpp>
#include <condition_variable>
#include <mutex>
#include "../d435/d435_driver.hpp"
#include "base_camera_device.hpp"
namespace camera{
class D435IrCameraDevice : public BaseCameraDevice{
 public:
    D435IrCameraDevice(const std::string &device_sn,
                       ImgMsgCallback callback,
                       const std::string &name,
                       sros::device::DeviceID device_id,
                       const std::string& stereo_camera_type) :
        device_sn_(device_sn),
        stereo_camera_type_(stereo_camera_type),
        BaseCameraDevice(callback, name, device_id,
                         sros::device::DEVICE_COMM_INTERFACE_TYPE_USB_0,
                         sros::device::DEVICE_MOUNT_SROS){
        d435_driver_.reset(new d435::D435Driver(device_sn_, stereo_camera_type_, true));
        setSerialNo(device_sn_);
    }

    virtual bool isOpened() { return d435_driver_->isOpened(); }

    virtual bool isEnabled() { return enable_state_; }

    virtual bool open(){
        ok_ = true;
        setStateInitialization();
        const int try_times_thres = 1;
        int curr_time = 0;
        while (curr_time++ < try_times_thres){
            if(d435_driver_->init()){
                LOG(INFO) << "succesfully to open D435 camera!";
                setStateOK();
                break;
            }else{
                LOG(INFO) << "cannot open D435! try again! curr time is:" << curr_time;
            }
        }

        if(!thread_running_){
            creatHandleCameraThread();
        }

        if(d435_driver_->isOpened()){
            return true;
        }else{
            LOG(INFO) << "cannot open D435 camera!";
            setStateOpenFailed();
            return false;
        }
    }

    virtual void close(){
        ok_ = false;
        enable_state_ = false;
        d435_driver_.reset(new d435::D435Driver(device_sn_,stereo_camera_type_));
    }

    virtual bool enableCamera(){
        LOG(INFO) << "will enable D435!";
        toggleLightState(true);
        return wakeUpThread();
    }

    bool wakeUpThread(){
        enable_state_ = true;
        int wait_count = 0;
        while (idle_wait_state_){
            condition_.notify_one();
            if(idle_wait_state_){
                sleepFor10ms();
                if(wait_count++ > 50){
                    LOG(INFO) << "cannot wake up thread! will return false!";
                    return false;
                }
            }
        }
        return true;
    }

    virtual bool disableCamera() {
        LOG(INFO) << "will disable camera!";
        toggleLightState(false);
        enable_state_ = false;
        return true;
    }

    void getIrIntrinsic(std::vector<double> &intrinsic, std::vector<double> &distort) {
        intrinsic.clear();
        distort.clear();
        d435_driver_->getIrIntrinsic(intrinsic, distort);
    }

 private:
    void creatHandleCameraThread() { boost::thread(boost::bind(&D435IrCameraDevice::handleD435Camera, this));}

    void handleD435Camera(){
        thread_running_ = true;

        int64_t check_gpio_state_incre = 0;

        while (ok_){
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

            if(d435_driver_){
                if(!d435_driver_->isOpened()) {
                    LOG(INFO) << "have not opened D435! will reopen!";
                    if (d435_driver_->init()) {
                        setStateOK();
                        LOG(INFO) << "successfully to open D435!";
                    } else {
                        LOG(INFO) << "cannot open D435!";
                    }
                }

                if(d435_driver_->isOpened()){
                    int64_t frame_stamp = sros::core::util::get_time_in_us();
                    cv::Mat ir_image;
                    if(d435_driver_->getIRImage(ir_image,frame_stamp)){
                        if(enable_state_){
                            ImgWithStampInfoPtr img_with_stamp(new ImgWithStampInfo);
                            img_with_stamp->topic_name = "TOPIC_COLOR";
                            img_with_stamp->camera_name = getName();
                            img_with_stamp->stamp = frame_stamp;
                            img_with_stamp->img = ir_image.clone();
                            img_with_stamp->use_mat_info = true;
                            if(sendMsg){
                                sendMsg(img_with_stamp);
                            }
                        }

                        keepAlive();
                    }else{
                        LOG(INFO) << "D435 has closed! will reopen it!";
                        setStateOpenFailed();
                    }
                } else {
                    LOG(INFO) << "d435 is not opened!";
                }
            }
        }

        thread_running_ = false;
    }

    void toggleLightState(bool enable) {
        //TODO:需要沟通是否是用一个GPIO
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

    std::string device_sn_;
    std::shared_ptr<d435::D435Driver> d435_driver_;

    bool idle_wait_state_ = true;
    bool thread_running_ = false;
    bool enable_state_ = false;
    bool ok_ = false;

    std::mutex condition_mutex_;
    std::condition_variable_any condition_;

    //红外灯控制变量
    int svc100_led_on_value_ = 0;
    int control_port_ = 0;

    std::string stereo_camera_type_;

};

}


#endif  // SROS_D435_IR_CAMERA_DEVICCE_H
