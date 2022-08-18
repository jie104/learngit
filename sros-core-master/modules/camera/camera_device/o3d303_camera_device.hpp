//
// Created by lfc on 2019/12/18.
//

#ifndef SROS_O3D103_CAMERA_DEVICE_HPP
#define SROS_O3D103_CAMERA_DEVICE_HPP
#include <glog/logging.h>
#include <boost/thread.hpp>
#include "base_camera_device.hpp"
#include "ifm3d/driver/ifm3d_driver.hpp"

namespace camera {
class O3d303CameraDevice : public BaseCameraDevice {
 public:
    O3d303CameraDevice(std::string device_address, ImgMsgCallback callback, const std::string &name,
                       sros::device::DeviceID device_id)
        : device_address_(device_address),
          BaseCameraDevice(callback, name, device_id, sros::device::DEVICE_COMM_INTERFACE_TYPE_USB_0,
                           sros::device::DEVICE_MOUNT_SROS) {
        disableCamera();
        ifm_driver.setIPAddress(device_address);
        getFrame_ok_count = 0;
    }

    virtual ~O3d303CameraDevice() {}

    virtual bool open() {
        LOG(INFO) << "ifm3d open ...";
        disableCamera();
        sleep(1);
        if (ifm_driver.tryInit()) {
            setStateOK();
            setSerialNo(ifm_driver.getSerialNumberStr());
            setModelNo(ifm_driver.getDeviceNameStr());
            opened_ = true;
            LOG(INFO) << "ifm3d successfully to initialize camera:" << device_address_;
        }else{
            setStateInitialFailed();
            LOG(INFO) << "ifm3d cannot initialize camera:" << device_address_;
        }
        if(!thread_running_){
            creatHandleCameraThread();
        }
        return true;
    }

    virtual bool isOpened() { return opened_; }

    virtual void close() {
        LOG(INFO) << "ifm3d will close camera!";
        opened_ = false;
        enable_state_ = false;
    }

    virtual bool disableCamera() {
        LOG(INFO) << "ifm3d will disable camera!";
        enable_state_ = false;
        return true;  // 如果不返回, 程序崩溃 double free or corruption (out) 
    }

    virtual bool enableCamera() {
        LOG(INFO) << "ifm3d will enable camera!";
        return wakeUpThread();
    }

    virtual bool isEnabled() { return enable_state_; }

    virtual void setParameter(const std::string &name, const std::string value) {
        if(name.compare("o3d303_calibr") == 0 && value.compare("start") == 0) {
            LOG(INFO) << "ifm3d set Calibr Params";
            try {
                setStartCalibrParam();
            }
            catch (...) {
                LOG(INFO) << "ifm3d set Ifm3d Calibr Param failed !!!";
            }
        }
        else if(name.compare("o3d303_calibr") == 0 && value.compare("stop") == 0) {
            LOG(INFO) << "ifm3d set Work Params!!!";
            try {
                setStopCalibrParam();
            }
            catch (...) {
                LOG(INFO) << "ifm3d set Ifm3d Work Param failed !!!";
            }
        }
    }

    /**
     * @brief 标定开始，设置o3d相机标定工作参数.
     */
    void setStartCalibrParam() {
        setParamStateStart();
        if(!ifm_driver.setAvtiveAppImagerIndex(ifm::DEFAULT_CALIB_INDEX)) {
            LOG(ERROR) << "ifm3d setStartCalibrParam failed !!!";
        }
        setParamStateStop();
        LOG(INFO)<<"ifm3d calibr started";
    }

    /**
     * @brief 标定结束，设置o3d相机恢复正常工作参数.
     */
    void setStopCalibrParam() {
        setParamStateStart();
        if(!ifm_driver.setAvtiveAppImagerIndex(ifm::DEFAULT_NORMAL_INDEX)) {
            LOG(ERROR) << "setStopCalibrParam failed !!!";
        }
        setParamStateStop();
        LOG(INFO)<<"ifm3d calibr stopped";
    }

 private:
    bool wakeUpThread() {
        LOG(INFO)<<"ifm3d wakeUpThread";
        enable_state_ = true;
        int wait_count = 0;
        while (idle_wait_state_) {
            condition_.notify_one();
            if (idle_wait_state_) {
                sleepFor10ms();
                if (wait_count++ > 50) {
                    LOG(INFO) << "ifm3d cannot wake up thread! will return false!";
                    return false;
                }
            }
        }
        return true;
    }

    bool ok() { return opened_; }

    void creatHandleCameraThread() { boost::thread(std::bind(&O3d303CameraDevice::handleO3DCamera, this)); }

    void handleO3DCamera() {
        thread_running_ = true;
        int64_t last_publish_time = 0;
        int initIfm3dParam_count = 0;
        int getFrame_error_count = 0;
        LOG(INFO) << "ifm3d handle device!";
        while (ok()) {
            if (!enable_state_) {  //在未启用摄像头的状态下,需要每隔3s,获取一次摄像头数据,如果读不到,则对外报打开失败提醒;这里通过超时等待实现
                std::unique_lock<std::mutex> lock(condition_mutex_);
                idle_wait_state_ = true;
                if (!enable_state_) {
                    auto state = condition_.wait_for(lock, std::chrono::seconds(3));
                }
            }
            if (!ifm_driver.opened()) {
                LOG(INFO) << "will reopen ifm3d device!";
                if(!ifm_driver.init()) {
                    continue;
                }
                setStateOK();
                setSerialNo(ifm_driver.getSerialNumberStr());
                setModelNo(ifm_driver.getDeviceNameStr());
                ifm_driver.setInitParamFlag(false);
                initIfm3dParam_count = 0;
                getFrame_ok_count = 0;
            }

            if(!ifm_driver.getInitParamFlag()) {
                if(initIfm3dParam_count <= 10) {
                    try {
                        ifm_driver.initIfm3dParam();
                        initIfm3dParam_count++;
                    }
                    catch (...) {
                        initIfm3dParam_count = 0;
                        LOG(INFO) << "ifm3d initIfm3dParam failed !!!";
                    }
                }
            }

            idle_wait_state_ = false;
            auto get_frame_beg = sros::core::util::get_time_in_ms();
            cv::Mat confidence_img, xyz_img, amplitude_img;
            uint64_t frame_stamp = 0;
            float fork_height = g_state.fork_height_encoder; // 获取叉车叉臂的高度
            if(!getParamState()) {
                if (ifm_driver.getFrame(frame_stamp, confidence_img, amplitude_img, xyz_img, fork_height)) {
                    auto curr_time = sros::core::util::get_time_in_ms();
                    if ((curr_time - get_frame_beg) > time_out_500ms_) {
                        LOG(WARNING) << "ifm3d camera " << getName() << "get frame timeout!";
                    }
                    if (!confidence_img.empty() && !xyz_img.empty()) {
                        keepAlive();
                        getFrame_error_count = 0;
                        if (enable_state_) {  //如果当前状态为disable,则不外发数据
                            if (sendMsg) {
                                ImgWithStampInfoPtr img(new ImgWithStampInfo);
                                img->stamp = frame_stamp;
                                img->img = confidence_img;
                                img->xyz_img = xyz_img;
                                img->amplitude_img = amplitude_img;
                                img->topic_name = topic_name_;
                                img->camera_name = getName();
                                img->use_mat_info = true;
                                img->fork_height_encoder = fork_height;
                                sendMsg(img);
    //                            LOG(ERROR) << "send o3d3xx("<<frame_stamp<<") frame time point:" << sros::core::util::get_time_in_us();   // add by zhangxu at 2020/11/30
                            }
                        }
                        if((getFrame_ok_count++ % 30) == 0) {
                            LOG(WARNING) << "ifm3d Got " << getFrame_ok_count << " frames, state " << ((true == enable_state_) ? "enabled" : "not enabled!!!");
                        }
                    } else {
                        LOG(WARNING) << "ifm3d current frame empty!";
                    }
                } else {
                    getFrame_error_count ++;
                    LOG(ERROR) << "ifm3d cannot get frame! getFrame_error_count: " << getFrame_error_count;
                    if(getFrame_error_count == 3) {
                        LOG(ERROR) << "ifm3d cannot get frame! will close and reopen!";
                        ifm_driver.close();
                    } else if(getFrame_error_count > 5) {
                        LOG(ERROR) << "ifm3d cannot get frame! will close and reopen! and notify the user!";
                        getFrame_error_count = 0;
                        ifm_driver.close();
                        setStateOpenFailed();
                    }
                }
            }
            
        }
        thread_running_ = false;
    }

    void setParamStateStart() {
        set_param_state_ = true;
    }

    void setParamStateStop() {
        set_param_state_ = false;
    }

    bool getParamState() {
        return set_param_state_;
    }

 private:
    bool idle_wait_state_ = true;
    bool thread_running_ = false;
    bool opened_ = true;
    bool enable_state_;
    bool set_param_state_ = false;

    ifm::Ifm3dDriver ifm_driver;

    std::mutex condition_mutex_;
    std::condition_variable_any condition_;

    std::string device_address_;
    const std::string topic_name_ = "IFM3D_IMG";
    const int time_out_500ms_ = 500;

    uint32_t getFrame_ok_count;
};
}  // namespace camera

#endif  // SROS_O3D103_CAMERA_DEVICE_HPP
