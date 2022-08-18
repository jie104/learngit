//
// Created by lfc on 2021/3/15.
//

#ifndef SROS_CQ_CAMERA_DEVICE_HPP
#define SROS_CQ_CAMERA_DEVICE_HPP

#include <core/src.h>
#include <core/state.h>
#include <glog/logging.h>
#include <boost/thread.hpp>
#include <string>
#include "../CqUsbCam/CqUsbCam.h"
#include "base_camera_device.hpp"

namespace camera {
    class CqCameraDevice : public BaseCameraDevice {
    public:
        CqCameraDevice(std::string device_address, ImgMsgCallback callback, const std::string &name,
                       sros::device::DeviceID device_id)
                : camera_name_(device_address),
                  BaseCameraDevice(callback, name, device_id, sros::device::DEVICE_COMM_INTERFACE_TYPE_USB_0,
                                   sros::device::DEVICE_MOUNT_SROS) {

        }

        ~CqCameraDevice(){
            close();
        }

        virtual bool open() {

            CqUsbCam_open_flag = false;
            disableCamera();
            setStateInitialization();
            const int try_times_thresh = 1;
            int curr_time = 0;
            while (curr_time++ < try_times_thresh) {
                CqUsbCam_open_flag = initCamera(CqUsbCam_default_Id);
                if (CqUsbCam_open_flag) {
                    LOG(INFO) << "open CqUsbCam Successful";
                    setStateOK();
                    break;
                } else {
                    LOG(INFO) << "cannot open ! try again!";
                }
            }

            if (!isOpened()) {
                LOG(ERROR)<<"open CqUsbCam Failure: cannot open CqUsbCam "<<CqUsbCam_default_Id;
                setStateOpenFailed();
                return false;
            }


            int read_result = cam0.StartCap(g_height, g_width, std::bind(&CqCameraDevice::handleCqCamera, this, std::placeholders::_1));
            LOG(INFO)<<"open read_result : "<<read_result;
            if(read_result != 0) {
                CqUsbCam_read_flag = false;
                LOG(ERROR) << "open CqUsbCam Failure: CqUsbCam readCap failure";
                return false;
            }
            CqUsbCam_read_flag = true;
            initCq = true;
            LOG(INFO) << "open CqUsbCam Successful!";

            return true;
        }

        virtual bool isOpened() {
            return CqUsbCam_open_flag;
        }

        virtual void close() {
            CqUsbCam_open_flag = false;
            if(this->CqUsbCam_read_flag){
                cam0.StopCap();
                CqUsbCam_read_flag = false;
            }
            disableCamera();
            cam0.ReleaseInterface();
            cam0.CloseUSB();

            LOG(INFO) << "close CqUsbCam!";
        }

        virtual bool enableCamera() {
            if (!CqUsbCam_open_flag || initCq) {
                initCq = false;
                setStateInitialization();
                CqUsbCam_open_flag = initCamera(CqUsbCam_default_Id);
                if (!CqUsbCam_open_flag) {
                    LOG(ERROR)<<"enable CqUsbCam Failure: cannot open CqUsbCam "<<CqUsbCam_default_Id;
                    return false;
                }
                setStateOK();
            }

            if(CqUsbCam_read_flag) {
                cam0.StopCap();
            }

            enable_state_ = true;
            int read_result = cam0.StartCap(g_height, g_width, std::bind(&CqCameraDevice::handleCqCamera, this, std::placeholders::_1));
            LOG(INFO)<<"enable read_result : "<<read_result;
            if(read_result != 0){
                CqUsbCam_read_flag = false;
                LOG(ERROR) << "enable CqUsbCam Failure: CqUsbCam readCap failure";
                return false;
            }
            CqUsbCam_read_flag = true;

            LOG(INFO) << "enable CqUsbCam Successful!";

            return true;
        }

        virtual bool disableCamera() {
//            if(this->CqUsbCam_read_flag){
//                cam0.StopCap();
//                CqUsbCam_read_flag = false;
//            }

            enable_state_ = false;

            LOG(INFO) << "disable CqUsbCam!";
            return true;
        }

        virtual bool isEnabled(){
            return enable_state_;
        };

        void handleCqCamera(void *frameData) {
            CImgFrame *m_frame = (CImgFrame *) frameData;
            cv::Mat frame(g_height, g_width, CV_8UC1, (unsigned char *) m_frame->m_imgBuf);
            cv::Mat image_data = frame.clone();

            //添加发送msg,但使用假的stamp
            ImgWithStampInfoPtr img_with_stamp(new ImgWithStampInfo);
            img_with_stamp->camera_name = getName();
            img_with_stamp->topic_name = "TOPIC_COLOR";
            img_with_stamp->stamp = 0;//TODO stamp并没有编写
            img_with_stamp->img = image_data;

            if(!image_data.empty()){
//                LOG(INFO)<<enable_state_;
                keepAlive();
                if(enable_state_){
                    if (sendMsg) {
                        sendMsg(img_with_stamp);
                    }
                }
            } else{
                LOG(INFO)<<"test CqUsbCam image data is empty!";
                setStateOpenFailed();
                CqUsbCam_open_flag = false;
            }
        }

        bool initCamera(int CqUsbCam_Id){
            cam0.SelectSensor(sensor);
            int usbCnt = CCqUsbCam::OpenUSB();
            LOG(INFO) << "CqUsbCam Count: " << usbCnt;

            if (usbCnt <= 0) {
                LOG(ERROR) << "CqUsbCam Exiting ...";
                return false;
            }

            cam0.ClaimInterface(CqUsbCam_Id);

            return true;
        }


    private:
        void checkspeed() {
            unsigned int speed = 0;
            cam0.GetUsbSpeed(speed);
            if (speed == LIBUSB_SPEED_SUPER) {
                LOG(INFO) << "USB 3.0 device found on cam0!";
                cam0.SendUsbSpeed2Fpga(LIBUSB_SPEED_SUPER);
            } else if (speed == LIBUSB_SPEED_HIGH) {
                LOG(INFO) << "USB 2.0 device found on cam0!";
                cam0.SendUsbSpeed2Fpga(LIBUSB_SPEED_HIGH);
            } else {
                LOG(INFO) << "Device speed unknown on cam0!";
            }

        }

        int CqUsbCam_default_Id = 0;
        bool CqUsbCam_open_flag = false;
        bool CqUsbCam_read_flag = false;
        bool enable_state_ = false;
        bool initCq = false;

        CCqUsbCam cam0;
        std::string camera_name_;
        const std::string sensor = "MT9V034";
        unsigned int g_width = 752;
        unsigned int g_height = 480;
    };
}
#endif  // SROS_CQ_CAMERA_DEVICE_HPP
