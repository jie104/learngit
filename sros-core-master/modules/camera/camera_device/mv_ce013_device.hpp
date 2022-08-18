/**
 * @file MVCE013Device.h
 * @brief 简述文件内容
 *
 * 此处写文件的详细内容，注意需要与上下内容用空格分开。
 *
 * @author zhangxu@standard-robots.com
 * @date create date：2021/6/4
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef SROS_MVCE013Device_H
#define SROS_MVCE013Device_H

// INCLUDE
#include <utility>
#include <boost/thread.hpp>
#include "base_camera_device.hpp"
#include "mv_ce013/include/MvCameraControl.h"

#include "core/core.h"
#include "core/module.h"
#include "core/settings.h"
#include "core/msg/common_msg.hpp"
#include "core/msg/point_cloud_msg.hpp"

// CODE
namespace camera {

static uint64_t get_time_in_ns() {
    return static_cast<uint64_t>(std::chrono::high_resolution_clock::now().time_since_epoch().count());
}
void __stdcall cbException(unsigned int nMsgType, void* pUser);

/**
 * @description : TODO
 * @author      : zhangxu
 * @date        : 2021/6/4 下午9:15
 */
class MVCE013Device : public BaseCameraDevice {
public:
    MVCE013Device(unsigned int index, ImgMsgCallback callback, const std::string &name,sros::device::DeviceID device_id)
            : index_(index), BaseCameraDevice(std::move(callback), name, device_id, sros::device::DEVICE_COMM_INTERFACE_TYPE_ETH_1,
                     sros::device::DEVICE_MOUNT_SROS) {
        disableCamera();
    }

    virtual ~MVCE013Device() {
        close();
    };

    virtual bool open() {
        disableCamera();
        opened_ = true;
        setStateInitialization();

        const int try_times_thresh = 5;
        int curr_time = 0;
        while (curr_time++ < try_times_thresh) {
            if (init()) {
                LOG(INFO) << "succesfully to open MVCE013 camera!";
                setStateOK();
                break;
            } else {
                LOG(INFO) << "cannot open MVCE013! try again! curr time is:" << curr_time;
                usleep(1e5); // sleep 100ms again
                // return false;
            }
        } 

        opened_=true;
        setConnected(true);
        LOG(INFO) <<"MVCE013Device open sucess!";
        if(!thread_running_)
            creatHandleCameraThread();
        return true;
    }

    virtual void close() {
        LOG(INFO) << "closing！" ;
        int ret = MV_OK;
        // ch:停止取流 | en:Stop grab image
        opened_=false;
        enable_state_=false;
        ret = MV_CC_StopGrabbing(handle);
        if (MV_OK != ret)
        {
            LOG(INFO) << "Stop Grabbing fail! ret [0x" << ret << "]";
            //break;
        }

        // ch:关闭设备 | Close device
        ret = MV_CC_CloseDevice(handle);
        if (MV_OK != ret)
        {
            LOG(INFO) << "ClosDevice fail! ret [0x" << ret << "]";
            //break;
        }

        // ch:销毁句柄 | Destroy handle
        ret = MV_CC_DestroyHandle(handle);
        if (MV_OK != ret)
        {
            LOG(INFO) << "Destroy Handle fail! ret [0x" << ret << "]";
            //break;
        }
    }

    virtual bool enableCamera() {
        LOG(INFO) << "will enable camera!";
        return wakeUpThread();
    }

    virtual bool disableCamera() override {
        LOG(INFO) << "will disable camera!";
        enable_state_ = false;
        return true;
    }

    virtual bool isOpened() {
        return opened_;
    }

    virtual bool isEnabled() {
        return enable_state_;
    }

 private:
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

    bool ok() { return opened_; }

    void creatHandleCameraThread() { boost::thread(std::bind(&MVCE013Device::getCameraData, this)); }

    void getCameraData(){
        LOG(INFO) << "getCameraData thread start.";
        thread_running_ = true;
        int ret = MV_OK;
        MV_FRAME_OUT stOutFrame = {0};
       
        bool first=true;
        if (!sendMsg) {
            LOG(ERROR) << "sendMsg is NULL!";
            return ;
        }
        
        while(ok())
        {
            //掉线重连
            if(!conneted_){
                LOG(INFO) << "MVCE013 camera was unconneted,now reconnect";
                close();
                if(!open()){
                    LOG(INFO) << "MVCE013 camera failured to open, sleep 1s agent reconnect!";
                    sleep(1);
                    continue;
                }
            }
            keepAlive();
            if (!enable_state_) {  //在未启用摄像头的状态下,SVC100需要每隔1s,获取一次摄像头数据,如果读不到,则对外报打开失败提醒;这里通过超时等待实现
                std::unique_lock<std::mutex> lock(condition_mutex_);
                idle_wait_state_ = true;
                if (!enable_state_) {
                    auto state = condition_.wait_for(lock, std::chrono::seconds(1));
                }
            }
            idle_wait_state_ = false;

            if(!enable_state_) continue;

            memset(&stOutFrame, 0, sizeof(MV_FRAME_OUT));
            uint64_t start_time = sros::core::util::get_time_in_us();
            ret = MV_CC_GetImageBuffer(handle, &stOutFrame, 1000);
            if (ret == MV_OK) {
                //申请缓冲区
                if(!save_img_buf_)
                {
                    img_buf_size_ = stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight * 3;
                    save_img_buf_ = (unsigned char*)malloc(img_buf_size_);
                }
                memset(save_img_buf_,0,img_buf_size_);
                //彩色 or 黑白
                bool isMono=isMonoPixelFormat(stOutFrame.stFrameInfo.enPixelType);

                cv::Mat img_data;
                if (isMono)
                {
                    img_data = cv::Mat(stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nWidth, CV_8UC1, stOutFrame.pBufAddr);
                    ImgWithStampInfoPtr img_with_stamp(new ImgWithStampInfo);
                    img_with_stamp->camera_name = getName();
                    img_with_stamp->topic_name = "TOPIC_COLOR";
                    img_with_stamp->img = img_data.clone();
                    img_with_stamp->stamp = sros::core::util::get_time_in_us();
                    sendMsg(img_with_stamp);
                }
                else
                {
                    // 转换图像格式为BGR8
                    if(convert2BGR(stOutFrame,img_data))
                    {
                        ImgWithStampInfoPtr img_with_stamp(new ImgWithStampInfo);
                        img_with_stamp->camera_name = getName();
                        img_with_stamp->topic_name = "TOPIC_COLOR";
                        img_with_stamp->img = img_data.clone();
                        img_with_stamp->stamp = sros::core::util::get_time_in_us();
                        sendMsg(img_with_stamp);
                    }

                }
                //帧率平滑
                uint64_t end_time = sros::core::util::get_time_in_us();
                uint64_t cost_time = end_time-start_time;
                if( cost_time < 1000*1000/fps_ )
                    usleep((1000*1000/fps_)-cost_time);
                
            }
            else {
                LOG(INFO) << "No data[0x" <<std::hex<<ret << "]";
            }

            if(NULL != stOutFrame.pBufAddr)
            {
                ret = MV_CC_FreeImageBuffer(handle, &stOutFrame);
                if(ret != MV_OK)
                {
                    LOG(INFO) << "Free Image Buffer fail! ret [0x" <<std::hex<<ret << "]";
                }
            }
        }
        if(save_img_buf_ != NULL){
            free(save_img_buf_);
            save_img_buf_=NULL;
        }
        thread_running_ = false;
        LOG(INFO) << "getCameraData thread exit.";
    }

    bool init() {
        int ret = MV_OK;
    
        if(!enumDevice())
            return false;

        // 只打开第0号相机设备
        index_ = 0;

        // ch:选择设备并创建句柄
        if (index_ >= dev_list_.nDeviceNum) {
            LOG(ERROR) << "Intput error! Index:"<<index_<<",DeviceList.nDeviceNum:"<<dev_list_.nDeviceNum;
            return false;
        }

        ret = MV_CC_CreateHandle(&handle, dev_list_.pDeviceInfo[index_]);
        if (MV_OK != ret) {
            LOG(ERROR) << "Create Handle fail! ret [0x" << ret << "]";
            return false;
        }

          // ch:打开设备 | en:Open device
        ret = MV_CC_OpenDevice(handle);
        if (MV_OK != ret) {
            LOG(ERROR) << "Open Device fail! ret [0x"<<ret<<"]";
            return false;
        }

        // ch:探测网络最佳包大小(只对GigE相机有效)
        optimalPackageSize();

        // ch:设置触发模式为off
        if(!setTriggerMode())
            return false;

        // ch:设置曝光模式为自动曝光
        if(!setExposureMode(0))
            return false;

        // ch:设置曝光模式为手动曝光
        float exposure_time = 5000.f;
        ret = MV_CC_SetFloatValue(handle, "ExposureTime", exposure_time); // 自动曝光导致在黑暗情况下，或塑料材质二维码，成像，不均匀曝光时间us
        if (MV_OK != ret)
        {
            LOG(WARNING) << "Set ExposureTime fail! ret [0x" << ret << "]";
            return false;
        }

        // ch:配置相机心跳包模式:  开启，3000ms
        setHeartbeatMode(true, 3000);

        // 注册异常回调 
        ret = MV_CC_RegisterExceptionCallBack(handle, cbException, static_cast<void*>(this));
        if (MV_OK != ret)
        {
            LOG(ERROR)<< "MV_CC_RegisterExceptionCallBack fail! ret="<< ret;
            return false;
        }

        // ch:获取数据包大小 | en:Get payload size
        MVCC_INTVALUE stParam;
        memset(&stParam, 0, sizeof(MVCC_INTVALUE));
        ret = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
        if (MV_OK != ret)
        {
            LOG(ERROR) << "Get PayloadSize fail! ret [0x"<<ret<<"]";
            return false;
        }

        payloadsize_ = stParam.nCurValue;
        LOG(INFO) << "Get PayloadSize :"<<payloadsize_;

        // ch:开始取流 | en:Start grab image
        ret = MV_CC_StartGrabbing(handle);
        if (MV_OK != ret)
        {
            LOG(ERROR) << "Start Grabbing fail! ret [0x" << ret << "]";
            return false;
        }
        return true;
    }

    // ch:枚举设备 | en:Enum device
    bool enumDevice() {
        int ret = MV_OK;
        memset(&dev_list_, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        ret = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &dev_list_);
        if (MV_OK != ret) {
            LOG(INFO) << "Enum Devices fail! ret [0x" << ret << "]";
            return false;
        }

        if (dev_list_.nDeviceNum > 0) {
            for (unsigned int i = 0; i < dev_list_.nDeviceNum; i++) {
                LOG(INFO) << "[device "<< i << "]:";
                MV_CC_DEVICE_INFO *pDeviceInfo = dev_list_.pDeviceInfo[i];
                if (nullptr == pDeviceInfo) {
                    break;
                }
                PrintDeviceInfo(pDeviceInfo);
            }
        } else {
            LOG(INFO) << "Find No Devices!";
            return false;
        }
        return true;
    }

    // 探测网络最佳包大小(只对GigE相机有效)
    void optimalPackageSize() {
        if (dev_list_.pDeviceInfo[index_]->nTLayerType == MV_GIGE_DEVICE)
        {
            int nPacketSize = MV_CC_GetOptimalPacketSize(handle);
            if (nPacketSize > 0)
            {
                int ret = MV_CC_SetIntValue(handle,"GevSCPSPacketSize",nPacketSize);
                if(ret != MV_OK)
                {
                    LOG(INFO) << "Warning: Set Packet Size fail ret [0x" << ret << "]!";
                } else {
                    LOG(INFO) << "nPacketSize = " << nPacketSize;
                }
            }
            else {
                LOG(WARNING) << "Warning: Get Packet Size fail ret [0x" << nPacketSize << "]!";
            }
        }
    }

    // 设置触发模式为off
    bool setTriggerMode() {
        int ret = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
        if (MV_OK != ret)
        {
            LOG(WARNING) << "Set Trigger Mode fail! ret [0x" << ret << "]";
            return false;
        }
        return true;
    }

    // 设置曝光模式 默认使用自动曝光 (mode: 0:Off 1:Once 2:Continuous)
    bool setExposureMode(int mode = 2) {
        int ret = MV_CC_SetEnumValue(handle, "ExposureAuto", mode);
        if (MV_OK != ret)
        {
            LOG(WARNING) << "Set Exposure Mode fail! ret [0x" << ret << "]";
            return false;
        }
        return true;
    }

    bool setHeartbeatMode(bool enable, unsigned int heartbeatTimeout) {
        int ret = MV_CC_SetEnumValue(handle, "DeviceLinkHeartbeatMode", enable);
        if (MV_OK != ret){
            MV_CC_SetEnumValue(handle, "GevGVCPHeartbeatDisable", !enable);
            MV_CC_SetHeartBeatTimeout(handle, heartbeatTimeout);
            LOG(WARNING) << "Set Heartbeat Mode fail! ret [0x" << ret << "]";
            return false;
        }
        return true;
    }

    static bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
    {
        if (nullptr == pstMVDevInfo)
        {
            LOG(INFO) << "The Pointer of pstMVDevInfo is NULL!";
            return false;
        }
        if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE) {
            int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
            int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
            int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
            int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

            // ch:打印当前相机ip和用户自定义名字 | en:print current ip and user defined name
            LOG(INFO) << "Device Model Name: " << pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName;
            LOG(INFO) << "CurrentIp: " << nIp1 << "." << nIp2 << "." << nIp3 << "." << nIp4;
            LOG(INFO) << "UserDefinedName: " << pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName << std::endl;
        }
        else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
        {
            LOG(INFO) << "UserDefinedName: " << pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName;
            LOG(INFO) << "Serial Number: " << pstMVDevInfo->SpecialInfo.stUsb3VInfo.chSerialNumber;
            LOG(INFO) << "Device Number: " << pstMVDevInfo->SpecialInfo.stUsb3VInfo.nDeviceNumber << std::endl;
        }
        else
        {
            LOG(INFO) << "Not support.";
        }

        return true;
    }

    bool convert2BGR(MV_FRAME_OUT &outFrame,cv::Mat &img_data)
    {
        // 转换图像格式为BGR8
        MV_CC_PIXEL_CONVERT_PARAM stConvertParam = { 0 };
        stConvertParam.nWidth = outFrame.stFrameInfo.nWidth;             
        stConvertParam.nHeight = outFrame.stFrameInfo.nHeight;           
        stConvertParam.pSrcData = outFrame.pBufAddr;             
        stConvertParam.nSrcDataLen = outFrame.stFrameInfo.nFrameLen;   
        stConvertParam.enSrcPixelType = outFrame.stFrameInfo.enPixelType;  
        stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
        stConvertParam.pDstBuffer = save_img_buf_;   
        stConvertParam.nDstBufferSize = img_buf_size_;                    
        int ret = MV_CC_ConvertPixelType(handle ,&stConvertParam);
        if(ret == MV_OK)
        {
            img_data = cv::Mat(outFrame.stFrameInfo.nHeight, outFrame.stFrameInfo.nWidth, CV_8UC3, save_img_buf_);
            return true;
        }else{
            LOG(INFO) << "MV_CC_ConvertPixelType failed, ret=[0x"<<std::hex<<ret<<"]";
            return false;
        }
    }

    bool isMonoPixelFormat(MvGvspPixelType enType)
    {
        switch (enType)
        {
            case PixelType_Gvsp_Mono8:
            case PixelType_Gvsp_Mono10:
            case PixelType_Gvsp_Mono10_Packed:
            case PixelType_Gvsp_Mono12:
                return true;
            default:
                return false;
        }
    }
    
public:
    unsigned int getFps() { return fps_; }
    void setFps(unsigned int fps) { fps_=fps;}
    void setConnected(bool flag) { conneted_=flag; }

private:
    bool thread_running_ = false;
    bool opened_=false;
    bool idle_wait_state_ = true;
    bool enable_state_ = false;
    bool conneted_ = false;

    unsigned int fps_ = 40;
    void *handle = nullptr;
    unsigned int index_ = 0;
    unsigned int payloadsize_ = 0;
    unsigned int img_buf_size_ = 0;
    unsigned char *save_img_buf_ = NULL;

    std::mutex condition_mutex_;
    std::condition_variable_any condition_;

    MV_CC_DEVICE_INFO_LIST dev_list_{};
};

void __stdcall cbException(unsigned int nMsgType, void* pUser)
{
    LOG(INFO) << "recv MVCE013Device cbException envent! nMsgType:"<<nMsgType;
    MVCE013Device* mv_camera_ptr=static_cast<MVCE013Device*>(pUser);

    mv_camera_ptr->setConnected(false);


}

}
#endif  // SROS_MVCE013Device_H
   
