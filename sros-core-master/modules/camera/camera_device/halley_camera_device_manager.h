//
// Created by ehl on 2022/07/07.
//
#ifndef SROS_HALLEY_CAMERA_DEVICE_MANAGER_HPP
#define SROS_HALLEY_CAMERA_DEVICE_MANAGER_HPP

#include <math.h>
#include <boost/thread.hpp>
#include "halley_camera_device.h"

namespace camera {

class HalleyCameraDeviceManager : public OpenNI::DeviceConnectedListener,
	public OpenNI::DeviceDisconnectedListener,
	public OpenNI::DeviceStateChangedListener
{

 private:
    std::vector< std::shared_ptr<DabaiCameraDevice> > daBaiCameraDevieList_;
    bool initialized;

 public:
    static HalleyCameraDeviceManager *getInstance()
    {
        static HalleyCameraDeviceManager manager;
        return &manager;
    }

    bool OpenNIinitialize();
    std::vector<std::string> getAllDevices();

    // 在设备重连后执行,更新数据库的配置参数SN的取值范围
    void updateUserSettingsValueRange(const std::string new_sn);

    void registerCamera(std::shared_ptr<DabaiCameraDevice> camera_device_ptr);

	virtual void onDeviceStateChanged(const openni::DeviceInfo* pInfo, DeviceState state);

	virtual void onDeviceConnected(const openni::DeviceInfo* pInfo);

	virtual void onDeviceDisconnected(const openni::DeviceInfo* pInfo);
};

}  // namespace camera

#endif
