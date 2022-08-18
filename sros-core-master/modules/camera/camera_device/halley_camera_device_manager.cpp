//
// Created by ehl on 2022/07/07.
//
#include "halley_camera_device_manager.h"
#include "core/settings.h"

using namespace camera;

bool HalleyCameraDeviceManager::OpenNIinitialize() {
    LOG(INFO)<<"halley_driver test init OpenNI driver";
    
    if (!initialized)
    {
        openni::Status rc = OpenNI::initialize();
        if (rc != STATUS_OK)
        {
            LOG(INFO) << "OpenNI Initialize failed " << OpenNI::getExtendedError();
            return false;
        }

        initialized = true;
    }
    
    
    LOG(INFO) << "add deviceListenerPtr to OpenNI";
    HalleyCameraDeviceManager *deviceListenerPtr = camera::HalleyCameraDeviceManager::getInstance();
    OpenNI::addDeviceConnectedListener(deviceListenerPtr);
    OpenNI::addDeviceDisconnectedListener(deviceListenerPtr);
    OpenNI::addDeviceStateChangedListener(deviceListenerPtr);

    return true;
}

std::vector<std::string> HalleyCameraDeviceManager::getAllDevices() {
    std::vector<std::string> uri_sn_numbers_null;
    std::vector<std::string> uri_sn_numbers;
    openni::Array<openni::DeviceInfo> devBindList;
    openni::Status rc;
    openni::OpenNI::enumerateDevices(&devBindList);
    int devNum = devBindList.getSize();
    LOG(INFO)<<"scan halley enumerate Device Number: " <<  devNum;
    //轮询所有的相机URI
    for(uint32_t i = 0; i < devNum; i++) {
        std::string str_uri(devBindList[i].getUri());
        uri_sn_numbers.emplace_back();
        uri_sn_numbers.back() = devBindList[i].getUri();
        // LOG(INFO)<<"scan halley Uri: " << str_uri;
        openni::Device openni_device ;
        try {
            // LOG(INFO)<<"scan halley openni_device.open";
            rc = openni_device.open(uri_sn_numbers[2*i].c_str());
        } catch (const std::exception &e) {
            LOG(ERROR) << e.what();
        }
        if (rc != openni::STATUS_OK) {
            LOG(INFO)<<"!!!!!ERROR: [open camera failed] Scan halley. " << uri_sn_numbers[i].c_str() << ". " << openni::OpenNI::getExtendedError();
            return uri_sn_numbers_null;
        }
        char serNumber[12]={0};
        int dataSize = sizeof(serNumber);
        openni_device.getProperty(16, (uint8_t*)&serNumber,&dataSize);
        openni_device.close();
        // delete openni_device;
        uri_sn_numbers.emplace_back();
        uri_sn_numbers.back() = serNumber;
        LOG(INFO) << "scan halley found Uri: " << uri_sn_numbers[2*i] << "  SN: " << uri_sn_numbers[2*i+1] ;
    }
    return uri_sn_numbers;
}

void HalleyCameraDeviceManager::updateUserSettingsValueRange(const std::string new_sn) {
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
    for(auto& stereo_camera_sn : sns){
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

void HalleyCameraDeviceManager::registerCamera(std::shared_ptr<DabaiCameraDevice> camera_device_ptr)
{
    daBaiCameraDevieList_.push_back(camera_device_ptr);
}

void HalleyCameraDeviceManager::onDeviceStateChanged(const openni::DeviceInfo* pInfo, DeviceState state)
{
    LOG(INFO) << "Device " << pInfo->getUri() << " error state changed to " << state;
}

 void HalleyCameraDeviceManager::onDeviceConnected(const openni::DeviceInfo* pInfo)
{
    LOG(INFO) << "Device connected: " << pInfo->getUri();
    // 根据 Uri 读取 序列号, 传给 DabaiCameraDevice 对象处理
    if (daBaiCameraDevieList_.size() == 0) {
        LOG(INFO) << "DabaiCameraDevice not set by user"; // 此处表示配置参数可能没打开任何一个相机
    } else {
        LOG(INFO) << "openni_device.open(uri) try to get the serial number";
        // get the real serial_number
        openni::Device openni_device;
        openni::Status rc = openni_device.open(pInfo->getUri());
        if (rc != openni::STATUS_OK) {
            LOG(INFO)<<"!!!!!ERROR: [open camera failed] Scan halley. " << pInfo->getUri() << ". " << openni::OpenNI::getExtendedError();
            return;
        }
        char serial_number[12] = {0};
        int dataSize = sizeof(serial_number);
        openni_device.getProperty(16, (uint8_t*)&serial_number,&dataSize);
        openni_device.close();
        // check if match the user setting
        for (int i = 0; i < daBaiCameraDevieList_.size(); i++)
        {
            std::string serial_number_set_by_user = daBaiCameraDevieList_[i]->getCameraSN();
            LOG(INFO) << "target: " << serial_number_set_by_user << " new: " << serial_number;
            if( !strcmp(serial_number_set_by_user.c_str(), serial_number)) {
                LOG(INFO) << "============= serial number matched ==============";
                daBaiCameraDevieList_[i]->setCameraUri( pInfo->getUri() );  // 打开序列号匹配的 Uri
                daBaiCameraDevieList_[i]->open();
                if(daBaiCameraDevieList_[i]->registerFrameDataCallback()) {
                    LOG(INFO) << "register data callback func OK";
                }
                break;
            }
        }
        // update settings value range
        updateUserSettingsValueRange(serial_number);
    }
}

void HalleyCameraDeviceManager::onDeviceDisconnected(const openni::DeviceInfo* pInfo)
{
    LOG(INFO) << "Device disconnected: " << pInfo->getUri() ;
    // 根据设备 Uri 关闭对应的 DabaiCameraDevice 对象处理
    if (daBaiCameraDevieList_.size() == 0) {
        LOG(INFO) << "DabaiCameraDevice not set by user";  // 此处表示配置参数可能没打开任何一个相机
    } else {
        for (int i = 0; i < daBaiCameraDevieList_.size(); i++)
        {
            std::string uri = daBaiCameraDevieList_[i]->getCameraUri();
            if( !strcmp(uri.c_str(), pInfo->getUri())) {
                daBaiCameraDevieList_[i]->close();       // 关闭uri匹配的设备
                break;
            }
        }
    }
}
