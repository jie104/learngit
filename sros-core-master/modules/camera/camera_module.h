#ifndef CAMERA_MODULE_H_
#define CAMERA_MODULE_H_

#include "camera.h"
#include "core/core.h"
#include "core/module.h"
#include "core/state.h"

#include <iostream>

#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "camera_device/base_camera_device.hpp"
#include "img_with_stamp_info.hpp"
#include "stereo_module_para.hpp"
#include "Eigen/Dense"
//#include <opencv2/objdetect/objdetect.hpp>
//#include <opencv2/ml/ml.hpp>

//#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

namespace camera {
struct CameraDriverInfo {
    std::string camera_sn;      //哈雷相机SN 标签
    std::string serial_number;  //哈雷相机打开名称
    int camera_index;
    BaseCameraDevicePtr camera_device;
    std::shared_ptr<stereo::StereoModulePara> para;
    bool initialize_ok = false;
    bool stereoCamera_using_avoidance = true;
};

class CameraModule : public sros::core::Module {
 public:
    CameraModule();

    virtual ~CameraModule();

    virtual void run();

 private:
    void onEnableSVC100Msg(sros::core::base_msg_ptr msg);

    void onEnableD435Msg(sros::core::base_msg_ptr msg);

    void onEnableDabaiproMsg(sros::core::base_msg_ptr msg);

    void onEnableStereoCameraMsg(sros::core::base_msg_ptr msg, const std::vector<CameraDriverInfo> stereo_camera_devices);

    void sendImgMsg(ImgWithStampInfoPtr img);

    void sendMVCE013ImgMsg(camera::ImgWithStampInfoPtr img);

    std::string uniteStrsToDoubles(const std::vector<double> &values, const char seperator) {
        std::stringstream result;
        for (auto &value:values) {
            result << value << seperator;
        }
        return result.str();
    }

    std::vector<double> splitStrsToDoubles(const std::string &s, const char seperator) {
        std::vector<double> result;
        typedef std::string::size_type string_size;

        string_size i = 0;
        string_size j = 0;
        char c = s[0];
        if (c == '"')           //chip 设置的话一般带“”
            j = 1;
        //LOG(INFO) << "the s is:" << s;
        while (i < s.size()) {
            if (s[i] == seperator || i == s.size() - 1) {
                if (j != i || i == s.size() - 1) {
                    auto len = (s[i] != seperator && i == s.size() - 1) ? (s.size() - j) : i - j;
                    std::string item_s = s.substr(j, len);
                    if (item_s == "\"")
                        break;
                    try {
                        double item = stof(item_s);
                        result.push_back(item);
                    }catch (std::exception &e){
                        LOG(ERROR) << "throw error:" << e.what()<<item_s;
                    }
                }
                j = i + 1;
            }
            i++;
        }

        return result;
    }

    void creatD435Devices(std::vector<CameraDriverInfo>& d435_devices, const std::string& stereo_camera_type);

    void creatDabaiproDevices(std::map<int,CameraDriverInfo>& dabaipro_devices);

    void creatSvc100Devices(std::vector<CameraDriverInfo>& d435_devices);

    void creatAzureDevice();

    void creatSvc200Device();

    void createInstallCalibrationInfo(Eigen::Matrix3d &install_info,const std::string& para_name);

    void openParticularSVC100(BaseCameraDevicePtr svc100,std::string suffix,int svc100_led_on_value);

    bool distributeSerialNumber(std::map<std::string,std::string> &sns);

    bool distributeSVC200SerialNumber(std::map<std::string,std::string> &sns);

    void setCameraParamToConfig(const std::string &suffix,const std::vector<double> &camera_param,const std::vector<double> &disort_param,const std::vector<double> &vanish_param);

    void initializeD435(CameraDriverInfo & d435_info, const std::string& stereo_camera_type);

    void initializeIRD435(CameraDriverInfo &d435_info, const std::string& stereo_camera_type);

    void initializeDabaipro(CameraDriverInfo &d435_info);

    bool distributeSerialNumber(const std::vector<std::string> &sns, std::map<std::string,int> &sns_map);

    bool createDabaiDeviceManager(void);

 private:
    //    bool enable_publish_svc100_image_ = true;
    BaseCameraDevicePtr svc_100_up_camera_device;
    BaseCameraDevicePtr svc_100_down_camera_device;
    BaseCameraDevicePtr azure_device;
    BaseCameraDevicePtr svc_100_back_camera_device;
    BaseCameraDevicePtr svc_200_back_camera_device;
    BaseCameraDevicePtr svc_200_up_camera_device;
    BaseCameraDevicePtr svc_200_down_camera_device;
    BaseCameraDevicePtr o3d303_camera_device;
    BaseCameraDevicePtr mv_ce013_device;
    std::vector<CameraDriverInfo> d435_devices;
    std::map<int,CameraDriverInfo> dabaipro_devices;
    std::vector<CameraDriverInfo> svc100_devices;
};

}  // namespace camera

#endif
