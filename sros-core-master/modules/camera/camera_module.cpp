
#include "camera_module.h"
#include "camera.h"

#include <opencv2/videoio/videoio_c.h>
#include <fstream>
#include <opencv2/videoio/videoio.hpp>
#include <sstream>
//#include <opencv2/contrib/contrib.hpp>
#include <core/util/timer.h>

#include "camera_device/svc100_camera_device.hpp"
#include "camera_device/svc200_camera_device.h"
#include "camera_device/o3d303_camera_device.hpp"
#include "core/device/device_manager.h"
#include "core/msg/common_msg.hpp"
#include "core/msg/image_msg.hpp"
#include "core/settings.h"
#include "core/src.h"
#include "core/task/task_manager.h"
#include "core/version.h"
#include "SonixCameraLib/sr_sonix_camera_interface.hpp"
#include "camera_device_manage.hpp"
#include "camera_device/d435_camera_device.hpp"
#include "camera_device/d435_ir_camera_devicce.hpp"
#include "camera_device/azure_camera_device.hpp"
#include "camera_device/cq_camera_device.hpp"
#include "camera_device/mv_ce013_device.hpp"
#include "camera_device/halley_camera_device_manager.h"
#include "ChipVersion.h"

const std::string IMAGE_FILE_DIR = "/tmp/";

using namespace sros::core;

namespace camera {

const int D435_COLOR_CAMERA_ID = 0;
const int D435_DEPTH_CAMERA_ID = 2;

/// 从设备读取图片并进行与算法无关的预处理（畸变矫正等）
CameraModule::CameraModule() : Module("CameraModule") {}

CameraModule::~CameraModule() {}

void CameraModule::run() {
    sleep(1);//等待src启动完成
    auto &s = sros::core::Settings::getInstance();

    auto enable_camera_module = (s.getValue<std::string>("main.enable_camera_module", "False") == "True");
    if (!enable_camera_module) {
        LOG(WARNING) << "Camera Module is disabled!";
        return;
    }

    // NXP暂不开放ifm相机
    if (CHIP_VERSION == 1) {
        auto enable_O3D_camera = (s.getValue<std::string>("camera.enable_ifm_o3d303_camera", "False") == "True");
        if (enable_O3D_camera) {
            LOG(WARNING) << "O3D303 is not supported!";
            std::string o3d_ip_address = s.getValue<std::string>(
                "camera.ifm_o3d303_ip_address", "192.168.1.69");
            o3d303_camera_device.reset(
                new O3d303CameraDevice(o3d_ip_address, boost::bind(&CameraModule::sendImgMsg, this, _1),
                                    sros::device::DEVICE_CAMERA_O3D303, sros::device::DEVICE_ID_CAMERA_O3D303));
            o3d303_camera_device->open();
            sros::device::DeviceManager::getInstance()->addDevice(o3d303_camera_device);
            const auto enable_publish_image_when_action =
                (s.getValue<std::string>("camera.ifm_o3d303_enable_publish_image_when_action", "False") == "True");
            if (!enable_publish_image_when_action) {
                o3d303_camera_device->enableCamera();
            }
        }
    }

    auto svc_stereo_camera_type = s.getValue<std::string>("camera.svc_obstacle_device_type", "d435");
    if (svc_stereo_camera_type == "d435" || svc_stereo_camera_type == "d430")
    {
        d435_devices.clear();
        creatD435Devices(d435_devices, svc_stereo_camera_type);
        subscribeTopic("TIMER_50MS", CALLBACK(&CameraModule::onEnableD435Msg));
    }else{
        dabaipro_devices.clear();

        // 启动 Dabai 相机设备管理器 和 监听
        if(createDabaiDeviceManager()){
            creatDabaiproDevices(dabaipro_devices);
        }
        subscribeTopic("TIMER_50MS", CALLBACK(&CameraModule::onEnableDabaiproMsg));
    }

    // NXP暂不开放海康相机
    if (CHIP_VERSION == 1) {
        auto enable_mvs_camera = (s.getValue<std::string>("camera.enable_mvs_camera", "True") == "True");
        if(enable_mvs_camera){
            LOG(INFO) << "reset mv_ce013_device";
            mv_ce013_device.reset(
                    new MVCE013Device(0, boost::bind(&CameraModule::sendImgMsg, this, _1),
                            sros::device::DEVICE_CAMERA_MV_CE013, sros::device::DEVICE_ID_CAMERA_MV_CE013));
            sros::device::DeviceManager::getInstance()->addDevice(mv_ce013_device);
            LOG(INFO) << "open mv_ce013_device";
            mv_ce013_device->open();
        }
    }

    creatSvc100Devices(svc100_devices);
    subscribeTopic("TOPIC_SVC100_ENABLE_PUBLISH", CALLBACK(&CameraModule::onEnableSVC100Msg));
    dispatch();
}

void CameraModule::onEnableSVC100Msg(sros::core::base_msg_ptr msg) {
    auto &s = sros::core::Settings::getInstance();
    auto m = std::dynamic_pointer_cast<sros::core::CommonMsg>(msg);

    auto enableCamera = [](bool enable_function, bool enable_image, BaseCameraDevicePtr camera_device) {
      if (!enable_image) {  //如果接收到关闭svc100指令
          if (!enable_function) {  //且当前参数配置为在任何时候都打开svc100,则不用关闭,直接返回.
              if (!camera_device->isEnabled()) {
                  camera_device->enableCamera();
              }
          } else {
              camera_device->disableCamera();
          }
      } else {
          camera_device->enableCamera();
          //如果相机需要重新打开
      }
    };
    // 根据m->flag设置是否对外发布image
    bool enable_publish_svc100_up = true;
    if (m->str_0_ == "" || m->str_0_ == sros::device::DEVICE_SVC100_UP) {
        if (svc_100_up_camera_device) {
            const auto enable_function = (s.getValue<std::string>("camera.svc100_enable_publish_image_when_action", "False") == "True");
            bool enable_publish_svc100_image = m->flag;

            LOG(INFO) << "onEnableSVC100Msg() => " << enable_publish_svc100_image;
            enableCamera(enable_function, enable_publish_svc100_image, svc_100_up_camera_device);
        }
    } else if (m->str_0_ == sros::device::DEVICE_SVC100_DOWN) {
        if (svc_100_down_camera_device) {
            const auto enable_function =
                (s.getValue<std::string>("camera.svc100_down_enable_publish_image_when_action", "False") == "True");
            bool enable_publish_svc100_image = m->flag;

            LOG(INFO) << "onEnableSVC100Msg() => " << enable_publish_svc100_image;
            enableCamera(enable_function, enable_publish_svc100_image, svc_100_down_camera_device);
        }
    } else if (m->str_0_ == sros::device::DEVICE_CAMERA_UP) {
        if (svc_200_up_camera_device) {
            const auto enable_function =
                (s.getValue<std::string>("camera.up_enable_publish_image_when_action", "False") == "True");
            bool enable_publish_svc200_image = m->flag;

            LOG(INFO) << "onEnableSVC200Msg() => " << enable_publish_svc200_image;
            enableCamera(enable_function, enable_publish_svc200_image, svc_200_up_camera_device);
        }
    } else if (m->str_0_ == sros::device::DEVICE_CAMERA_DOWN) {
        if (svc_200_down_camera_device) {
            const auto enable_function =
                (s.getValue<std::string>("camera.down_enable_publish_image_when_action", "False") == "True");
            bool enable_publish_svc200_image = m->flag;

            LOG(INFO) << "onEnableSVC200Msg() => " << enable_publish_svc200_image;
            enableCamera(enable_function, enable_publish_svc200_image, svc_200_down_camera_device);
        }
    } else if(m->str_0_ == sros::device::DEVICE_CAMERA_BACK) {
        if (svc_200_back_camera_device) {
            const auto enable_function =
                (s.getValue<std::string>("camera.back_enable_publish_image_when_action", "False") == "True");
            bool enable_publish_svc200_image = m->flag;

            LOG(INFO) << "onEnableSVC200Msg() => " << enable_publish_svc200_image;
            enableCamera(enable_function, enable_publish_svc200_image, svc_200_back_camera_device);
        }
    } else if (m->str_0_ == sros::device::DEVICE_CAMERA_BACKWARD) {
        bool enable_when_action =
            (s.getValue<std::string>("camera.azure_enable_publish_image_when_action", "False") == "True");
        if(azure_device) {
            bool enable_publish_azure_image = m->flag;
            LOG(INFO) << "onEnableAzureCamera() => " << enable_publish_azure_image;
            enableCamera(enable_when_action, enable_publish_azure_image, azure_device);
        }

        if (svc_100_back_camera_device) {
            bool enable_publish_back_image = m->flag;
            LOG(INFO) << "onEnableBackCamera() => " << enable_publish_back_image;
            enableCamera(enable_when_action, enable_publish_back_image, svc_100_back_camera_device);
        }
    } else if (m->str_0_ == sros::device::DEVICE_CAMERA_O3D303) {
        if (o3d303_camera_device) {
            const auto enable_function = (s.getValue<std::string>("camera.ifm_o3d303_enable_publish_image_when_action", "False") == "True");
            bool enable_publish_o3d303_image = m->flag;

            LOG(INFO) << "onEnableO3d303Msg() => " << enable_publish_o3d303_image;
            if (!enable_publish_o3d303_image) {  //如果接收到关闭svc100指令
                if (!enable_function) {  //且当前参数配置为在任何时候都打开svc100,则不用关闭,直接返回.
                    if (!o3d303_camera_device->isEnabled()) {
                        o3d303_camera_device->enableCamera();
                    }
                } else {
                    o3d303_camera_device->disableCamera();
                }
            } else {
                o3d303_camera_device->enableCamera();
                //如果相机需要重新打开
            }
        }
    } else if (m->str_0_ == sros::device::DEVICE_CAMERA_D435_2) {
        for (auto &d435 : d435_devices) {
            if (d435.camera_device->getName() == m->str_0_) {
                LOG(INFO) << "onEnableD435_2Msg() => " << m->flag;
                if (m->flag) {
                    d435.camera_device->enableCamera();
                } else {
                    d435.camera_device->disableCamera();
                }
            }
        }
    } else if (m->str_0_ == sros::device::DEVICE_CAMERA_MV_CE013) {
        if (mv_ce013_device) {
            const auto enable_function =
                (s.getValue<std::string>("camera.mvs_ce013_enable_publish_image_when_action", "False") == "True");

            bool enable_publish_mvs_ce013_image = m->flag;

            LOG(INFO) << "onEnableMVSCE013Msg() => " << enable_publish_mvs_ce013_image;
            if (!enable_publish_mvs_ce013_image) {  //如果接收到关闭svc100指令
                if (!enable_function) {  //且当前参数配置为在任何时候都打开相机,则不用关闭,直接返回.
                    if (!mv_ce013_device->isEnabled()) {
                        mv_ce013_device->enableCamera();
                    }
                } else {
                    mv_ce013_device->disableCamera();
                }
            } else {
                mv_ce013_device->enableCamera();
                //如果相机需要重新打开
            }
        }
    } else if(m->str_0_ == sros::device::DEVICE_CAMERA_D435_3){
        for (auto &d435 : d435_devices) {
            if (d435.camera_device->getName() == m->str_0_) {
                LOG(INFO) << "onEnableD435_3Msg() => " << m->flag;
                if (m->flag) {
                    d435.camera_device->enableCamera();
                }else{
                    d435.camera_device->disableCamera();
                }
            }
        }
    }else if(m->str_0_ == sros::device::DEVICE_CAMERA_D435_4){
        for (auto &d435 : d435_devices) {
            if (d435.camera_device->getName() == m->str_0_) {
                LOG(INFO) << "onEnableD435_3Msg() => " << m->flag;
                if (m->flag) {
                    d435.camera_device->enableCamera();
                }else{
                    d435.camera_device->disableCamera();
                }
            }
        }
    }

}

void CameraModule::sendImgMsg(camera::ImgWithStampInfoPtr img) {
    if (img) {
        if (img->use_mat_info) {
            auto msg = std::make_shared<ImageMsg>(img->topic_name);
            msg->setCameraName(img->camera_name);
            msg->setTimestamp(img->stamp);
            msg->setMat(img->img);
            msg->is_real_time_ = true;
            if (!img->amplitude_img.empty()) {
                msg->setAmplitudeMat(img->amplitude_img); // add by zhangxu at 2020/11/30
            }
            if (!img->xyz_img.empty()) {
                msg->mat_xyz_ = img->xyz_img;
            }
            msg->setForkHeightEncoder(img->fork_height_encoder); // add by lijunhong at 2021/12/13
            sendMsg(msg);
        }
        if (img->use_points_info) {
            sendMsg(img->points);
        }
    }
}

void CameraModule::sendMVCE013ImgMsg(camera::ImgWithStampInfoPtr img) {
    LOG(INFO) << "sendMVCE013ImgMsg() " << get_time_in_ns() / 1000.0f;
}

bool CameraModule::distributeSerialNumber(std::map<std::string, std::string> &sns) {
    auto &s = sros::core::Settings::getInstance();
    auto enable_up = (s.getValue<std::string>("camera.enable_svc100_camera", "False") == "True");
    auto enable_down = (s.getValue<std::string>("camera.enable_svc100_down_camera", "False") == "True");
    int enable_count = (int)enable_up + (int)enable_down;
    std::map<std::string, Device::CameraDevice> svc100_devices;
    int totally_count = 0;
    V4L2Camera::getAllSVC100Devices(svc100_devices,totally_count);

    auto up_info = s.getItemInfo("camera.svc100_up_serial_number");
    auto down_info = s.getItemInfo("camera.svc100_down_serial_number");
    std::string value_range;
    LOG(INFO) << "existed device numbers:" << totally_count << ", valid size:" << svc100_devices.size();
    if(totally_count!=svc100_devices.size()) {
        svc100_devices.clear();
    }
    for (auto& svc:svc100_devices) {
        LOG(INFO) << "svc sn:" << svc.first;
        value_range += svc.first;
        value_range += ';';
    }
    value_range += "NA;";

    LOG(INFO) << "up sn:" << up_info.value << ",down sn:" << down_info.value;
    LOG(INFO) << "origin value:" << down_info.value_range << "," << up_info.value_range << "," << value_range;
    down_info.value_range = value_range;
    up_info.value_range = value_range;
    s.setItemInfo(down_info);
    s.setItemInfo(up_info);
    if (svc100_devices.empty()) {
        LOG(ERROR) << "cannot distribute svc100 device!";
        return false;
    }
    const std::string svc100_up_name = sros::device::DEVICE_SVC100_UP;
    const std::string svc100_down_name = sros::device::DEVICE_SVC100_DOWN;
    if (totally_count == 1 && enable_count == 1) {//如果当前打开的摄像头只有一个,且搜索到的设备只有一个,那么直接打开指定设备,不需要设置参数
        auto svc100_iter = svc100_devices.begin();
        if (enable_up) {
            sns[svc100_up_name] = svc100_iter->first;
        } else if (enable_down) {
            sns[svc100_down_name] = svc100_iter->first;
        }
    } else {
        for (auto &device : svc100_devices) {//其他情况使用默认方式打开.
            if(device.first == up_info.value) {//等于上视二维码
                if (down_info.value == up_info.value) {
                    LOG(ERROR) << "down sn value is wrong! will not open it!";
                }
                LOG(INFO) << "get up device sn!" << device.first;
                sns[svc100_up_name] = device.first;
            } else if (device.first == down_info.value) {
                LOG(INFO) << "get down device sn!" << device.first;
                sns[svc100_down_name] = device.first;
            }
        }
    }
    for (auto& sn:sns) {
        auto device = svc100_devices.find(sn.second);
        if (device != svc100_devices.end()) {
            std::string suffix;
            if (sn.first == svc100_down_name) {
                suffix = "down";
            }
            setCameraParamToConfig(suffix, device->second.intrinsic, device->second.distort, device->second.vanish);
        }
    }
    return true;
}

bool CameraModule::distributeSVC200SerialNumber(std::map<std::string, std::string> &sns) {
    auto &s = sros::core::Settings::getInstance();
    auto svc200_singleton = SVC200ChipSelectSingleton::getInstance();
    if (!svc200_singleton) {
        LOG(INFO) << "Get singleton prt fail, again. ";
        svc200_singleton = SVC200ChipSelectSingleton::getInstance();
        if (!svc200_singleton) {
            LOG(INFO) << "Get singleton prt fail, abort. ";
            return false;
        }
    }
    auto dev_map_ = svc200_singleton->dev_map_;
    auto up_info = s.getItemInfo("camera.svc100_up_serial_number");
    auto down_info = s.getItemInfo("camera.svc100_down_serial_number");
    auto back_info = s.getItemInfo("camera.svc100_back_serial_number");
    std::string value_range;
    for (auto &dev_map : dev_map_) {
        LOG(INFO) << "svc sn:" << dev_map.first;
        value_range += dev_map.first;
        value_range += ';';
    }
    value_range += "NA;";

    LOG(INFO) << "up sn:" << up_info.value << ",down sn:" << down_info.value << ",back sn:" << back_info.value;
    LOG(INFO) << "origin value:" << down_info.value_range << "," << up_info.value_range << ","
                << back_info.value_range << "," << value_range;
    down_info.value_range = value_range;
    up_info.value_range = value_range;
    back_info.value_range = value_range;
    s.setItemInfo(down_info);
    s.setItemInfo(up_info);
    s.setItemInfo(back_info);
    if (dev_map_.empty()) {
        LOG(ERROR) << "cannot distribute svc200 device!";
        return false;
    }
    const std::string svc100_up_name = sros::device::DEVICE_SVC100_UP;
    const std::string svc100_down_name = sros::device::DEVICE_SVC100_DOWN;
    const std::string back_name = sros::device::DEVICE_CAMERA_BACK;
    for (auto &dev_map : dev_map_) {
        if (dev_map.first == up_info.value) {
            if (down_info.value == up_info.value || back_info.value == up_info.value) {
                LOG(ERROR) << "down sn value is wrong! will not open it!";
            }
            LOG(INFO) << "get up device sn!" << dev_map.first;
            sns[svc100_up_name] = dev_map.first;
            s.setValue("camera.up_serial_number", dev_map.first);
        } else if (dev_map.first == down_info.value) {
            if (back_info.value == down_info.value) {
                LOG(ERROR) << "back sn value is wrong! will not open it!";
            }
            LOG(INFO) << "get down device sn!" << dev_map.first;
            sns[svc100_down_name] = dev_map.first;
            s.setValue("camera.down_serial_number", dev_map.first);
        } else if (dev_map.first == back_info.value) {
            LOG(INFO) << "get back device sn!" << dev_map.first;
            sns[back_name] = dev_map.first;
            s.setValue("camera.back_serial_number", dev_map.first);
        }
    }
    std::vector<double> camera_param, disort_param, vanish_param;
    for (auto &sn:sns) {
        auto device = dev_map_.find(sn.second);
        if (device != dev_map_.end()) {
            std::string suffix;
            if (sn.first == svc100_down_name) {
                suffix = "down";
            }
            if (sn.first == back_name) {
                suffix = "back";
            }
            for (int i = 0; i < 9; i++) {
                camera_param.push_back(device->second.cameraParam.vAlgorithm.intrinsic[i]);
            }
            for (int i = 0; i < 5; i++) {
                disort_param.push_back(device->second.cameraParam.vAlgorithm.distort[i]);
            }
            for (int i = 0; i < 2; i++) {
                vanish_param.push_back(device->second.cameraParam.vAlgorithm.vanishingPoint[i]);
            }
            setCameraParamToConfig(suffix, camera_param, disort_param, vanish_param);
            camera_param.clear();
            disort_param.clear();
            vanish_param.clear();
        }
    }
    return true;
}

void CameraModule::setCameraParamToConfig(const std::string &suffix, const std::vector<double> &camera_param,
                                          const std::vector<double> &disort_param,
                                          const std::vector<double> &vanish_param) {
    std::string camera_suffix;
    if (suffix.empty()) {
        camera_suffix = "camera.svc100_";
    } else {
        camera_suffix = "camera.svc100_" + suffix + "_";
    }

    auto &s = sros::core::Settings::getInstance();
    auto string_value = uniteStrsToDoubles(camera_param, ';');
    s.setValue<std::string>(camera_suffix + "inner_param", string_value);
    LOG(INFO) << camera_suffix + "inner_param" << ":" << string_value;
    string_value = uniteStrsToDoubles(disort_param, ';');
    LOG(INFO) << camera_suffix + "disort_param" << ":" << string_value;
    s.setValue<std::string>(camera_suffix + "disort_param", string_value);

    string_value = uniteStrsToDoubles(vanish_param, ';');
    LOG(INFO) << camera_suffix + "vanish_point" << ":" << string_value;
    s.setValue<std::string>(camera_suffix + "vanish_point", string_value);
}

void CameraModule::openParticularSVC100(BaseCameraDevicePtr svc100, std::string suffix, int svc100_led_on_value) {
    std::string camera_suffix;
    if (suffix.empty()) {
        camera_suffix = "camera.svc100_";
    } else {
        camera_suffix = "camera.svc100_" + suffix + "_";
    }
    auto &s = sros::core::Settings::getInstance();
    std::string led_bits = camera_suffix + "led_bits";
    std::string fps = camera_suffix + "fps";
    std::string action = camera_suffix + "enable_publish_image_when_action";
    int svc100_led_bits = s.getValue<int>(led_bits, 5);
    int camera_fps = s.getValue<int>(fps, 40);
    auto enable_publish_image_when_action = (s.getValue<std::string>(action, "False") == "True");
    svc100->setParameter("svc100_led_on_value", svc100_led_on_value);
    svc100->setParameter("svc100_led_bits", svc100_led_bits);
    svc100->setFPS(camera_fps);
    LOG(INFO) << led_bits << "," << svc100_led_bits << "," << fps << "," << camera_fps << "," << action << ","
              << enable_publish_image_when_action;
    if (svc100->open()) {
        if (!enable_publish_image_when_action) {
            svc100->enableCamera();
        }
    } else {
        LOG(INFO) << "err to open " << svc100->getName();
    }
}

bool CameraModule::distributeSerialNumber(const std::vector<std::string> &sns, std::map<std::string, int> &sns_map) {
    auto &s = sros::core::Settings::getInstance();
    bool enable_stereo_camera_1 = (s.getValue<std::string>("camera.enable_d435_camera", "True") == "True");
    bool enable_stereo_camera_2 = (s.getValue<std::string>("camera.enable_d435_2_camera", "True") == "True");
    bool enable_stereo_camera_3 = (s.getValue<std::string>("camera.enable_d435_3_camera", "True") == "True");
    bool enable_stereo_camera_4 = (s.getValue<std::string>("camera.enable_d435_4_camera", "True") == "True");
    auto stereo_camera_1_info = s.getItemInfo("camera.d435_1_serial_number");
    auto stereo_camera_2_info = s.getItemInfo("camera.d435_2_serial_number");
    auto stereo_camera_3_info = s.getItemInfo("camera.d435_3_serial_number");
    auto stereo_camera_4_info = s.getItemInfo("camera.d435_4_serial_number");
    LOG(INFO) << "user set SN old value_range: " << stereo_camera_1_info.value_range << ";" << stereo_camera_2_info.value_range << ";" << \
        stereo_camera_3_info.value_range << ";" << stereo_camera_4_info.value_range;

    // 把实际存在的相机序列号 sns 放进数据库的value_range
    std::string value_range;
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

    // 当实际的sns为空时只警告不报错
    if (sns.empty()) {
        LOG(WARNING) << "actual stereo_camera SN null! use the user's config";
        // return true;
    }
    //  用户enable的SN, 都要尝试打开
    int enabled_numbers = (enable_stereo_camera_1 + enable_stereo_camera_2 + enable_stereo_camera_3 + enable_stereo_camera_4);
    if (sns.size() == 1 && enable_stereo_camera_1 && !enable_stereo_camera_2 && !enable_stereo_camera_3 && !enable_stereo_camera_4) {
        //默认情况下,如果只识别到一个
        LOG(INFO) << "user set one stereo_camera, will set to stereo_camera 1!";
        stereo_camera_1_info.value = sns[0];
        s.setItemInfo(stereo_camera_1_info);
        if (enable_stereo_camera_1) {
            sns_map[sns[0]] = 1;
        }
    } else {
        // for (auto &sn : sns) {
        //     if ((sn == stereo_camera_1_info.value) && enable_stereo_camera_1) {
        //         sns_map[sn] = 1;
        //     } else if ((sn == stereo_camera_2_info.value) && enable_stereo_camera_2) {
        //         sns_map[sn] = 2;
        //     } else if ((sn == stereo_camera_3_info.value) && enable_stereo_camera_3) {
        //         sns_map[sn] = 3;
        //     } else if ((sn == stereo_camera_4_info.value) && enable_stereo_camera_4) {
        //         sns_map[sn] = 4;
        //     } else {
        //         LOG(ERROR) << "user not config the sn:" << sn;
        //     }
        // }
        if (!(stereo_camera_1_info.value.empty()) && enable_stereo_camera_1) {
            sns_map[stereo_camera_1_info.value] = 1;
        }
        if (!(stereo_camera_2_info.value.empty()) && enable_stereo_camera_2) {
            sns_map[stereo_camera_2_info.value] = 2;
        }
        if (!(stereo_camera_3_info.value.empty()) && enable_stereo_camera_3) {
            sns_map[stereo_camera_3_info.value] = 3;
        }
        if (!(stereo_camera_4_info.value.empty()) && enable_stereo_camera_4) {
            sns_map[stereo_camera_4_info.value] = 4;
        }
    }

    if (sns_map.empty()) {
        LOG(ERROR) << "user config SN null ERROR";
        return false;
    } else if(sns_map.size() != enabled_numbers) {
        LOG(WARNING) << "user config SN numbers: " << sns_map.size() << " ? but enabled:" << enabled_numbers;
    }
    LOG(INFO) << "distribute Serial Number " << sns_map.size();
    return true;
}

void CameraModule::initializeD435(CameraDriverInfo &d435_info, const std::string& stereo_camera_type) {
    LOG(INFO) << "d435 sn:" << d435_info.camera_sn;
    auto &s = sros::core::Settings::getInstance();
    std::string d435_device_name = sros::device::DEVICE_CAMERA_D435;
    auto d435_device_id = sros::device::DEVICE_ID_CAMERA_D435;
    int index_count = d435_info.camera_index;
    std::string d435_device_name_temp = sros::device::DEVICE_CAMERA_D435;
    if (index_count > 1 && index_count < 5) {
        for(int i = 2;i <= index_count;i++){
            d435_device_name = d435_device_name_temp + "_" + std::to_string(i);
            if(i == 2){
                d435_device_id = sros::device::DEVICE_ID_CAMERA_D435_2; // D435 2 的ID相比于D435 1 加了1
            } else if(i == 3){
                d435_device_id = sros::device::DEVICE_ID_CAMERA_D435_3; // D435 3 的ID相比于D435 1 加了2
            } else if(i == 4){
                 d435_device_id = sros::device::DEVICE_ID_CAMERA_D435_4; // D435 4 的ID相比于D435 1 加了3
             }

        }
    }
    auto &para = d435_info.para;
    if (!para) {
        para.reset(new stereo::StereoModulePara);
    }
    std::shared_ptr<camera::D435CameraDevice> d435_device;
    d435_device.reset(new camera::D435CameraDevice(
        para, d435_info.camera_sn, boost::bind(&CameraModule::sendImgMsg,this,_1), d435_device_name, d435_device_id, stereo_camera_type));
    sros::device::DeviceManager::getInstance()->addDevice(d435_device);
    d435_info.camera_device = d435_device;
    d435_device->open();
    d435_device->disableCamera();

    std::string calibrate_state = "camera.calibrate_";
    std::string camera_prefix = "camera.";
    std::string suffix_str = "d435_";
    std::string calibrate_state_name = "camera.calibrate_camera_install_err";
    std::string calibrate_matrix_name = "camera.stereo_camera_calibration_matrix";
    if (index_count == 2) {
        suffix_str = "d435_2_";
        calibrate_state_name = calibrate_state + suffix_str + "install_err";
        calibrate_matrix_name = camera_prefix + suffix_str + "calibration_matrix";
    }
    if (index_count == 3) {
        suffix_str = "d435_3_";
        calibrate_state_name = calibrate_state + suffix_str + "install_err";
        calibrate_matrix_name = camera_prefix + suffix_str + "calibration_matrix";
    }
    if (index_count == 4) {
        suffix_str = "d435_4_";
        calibrate_state_name = calibrate_state + suffix_str + "install_err";
        calibrate_matrix_name = camera_prefix + suffix_str + "calibration_matrix";
    }

    std::string install_x = "install_x_offset";
    para->d435_install_x = s.getValue(camera_prefix + suffix_str + install_x, 390.0) * sros::core::MM_TO_M;
    std::string install_y = "install_y_offset";
    para->d435_install_y = s.getValue(camera_prefix + suffix_str + install_y, 0.0) * sros::core::MM_TO_M;
    std::string install_yaw = "install_yaw";
    para->d435_install_yaw = s.getValue(camera_prefix + suffix_str + install_yaw, 0.0) * sros::core::DEG_TO_RAD;

    LOG(INFO) << "para:" << para->d435_install_x << "," << para->d435_install_y << "," << para->d435_install_yaw;

    para->obstacle_min_height = s.getValue("obstacle.stereo_points_min_height", 30.0) * sros::core::MM_TO_M;
    para->obstacle_max_height = s.getValue("camera.d435_detect_max_height", 1000.0) * sros::core::MM_TO_M;
    para->obstacle_max_dist = s.getValue("camera.d435_detect_max_dist", 1500.0) * sros::core::MM_TO_M;
    auto obssize = s.getValue<int>("obstacle.d435_obssize", 3);
    d435_device->setOBSZISE(obssize);
    d435_device->setHeightThresh(para->obstacle_min_height, para->obstacle_max_height);
    d435_device->setDetectDistThresh(para->obstacle_max_dist);

    std::string install_theta = "install_pitch";
    para->d435_install_pitch =
        s.getValue(camera_prefix + suffix_str + install_theta, 32.7) * sros::core::DEG_TO_RAD;

    std::string install_roll = "install_roll";
    para->d435_install_roll =
        s.getValue(camera_prefix + suffix_str + install_roll, 0.0) * sros::core::DEG_TO_RAD;

    std::string calibration_height = "calibration_height";
    para->d435_calibration_height =
        s.getValue(camera_prefix + suffix_str + calibration_height, 0) * sros::core::MM_TO_M;

    std::string install_height = "install_height";
    para->d435_install_height =
        s.getValue(camera_prefix + suffix_str + install_height, 250.0) * sros::core::MM_TO_M;
    if (para->d435_install_height < 0.0) {
        para->d435_install_height = -para->d435_install_height;
    }
    bool calibrate_camera_install_err = false;
    calibrate_camera_install_err = (s.getValue<std::string>(calibrate_state_name, "False") == "True");
    if (calibrate_camera_install_err) {
        Eigen::Quaternionf quat;
        d435::D435ObstacleDetector::EulerToQuaternion(para->d435_install_yaw, para->d435_install_roll,
                                                      para->d435_install_pitch, quat);
        if (d435_device->calibrateCameraInstallErr(quat, para->d435_calibration_height,
                                                   para->d435_install_height,
                                                   para->d435_install_calibration_matrix)) {
            auto matrix_data = para->d435_install_calibration_matrix.data();
            std::vector<double> values;
            for (int i = 0; i < 9; ++i) {
                values.push_back(matrix_data[i]);
            }
            auto string_value = uniteStrsToDoubles(values, ';');
            LOG(INFO) << "string value:" << string_value << ",calibrate name:" << calibrate_matrix_name;
            s.setValue<std::string>(calibrate_state_name, "False");

//            //TODO:set angle updata
//            Eigen::Vector3d eulerAngle = para->d435_install_calibration_matrix.eulerAngles(0,1,2);//(0,1,2)表示旋转顺序XYZ，数字越小表示优先级越高
//
//            para->d435_install_pitch = eulerAngle(1);
//            para->d435_install_roll = eulerAngle(0);
//            LOG(INFO)<<"set angle updata"<< "  yaw : "<<eulerAngle(2) *RAD_TO_DEGREE<<"   pitch :"<<eulerAngle(1) * RAD_TO_DEGREE;
//
//            s.setValue<double>(camera_prefix + suffix_str + install_theta,para->d435_install_pitch );
//            s.setValue<double>(camera_prefix + suffix_str + install_yaw,para->d435_install_roll );

            s.setValue<std::string>(calibrate_matrix_name, string_value);
            s.setValue<double>(camera_prefix + suffix_str + "install_height", para->d435_install_height / sros::core::MM_TO_M);
        } else {
            LOG(INFO) << "err to calibrate stereo camera!";
        }
    }
    createInstallCalibrationInfo(para->d435_install_calibration_matrix,calibrate_matrix_name);
    Eigen::Matrix3f calibration_matrix_f;
    Eigen::Quaternionf quat;
    if (!para->d435_install_calibration_matrix.isIdentity(1e-5)) {
        Eigen::Quaterniond quaternion(para->d435_install_calibration_matrix);
        d435::D435ObstacleDetector::QuaternionToEuler(quaternion, para->d435_install_yaw, para->d435_install_roll,
                                                      para->d435_install_pitch);
        s.setValue<double>(camera_prefix + suffix_str + install_theta, para->d435_install_pitch / M_PI * 180);
        s.setValue<double>(camera_prefix + suffix_str + install_yaw, para->d435_install_yaw / M_PI * 180);
        s.setValue<std::string>(calibrate_matrix_name, "1;0;0;0;1;0;0;0;1");
        LOG(INFO) << "set config " << camera_prefix + suffix_str + install_theta << "value: " << para->d435_install_pitch / M_PI * 180;
        LOG(INFO) << "set config " << camera_prefix + suffix_str + install_yaw << "value: " << para->d435_install_pitch / M_PI * 180;
        LOG(INFO) << "set config " << calibrate_matrix_name << "value: " << "1;0;0;0;1;0;0;0;1";

    }
    d435::D435ObstacleDetector::EulerToQuaternion(0.0, para->d435_install_roll,
                                                  para->d435_install_pitch, quat);
    calibration_matrix_f = quat.matrix();
    para->d435_install_calibration_matrix = calibration_matrix_f.cast<double>();

    d435_device->updateCameraInstallErr(para->d435_install_calibration_matrix, para->d435_install_height);
    para->enable_filter_only_load_full =
        (s.getValue<std::string>("obstacle.rack_enable_filter_only_load_full", "True") == "True");
    para->enable_remove_rack_leg = s.getValue<std::string>("obstacle.enable_remove_rack_leg", "False") == "True";

    para->rack_leg_wheel_center_length =
        s.getValue<double>("rack.rack_leg_center_length", 1060) * sros::core::MM_TO_M;
    para->rack_leg_wheel_center_width = s.getValue<double>("rack.rack_leg_center_width", 600) * sros::core::MM_TO_M;
    para->rack_wheel_rotate_radius = s.getValue<double>("rack.rack_wheel_rotate_radius", 150) * sros::core::MM_TO_M;

    LOG(INFO) << "para info:"
              << "install x:" << para->d435_install_x << ",install_y," << para->d435_install_y << ",install_yaw,"
              << para->d435_install_yaw << ",rack_length," << para->rack_leg_wheel_center_length << ",width,"
              << para->rack_leg_wheel_center_width << "," << para->rack_wheel_rotate_radius;
}

void CameraModule::initializeIRD435(CameraDriverInfo &d435_info, const std::string& stereo_camera_type) {
    LOG(INFO) << "d435 ir sn:" << d435_info.camera_sn;
    auto &s = sros::core::Settings::getInstance();
    std::string d435_device_name = sros::device::DEVICE_CAMERA_D435;
    auto d435_device_id = sros::device::DEVICE_ID_CAMERA_D435;
    int index_count = d435_info.camera_index;
//    if (index_count > 1) {
//        d435_device_name = d435_device_name + "_" + std::to_string(index_count);
//        d435_device_id = sros::device::DEVICE_ID_CAMERA_D435_2;  // D435 2 的ID相比于D435 1 加了1
//    }
    std::string d435_device_name_temp = sros::device::DEVICE_CAMERA_D435;
    if (index_count > 1 && index_count < 5) {
        for(int i = 2;i < index_count;i++){
            d435_device_name = d435_device_name_temp + "_" + std::to_string(i+1);
            if(i == 2){
                d435_device_id = sros::device::DEVICE_ID_CAMERA_D435_2; // D435 2 的ID相比于D435 1 加了1
            } if(i == 3){
                d435_device_id = sros::device::DEVICE_ID_CAMERA_D435_3; // D435 3 的ID相比于D435 1 加了2
            } if(i == 4){
                d435_device_id = sros::device::DEVICE_ID_CAMERA_D435_4; // D435 4 的ID相比于D435 1 加了3
            }

        }
    }

    std::shared_ptr<camera::D435IrCameraDevice> d435_device;
    d435_device.reset(new camera::D435IrCameraDevice(
        d435_info.camera_sn, boost::bind(&CameraModule::sendImgMsg, this, _1), d435_device_name, d435_device_id, stereo_camera_type));
    sros::device::DeviceManager::getInstance()->addDevice(d435_device);
    d435_info.camera_device = d435_device;
    d435_device->open();
    d435_device->disableCamera();
    if (d435_device->isOpened()) {
        //设置d435参数到matrix界面（用来识别fm码的相机）
        std::vector<double> intrinsic;
        std::vector<double> distort;
        d435_device->getIrIntrinsic(intrinsic, distort);

        auto string_value = uniteStrsToDoubles(intrinsic, ';');
        s.setValue<std::string>("camera.d435_inner_param", string_value);
        LOG(INFO) << "camera.d435_inner_param" << ":" << string_value;

        string_value = uniteStrsToDoubles(distort, ';');
        s.setValue<std::string>("camera.d435_disort_param", string_value);
        LOG(INFO) << "camera.d435_disort_param" << ":" << string_value;
    }
}

void CameraModule::initializeDabaipro(CameraDriverInfo &dabaipro_info) {
    LOG(INFO) << "halley dabaipro camera serial_number:" << dabaipro_info.camera_sn;
    auto &s = sros::core::Settings::getInstance();
    std::string dabaipro_device_name = sros::device::DEVICE_CAMERA_DEPTH;
    auto dabaipro_device_id = sros::device::DEVICE_ID_CAMERA_DEPTH;
    int index_count = dabaipro_info.camera_index;
    if (index_count > 1) {
        dabaipro_device_name = dabaipro_device_name + "_" + std::to_string(index_count);
        dabaipro_device_id = sros::device::DEVICE_ID_CAMERA_DEPTH_2;  // Dabaipro 2 的ID相比于Dabaipro 1 加了1
    }
    auto &para = dabaipro_info.para;
    if (!para) {
        para.reset(new stereo::StereoModulePara);
    }

    std::shared_ptr<camera::DabaiCameraDevice> dabaipro_device;
    dabaipro_device.reset(new camera::DabaiCameraDevice(
        para, dabaipro_info.serial_number,dabaipro_info.camera_sn, boost::bind(&CameraModule::sendImgMsg,this,_1), dabaipro_device_name, dabaipro_device_id));
    
    sros::device::DeviceManager::getInstance()->addDevice(dabaipro_device);

    // register the device into deviceList
    camera::HalleyCameraDeviceManager::getInstance()->registerCamera(dabaipro_device);
    dabaipro_info.camera_device = dabaipro_device;
    dabaipro_device->open();
    dabaipro_device->disableCamera();

    std::string calibrate_state = "camera.calibrate_";
    std::string camera_prefix = "camera.";
    std::string suffix_str = "d435_";
    std::string calibrate_state_name = "camera.calibrate_camera_install_err";
    std::string calibrate_matrix_name = "camera.stereo_camera_calibration_matrix";
    if (index_count == 2) {
        suffix_str = "d435_2_";
        calibrate_state_name = calibrate_state + suffix_str + "install_err";
        calibrate_matrix_name = camera_prefix + suffix_str + "calibration_matrix";
    }
    std::string install_x = "install_x_offset";
    para->d435_install_x = s.getValue(camera_prefix + suffix_str + install_x, 390.0) * sros::core::MM_TO_M;
    std::string install_y = "install_y_offset";
    para->d435_install_y = s.getValue(camera_prefix + suffix_str + install_y, 0.0) * sros::core::MM_TO_M;
    std::string install_yaw = "install_yaw";
    para->d435_install_yaw = s.getValue(camera_prefix + suffix_str + install_yaw, 0.0) * sros::core::DEG_TO_RAD;

    LOG(INFO) << "para:" << para->d435_install_x << "," << para->d435_install_y << "," << para->d435_install_yaw;

    para->obstacle_min_height = s.getValue("obstacle.stereo_points_min_height", 30.0) * sros::core::MM_TO_M;
    para->obstacle_max_height = s.getValue("camera.d435_detect_max_height", 1000.0) * sros::core::MM_TO_M;
    para->obstacle_max_dist = s.getValue("camera.d435_detect_max_dist", 1500.0) * sros::core::MM_TO_M;
    dabaipro_device->setHeightThresh(para->obstacle_min_height, para->obstacle_max_height);
    dabaipro_device->setDetectDistThresh(para->obstacle_max_dist);

    std::string install_theta = "install_pitch";
    para->d435_install_pitch =
        s.getValue(camera_prefix + suffix_str + install_theta, 32.7) * sros::core::DEG_TO_RAD;

    std::string install_roll = "install_roll";
    para->d435_install_roll =
        s.getValue(camera_prefix + suffix_str + install_roll, 0.0) * sros::core::DEG_TO_RAD;

    std::string calibration_height = "calibration_height";
    para->d435_calibration_height =
        s.getValue(camera_prefix + suffix_str + calibration_height, 0) * sros::core::MM_TO_M;

    std::string install_height = "install_height";
    para->d435_install_height = s.getValue(camera_prefix + suffix_str + install_height, 250.0) * sros::core::MM_TO_M;
    if (para->d435_install_height < 0.0) {
        para->d435_install_height = -para->d435_install_height;
    }
    bool calibrate_camera_install_err = false;
    calibrate_camera_install_err = (s.getValue<std::string>(calibrate_state_name, "False") == "True");
    if (calibrate_camera_install_err) {
        Eigen::Quaternionf quat;
        threedim_camera::CameraObstacleDetector::EulerToQuaternion(para->d435_install_yaw, para->d435_install_roll,
                                                      para->d435_install_pitch, quat);
        if (dabaipro_device->calibrateCameraInstallErr(quat, para->d435_calibration_height,
                                                   para->d435_install_height, para->d435_install_calibration_matrix)) {
            auto matrix_data = para->d435_install_calibration_matrix.data();
            std::vector<double> values;
            for (int i = 0; i < 9; ++i) {
                values.push_back(matrix_data[i]);
            }
            auto string_value = uniteStrsToDoubles(values, ';');
            LOG(INFO) << "string value:" << string_value << ",calibrate name:" << calibrate_matrix_name;
            s.setValue<std::string>(calibrate_state_name, "False");
            s.setValue<std::string>(calibrate_matrix_name, string_value);
            s.setValue<double>(camera_prefix + suffix_str + "install_height", para->d435_install_height / sros::core::MM_TO_M);
        } else {
            LOG(INFO) << "err to calibrate stereo camera!";
        }
    }
    createInstallCalibrationInfo(para->d435_install_calibration_matrix,calibrate_matrix_name);
    Eigen::Matrix3f calibration_matrix_f;
    Eigen::Quaternionf quat;
    if (!para->d435_install_calibration_matrix.isIdentity(1e-5)) {
        Eigen::Quaterniond quaternion(para->d435_install_calibration_matrix);
        d435::D435ObstacleDetector::QuaternionToEuler(quaternion, para->d435_install_yaw, para->d435_install_roll,
                                                      para->d435_install_pitch);
        s.setValue<double>(camera_prefix + suffix_str + install_theta, para->d435_install_pitch / M_PI * 180);
        s.setValue<double>(camera_prefix + suffix_str + install_yaw, para->d435_install_yaw / M_PI * 180);
        s.setValue<std::string>(calibrate_matrix_name, "1;0;0;0;1;0;0;0;1");
    }
    d435::D435ObstacleDetector::EulerToQuaternion(0.0, para->d435_install_roll,
                                                  para->d435_install_pitch, quat);
    calibration_matrix_f = quat.matrix();
    para->d435_install_calibration_matrix = calibration_matrix_f.cast<double>();
    dabaipro_device->updateCameraInstallErr(para->d435_install_calibration_matrix, para->d435_install_height);
    para->enable_filter_only_load_full =
        (s.getValue<std::string>("obstacle.rack_enable_filter_only_load_full", "True") == "True");
    para->enable_remove_rack_leg = s.getValue<std::string>("obstacle.enable_remove_rack_leg", "False") == "True";

    para->rack_leg_wheel_center_length =
        s.getValue<double>("rack.rack_leg_center_length", 1060) * sros::core::MM_TO_M;
    para->rack_leg_wheel_center_width = s.getValue<double>("rack.rack_leg_center_width", 600) * sros::core::MM_TO_M;
    para->rack_wheel_rotate_radius = s.getValue<double>("rack.rack_wheel_rotate_radius", 150) * sros::core::MM_TO_M;

    LOG(INFO) << "para info:"
    << "install x:" << para->d435_install_x << ",install_y," << para->d435_install_y << ",install_yaw,"
    << para->d435_install_yaw << ",rack_length," << para->rack_leg_wheel_center_length << ",width,"
    << para->rack_leg_wheel_center_width << "," << para->rack_wheel_rotate_radius;
    LOG(INFO) << "halley dabaipro initializeDabaipro done.";

    if(dabaipro_device->registerFrameDataCallback()) {
        LOG(INFO) << "register data callback func OK";
    }
}


void CameraModule::createInstallCalibrationInfo(Eigen::Matrix3d &install_info, const std::string &para_name) {
    auto infos = sros::core::Settings::getInstance().getValue<std::string>(para_name,
                                                                           "1;0;0;0;1;0;0;0;1");
    LOG(INFO) << "infos:" << infos;
    auto values = splitStrsToDoubles(infos, ';');
    if (values.size() == 9) {
        install_info = Eigen::Map<Eigen::Matrix3d>(values.data(), 3, 3);
    } else {
        LOG(ERROR) << "matrix is wrong!";
        install_info.setIdentity();
    }
}

void CameraModule::creatD435Devices(std::vector<CameraDriverInfo> &d435_devices, const std::string& stereo_camera_type) {
    std::vector<std::string> existed_d435;
    auto &s = sros::core::Settings::getInstance();
    bool enable_d435_1 = (s.getValue<std::string>("camera.enable_d435_camera", "True") == "True");
    bool enable_d435_2 = (s.getValue<std::string>("camera.enable_d435_2_camera", "True") == "True");
    bool enable_d435_3 = (s.getValue<std::string>("camera.enable_d435_3_camera", "True") == "True");
    bool enable_d435_4 = (s.getValue<std::string>("camera.enable_d435_4_camera", "True") == "True");
    auto d435_1_info = s.getItemInfo("camera.d435_1_serial_number");
    auto d435_2_info = s.getItemInfo("camera.d435_2_serial_number");
    auto d435_3_info = s.getItemInfo("camera.d435_3_serial_number");
    auto d435_4_info = s.getItemInfo("camera.d435_4_serial_number");
    auto d435_2_using_avoidance = (s.getValue<std::string>("camera.d435_2_using_avoidance", "True") == "True");
    auto d435_3_using_avoidance = (s.getValue<std::string>("camera.d435_3_using_avoidance", "True") == "True");
    auto d435_4_using_avoidance = (s.getValue<std::string>("camera.d435_4_using_avoidance", "True") == "True");

    if (enable_d435_1) {
        existed_d435.push_back(d435_1_info.value);
    }
    else if (enable_d435_2) {
        existed_d435.push_back(d435_2_info.value);
    }
    else if (enable_d435_3) {
        existed_d435.push_back(d435_3_info.value);
    }
    else if (enable_d435_4) {
        existed_d435.push_back(d435_4_info.value);
    }

    auto d435_devices_sn = d435::D435Driver::getAllDevices();
    if (d435_devices_sn.size() <= existed_d435.size()) {
        for (auto &exist : existed_d435) {
            bool finded_d435 = false;
            for (auto &d435 : d435_devices_sn) {
                if (d435 == exist) {
                    finded_d435 = true;
                }
            }
            // if (!finded_d435) {
            //     d435_devices_sn.push_back(exist);
            // }
        }
    }

    std::map<std::string, int> sns_map;
    if (!distributeSerialNumber(d435_devices_sn, sns_map)) {
        LOG(ERROR) << "cannot distribute d435 SN!";
        return;
    }

    //设备容器本身已经包含相机索引信息，故直接通过遍历初始化设备容器
    uint16_t index_count = 0;

    LOG(INFO)<<"sns_map: "<<sns_map.size();
    LOG(INFO)<<"first d435_devices.resize: "<<d435_devices.size();

    d435_devices.resize(sns_map.size());

    LOG(INFO)<<"second d435_devices.resize: "<<d435_devices.size();

    for (auto &d435_sn : sns_map) {
        auto camera_index = d435_sn.second;
        LOG(INFO)<<"camera_index  :"<<camera_index;

        auto &d435_device = d435_devices[index_count++];
        d435_device.camera_index = camera_index;

        LOG(INFO)<<"camera_index  :"<<camera_index<<",,,d435_device.camera_index :"<<d435_device.camera_index ;

        d435_device.camera_sn = d435_sn.first;
        d435_device.stereoCamera_using_avoidance = true;//默认使用避障

        if(!d435_2_using_avoidance && d435_2_info.value == d435_sn.first){
            d435_device.stereoCamera_using_avoidance = d435_2_using_avoidance;
            initializeIRD435(d435_device, stereo_camera_type);
        } else {
            initializeD435(d435_device, stereo_camera_type);
        }
        d435_device.initialize_ok = true;
    }
}

bool CameraModule::createDabaiDeviceManager(void) {
    return HalleyCameraDeviceManager::getInstance()->OpenNIinitialize();
}

void CameraModule::creatDabaiproDevices(std::map<int,CameraDriverInfo> &dabaipro_devices) {

    std::vector<std::string> user_set_camera_serial_number_list;
    std::vector<std::string> scanned_device_uri_list;
    std::vector<std::string> scanned_camera_serial_number_list;

    auto &s = sros::core::Settings::getInstance();
    bool enable_dabaipro_1 = (s.getValue<std::string>("camera.enable_d435_camera", "True") == "True");
    bool enable_dabaipro_2 = (s.getValue<std::string>("camera.enable_d435_2_camera", "True") == "True");
    bool enable_dabaipro_3 = (s.getValue<std::string>("camera.enable_d435_3_camera", "True") == "True");
    bool enable_dabaipro_4 = (s.getValue<std::string>("camera.enable_d435_4_camera", "True") == "True");
    auto dabaipro_1_info = s.getItemInfo("camera.d435_1_serial_number");
    auto dabaipro_2_info = s.getItemInfo("camera.d435_2_serial_number");
    auto dabaipro_3_info = s.getItemInfo("camera.d435_3_serial_number");
    auto dabaipro_4_info = s.getItemInfo("camera.d435_4_serial_number");
    // auto dabaipro_2_using_avoidance = (s.getValue<std::string>("camera.d435_2_using_avoidance", "True") == "True");

    if (enable_dabaipro_1) {
        user_set_camera_serial_number_list.push_back(dabaipro_1_info.value);
    }
    if (enable_dabaipro_2) {
        user_set_camera_serial_number_list.push_back(dabaipro_2_info.value);
    }
    if (enable_dabaipro_3) {
        user_set_camera_serial_number_list.push_back(dabaipro_3_info.value);
    }
    if (enable_dabaipro_4) {
        user_set_camera_serial_number_list.push_back(dabaipro_4_info.value);
    }
    // auto rc1 = astra_dev::AstraDriver::OpenNIAllinitDevices();
    auto scanned_device_info = HalleyCameraDeviceManager::getInstance()->getAllDevices();
    for (auto i = 0; i < scanned_device_info.size()/2; i++)
    {
        scanned_device_uri_list.emplace_back();
        scanned_device_uri_list.back() = scanned_device_info[2*i];

        scanned_camera_serial_number_list.emplace_back();
        scanned_camera_serial_number_list.back() = scanned_device_info[2*i+1];
        // LOG(INFO) << "halley found Uri & SN: "<< scanned_device_uri_list[i] << " " << scanned_camera_serial_number_list[i];
    }

    auto camera_serial_number_list = scanned_camera_serial_number_list;
    if (camera_serial_number_list.size() <= user_set_camera_serial_number_list.size()) {
        for (auto &user_serial : user_set_camera_serial_number_list) {
            bool found_dabaipro = false;
            for (auto &dabaipro : camera_serial_number_list) {
                if (dabaipro == user_serial) {
                    found_dabaipro = true;
                }
            }
            // if (!found_dabaipro) {
            //     camera_serial_number_list.push_back(user_serial);
            // }
        }
    }

    std::map<std::string, int> sns_map;
    if(!distributeSerialNumber(camera_serial_number_list, sns_map)){
        LOG(ERROR) << "cannot distribute halley dabaipro camera SN!";
        return;
    }
    
    // 设备用map替换，防止vector下标越界
    for (auto &dabaipro_sn : sns_map) {
        auto camera_index = dabaipro_sn.second;
        LOG(INFO) << "dabaipro_sn " << dabaipro_sn.first << " index " << dabaipro_sn.second;
        auto &dabaipro_device = dabaipro_devices[camera_index];
        dabaipro_device.camera_index = camera_index;
        dabaipro_device.camera_sn = dabaipro_sn.first; //相机SN码
        dabaipro_device.stereoCamera_using_avoidance = true;//默认是使用避障
        
        for (auto i = 0; i < scanned_device_info.size()/2; i++)
        {
            if(scanned_camera_serial_number_list[i] == dabaipro_device.camera_sn){
                dabaipro_device.serial_number = scanned_device_uri_list[i] ;
                LOG(INFO) << "halley initial config camera serial number: "<< scanned_camera_serial_number_list[i] <<" serial_number:"<< dabaipro_device.serial_number;
            }
        }

        LOG(INFO) << "init dabaipro_device SN: " << dabaipro_device.camera_sn << " Uri:" << dabaipro_device.serial_number; 
        initializeDabaipro(dabaipro_device);
        dabaipro_device.initialize_ok = true;
    }
}


void CameraModule::onEnableD435Msg(sros::core::base_msg_ptr msg) {
    auto &s = sros::core::Settings::getInstance();
    for (auto &d435_info : d435_devices) {
        auto &d435_device = d435_info.camera_device;
        if (!d435_info.initialize_ok) {
            continue;
        }
        if (!d435_info.stereoCamera_using_avoidance) {//如果不使用避障，则不用在这里就行使能和关闭
            continue;
        }
        auto &para = d435_info.para;
        if (g_state.isNeedAvoidObaNavState()||g_state.isManualControl()) {  // 若当前的导航状态不需要避障，就直接返回,目的是节约CPU占用
            para->enable_remove_rack_leg = false;
            if (g_state.load_state == sros::core::LOAD_FULL) {
                para->enable_remove_rack_leg = true;
                bool enable_stereo_points_when_load_full = (
                        s.getValue<std::string>("obstacle.enable_stereo_points_when_load_full", "True") == "True");
                if (!enable_stereo_points_when_load_full) {
                    if (d435_device->isEnabled()) {
                        d435_device->disableCamera();
                    }
                    continue;
                }
            }
            if (!d435_device->isEnabled()) {
                d435_device->enableCamera();
            }
        } else {
            if (d435_device->isEnabled()) {
                d435_device->disableCamera();
            }
        }
    }

    bool enable_record_d435_1 = (s.getValue<std::string>("debug.enable_record_d435_1_data", "False") == "True");
    if (enable_record_d435_1) {
        s.setValue("debug.enable_record_d435_1_data", "False");
        if (d435_devices.size() >= 1) {
            d435_devices[0].camera_device->startRecord();
        }
    }
    bool enable_record_d435_2 = (s.getValue<std::string>("debug.enable_record_d435_2_data", "False") == "True");
    if (enable_record_d435_2) {
        s.setValue("debug.enable_record_d435_2_data", "False");
        if (d435_devices.size() >= 2) {
            d435_devices[1].camera_device->startRecord();
        }
    }
    bool enable_record_d435_3 = (s.getValue<std::string>("debug.enable_record_d435_3_data", "False") == "True");
    if (enable_record_d435_3) {
        s.setValue("debug.enable_record_d435_3_data", "False");
        if (d435_devices.size() >= 3) {
            d435_devices[2].camera_device->startRecord();
        }
    }
    bool enable_record_d435_4 = (s.getValue<std::string>("debug.enable_record_d435_4_data", "False") == "True");
    if (enable_record_d435_4) {
        s.setValue("debug.enable_record_d435_4_data", "False");
        if (d435_devices.size() >= 4) {
            d435_devices[3].camera_device->startRecord();
        }
    }


    double saturation = s.getValue<double>("camera.svc100_saturation",40);
    if(svc_100_down_camera_device){
       svc_100_down_camera_device->setParameter("saturation",saturation);
    }
}

void CameraModule::onEnableDabaiproMsg(sros::core::base_msg_ptr msg) {
    auto &s = sros::core::Settings::getInstance();

    for (auto &dev : dabaipro_devices) {
        auto& dabaipro_info = dev.second;
        auto &dabaipro_device = dabaipro_info.camera_device;
        if (!dabaipro_info.initialize_ok) {
            continue;
        }
        if (!dabaipro_info.stereoCamera_using_avoidance) {//如果不使用避障，则不用在这里就行使能和关闭
            continue;
        }
        auto &para = dabaipro_info.para;
        if (g_state.isNeedAvoidObaNavState()) {  // 若当前的导航状态不需要避障，就直接返回,目的是节约CPU占用
            para->enable_remove_rack_leg = false;
            if (g_state.load_state == sros::core::LOAD_FULL) {
                para->enable_remove_rack_leg = true;
                bool enable_stereo_points_when_load_full = (s.getValue<std::string>("obstacle.enable_stereo_points_when_load_full", "True") == "True");
                if (!enable_stereo_points_when_load_full) {
                    if (dabaipro_device->isEnabled()) {
                        dabaipro_device->disableCamera();
                    }
                    continue;
                }
            }
            if (!dabaipro_device->isEnabled()) {
                dabaipro_device->enableCamera();
            }
        }else{
            if (dabaipro_device->isEnabled()) {
                dabaipro_device->disableCamera();
            }
        }
    }

    bool enable_record_dabaipro_1 = (s.getValue<std::string>("debug.enable_record_d435_1_data", "False") == "True");
    if (enable_record_dabaipro_1) {
        s.setValue("debug.enable_record_d435_1_data", "False");
        if (dabaipro_devices.size() >= 1) {
            dabaipro_devices[0].camera_device->startRecord();
        }
    }
    bool enable_record_dabaipro_2 = (s.getValue<std::string>("debug.enable_record_d435_2_data", "False") == "True");
    if (enable_record_dabaipro_2) {
        s.setValue("debug.enable_record_d435_2_data", "False");
        if (dabaipro_devices.size() >= 2) {
            dabaipro_devices[1].camera_device->startRecord();
        }
    }
}


void CameraModule::onEnableStereoCameraMsg(sros::core::base_msg_ptr msg, const std::vector<CameraDriverInfo> stereo_camera_devices) {
    auto &s = sros::core::Settings::getInstance();

    for (auto &stereo_camera_info : stereo_camera_devices) {
        auto &stereo_camera_device = stereo_camera_info.camera_device;
        if (!stereo_camera_info.initialize_ok) {
            continue;
        }
        if (!stereo_camera_info.stereoCamera_using_avoidance) {//如果不使用避障，则不用在这里就行使能和关闭
            continue;
        }
        auto &para = stereo_camera_info.para;
        if (g_state.isNeedAvoidObaNavState()) {  // 若当前的导航状态不需要避障，就直接返回,目的是节约CPU占用
            para->enable_remove_rack_leg = false;
            if (g_state.load_state == sros::core::LOAD_FULL) {
                para->enable_remove_rack_leg = true;
                bool enable_stereo_points_when_load_full = (s.getValue<std::string>("obstacle.enable_stereo_points_when_load_full", "True") == "True");
                if (!enable_stereo_points_when_load_full) {
                    if (stereo_camera_device->isEnabled()) {
                        stereo_camera_device->disableCamera();
                    }
                    continue;
                }
            }
            if (!stereo_camera_device->isEnabled()) {
                stereo_camera_device->enableCamera();
            }
        }else{
            if (stereo_camera_device->isEnabled()) {
                stereo_camera_device->disableCamera();
            }
        }
    }

    bool enable_record_stereo_camera_1 = (s.getValue<std::string>("debug.enable_record_d435_1_data", "False") == "True");
    if (enable_record_stereo_camera_1) {
        s.setValue("debug.enable_record_d435_1_data", "False");
        if (stereo_camera_devices.size() >= 1) {
            stereo_camera_devices[0].camera_device->startRecord();
        }
    }
    bool enable_record_stereo_camera_2 = (s.getValue<std::string>("debug.enable_record_d435_2_data", "False") == "True");
    if (enable_record_stereo_camera_2) {
        s.setValue("debug.enable_record_d435_2_data", "False");
        if (stereo_camera_devices.size() >= 2) {
            stereo_camera_devices[1].camera_device->startRecord();
        }
    }
}


void CameraModule::creatSvc100Devices(std::vector<CameraDriverInfo> &svc100_devices) {
    auto &s = sros::core::Settings::getInstance();
    auto svc_type = s.getValue<std::string>("camera.svc_device_type", "svc100");
    std::map<std::string, std::string> sns_map;
    if (svc_type == "svc100") {
        distributeSerialNumber(sns_map);
    } else if (svc_type == "svc200") {
        distributeSVC200SerialNumber(sns_map);
    }
    int svc100_led_on_value = s.getValue<int>("camera.svc100_led_on_value", 0);
    auto enable_svc100_camera = (s.getValue<std::string>("camera.enable_svc100_camera", "False") == "True");
    if (enable_svc100_camera) {
        std::string svc100_camera_name = sros::device::DEVICE_SVC100_UP;
        std::string svc100_camera;
        if (svc_type == "svc100") {
            bool use_sn_to_open = true;
            auto camera_sn = sns_map.find(svc100_camera_name);
            if (camera_sn == sns_map.end()) {
                LOG(WARNING) << "cannot open! will use default method to open the camera!" << svc100_camera_name;
                svc100_camera = s.getValue<std::string>(
                        "camera.svc100_up_usb_device",
                        "/dev/v4l/by-path/platform-tegra-xhci-usb-0:2.2.3:1.0-video-index0");
                use_sn_to_open = false;
            } else {
                svc100_camera = camera_sn->second;
            }
            LOG(INFO) << "camera name:" << svc100_camera << "," << svc100_camera_name;
            svc_100_up_camera_device.reset(new SVC100CameraDevice(
                    svc100_camera, use_sn_to_open,
                    std::bind(&CameraModule::sendImgMsg, this, std::placeholders::_1),
                    svc100_camera_name, sros::device::DEVICE_ID_SVC100_UP));

        } else if (svc_type == "svc200") {
            svc100_camera = s.getValue<std::string>("camera.svc100_up_serial_number", "");
            svc_100_up_camera_device.reset(
                    new SVC200CameraDevice(svc100_camera, boost::bind(&CameraModule::sendImgMsg, this, _1),
                                           svc100_camera_name, sros::device::DEVICE_ID_SVC200_UP));
        }
        sros::device::DeviceManager::getInstance()->addDevice(svc_100_up_camera_device);
        svc_100_up_camera_device->setSerialNo(svc100_camera);
        openParticularSVC100(svc_100_up_camera_device, "", svc100_led_on_value);
        svc100_devices.emplace_back();
        svc100_devices.back().camera_sn = svc100_camera;
        svc100_devices.back().camera_index = 1;
        svc100_devices.back().camera_device = svc_100_up_camera_device;
    } else {
        LOG(WARNING) << "SVC100 up device is disabled.";
    }

    auto enable_svc100_down_camera = (s.getValue<std::string>("camera.enable_svc100_down_camera", "False") == "True");
    if (enable_svc100_down_camera) {
        std::string svc100_down_camera;
        std::string svc100_down_camera_name = sros::device::DEVICE_SVC100_DOWN;
        if (svc_type == "svc100") {
            auto down_camera_sn = sns_map.find(svc100_down_camera_name);
            bool use_sn_to_open = true;
            if (down_camera_sn == sns_map.end()) {
                LOG(WARNING) << "cannot open! will use default method to open the camera!"
                             << svc100_down_camera_name;
                svc100_down_camera = s.getValue<std::string>(
                        "camera.svc100_down_usb_device",
                        "/dev/v4l/by-path/platform-tegra-xhci-usb-0:2.2.1:1.0-video-index0");
                use_sn_to_open = false;
            } else {
                svc100_down_camera = down_camera_sn->second;
            }
            LOG(INFO) << "camera name:" << svc100_down_camera << "," << svc100_down_camera_name;
            svc_100_down_camera_device.reset(
                    new SVC100CameraDevice(svc100_down_camera, use_sn_to_open,
                                           boost::bind(&CameraModule::sendImgMsg, this, _1),
                                           svc100_down_camera_name, sros::device::DEVICE_ID_SVC100_DOWN));

        } else if (svc_type == "svc200") {
            svc100_down_camera = s.getValue<std::string>("camera.svc100_down_serial_number", "");
            LOG(INFO) << "svc100_down_camera:"<<svc100_down_camera;
            svc_100_down_camera_device.reset(
                    new SVC200CameraDevice(svc100_down_camera, boost::bind(&CameraModule::sendImgMsg, this, _1),
                                           svc100_down_camera_name, sros::device::DEVICE_ID_SVC200_DOWN));
//                svc_100_down_camera_device -> setParameter("SVC_AUTO_GAIN_EXPO", svc100_led_on_value);

        }
        sros::device::DeviceManager::getInstance()->addDevice(svc_100_down_camera_device);
        svc_100_down_camera_device->setSerialNo(svc100_down_camera);
        openParticularSVC100(svc_100_down_camera_device, "down", svc100_led_on_value);
        svc100_devices.emplace_back();
        svc100_devices.back().camera_sn = svc100_down_camera;
        svc100_devices.back().camera_index = 2;
        svc100_devices.back().camera_device = svc_100_down_camera_device;
    } else {
        LOG(WARNING) << "SVC100 down device is disabled.";
    }

    auto enable_back = (s.getValue<std::string>("camera.enable_back_camera", "False") == "True");
    if (enable_back && svc_type == "svc200") {
        LOG(WARNING) << "!!!!!SVC200 back device enable";
        std::string svc200_camera_name = sros::device::DEVICE_CAMERA_BACKWARD;
        std::string camera_address = s.getValue<std::string>("camera.svc100_back_serial_number", "");
        svc_100_back_camera_device.reset(new SVC200CameraDevice(camera_address,
                                                                boost::bind(&CameraModule::sendImgMsg, this, _1),
                                                                svc200_camera_name,
                                                                sros::device::DEVICE_ID_SVC200_BACK));
        sros::device::DeviceManager::getInstance()->addDevice(svc_100_back_camera_device);
        svc_100_back_camera_device->setSerialNo(camera_address);
        svc100_devices.emplace_back();
        svc100_devices.back().camera_sn = camera_address;
        svc100_devices.back().camera_index = 3;
        svc100_devices.back().camera_device = svc_100_back_camera_device;
    } else {
        LOG(WARNING) << "SVC200 BACK device is disabled.";
    }
}

void CameraModule::creatAzureDevice() {
    auto &s = sros::core::Settings::getInstance();
    auto enable_azure_camera = (s.getValue<std::string>("camera.enable_azure_camera", "True") == "True");
    if (enable_azure_camera) {
        std::string camera_address = s.getValue<std::string>(
                "camera.azure_camera_device_address",
                "/dev/v4l/by-id/usb-Global_Shutter_Camera_Global_Shutter_Camera_01.00.00-video-index0");
        std::string camera_name = sros::device::DEVICE_CAMERA_BACKWARD;
        azure_device.reset(
                new AzureCameraDevice(camera_address, boost::bind(&CameraModule::sendImgMsg, this, _1), camera_name,
                                      sros::device::DEVICE_ID_SVC200_BACKWARD));
        sros::device::DeviceManager::getInstance()->addDevice(azure_device);
        azure_device->open();
        bool enable_when_action =
                (s.getValue<std::string>("camera.azure_enable_publish_image_when_action", "False") == "True");
        if (!enable_when_action) {
            azure_device->enableCamera();
        }
    }
}
//
//    void CameraModule::creatSvc200Device() {
//
//        std::string camera_sn = "3120012109220001";
//        svc_100_down_camera_device.reset(new SVC200CameraDevice(camera_sn,boost::bind(&CameraModule::sendImgMsg, this, _1), sros::device::DEVICE_SVC100_DOWN, sros::device::DEVICE_ID_SVC200_RIGHT));
//        sros::device::DeviceManager::getInstance()->addDevice(svc_100_down_camera_device);
//        svc_100_down_camera_device->enableCamera();
//    }
}

