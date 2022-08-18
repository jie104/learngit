//
// Created by lfc on 2019/12/18.
//
#include "ifm3d_driver.hpp"
#include <core/util/time.h>
#include <ifm3d/camera.h>
#include <ifm3d/fg.h>
#include <ifm3d/image.h>
#include <ifm3d/camera/err.h>
#include "core/state.h"

namespace ifm {

    // 默认的配置(JSON-string格式), 包含两组"Imager"参数,分别是用于标定的,和用于正常工作状态的
    const json JSON_DEFAULT_O3X_CONFIG =
    R"(
         {
            "ifm3d":
            {
              "Apps":
              [
                {
                  "Description": "normal running mode",
                  "Imager": {
                      "ContinuousAutoExposure": "false",
                      "EnableFilterAmplitudeImage": "true",
                      "EnableFilterDistanceImage": "true",
                      "ExposureTime": "2000",
                      "ExposureTimeList": "100;2000",
                      "FrameRate": "30",
                      "Resolution": "1",
                      "SpatialFilter": {
                          "MaskSize": "0"
                        },
                      "SpatialFilterType": "1",
                      "TemporalFilter": {},
                      "TemporalFilterType":"0",
                      "Type":"upto30m_moderate"
                  },
                  "Index": "1",
                  "Name": "Detect_Card_Application",
                  "TriggerMode": "2"
                },
                {
                  "Description": "calibration mode",
                  "Imager": {
                      "ContinuousAutoExposure": "false",
                      "EnableFilterAmplitudeImage": "true",
                      "EnableFilterDistanceImage": "true",
                      "ExposureTime": "2200",
                      "ExposureTimeList": "2200",
                      "FrameRate": "5",
                      "Resolution": "1",
                      "SpatialFilter": {
                          "MaskSize": "0"
                        },
                      "SpatialFilterType": "1",
                      "TemporalFilter": {
                        "AdaptiveExponentialStrength": "1"
                      },
                      "TemporalFilterType":"2",
                      "Type":"under5m_low"
                  },
                  "Index": "2",
                  "Name": "calibr_params",
                  "TriggerMode": "1"
                }
              ]
            }
          }
      )"_json;

inline uint64_t get_time_in_ns() {
    return static_cast<uint64_t>(std::chrono::high_resolution_clock::now().time_since_epoch().count());
}



Ifm3dDriver::Ifm3dDriver() {
    camera_ip_ = "192.168.71.69";
    xmlrpc_port_ = (int)ifm3d::DEFAULT_XMLRPC_PORT;
    password_ = ifm3d::DEFAULT_PASSWORD;
    int schema_mask = (int)ifm3d::DEFAULT_SCHEMA_MASK;
    serial_number_str_ = "";
}
bool Ifm3dDriver::init(uint16_t mask) {
    std::lock_guard<std::mutex> lock(this->mutex_);
    opened_ = false;

    try {
        LOG(INFO) << "Running dtors...";
        this->im_.reset();
        this->fg_.reset();
        this->cam_.reset();
        LOG(INFO) << "ip:" << camera_ip_;
        LOG(INFO) << "port:" << xmlrpc_port_;
        LOG(INFO) << "passwd:" << password_;
        LOG(INFO) << "Initializing camera...";
        this->cam_ = ifm3d::Camera::MakeShared(this->camera_ip_, this->xmlrpc_port_, this->password_);
        sleep(1);

        LOG(INFO) << "Initializing framegrabber... mask: " << mask;
        this->fg_ = std::make_shared<ifm3d::FrameGrabber>(cam_, mask);

        LOG(INFO) << "Initializing image buffer...";
        this->im_ = std::make_shared<ifm3d::ImageBuffer>();
        if (cam_->IsO3X()) {
            serial_number_str_ = "O3X";
        } else {
            serial_number_str_ = "O3D";
        }
        opened_ = true;
        LOG(INFO) << "opened success.";
    } catch (const ifm3d::error_t& ex) {
        LOG(ERROR) << "ifm3d error: " << ex.code() << " " << ex.what();
        this->im_.reset();
        this->fg_.reset();
        this->cam_.reset();
        opened_ = false;
        if(IFM3D_XMLRPC_TIMEOUT == ex.code()) {
            LOG(INFO) << "Can not ping the camera, waiting 1 second ...";
            sleep(1);
        }
    }catch (...){
        LOG(ERROR) << "unexpet err!";
        this->im_.reset();
        this->fg_.reset();
        this->cam_.reset();
        opened_ = false;
    }

    return opened_;
}

bool Ifm3dDriver::getFrame(uint64_t& stamp, cv::Mat& confidence_img, cv::Mat& xyz_img) {
    std::lock_guard<std::mutex> lock(this->mutex_);
    bool retval = false;
    if (!fg_ || !im_ || !cam_) {
        LOG(INFO) << "have not initialize!";
        return false;
    }
    try {
        if(getSoftwareTriggerModeEnabled()) {
            fg_->SWTrigger();
        }
        retval = fg_->WaitForFrame(im_.get(), timeout_millis_);
        stamp = sros::core::util::get_time_in_us();
        if (retval) {
            xyz_img = im_->XYZImage();
            confidence_img = im_->ConfidenceImage();
            if (xyz_img.empty()) {
                LOG(INFO) << "xyz empty!";
            }
            if (confidence_img.empty()) {
                LOG(INFO) << "conf empty!";
            }
        } else {
            LOG(INFO) << "cannot get img!";
        }
    } catch (const std::exception& ex) {
        LOG(ERROR)  << ": " << ex.what();
        retval = false;
    }catch (...){
        LOG(ERROR) << "unexception error!";
        retval = false;
    }
    return retval;
}

bool Ifm3dDriver::getFrame(uint64_t& stamp, cv::Mat& confidence_img,
                           cv::Mat &amplitude_img, cv::Mat& xyz_img, float& fork_height) {
    std::lock_guard<std::mutex> lock(this->mutex_);
    bool retval = false;
    if (!fg_ || !im_ || !cam_) {
        LOG(INFO) << "have not initialize!";
        return false;
    }
    try {
        if(getSoftwareTriggerModeEnabled()) {
            fg_->SWTrigger();
        }
        stamp = sros::core::util::get_time_in_us(); // 将时间戳定在软触发时候 changed by lijunhong 20211228
        retval = fg_->WaitForFrame(im_.get(), timeout_millis_);

//        LOG(ERROR) << "get o3d3xx("<<stamp<<") frame time point: " << sros::core::util::get_time_in_us();

        if (retval) {
            xyz_img = im_->XYZImage();
            confidence_img = im_->ConfidenceImage();
            amplitude_img = im_->AmplitudeImage();
            if (xyz_img.empty()) {
                LOG(INFO) << "xyz empty!";
            }
            if (confidence_img.empty()) {
                LOG(INFO) << "conf empty!";
            }
            if (amplitude_img.empty()) {
                LOG(INFO) << "ampl empty!";
            }
        } else {
            LOG(INFO) << "cannot get img!";
        }
    } catch (const std::exception& ex) {
        LOG(ERROR)  << ": " << ex.what();
        retval = false;
    }catch (...){
        LOG(ERROR) << "unexception error!";
        retval = false;
    }
    return retval;
}

bool Ifm3dDriver::initIfm3dParam() {
    init_param_flag_ = false;
    bool retval = false;
    int idx = -1;
    int error_code = 0;
    try {
        if (cam_->IsO3X()) {
            LOG(WARNING) << "O3X device initIfm3dParam OK";
            init_param_flag_ = true;
            return true;
        }

        // if only one apps, copy from index 1 and then init
        json app_list = cam_->ApplicationList();
        if(app_list.size() <= 1) {
            // check if idx 1 exist or active, if NOT, create a default one
            idx = cam_->ActiveApplication();
            LOG(INFO) << "active idx: " << idx;
            if(1 != idx) {
                std::vector<std::string> app_types = cam_->ApplicationTypes();
                for(auto& s : app_types) {
                    idx = cam_->CreateApplication(s);
                    app_list = cam_->ApplicationList();
                    LOG(INFO) << "app_list.size(): " << app_list.size();
                }
            }
            LOG(INFO) << "copy apps from index " << idx;
            cam_->CopyApplication(idx);
            app_list = cam_->ApplicationList();
            if(app_list.size() >= 2) {
                LOG(INFO) << "copy apps success";
            }
        }

        LOG(INFO) << "set default config start";
        cam_->FromJSON(JSON_DEFAULT_O3X_CONFIG);
        LOG(INFO) << "set default config done";
        
        // Mark the default application as active
        setAvtiveAppImagerIndex(DEFAULT_NORMAL_INDEX);
        retval = true;
        // LOG(INFO) << "read out param";
        // json dump = cam_->ToJSON();
        // LOG(INFO) << "dump: apps 0 Type:" << dump["ifm3d"]["Apps"][0]["Imager"]["Type"].get<std::string>();
        // LOG(INFO) << "dump: apps 1 Type:" << dump["ifm3d"]["Apps"][1]["Imager"]["Type"].get<std::string>();
    } catch (const ifm3d::error_t& ex) {
        LOG(ERROR) << "error: " << ex.code() << " " << ex.what();
        error_code = ex.code();
        retval = false;
    } catch (const std::exception& ex) {
        LOG(ERROR)  << ": " << ex.what();
        retval = false;
    }catch (...){
        LOG(ERROR) << "unexception error!";
        retval = false;
    }
    if(!retval) {
        if (IFM3D_SESSION_ALREADY_ACTIVE == error_code) {
            retval = rebootIfm3dCamera();
        } else if(IFM3D_INVALID_APP_INDEX == error_code) {
            LOG(WARNING) << "create default Application";
            std::vector<std::string> app_types = cam_->ApplicationTypes();
            for(auto& s : app_types) {
                idx = cam_->CreateApplication(s);
            }
        }
        LOG(ERROR) << "initIfm3dParam error !!!"; 
        return false;
    }
    init_param_flag_ = true;
    LOG(INFO) << "initIfm3dParam successful"; 
    return true;
}

bool Ifm3dDriver::setIfm3dCalibrParam(std::string name_str[], std::string value_str[], int size) {
    bool flag_s = false;
    flag_s = setAvtiveAppImagerIndex(1);
    return flag_s;
}

bool Ifm3dDriver::setIfm3dParam(std::string name, std::string value) {

    bool flag_s = false;
    int idx = cam_->ActiveApplication();
    json config = cam_->ToJSON();
    if(name.compare("MaskSize") == 0) {
        std::string dataToString = config["ifm3d"]["Apps"][idx - 1]["Imager"]["SpatialFilterType"];

        if(atoi(dataToString.c_str()) > 0 && atoi(dataToString.c_str()) <= 3) {
            dataToString = config["ifm3d"]["Apps"][idx - 1]["Imager"]["SpatialFilter"][name];
            LOG(INFO) << name.c_str() << " = " << dataToString.c_str();
            config["ifm3d"]["Apps"][idx - 1]["Imager"]["SpatialFilter"][name] = value;
            cam_->FromJSON(config);
            flag_s = true;
        }
        else {
            LOG(ERROR) << "没有打开空间滤波器 ！！！";
        }
    }
    else if (name.compare("MedianLength") == 0 || name.compare("AdaptiveExponentialStrength") == 0) {
        std::string dataToString = config["ifm3d"]["Apps"][idx - 1]["Imager"]["TemporalFilterType"];

        if(name.compare("MedianLength") == 0) {
            if(atoi(dataToString.c_str()) == 3) {
                dataToString = config["ifm3d"]["Apps"][idx - 1]["Imager"]["TemporalFilter"][name];
                LOG(INFO) << name.c_str() << " = " << dataToString.c_str();
                config["ifm3d"]["Apps"][idx - 1]["Imager"]["TemporalFilter"][name] = value;
                cam_->FromJSON(config);
                flag_s = true;
            }
            else {
                LOG(ERROR) << "没有打开临时滤波器类型：中值滤波器 ！！！";
            }
        }

        if(name.compare("AdaptiveExponentialStrength") == 0) {
            if(atoi(dataToString.c_str()) == 2) {
                dataToString = config["ifm3d"]["Apps"][idx - 1]["Imager"]["TemporalFilter"][name];
                LOG(INFO) << name.c_str() << " = " << dataToString.c_str();
                config["ifm3d"]["Apps"][idx - 1]["Imager"]["TemporalFilter"][name] = value;
                cam_->FromJSON(config);
                flag_s = true;
            }
            else {
                LOG(ERROR) << "没有打开临时滤波器类型：指数滤波器 ！！！";
            }
        }
    }
    else if(name.compare("TriggerMode") == 0) {
        std::string dataToString = config["ifm3d"]["Apps"][idx - 1][name];
        LOG(INFO) << name.c_str() << " = " << dataToString.c_str();
        config["ifm3d"]["Apps"][idx - 1][name] = value;
        cam_->FromJSON(config);
        flag_s = true;
    }
    else {
        if(name.compare("ExposureTime") == 0 || name.compare("ExposureTimeRatio") == 0 || name.compare("ContinuousAutoExposure") == 0) {
            std::string dataToString = config["ifm3d"]["Apps"][idx - 1]["Imager"]["Type"];
            if(dataToString.find("high") == -1) {
                dataToString = config["ifm3d"]["Apps"][idx - 1]["Imager"][name];
                LOG(INFO) << name.c_str() << " = " << dataToString.c_str();
                config["ifm3d"]["Apps"][idx - 1]["Imager"][name] = value;
                cam_->FromJSON(config);
                flag_s = true;

                return flag_s;
            }
            else {
                LOG(ERROR) << "处于高曝光模式，无法设置 ！！！";
                return flag_s;
            }
        }
        else if(name.compare("Type") == 0) {
            std::string dataToString = config["ifm3d"]["Apps"][idx - 1]["Imager"]["ContinuousAutoExposure"];
            if(dataToString.compare("false") == 0 || (dataToString.compare("true") == 0 && value.find("high") == -1)) {
                dataToString = config["ifm3d"]["Apps"][idx - 1]["Imager"][name];
                LOG(INFO) << name.c_str() << " = " << dataToString.c_str();
                config["ifm3d"]["Apps"][idx - 1]["Imager"][name] = value;
                cam_->FromJSON(config);
                flag_s = true;

                return flag_s;
            }
            else {
                LOG(ERROR) << "处于自动曝光模式，无法设置 ！！！";
                return flag_s;
            }
        }

        std::string dataToString = config["ifm3d"]["Apps"][idx - 1]["Imager"][name];
        LOG(INFO) << name.c_str() << " = " << dataToString.c_str();
        config["ifm3d"]["Apps"][idx - 1]["Imager"][name] = value;
        cam_->FromJSON(config);
        flag_s = true;
    }

    return flag_s;
}

bool Ifm3dDriver::getIfm3dParam(std::string name) {
    
    bool flag_g = false;
    int idx = cam_->ActiveApplication();
    json config = cam_->ToJSON();
    // std::string testData = config.dump(4);
    // printf("config = \n%s\n", testData.c_str());
    if(name.compare("MaskSize") == 0) {
        std::string dataToString = config["ifm3d"]["Apps"][idx - 1]["Imager"]["SpatialFilterType"];
        if(atoi(dataToString.c_str()) > 0 && atoi(dataToString.c_str()) <= 3) {
            dataToString = config["ifm3d"]["Apps"][idx - 1]["Imager"]["SpatialFilter"][name];
            LOG(INFO) << name.c_str() << " = " << dataToString.c_str();
            flag_g = true;
        }
        else {
            LOG(ERROR) << "没有打开空间滤波器 ！！！";
        }
    }
    else if(name.compare("MedianLength") == 0 || name.compare("AdaptiveExponentialStrength") == 0) {
        std::string dataToString = config["ifm3d"]["Apps"][idx - 1]["Imager"]["TemporalFilterType"];
        if(name.compare("MedianLength") == 0) {
            if(atoi(dataToString.c_str()) == 3) {
                dataToString = config["ifm3d"]["Apps"][idx - 1]["Imager"]["TemporalFilter"][name];
                LOG(INFO) << name.c_str() << " = " << dataToString.c_str();
                flag_g = true;
            }
            else {
                LOG(ERROR) << "没有打开临时滤波器：中值滤波器 ！！！";
            }
        }

        if(name.compare("AdaptiveExponentialStrength") == 0) {
            if(atoi(dataToString.c_str()) == 2) {
                dataToString = config["ifm3d"]["Apps"][idx - 1]["Imager"]["TemporalFilter"][name];
                LOG(INFO) << name.c_str() << " = " << dataToString.c_str();
                flag_g = true;
            }
            else {
                LOG(ERROR) << "没有打开临时滤波器：指数滤波器 ！！！";
            }
        }
    }
    else if(name.compare("TriggerMode") == 0) {
        std::string dataToString = config["ifm3d"]["Apps"][idx - 1][name];
        LOG(INFO) << name.c_str() << " = " << dataToString.c_str();
        flag_g = true;
    }
    else {
        if(name.compare("ExposureTime") == 0 || name.compare("ExposureTimeRatio") == 0) {
            std::string dataToString = config["ifm3d"]["Apps"][idx - 1]["Imager"]["Type"];
            if(dataToString.find("high") == -1) {
                dataToString = config["ifm3d"]["Apps"][idx - 1]["Imager"][name];
                LOG(INFO) << name.c_str() << " = " << dataToString.c_str();
                flag_g = true;

                return flag_g;
            }
            else {
                LOG(ERROR) << "处于高曝光模式，无法单独获得曝光值 ！！！";
                return flag_g;
            }
        }
        std::string dataToString = config["ifm3d"]["Apps"][idx - 1]["Imager"][name];
        LOG(INFO) << name.c_str() << " = " << dataToString.c_str();
        flag_g = true;
    }
    return flag_g;
}

int Ifm3dDriver::getAvtiveAppsIndex()
{
    int idx = -1;
    json app_list = cam_->ApplicationList();
    for (auto& a : app_list)
    {
      if (a["Active"].get<bool>())
        {
          idx = a["Index"].get<int>();
          break;
        }
    }
    if(-1 == idx) {
        LOG(INFO) << "no app config active. set the default ImagerIndex active.";
        setAvtiveAppImagerIndex(DEFAULT_NORMAL_INDEX);
        idx = 1;
    }
    LOG(INFO) << "read: app_list.size(): " << app_list.size() << " active idx: " << idx;
    return (idx > 0) ? (idx - 1) : 0;
}

bool Ifm3dDriver::setAvtiveAppImagerIndex(int index)
{
    try{
        if (cam_->IsO3X()) {
            LOG(WARNING) << "O3X set param OK";
            software_trigger_enabled_ = false;
            return true;
        }
        int idx = cam_->ActiveApplication();
        if(idx == index) {
            LOG(INFO) << "already active. ImagerIndex: " << idx;
        } else {
            // The rest of the test is invalid for O3X
            std::string json_str;
            if(DEFAULT_NORMAL_INDEX == index) {
                json_str = R"({"Device":{"ActiveApplication":"1"}})";
            } else if(DEFAULT_CALIB_INDEX== index) {
                json_str = R"({"Device":{"ActiveApplication":"2"}})";
            }
            cam_->FromJSONStr(json_str);
            LOG(INFO) << "set new active ImagerIndex: " << index;
        }
        // set the flag
        if(DEFAULT_CALIB_INDEX == index) {
            software_trigger_enabled_ = false;
        } else if(DEFAULT_NORMAL_INDEX == index) {
            software_trigger_enabled_ = true;
        }
    } catch (const std::exception& ex) {
        LOG(ERROR)  << ": " << ex.what();
        return false;
    }catch (...){
        LOG(ERROR) << "unexception error!";
        return false;
    }
    return true;
}

bool Ifm3dDriver::setIfm3dAppsImagerExposureTemporary(int exposurreTimeUs, int exposureTimeRatio) {
    if(exposurreTimeUs < 0) {
      LOG(WARNING) << "invalid param!!!";
      return false;
    }
    bool retval = false;
    try {
        cam_->RequestSession();
        
        std::unordered_map<std::string, std::string> params =
        {
          { "imager_001/ExposureTime", std::to_string(exposurreTimeUs) },
          { "imager_001/ExposureTimeRatio", "10" }
        };
        if(exposureTimeRatio > 0) {
          params["imager_001/ExposureTimeRatio"] = std::to_string(exposureTimeRatio);
        }
        cam_->SetTemporaryApplicationParameters(params);
        LOG(INFO) << "set new temporary ExposureTime: " << params["imager_001/ExposureTime"];
        LOG(INFO) << "new temporary ExposureTimeRatio: " << params["imager_001/ExposureTimeRatio"];

        cam_->CancelSession();
        retval = true;
    } catch (const std::exception& ex) {
        LOG(ERROR)  << ": " << ex.what();
        retval = false;
    }catch (...){
        LOG(ERROR) << "unexception error!";
        retval = false;
    }
    return retval;
}

bool Ifm3dDriver::setIfm3dAppsImagerExposureParams(int exposurreTimeUs, int exposureTimeRatio) {
    if((exposurreTimeUs <= 0) || (exposureTimeRatio < 0)) {
      LOG(WARNING) << "invalid param!!!";
      return false;
    }
    bool retval = false;
    try {
    	  LOG(INFO) << "read out param";
        json dump = cam_->ToJSON();
        if (dump["/ifm3d/Apps"_json_pointer].empty()) {
            return retval;
        }
        int idx = getAvtiveAppsIndex(); 
        dump["ifm3d"]["Apps"][idx]["Index"] = std::to_string(idx+1);
        dump["ifm3d"]["Apps"][idx]["Imager"]["ExposureTime"] = std::to_string(exposurreTimeUs);
        if(exposureTimeRatio > 0) {
          dump["ifm3d"]["Apps"][idx]["Imager"]["ExposureTimeRatio"] = std::to_string(exposureTimeRatio);
        }

        LOG(INFO) << "new Index: " << dump["ifm3d"]["Apps"][idx]["Index"];
        LOG(INFO) << "new ExposureTime: " << dump["ifm3d"]["Apps"][idx]["Imager"]["ExposureTime"] ;
        LOG(INFO) << "new ExposureTimeRatio: " << dump["ifm3d"]["Apps"][idx]["Imager"]["ExposureTimeRatio"] ;

        LOG(INFO) << "write " << "idx: " << idx;
        cam_->FromJSON(dump);
        LOG(INFO) << "write config end";
        retval = true;
    } catch (const std::exception& ex) {
        LOG(ERROR)  << ": " << ex.what();
        retval = false;
    }catch (...){
        LOG(ERROR) << "unexception error!";
        retval = false;
    }
    return retval;
}


bool Ifm3dDriver::setIfm3dAppsImagerTypeParam(const std::string str_type_param) {
    if (str_type_param.empty() || str_type_param.find("_") == std::string::npos) {
      LOG(WARNING) << "invalid param!!!";
      return false;
    }
    bool retval = false;
    try {
    	  LOG(INFO) << "read out param";
        json dump = cam_->ToJSON();
        if (dump["/ifm3d/Apps"_json_pointer].empty()) {
            return retval;
        }
        int idx = getAvtiveAppsIndex();
        dump["ifm3d"]["Apps"][idx]["Imager"]["Type"] = str_type_param;
        LOG(INFO) << "new Type: " << dump["ifm3d"]["Apps"][idx]["Imager"]["Type"] ;

        LOG(INFO) << "write " << "idx: " << idx;
        cam_->FromJSON(dump);
        LOG(INFO) << "write config end";
        retval = true;
    } catch (const std::exception& ex) {
        LOG(ERROR)  << ": " << ex.what();
        retval = false;
    }catch (...){
        LOG(ERROR) << "unexception error!";
        retval = false;
    }
    return retval;
}

/**
  * read and write json config
  *         
  * @param[in] str_json_config  example:
        std::string strJsonConfig =
          R"(
            {
              "ifm3d":
              {
                "Apps":
                [
                  {
                  "Index": "1",  // 不可省略,否则会产生新的 Application
                  "Imager": {
                    "ExposureTime": "2000",
                    "ExposureTimeRatio": "10",
                    "Type":"under5m_low"
                    }
                  }
                ]
              }
            }
          )";
*/
bool Ifm3dDriver::setIfm3dJsonStrParam(const std::string str_json_config) {
    if (str_json_config.empty() || str_json_config.find("Apps") == std::string::npos) {
      return false;
    }
    bool retval = false;
    try {
        int idx = cam_->ActiveApplication();
        LOG(INFO) << "write " << "idx: " << idx;
        if(!str_json_config.empty()) {
          LOG(INFO) << str_json_config;
          cam_->FromJSONStr(str_json_config);
        }
        LOG(INFO) << "write config end";
        retval = true;
    } catch (const std::exception& ex) {
        LOG(ERROR)  << ": " << ex.what();
        retval = false;
    }catch (...){
        LOG(ERROR) << "unexception error!";
        retval = false;
    }
    return retval;
}

// !谨慎使用!
// 设置 SW 触发模式, 会报错 : vector::_M_fill_insert 
// 在 SW 模式, 调用 fg->SWTrigger(); 后 WaitForFrame 会超时 > 3秒,或出错.
// 在 FREE_RUN 模式, 调用 fg->SWTrigger(); 正常, 但有提示:Are you software triggering in free-run mode?
bool Ifm3dDriver::setSoftwareTriggerMode(const bool enable) {
    bool retval = false;
    try {
        if (cam_->IsO3X()) {
            LOG(WARNING) << "O3X device no software trigger";
            software_trigger_enabled_ = false;
            return true;
        }
        // mark the current active application as sw triggered
        int idx = cam_->ActiveApplication();
        if(idx < 0) {
            return false;
        }
        json config = cam_->ToJSON();
        if(enable) {
            config["ifm3d"]["Apps"][idx - 1]["TriggerMode"] = std::to_string(static_cast<int>(ifm3d::Camera::trigger_mode::SW));
        } else {
            config["ifm3d"]["Apps"][idx - 1]["TriggerMode"] = std::to_string(static_cast<int>(ifm3d::Camera::trigger_mode::FREE_RUN));
        }
        cam_->FromJSON(config);
        LOG(INFO) << "write config end";
        software_trigger_enabled_ = enable;
        retval = true;
    } catch (const std::exception& ex) {
        LOG(ERROR)  << ": " << ex.what();
        retval = false;
    }catch (...){
        LOG(ERROR) << "unexception error!";
        retval = false;
    }
    return retval;
}

bool Ifm3dDriver::getSoftwareTriggerModeEnabled() {
    return software_trigger_enabled_;
}

bool Ifm3dDriver::rebootIfm3dCamera() {
    bool retval = false;
    try {
        LOG(WARNING)  << "Reboot ifm3d camera ...";
        cam_->Reboot(ifm3d::Camera::boot_mode::PRODUCTIVE);
        this->im_.reset();
        this->fg_.reset();
        this->cam_.reset();
        opened_ = false;
        retval = true;
        LOG(WARNING)  << "Reboot ifm3d camera waiting >5 seconds ...";
        sleep(5);
    } catch (const std::exception& ex) {
        LOG(ERROR)  << ": " << ex.what();
        retval = false;
    }catch (...){
        LOG(ERROR) << "unexception error!";
        retval = false;
    }
    if(!retval) {
        LOG(ERROR) << "Reboot error!";
        return false;
    }
    return retval;
}

const std::string Ifm3dDriver::getSerialNumberStr() {
    return serial_number_str_;
}

const std::string Ifm3dDriver::getDeviceNameStr() {
    return "IFM3D";
}

} // namespace ifm
