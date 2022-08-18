//
// The MIT License (MIT)
//
// Copyright (c) 2019 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "lds_lidar.h"

#include <stdio.h>
#include <string.h>
#include <thread>
#include <memory>

#include "glog/logging.h"

/** Const varible ------------------------------------------------------------------------------- */
/** User add broadcast code here */
static const char* local_broadcast_code_list[] = {
  "000000000000001",
};

/** For callback use only */
LdsLidar* g_lidars = nullptr;

/** Lds lidar function ---------------------------------------------------------------------------*/
LdsLidar::LdsLidar() {
  auto_connect_mode_ = true;
  whitelist_count_   = 0;
  is_initialized_    = false;

  lidar_count_       = 0;
  memset(broadcast_code_whitelist_, 0, sizeof(broadcast_code_whitelist_));

  memset(lidars_, 0, sizeof(lidars_));
  for (uint32_t i=0; i<kMaxLidarCount; i++) {
    lidars_[i].handle = kMaxLidarCount; 
    /** Unallocated state */
    lidars_[i].connect_state = kConnectStateOff;
  }
}

LdsLidar::~LdsLidar() {
}

int LdsLidar::InitLdsLidar(std::vector<std::string>& broadcast_code_strs) {

  if (is_initialized_) {
    LOG(INFO) << "LiDAR data source is already inited!";
    return -1;
  }

  if (!Init()) {
    Uninit();
    LOG(INFO) << "livox init fail!";
    return -1;
  }

  LivoxSdkVersion _sdkversion;
  GetLivoxSdkVersion(&_sdkversion);
  LOG(INFO) << "Livox SDK version "<< _sdkversion.major << "." << _sdkversion.minor << "." << _sdkversion.patch;

  SetBroadcastCallback(LdsLidar::OnDeviceBroadcast);
  SetDeviceStateUpdateCallback(LdsLidar::OnDeviceChange);

  /** Add commandline input broadcast code */
  for (auto input_str : broadcast_code_strs) {
    LdsLidar::AddBroadcastCodeToWhitelist(input_str.c_str());
  }

  /** Add local broadcast code */
  LdsLidar::AddLocalBroadcastCode();

  if (whitelist_count_) {
    LdsLidar::DisableAutoConnectMode();
    LOG(INFO) << "Disable auto connect mode!";

    LOG(INFO) << "List all broadcast code in whiltelist:";
    for (uint32_t i=0; i<whitelist_count_; i++) {
      LOG(INFO) <<  broadcast_code_whitelist_[i];
    }
  } else {
    LdsLidar::EnableAutoConnectMode();
    LOG(INFO) << "No broadcast code was added to whitelist, swith to automatic connection mode!";;
  }

  /** Start livox sdk to receive lidar data */
  if (!Start()) {
    Uninit();
    LOG(INFO) << "livox init fail!";
    return -1;
  }

  /** Add here, only for callback use */
  if (g_lidars == nullptr) {
    g_lidars = this;
  }
  is_initialized_= true;
  LOG(INFO) << "livox init success!";

  return 0;
}



int LdsLidar::DeInitLdsLidar(void) {

  if (!is_initialized_) {
    LOG(INFO) << "LiDAR data source is not exit";
    return -1;
  }

  Uninit();
  LOG(INFO) << "Livox SDK Deinit completely!";

  return 0;
}


/** Static function in LdsLidar for callback or event process ------------------------------------*/

static uint64_t get_time_in_ns() {
  return static_cast<uint64_t>(std::chrono::high_resolution_clock::now().time_since_epoch().count());
}

static uint64_t get_time_in_ms() {
  return get_time_in_ns() / 1000000L;
}

/** Receiving point cloud data from Livox LiDAR. */
void LdsLidar::GetLidarDataCb(uint8_t handle, LivoxEthPacket *data,
                              uint32_t data_num, void *client_data) {
  using namespace std;

  LdsLidar* lidar_this = static_cast<LdsLidar *>(client_data);
  LivoxEthPacket* eth_packet = data;

  if (!data || !data_num || (handle >= kMaxLidarCount)) {
    return;
  }

  if (eth_packet) {
    lidar_this->data_recveive_count_[handle] ++;
//      if (lidar_this->data_recveive_count_[handle] % 1000 == 0) {
//        LOG(INFO) << "receive packet hand=" << unsigned(handle) << " count=" << lidar_this->data_recveive_count_[handle];
//      }
      /** Parsing the timestamp and the point cloud data. */
      uint64_t cur_timestamp = *((uint64_t *)(data->timestamp));
      if(data ->data_type == kCartesian) {
        LivoxRawPoint *p_point_data = (LivoxRawPoint *)data->data;
      }else if ( data ->data_type == kSpherical) {
        LivoxSpherPoint *p_point_data = (LivoxSpherPoint *)data->data;
      }else if ( data ->data_type == kExtendCartesian) {
        LivoxExtendRawPoint *p_point_data = (LivoxExtendRawPoint *) data->data;
        std::lock_guard<std::mutex> lock(lidar_this->mutexs_[handle]);
        LivoxExtendRawPointFrame *frame_handle = &lidar_this->frames_[handle];
        frame_handle->points.resize(data_num);
        for (size_t i = 0; i < data_num; i++) {
          frame_handle->points[i] = p_point_data[i];
        }
        frame_handle->time = get_time_in_ns();
      }else if ( data ->data_type == kExtendSpherical) {
        LivoxExtendSpherPoint *p_point_data = (LivoxExtendSpherPoint *)data->data;
      }else if ( data ->data_type == kDualExtendCartesian) {
        LivoxDualExtendRawPoint *p_point_data = (LivoxDualExtendRawPoint *)data->data;
      }else if ( data ->data_type == kDualExtendSpherical) {
        LivoxDualExtendSpherPoint *p_point_data = (LivoxDualExtendSpherPoint *)data->data;
      }else if ( data ->data_type == kImu) {
        LivoxImuPoint *p_point_data = (LivoxImuPoint *)data->data;
      }else if ( data ->data_type == kTripleExtendCartesian) {
        LivoxTripleExtendRawPoint *p_point_data = (LivoxTripleExtendRawPoint *)data->data;
      }else if ( data ->data_type == kTripleExtendSpherical) {
        LivoxTripleExtendSpherPoint *p_point_data = (LivoxTripleExtendSpherPoint *)data->data;
      }
  }
}

void LdsLidar::OnDeviceBroadcast(const BroadcastDeviceInfo *info) {
  if (info == nullptr || g_lidars == nullptr) {
    return;
  }

  if (info->dev_type == kDeviceTypeHub) {
    LOG(INFO) << "In lidar mode, couldn't connect a hub : " << info->broadcast_code;
    return;
  }

  if (g_lidars->IsAutoConnectMode()) {
    LOG(INFO) << "In automatic connection mode, will connect " << info->broadcast_code;
  } else {
    if (!g_lidars->FindInWhitelist(info->broadcast_code)) {
      LOG(INFO) << "Not in the whitelist, please add "<<info->broadcast_code<<" to if want to connect!";
      return;
    }
  }

  livox_status result = kStatusFailure;
  uint8_t handle = 0;
  result = AddLidarToConnect(info->broadcast_code, &handle);
  if (result == kStatusSuccess && handle < kMaxLidarCount) {
    SetDataCallback(handle, LdsLidar::GetLidarDataCb, (void *)g_lidars);
    LidarDevice* p_lidar = &(g_lidars->lidars_[handle]);
    p_lidar->handle = handle;
    p_lidar->connect_state = kConnectStateOff;
    p_lidar->config.enable_fan = true;
    p_lidar->config.return_mode = kStrongestReturn;
    p_lidar->config.coordinate = kCoordinateCartesian;
    p_lidar->config.imu_rate = kImuFreq200Hz;
  } else {
    LOG(INFO) << "Add lidar to connect is failed : " << result << " " << handle;
  }
}

/** Callback function of changing of device state. */
void LdsLidar::OnDeviceChange(const DeviceInfo *info, DeviceEvent type) {
  if (info == nullptr) {
    return;
  }

  uint8_t handle = info->handle;
  if (handle >= kMaxLidarCount) {
    return;
  }

  LidarDevice* p_lidar = &(g_lidars->lidars_[handle]);
  if (type == kEventConnect) {
    QueryDeviceInformation(handle, DeviceInformationCb, g_lidars);
    if (p_lidar->connect_state == kConnectStateOff) {
      p_lidar->connect_state = kConnectStateOn;
      p_lidar->info = *info;
    }
    LOG(INFO) << "[WARNING] Lidar sn: [" << info->broadcast_code << "] Connect!!!";
  } else if (type == kEventDisconnect) {
    p_lidar->connect_state = kConnectStateOff;
    LOG(INFO) << "[WARNING] Lidar sn: ["<< info->broadcast_code << "] Disconnect!!!";
  } else if (type == kEventStateChange) {
    p_lidar->info = *info;
    LOG(INFO) << "[WARNING] Lidar sn: [" << info->broadcast_code <<"] StateChange!!!";
    LOG(INFO) << "LidarState=" << p_lidar->info.state;
  }

  if (p_lidar->connect_state == kConnectStateOn) {
    LOG(INFO) << "Device Working State " << p_lidar->info.state;
    if (p_lidar->info.state == kLidarStateInit) {
      LOG(INFO) << "Device State Change Progress " << p_lidar->info.status.progress;
    } else {
      LOG(INFO) << "Device State Error Code " << std::hex << p_lidar->info.status.status_code.error_code;
    }
    LOG(INFO) << "Device feature " << p_lidar->info.feature;
    SetErrorMessageCallback(handle, LdsLidar::LidarErrorStatusCb);

    /** Config lidar parameter */
    if (p_lidar->info.state == kLidarStateNormal) {
      if (p_lidar->config.coordinate != 0) {
        SetSphericalCoordinate(handle, LdsLidar::SetCoordinateCb, g_lidars);
      } else {
        SetCartesianCoordinate(handle, LdsLidar::SetCoordinateCb, g_lidars);
      }
      p_lidar->config.set_bits |= kConfigCoordinate;

      if (kDeviceTypeLidarMid40 != info->type) {
        LidarSetPointCloudReturnMode(handle, (PointCloudReturnMode)(p_lidar->config.return_mode),\
                                     LdsLidar::SetPointCloudReturnModeCb, g_lidars);
        p_lidar->config.set_bits |= kConfigReturnMode;
      }

      if (kDeviceTypeLidarMid40 != info->type && kDeviceTypeLidarMid70 != info->type) {
        LidarSetImuPushFrequency(handle, (ImuFreq)(p_lidar->config.imu_rate),\
                                 LdsLidar::SetImuRatePushFrequencyCb, g_lidars);
        p_lidar->config.set_bits |= kConfigImuRate;
      }

      p_lidar->connect_state = kConnectStateConfig;
    }
  }
}

/** Query the firmware version of Livox LiDAR. */
void LdsLidar::DeviceInformationCb(livox_status status, uint8_t handle, \
                         DeviceInformationResponse *ack, void *client_data) {
  if (status != kStatusSuccess) {
    LOG(INFO) << "Device Query Informations Failed : " << status;
  }
  if (ack) {
    LOG(INFO) << "firm ver: "
              << ack->firmware_version[0] << "."
              << ack->firmware_version[1] << "."
              << ack->firmware_version[2] << "."
              << ack->firmware_version[3];
  }
}

/** Callback function of Lidar error message. */
void LdsLidar::LidarErrorStatusCb(livox_status status, uint8_t handle, ErrorMessage *message) {
  static uint32_t error_message_count = 0;
  if (message != NULL) {
    ++error_message_count;
    if (0 == (error_message_count % 100)) {
      LOG(INFO) << "handle: " << handle;
      LOG(INFO) << "temp_status : " << message->lidar_error_code.temp_status;
      LOG(INFO) << "volt_status : " << message->lidar_error_code.volt_status;
      LOG(INFO) << "motor_status : " << message->lidar_error_code.motor_status;
      LOG(INFO) << "dirty_warn : " << message->lidar_error_code.dirty_warn;
      LOG(INFO) << "firmware_err : " << message->lidar_error_code.firmware_err;
      LOG(INFO) << "pps_status : " << message->lidar_error_code.device_status;
      LOG(INFO) << "fan_status : " << message->lidar_error_code.fan_status;
      LOG(INFO) << "self_heating : " << message->lidar_error_code.self_heating;
      LOG(INFO) << "ptp_status : " << message->lidar_error_code.ptp_status;
      LOG(INFO) << "time_sync_status : " << message->lidar_error_code.time_sync_status;
      LOG(INFO) << "system_status : " << message->lidar_error_code.system_status;
    }
  }
}

void LdsLidar::ControlFanCb(livox_status status, uint8_t handle, \
                           uint8_t response, void *client_data) {

}

void LdsLidar::SetPointCloudReturnModeCb(livox_status status, uint8_t handle, \
                                         uint8_t response, void *client_data) {
  LdsLidar* lds_lidar = static_cast<LdsLidar *>(client_data);

  if (handle >= kMaxLidarCount) {
    return;
  }
  LidarDevice* p_lidar = &(lds_lidar->lidars_[handle]);

  if (status == kStatusSuccess) {
    p_lidar->config.set_bits &= ~((uint32_t)(kConfigReturnMode));
    LOG(INFO) << "Set return mode success!";

    if (!p_lidar->config.set_bits) {
      LidarStartSampling(handle, LdsLidar::StartSampleCb, lds_lidar);
      p_lidar->connect_state = kConnectStateSampling;
    }
  } else {
    LidarSetPointCloudReturnMode(handle, (PointCloudReturnMode)(p_lidar->config.return_mode),\
                                 LdsLidar::SetPointCloudReturnModeCb, lds_lidar);
    LOG(INFO) << "Set return mode fail, try again!";
  }
}

void LdsLidar::SetCoordinateCb(livox_status status, uint8_t handle, \
                               uint8_t response, void *client_data) {
  LdsLidar* lds_lidar = static_cast<LdsLidar *>(client_data);

  if (handle >= kMaxLidarCount) {
    return;
  }
  LidarDevice* p_lidar = &(lds_lidar->lidars_[handle]);

  if (status == kStatusSuccess) {
    p_lidar->config.set_bits &= ~((uint32_t)(kConfigCoordinate));
    LOG(INFO) << "Set coordinate success!";

    if (!p_lidar->config.set_bits) {
       LidarStartSampling(handle, LdsLidar::StartSampleCb, lds_lidar);
       p_lidar->connect_state = kConnectStateSampling;
    }
  } else {
    if (p_lidar->config.coordinate != 0) {
      SetSphericalCoordinate(handle, LdsLidar::SetCoordinateCb, lds_lidar);
    } else {
      SetCartesianCoordinate(handle, LdsLidar::SetCoordinateCb, lds_lidar);
    }

    LOG(INFO) << "Set coordinate fail, try again!";
  }
}

void LdsLidar::SetImuRatePushFrequencyCb(livox_status status, uint8_t handle, \
                                         uint8_t response, void *client_data) {
  LdsLidar* lds_lidar = static_cast<LdsLidar *>(client_data);

  if (handle >= kMaxLidarCount) {
    return;
  }
  LidarDevice* p_lidar = &(lds_lidar->lidars_[handle]);

  if (status == kStatusSuccess) {
    p_lidar->config.set_bits &= ~((uint32_t)(kConfigImuRate));
    LOG(INFO) << "Set imu rate success!";

    if (!p_lidar->config.set_bits) {
      LidarStartSampling(handle, LdsLidar::StartSampleCb, lds_lidar);
      p_lidar->connect_state = kConnectStateSampling;
    }

  } else {
    LidarSetImuPushFrequency(handle, (ImuFreq)(p_lidar->config.imu_rate),\
                             LdsLidar::SetImuRatePushFrequencyCb, lds_lidar);
    LOG(INFO) << "Set imu rate fail, try again!";
  }
}

/** Callback function of starting sampling. */
void LdsLidar::StartSampleCb(livox_status status, uint8_t handle, \
                             uint8_t response, void *client_data) {
  LdsLidar* lds_lidar = static_cast<LdsLidar *>(client_data);

  if (handle >= kMaxLidarCount) {
    return;
  }

  LidarDevice* p_lidar = &(lds_lidar->lidars_[handle]);
  if (status == kStatusSuccess) {
    if (response != 0) {
      p_lidar->connect_state = kConnectStateOn;
      LOG(INFO) << "Lidar start sample fail : state["<< status <<"] handle["<< handle <<"] res["<< response <<"]";
    } else {
      LOG(INFO) << "Lidar start sample success";
   }
  } else if (status == kStatusTimeout) {
    p_lidar->connect_state = kConnectStateOn;
    LOG(INFO) << "Lidar start sample timeout : state["<< status <<"] handle["<< handle <<"] res["<< response <<"]";
  }
}

/** Callback function of stopping sampling. */
void LdsLidar::StopSampleCb(livox_status status, uint8_t handle, \
                            uint8_t response, void *client_data) {
}

/** Add broadcast code to whitelist */
int LdsLidar::AddBroadcastCodeToWhitelist(const char* bd_code) {
  if (!bd_code || (strlen(bd_code) > kBroadcastCodeSize) || \
      (whitelist_count_ >= kMaxLidarCount)) {
    return -1;
  }

  if (LdsLidar::FindInWhitelist(bd_code)) {
    LOG(INFO) << bd_code << " is alrealy exist!";
    return -1;
  }

  strcpy(broadcast_code_whitelist_[whitelist_count_], bd_code);
  ++whitelist_count_;

  return 0;
}

void LdsLidar::AddLocalBroadcastCode(void) {
  for (size_t i=0; i<sizeof(local_broadcast_code_list)/sizeof(intptr_t); ++i) {
    std::string invalid_bd = "000000000";
    LOG(INFO) << "Local broadcast code : " << local_broadcast_code_list[i];
    if ((kBroadcastCodeSize == strlen(local_broadcast_code_list[i]) + 1) && \
        (nullptr == strstr(local_broadcast_code_list[i], invalid_bd.c_str()))) {
      LdsLidar::AddBroadcastCodeToWhitelist(local_broadcast_code_list[i]);
    } else {
      LOG(INFO) << "Invalid local broadcast code : " << local_broadcast_code_list[i];
    }
  }
}

bool LdsLidar::FindInWhitelist(const char* bd_code) {
  if (!bd_code) {
    return false;
  }

  for (uint32_t i=0; i<whitelist_count_; i++) {
    if (strncmp(bd_code, broadcast_code_whitelist_[i], kBroadcastCodeSize) == 0) {
      return true;
    }
  }

  return false;
}

bool LdsLidar::GetFrame(const int handle, LivoxExtendRawPointFrame &frame) {
    if (!is_initialized_) {
        LOG(INFO) << "is_initialized=false";
        return false;
    }
    if (frames_[handle].points.empty()) {
        LOG(INFO) << "frames_[handle].points is empty";
        return false;
    }
    std::lock_guard<std::mutex> lock(mutexs_[handle]);
    LivoxExtendRawPointFrame *frame_handle = &frames_[handle];
    frame.points.resize(frame_handle->points.size());
    copy(frame_handle->points.begin(), frame_handle->points.end(), frame.points.begin());
    frame.time = frame_handle->time;
    return true;
}
