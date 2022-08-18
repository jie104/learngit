/**
 * @file SH200.h
 *
 * @author perry
 * @date 2021年9月13日
 *
 * @describe
 *
 * @copyright Copyright (c) 2021 Standard Robots Co., Ltd. All rights reserved.
 */
#ifndef CORE_HARDWARE_SH200_H_
#define CORE_HARDWARE_SH200_H_

#include "core/device/SRIODevice.h"

namespace sros {
namespace device {


using SH200TofDataCallback = std::function<void(uint8_t, uint16_t, uint16_t)>;  // (id, distance, strength)

class SH200 : public SRIODevice {
 public:
   enum dev_model{
      SR_DEV_NULL = 0,
      SR_DEV_SH100,
      SR_DEV_SH200,
   };

   enum tof_type {
      SR_TYPE_NULL = 0,
      SR_TYPE_VL53L1X,
      SR_TYPE_TFMINI_PLUS,
   };

   enum tof_dist_mode {
      SR_NULL = 0,
      SR_VL53L1X_DISTANCEMODE_SHORT,
      SR_VL53L1X_DISTANCEMODE_MEDIUM,  //默认中等
      SR_VL53L1X_DISTANCEMODE_LONG,
   };

   SH200(const std::string &name, DeviceID device_id, DeviceCommInterfaceType device_comm_interface_type,
          std::shared_ptr<IOInterface> io_interface);

   void setTofDataCallback(SH200TofDataCallback func) { sh200_tof_data_callback_ = func; }

   //初始化协议
   void initProtocol();
    
   //获取设备型号
   std::string getDeviceModelStr();

   //获取设备版本
   std::string getDeviceVersionStr();

 protected:
   void onDataRecive(const std::vector<uint8_t> &data) final;

 private:
   //激活设备
   bool openDevice();

   //关闭设备
   bool closeDevice();

   //获取设备型号
   int getDeviceModel();

   //获取设备版本
   int getDeviceVersion();

   //进入配置模式
   bool enterConfigMode();

   //退出配置模式
   bool leaveConfigMode();

   //配置TOF数量
   bool setTofNums(uint8_t tof_nums);

   //配置TOF设备类型
   bool setTofDevType(tof_type tof_type);

   //配置TOF测距模式
   bool setTofDistMode(tof_dist_mode tof_dist_mode);

 private:
    SH200TofDataCallback sh200_tof_data_callback_ = nullptr;
    AsyncConditionVariable<int> dev_model_variable_;
    AsyncConditionVariable<int> dev_version_variable_;
};

typedef std::shared_ptr<SH200> SH200_ptr;

}  // namespace device
}  // namespace sros

#endif  // CORE_HARDWARE_SH200_H_
