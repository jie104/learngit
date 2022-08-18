/**
 * @file SH00.h
 *
 * @author pengjiali
 * @date 2019年5月21日
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */
#ifndef CORE_HARDWARE_SH100_H_
#define CORE_HARDWARE_SH100_H_

#include "core/device/SRIODevice.h"

namespace sros {
namespace device {

class TofData {
 public:
    uint16_t distance = 0;  // 取值范围为 (0,1200]，单位cm
    uint16_t strength = 0;  // 信号强度，取值范围为 [20, 2000]
};

using NewTofDataCallback = std::function<void(uint8_t, uint16_t, uint16_t)>;  // (id, distance, strength)

class SH100 : public SRIODevice {
 public:

    enum dev_model{
      SR_DEV_NULL = 0,
      SR_DEV_SH100,
      SR_DEV_SH200,
   };

    SH100(const std::string &name, DeviceID device_id, DeviceCommInterfaceType device_comm_interface_type,
          std::shared_ptr<IOInterface> io_interface);

    //    void getTofData(std::vector<TofData> &data);
    void setTofDataCallback(NewTofDataCallback func) { new_tof_data_callback_ = func; }

    bool syncSetCanPeriod(uint16_t period_ms);  // 设置can周期，取值范围：[10,
                                                // 1000]必须是10的倍数，单位ms，如TOF的上传周期也通过这个来配置
   //获取设备型号
   std::string getDeviceModelStr();

   //获取设备版本
   std::string getDeviceVersionStr();

 protected:
    void onDataRecive(const std::vector<uint8_t> &data) final;

   //获取设备型号
   int getDeviceModel();

   //获取设备版本
   int getDeviceVersion();

 private:
   NewTofDataCallback new_tof_data_callback_ = nullptr;

   AsyncConditionVariable<int> dev_model_variable_;
   AsyncConditionVariable<int> dev_version_variable_;

};

typedef std::shared_ptr<SH100> SH100_ptr;

}  // namespace device
}  // namespace sros

#endif  // CORE_HARDWARE_SH100_H_
