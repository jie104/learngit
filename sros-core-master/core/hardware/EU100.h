/**
 * @file EU100.h
 *
 * @author pengjiali
 * @date 2019年5月19日
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */
#ifndef CORE_HARDWARE_EU100_H_
#define CORE_HARDWARE_EU100_H_

#include "core/device/SRIODevice.h"

namespace sros {
namespace device {

enum EU100Type {
    EU100_TYPE_JACKSOLO = 0x11,
    EU100_TYPE_JEUIO = 0x21,
    EU100_TYPE_HD24CAN = 0x31,
    EU100_TYPE_AEROPOD1 = 0x32,
};

using NewEU100IODataCallback = std::function<void(uint8_t, uint8_t)>;  // (DI, DO)

class EU100 : public SRIODevice {
 public:
    EU100(const std::string &name, DeviceID device_id, DeviceCommInterfaceType device_comm_interface_type,
          std::shared_ptr<IOInterface> io_interface);
    //    void getIO(uint8_t &DI, uint8_t &DO);

    void setIODataCallback(NewEU100IODataCallback func) { new_eu100_id_data_callback_ = func; }

    bool syncSetCanPeriod(uint16_t period_ms);  // 设置can周期，取值范围：[10,
                                                // 1000]必须是10的倍数，单位ms，如TOF的上传周期也通过这个来配置

    /**
     * 异步设置EU100 io
     * @param mask 掩码，当某一位为1时表示控制此位，0就不控制此位
     * @param value 需要改变的值
     */
    void setIO(uint8_t mask, uint8_t value);
 protected:
    void onDataRecive(const std::vector<uint8_t> &data) final;

 private:
    NewEU100IODataCallback new_eu100_id_data_callback_ = nullptr;
    EU100Type eu100_type_ = EU100_TYPE_JEUIO;  // eu100 的类型
};

typedef std::shared_ptr<EU100> EU100_ptr;

}  // namespace device
}  // namespace sros

#endif  // CORE_HARDWARE_EU100_H_
