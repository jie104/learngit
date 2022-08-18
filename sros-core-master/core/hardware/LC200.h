/**
 * @file LC200.h
 *
 * @author cwt
 * @date 2021年10月13日
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */
#ifndef CORE_HARDWARE_LC200_H_
#define CORE_HARDWARE_LC200_H_

#include "LC.h"

namespace sros {
namespace device {

class LC200 : public SRIODevice {
 public:
    LC200(const std::string &name, DeviceID device_id, DeviceCommInterfaceType device_comm_interface_type,
          std::shared_ptr<IOInterface> io_interface);

    class LedState {
     public:
        LedState() = default;

        LedState(LedColor color, LedType type, uint16_t period = 1000) : color(color), type(type), period(period) {}

        void setState(LedColor color, LedType type, uint16_t period = 1000) {
            this->color = color;
            this->type = type;
            this->period = period;
        }

        bool operator==(const LedState &other) {
            return other.color == color && other.type == type && other.period == period;
        }

        LedColor color = LED_OFF;
        LedType type = LED_STATIC;
        uint16_t period = 1000;
    };

    uint32_t getCanId() const { return can_id_; }

    bool setLed(LedNo no, const LedState &led_state);

    bool setLed(LedNo no, LedColor color, LedType type, uint16_t period = 1000);

 protected:
    void onDataRecive(const std::vector<uint8_t> &data) final;  // 处理收到的数据，各个设备需要重写此函数

 private:
    std::vector<uint8_t> buildLedCtrol(LedNo no, LedColor color, LedType type, uint16_t period = 1000) const;

    uint32_t can_id_ = 0x30a;  // 默认是0x30a
    SendMsgCallbackFunc send_msg_func_ = nullptr;

    LedState led_1_old_state_;
    LedState led_2_old_state_;
    LedState led_3_old_state_;
};

typedef std::shared_ptr<LC200> LC200_ptr;
}  // namespace device
}  // namespace sros
#endif  // CORE_HARDWARE_LC200_H_
