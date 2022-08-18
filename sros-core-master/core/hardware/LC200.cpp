/**
 * @file LC200.cpp
 *
 * @author cwt
 * @date 2021年10月13日
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */
#include "LC200.h"
#include <glog/logging.h>
#include <chrono>
#include <thread>

namespace sros {
namespace device {

LC200::LC200(const std::string &name, DeviceID device_id, DeviceCommInterfaceType device_comm_interface_type,
             std::shared_ptr<IOInterface> io_interface)
    : SRIODevice(name, device_id, device_comm_interface_type, io_interface) {
    synCmdReset();
    synCmdStart();
}

/**
 * 设置LED灯光
 * @param no
 * @param led_state
 * @note
 * @return
 */
bool LC200::setLed(LedNo no, const LedState &led_state) {
    static uint32_t led_1_same_count = 0;  // 记录相同的次数
    static uint32_t led_2_same_count = 0;  // 记录相同的次数
    static uint32_t led_3_same_count = 0;  // 记录相同的次数

    switch (no) {
        case LED_1: {
            if (led_1_old_state_ == led_state) {
                ++led_1_same_count;

                if (led_1_same_count >= 20 || led_1_same_count == 1) {
                    led_1_same_count = 1;
                    return asyncRequest(buildLedCtrol(no, led_state.color, led_state.type, led_state.period));
                }
            } else {
                led_1_same_count = 1;

                led_1_old_state_ = led_state;
                return asyncRequest(buildLedCtrol(no, led_state.color, led_state.type, led_state.period));
            }
            break;
        }
        case LED_2: {
            if (led_2_old_state_ == led_state) {
                ++led_2_same_count;

                if (led_2_same_count >= 20 || led_2_same_count == 1) {
                    led_2_same_count = 1;
                    return asyncRequest(buildLedCtrol(no, led_state.color, led_state.type, led_state.period));
                }
            } else {
                led_2_same_count = 1;

                led_2_old_state_ = led_state;
                return asyncRequest(buildLedCtrol(no, led_state.color, led_state.type, led_state.period));
            }
            break;
        }
        case LED_3: {
            if (led_3_old_state_ == led_state) {
                ++led_3_same_count;

                if (led_3_same_count >= 20 || led_3_same_count == 1) {
                    led_3_same_count = 1;
                    return asyncRequest(buildLedCtrol(no, led_state.color, led_state.type, led_state.period));
                }
            } else {
                led_3_same_count = 1;

                led_3_old_state_ = led_state;
                return asyncRequest(buildLedCtrol(no, led_state.color, led_state.type, led_state.period));
            }
            break;
        }
        default: {
            LOG(ERROR) << "UNREACHABLE";  // UNREACHABLE
            break;
        }
    }

    return false;
}

bool LC200::setLed(LedNo no, LedColor color, LedType type, uint16_t period) {
    LedState led_state(color, type, period);
    return setLed(no, led_state);
}

void LC200::onDataRecive(const std::vector<uint8_t> &data) {}

std::vector<uint8_t> LC200::buildLedCtrol(LedNo no, LedColor color, LedType type, uint16_t period) const {
    std::vector<uint8_t> data = {0xAA, (uint8_t)no, (uint8_t)type,          0x00,
                                 0x00, 0x00,        (uint8_t)(period >> 8), (uint8_t)((period)&0xFF)};
    const int red_index = 3;
    const int green_index = 4;
    const int blue_index = 5;

    switch (color) {
        case LED_OFF: {
            data[red_index] = 0x00;
            data[green_index] = 0x00;
            data[blue_index] = 0x00;
            break;
        }
        case LED_RED: {
            data[red_index] = 0xFF;
            data[green_index] = 0x00;
            data[blue_index] = 0x00;
            break;
        }
        case LED_GREEN: {
            data[red_index] = 0x00;
            data[green_index] = 0xFF;
            data[blue_index] = 0x00;
            break;
        }
        case LED_BLUE: {
            data[red_index] = 0x00;
            data[green_index] = 0x00;
            data[blue_index] = 0xFF;
            break;
        }
        case LED_YELLOW: {
            data[red_index] = 0xFF;
            data[green_index] = 0xFF;
            data[blue_index] = 0x00;
            break;
        }
        case LED_MAGENTA: {
            data[red_index] = 0xFF;
            data[green_index] = 0x00;
            data[blue_index] = 0xFF;
            break;
        }
        case LED_CYAN: {
            data[red_index] = 0x00;
            data[green_index] = 0xFF;
            data[blue_index] = 0xFF;
            break;
        }
        case LED_WHITE: {
            data[red_index] = 0xFF;
            data[green_index] = 0xFF;
            data[blue_index] = 0xFF;
            break;
        }
        default: {
            LOG(ERROR) << "UNREACHABLE";  // UNREACHABLE
            break;
        }
    }
    return data;
}

}  // namespace device
}  // namespace sros
