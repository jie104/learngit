#ifndef CORE_HARDWARE_LC_H_
#define CORE_HARDWARE_LC_H_

#include <functional>
#include <memory>
#include <vector>
#include "core/device/SRIODevice.h"

namespace sros {
namespace device {

class DeviceManager;

enum LedNo {
    LED_1 = 0x71,
    LED_2 = 0x72,
    LED_3 = 0x73,
};

enum LedType {
    LED_STATIC = 0x01,
    LED_BLINK = 0x02,
    LED_BREATH = 0x03,
};

enum LedColor {
    LED_OFF = 0,
    LED_RED = 1,      // RED = M+Y
    LED_GREEN = 2,    // GREEN = C+Y
    LED_BLUE = 4,     // BLUE = M+C
    LED_YELLOW = 3,   // YELLOW = R+G
    LED_MAGENTA = 5,  // Magenta = R+B 洋红
    LED_CYAN = 6,     // Cyan = G+B 青色
    LED_WHITE = 7,    // white = R+G+B
};

using SendMsgCallbackFunc = std::function<bool(uint32_t, const std::vector<uint8_t> &)>;
}
}

#endif