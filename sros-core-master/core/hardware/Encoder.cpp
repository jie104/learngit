/**
 * @file Encoder.cpp
 *
 * @author perry
 * @date 2022-05-18
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#include "Encoder.h"

namespace sros {
namespace device {

// 设置编码器位置值
void Encoder::setPosition(const int32_t _iPos) {
    iPosition = _iPos;
}

// 获取编码器数
int32_t Encoder::getPosition() {
    return iPosition;
}

uint32_t Encoder::faultCodeMapping(uint32_t raw_fault_code) {
    uint32_t new_fault_code = DEVICE_FAULT_OTHER;
    switch (raw_fault_code) {
        case 0x0001: {
            new_fault_code = 410;
            break;
        }
        case 0x0002: {
            new_fault_code = 411;
            break;
        }
        case 0x0004: {
            new_fault_code = 412;
            break;
        }
        case 0x0008: {
            new_fault_code = 413;
            break;
        }
        case 0x0010: {
            new_fault_code = 414;
            break;
        }
        case 0x0020: {
            new_fault_code = 415;
            break;
        }
        case 0x0040: {
            new_fault_code = 416;
            break;
        }
        case 0x0080: {
            new_fault_code = 417;
            break;
        }
        case 0x0100: {
            new_fault_code = 418;
            break;
        }
        case 0x0200: {
            new_fault_code = 419;
            break;
        }
        case 0x0400: {
            new_fault_code = 420;
            break;
        }
        case 0x0800: {
            new_fault_code = 421;
            break;
        }
        case 0x1000: {
            new_fault_code = 422;
            break;
        }
        default:{
            break;
        }
    }

    return new_fault_code;
}
}  // namespace device
}