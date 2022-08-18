/**
 * @file PMU.cpp
 *
 * @author pengjiali
 * @date 19-9-7.
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#include "PMU.h"
namespace sros {
namespace device {
uint32_t PMU::faultCodeMapping(uint32_t raw_fault_code) {
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
        default:{
            break;
        }
    }

    return new_fault_code;
}
}
}
