/**
 * @file EAC.cpp
 *
 * @author pengjiali
 * @date 2019年5月21日
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */
#include "EAC.h"

namespace sros {
namespace device {

uint32_t EAC::faultCodeMapping(uint32_t raw_fault_code) {
    uint32_t new_fault_code = DEVICE_FAULT_OTHER;
    switch (raw_fault_code) {
        case 19256: {
            new_fault_code = 310;
            break;
        }
        case 19257: {
            new_fault_code = 311;
            break;
        }
        case 19258: {
            new_fault_code = 312;
            break;
        }
        case 19259: {
            new_fault_code = 313;
            break;
        }
        default: {
            break;
        }
    }

    return new_fault_code;
}

}  // namespace device
}  // namespace sros
