/**
 * @file motor.cpp
 *
 * @author pengjiali
 * @date 19-9-6.
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#include "motor.h"

namespace sros {
namespace device {

uint32_t Motor::faultCodeMapping(uint32_t raw_fault_code) {
    uint32_t new_fault_code = DEVICE_FAULT_OTHER;
    switch (raw_fault_code) {
        case 0x00000001: {
            new_fault_code = 410;
            break;
        }
        case 0x00000002: {
            new_fault_code = 411;
            break;
        }
        case 0x00000004: {
            new_fault_code = 412;
            break;
        }
        case 0x00000008: {
            new_fault_code = 413;
            break;
        }
        case 0x00000010: {
            new_fault_code = 414;
            break;
        }
        case 0x00000020: {
            new_fault_code = 415;
            break;
        }
        case 0x00000040: {
            new_fault_code = 416;
            break;
        }
        case 0x00000080: {
            new_fault_code = 417;
            break;
        }
        case 0x00000100: {
            new_fault_code = 418;
            break;
        }
        case 0x00000200: {
            new_fault_code = 319;
            break;
        }
        case 0x00000400: {
            new_fault_code = 420;
            break;
        }
        case 0x00000800: {
            new_fault_code = 421;
            break;
        }
        case 0x00001000: {
            new_fault_code = 422;
            break;
        }
        case 0x00002000: {
            new_fault_code = 423;
            break;
        }
        case 0x00004000: {
            new_fault_code = 424;
            break;
        }
        case 0x00008000: {
            new_fault_code = 425;
            break;
        }
        case 0x00010001: {
            new_fault_code = 426;
            break;
        }
        case 0x020001: {
            new_fault_code = 427;
            break;
        }
        case 0x040001: {
            new_fault_code = 428;
            break;
        }
        case 0x080001: {
            new_fault_code = 429;
            break;
        }
        case 0x100001: {
            new_fault_code = 430;
            break;
        }
        case 0x200001: {
            new_fault_code = 431;
            break;
        }
        case 0x400001: {
            new_fault_code = 432;
            break;
        }
        case 0x800001: {
            new_fault_code = 433;
            break;
        }
        case 0x01000001: {
            new_fault_code = 434;
            break;
        }
        case 0x02000001: {
            new_fault_code = 335;
            break;
        }
        case 0x04000001: {
            new_fault_code = 336;
            break;
        }
        case 0x08000001: {
            new_fault_code = 437;
            break;
        }
        case 0x10000001: {
            new_fault_code = 438;
            break;
        }
        case 0x20000001: {
            new_fault_code = 439;
            break;
        }
        case 0x40000001: {
            new_fault_code = 440;
            break;
        }
        case 0x80000001: {
            new_fault_code = 441;
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
