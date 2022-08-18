//
// Created by lhx on 17-2-22.
//

#ifndef SROS_RS485_GPIO_H
#define SROS_RS485_GPIO_H
#include "ChipVersion.h"

enum RS485_MODE {
    RS485_RX,
    RS485_TX,
};

#if (CHIP_VERSION == 1)
    #define RS485_GPIO_PIN 234
#else
    #define RS485_GPIO_PIN 412
#endif

void init_rs485_gpio();

void toggle_rs485_mode(RS485_MODE mode);

void release_rs485_gpio();

#endif //SROS_RS485_GPIO_H
