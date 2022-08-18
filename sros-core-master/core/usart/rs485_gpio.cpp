//
// Created by lhx on 17-2-22.
//

#include "rs485_gpio.h"

#include "core/util/gpio_sysfs.h"

void init_rs485_gpio() {
    gpio_export(RS485_GPIO_PIN);
    gpio_direction(RS485_GPIO_PIN, GPIO_OUT);
    gpio_write(RS485_GPIO_PIN, GPIO_LOW);
}

void toggle_rs485_mode(RS485_MODE mode) {
    gpio_write(RS485_GPIO_PIN, (mode == RS485_TX) ? GPIO_HIGH : GPIO_LOW);
}

void release_rs485_gpio() {
    gpio_export(RS485_GPIO_PIN);
}
