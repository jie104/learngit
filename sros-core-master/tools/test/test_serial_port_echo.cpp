//
// Created by lhx on 18-11-15.
//
// Copyright [2018] Standard Robots Co.,Ltd
//

#include <stdlib.h>

#include <iostream>

#include "core/usart/SimpleSerial.hpp"

int main(int argc, char **argv) {
    if (argc < 3) {
        std::cerr << "Usage: ./test_serial_port_echo [DEVICE_NAME] [BAUD RATE]" << std::endl;
        return -1;
    }

    std::cout << "Start echo at " << argv[1] << " " << argv[2] << std::endl;

    SimpleSerial serial(argv[1], strtol(argv[2], NULL, 10));

    while (true) {
        auto c = serial.readChar();
        std::cout << c << std::flush;
        serial.writeChar(c);
    }

    return 0;
}
