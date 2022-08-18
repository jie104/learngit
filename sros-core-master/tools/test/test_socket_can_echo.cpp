//
// Created by lhx on 18-4-4.
//
// Copyright [2018] Standard Robots Co.,Ltd
//

#include <iostream>
#include <thread>
#include <memory>
#include <vector>
#include <functional>

#include "core/usart/socket_can.hpp"

SocketCan_ptr can;

void can_msg_callback(canid_t recv_can_id, const std::vector<uint8_t> &recv_data) {
    can->send(recv_can_id, recv_data);
}

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cerr << "Usage: ./test_socket_can_echo [DEVICE_NAME]" << std::endl;
    }

    can = std::make_shared<SocketCan>();

    can->open(argv[1], 0);

    can->setMessageCallbackFunc(std::bind(can_msg_callback, std::placeholders::_1, std::placeholders::_2));

    can->startRecv();

    return 0;
}

