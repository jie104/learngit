//
// Created by lhx on 18-4-4.
//
// Copyright 2018 StandardRobots
//

#include <iostream>
#include <thread>
#include <fstream>
#include <memory>
#include <vector>
#include <functional>

#include "core/usart/socket_can.hpp"

int msg_send_cnt = 0;
int msg_recv_cnt = 0;

bool result = false;

void can_msg_callback(canid_t, const std::vector<uint8_t> &recv_data) {
    std::vector<uint8_t> send_data {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};

    if (recv_data == send_data) {
        msg_recv_cnt += 1;
    }
}

bool can_test_case() {

    msg_send_cnt = 0;
    msg_recv_cnt = 0;

    auto can = std::make_shared<SocketCan>();

    can->open("can0", 2); // 读取超时设置为2s

    can->setMessageCallbackFunc(std::bind(can_msg_callback, std::placeholders::_1, std::placeholders::_2));

    std::thread t1([can] {
        can->startRecv();

//        std::cout << "t1 finished" << std::endl;
    });

    std::thread t2([can] {
        std::vector<uint8_t> data{0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};

        const int TOTAL_CNT = 10000;
        const int PERCENTAGE = (TOTAL_CNT / 100);
        const int SEND_INTERVAL_TIME = 50; // ms

        result = false;

        for (int i = 0; i < TOTAL_CNT; i++) {
            can->send(0x202, data);
            msg_send_cnt += 1;

            std::this_thread::sleep_for(std::chrono::milliseconds(SEND_INTERVAL_TIME));

            if (i % PERCENTAGE == 0) {
                std::cout << i / PERCENTAGE << "%\r" << std::flush;
            }

            if (msg_send_cnt != msg_recv_cnt) {
                std::cerr << "test case failed at " << msg_send_cnt << std::endl;

                can->close();

                result = false;

                return;
            }
        }

        result = true;
//        std::cout << "t2 finished" << std::endl;

        can->close();
    });

    t2.join();
    t1.join();

    return result;
}

int main(int argc, char **argv) {

    // 阻止glog输出日志到stderr
    google::InitGoogleLogging(argv[0]);    // 初始化log，参数argv[0]就是程序名
    google::SetStderrLogging(google::FATAL);
    FLAGS_logtostderr = false;
    FLAGS_alsologtostderr = false;

    std::ofstream out;

    const int CASE_CNT = 100;
    for (int i = 0; i < CASE_CNT; i++) {
        out.open("test_can_result.txt", std::ios::out);

        auto r = can_test_case();

        if (r) {
            out << "#" << i << "/" << CASE_CNT << " passed." << std::endl;
            std::cout  << "#" << i << "/" << CASE_CNT << " passed." << std::endl;
        } else {
            out << "#" << i << "/" << CASE_CNT << " failed." << std::endl;
            std::cout  << "#" << i << "/" << CASE_CNT << " failed." << std::endl;
        }

        out.close();


        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }

}

