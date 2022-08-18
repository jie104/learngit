/**
 * @file async_serial_test
 *
 * @author pengjiali
 * @date 2021/4/28.
 *
 * @describe
 *
 * @copyright Copyright (c) 2021 Standard Robots Co., Ltd. All rights reserved.
 */

#include <gtest/gtest.h>
#include <iostream>
#include <thread>
#include "core/usart/async_serial.h"

using namespace std;

TEST(AsyncSerial, AsioBuffer) {
    std::vector<char> v{'1', '2', '3', '4', '5', '6', '7', '8', '9', '0'};
    for (auto i=0; i<v.size(); ++i) {
        std::cout << (int)v[i] << ", ";
    }
    cout << endl;
    auto b = boost::asio::buffer(v.data(), v.size());
    char *p = (char *)b.begin();
    cout << (void*)v.data() << ", " << b.begin() << endl;
    for (auto i=0; i<v.size(); ++i) {
        std::cout << (int)p[i] << ", ";
    }
    cout << endl;
    v[0] = 'a';
    v[1] = 'a';
    v[2] = 'a';
    v[3] = 'a';
    for (auto i=0; i<v.size(); ++i) {
        std::cout << (int)p[i] << ", ";
    }
}

TEST(AsyncSerial, All) {
    CallbackAsyncSerial serial;
    serial.open("/dev/ttyUSB0", 460800);
    auto func = [&]() {
        srand(time(0));

        for (auto i = 0; i < 1e9; ++i) {
            std::vector<char> data(rand() % 500 + 10, 'a');
            serial.write(data);
            std::this_thread::sleep_for(std::chrono::microseconds(30000));
        }
        std::cout << "finish" << std::endl;
    };
    std::thread t1(func);
    std::thread t2(func);
    std::thread t3(func);
    std::thread t4(func);
    std::thread t5(func);
    //    std::thread t6([&](){
    //      std::this_thread::sleep_for(std::chrono::seconds(3));
    //      serial.close();
    //    });

    t1.join();
    t2.join();
    t3.join();
    t4.join();
    t5.join();
    //    t6.join();
}