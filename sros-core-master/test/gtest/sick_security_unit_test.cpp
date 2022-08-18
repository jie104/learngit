/**
 * @file sick_security_unit_test
 *
 * @author pengjiali
 * @date 20-1-20.
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#include <gtest/gtest.h>
#include <boost/asio.hpp>
#include <cstdlib>
#include <cstring>
#include <iostream>

using namespace boost;
using asio::ip::tcp;

enum { max_length = 60 };

TEST(sick_security_unit, all) {
    try {
        asio::io_service io_service;

        tcp::socket s(io_service);
        tcp::resolver resolver(io_service);
        asio::connect(s, resolver.resolve({"192.168.1.251", "9100"}));

        char reply[max_length];
        for (auto i = 0; i < 1; ++i) {
            size_t reply_length = asio::read(s, asio::buffer(reply, max_length));
            std::cout << "Reply is: ";
            for (auto j = 0; j < max_length; ++j) {
                std::cout << "0x" << std::hex << (uint)reply[j] << " ";
            }
            std::cout << "\n";

            std::cout << "emergency state is " << (int)reply[37] << "; emergency source is " << (int)reply[38]
                      << std::endl;
        }
    } catch (std::exception& e) {
        std::cerr << "Exception: " << e.what() << "\n";
    }
}