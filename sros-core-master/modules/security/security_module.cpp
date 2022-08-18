/**
 * @file security_module
 *
 * @author pengjiali
 * @date 20-1-21.
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#include "security_module.h"
#include <glog/logging.h>
#include <boost/asio.hpp>
#include "core/settings.h"
#include "core/logger.h"

using namespace boost;
using asio::ip::tcp;

namespace security {

void SecurityModule::run() {
    LOG(INFO) << "SecurityModule module start running";

    auto &s = sros::core::Settings::getInstance();
    if (s.getValue<std::string>("main.vehicle_controller_type", "VC300") != "VC400" &&
        s.getValue<int>("main.security_unit_type", 1) != 3) {
        LOG(INFO) << "vehicle controller type is not VC400 or not CE security unit, disable this module.";
        stop();
        return;
    }

    boost::thread t(boost::bind(&SecurityModule::handleSecurityState, this));

    LOG(INFO) << "SecurityModule module dispatch!";

    dispatch();
}

void SecurityModule::handleSecurityState() {
    try {
        asio::io_service io_service;

        tcp::socket s(io_service);
        tcp::resolver resolver(io_service);
        asio::connect(s, resolver.resolve({"192.168.1.251", "9100"}));

        const int frame_length = 60; // 协议帧长度

        char reply[frame_length];
        for (;;) {
            size_t reply_length = asio::read(s, asio::buffer(reply, frame_length));

            // FIXME(pengjiali): 此处为了适配PLC，没有按照标准的来，下一版改
            uint8_t safety_state = reply[38];
            if (safety_state == 0x04) {
                safety_state = 0x03;
            }
            uint16_t emergency_src = reply[39];
            //LOG(INFO) << "handleSecurityState safety_state: " << std::hex << (int)safety_state << ", emergency_src: 0x" << std::hex << emergency_src;
            security_state_handle_.setNewState((sros::core::EmergencyState)safety_state, emergency_src);
        }
    } catch (std::exception& e) {
        std::cerr << "Exception: " << e.what() << "\n";
    }
}


}
