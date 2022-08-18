//
// Created by lhx on 15-12-23.
//

#ifndef SROS_LASER_MODULE_H
#define SROS_LASER_MODULE_H

#include "core/core.h"

#include "core/log_helper.hpp"

#include "UTM30LX/hokuyo.h"

namespace laser {


const double PI = 3.1415926;

class Config {
public:
    const double min_ang = -PI*3/4;
    const double max_ang =  PI*3/4;
    const double time_offset = 0;
    const bool intensity = true;
    const bool calibrate_time = true;
    const bool allow_unsafe_settings = true;
    const int cluster = 1;
    const int skip = 1;
    const std::string port = "/dev/ttyACM0";
};

class LaserModule : public sros::core::Module {
public:
    LaserModule();

    virtual ~LaserModule();

    virtual void run();
private:
    void doOpen();
    void doStart();
    void doStop();
    void doClose();

    void publicScanData(hokuyo::LaserScan& scan);

    hokuyo::Laser laser_;
    hokuyo::LaserScan scan_;
    hokuyo::LaserConfig laser_config_;

    Config config_;
    std::string device_status_;
    std::string device_id_;

    enum DEVICE_STATE {
        OPENED,
        RUNNING,
        CLOSED,
    };
    DEVICE_STATE device_state_;

    int scan_no_;

    int publish_freq_;
};

}

#endif //SROS_LASER_MODULE_H
