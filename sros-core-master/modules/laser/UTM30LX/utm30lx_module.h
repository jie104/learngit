//
// Created by lfc on 16-11-2.
//

#ifndef SROS_UTM30LX_MODULE_H
#define SROS_UTM30LX_MODULE_H
#include <modules/laser/base_laser_module.h>
#include <math.h>
#include "hokuyo.h"

namespace laser{
const double PI = 3.1415926;
class Config {
public:
    const double min_ang = -PI*3.0/4.0;
    const double max_ang =  PI*3.0/4.0;
    const double time_offset = 0;
    const bool intensity = true;
    const bool calibrate_time = true;
    const bool allow_unsafe_settings = true;
    const int cluster = 1;
    const int skip = 1;
    const std::string port = "/dev/ttyACM0";
};
class Utm30lxModule: public BaseLaserModule{
public:

    Utm30lxModule();

    virtual ~Utm30lxModule();

    virtual bool doOpen();

    virtual void doStart();

    virtual void doStop();

    virtual void doClose();

    virtual bool getScanData(sros::core::LaserScan_ptr& scan_ptr);

private:
    hokuyo::Laser laser_;
    hokuyo::LaserScan scan_;
    hokuyo::LaserConfig laser_config_;

    Config config_;
    std::string device_status_;
    std::string device_id_;

};
}



#endif //SROS_UTM30LX_MODULE_H
