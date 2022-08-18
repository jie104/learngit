//
// Created by lfc on 16-11-3.
//

#ifndef SROS_UST10LX_MODULE_H
#define SROS_UST10LX_MODULE_H

#include <modules/laser/base_laser_module.h>
#include <math.h>
#include "urg_c_wrapper.h"

namespace laser {
class Ust10lxModule : public BaseLaserModule {
public:
    class Config {
    public:
        const double min_ang = -M_PI * 3 / 4;
        const double max_ang = M_PI * 3 / 4;
        const double time_offset = 0;
        const bool intensity = true;
        const bool calibrate_time = true;
        const bool allow_unsafe_settings = true;
        const int cluster = 1;
        const int skip = 1;
        const std::string port = "/dev/ttyACM0";
    };

    Ust10lxModule();

    virtual ~Ust10lxModule();

    virtual bool doOpen();

    virtual void doStart();

    virtual void doStop();

    virtual void doClose();

    virtual bool getScanData(sros::core::LaserScan_ptr& scan_ptr);

    virtual std::string getSerialNO() const {
        return laser_->getDeviceID();
    }

    virtual std::string getModelNO() const {
        return laser_->getProductName();
    }

    virtual std::string getVersionNO() const {
        return laser_->getProtocolVersion();
    }

private:
    std::shared_ptr<urg_node::URGCWrapper> laser_;
    std::string ip_address;
    int ip_port;
//    bool publish_intensity;
    bool publish_multiecho;
    Config config;
//    bool calibrate_time;



};
}


#endif //SROS_UST10LX_MODULE_H
