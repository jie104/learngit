//
// Created by lfc on 16-11-2.
//

#ifndef SROS_UAM05LP_MODULE_H
#define SROS_UAM05LP_MODULE_H
#include <modules/laser/base_laser_module.h>
#include <math.h>
#include "modules/laser/UTM30LX/hokuyo.h"
#include "modules/laser/UST10LX/urg_c_wrapper.h"
#include "core/circle_optimizer_set.hpp"

namespace laser{


class Uam05lpModule: public BaseLaserModule{
public:

class Config {
public:
    const double PI = 3.1415926;
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

    Uam05lpModule(std::string ip_address,int ip_port);

    virtual ~Uam05lpModule();

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

    bool serialOpen();
private:

    std::shared_ptr<urg_node::URGCWrapper> laser_;
    std::string ip_address_;
    int ip_port_;
//    bool publish_intensity;
    bool publish_multiecho;
    Config config;
//    bool calibrate_time;

    const int max_stamp_cache_size_ = 200;
    
    int64_t last_lidar_stamp = 0;
    circle::CircleOptimizerArray<int64_t> delta_timestamp_;

    // uart
     std::shared_ptr<urg_node::URGCWrapper> serial_laser_;
    hokuyo::Laser hokuyo_serial_laser_;
    hokuyo::LaserScan hokuyo_serial_scan_;
    hokuyo::LaserConfig hokuyo_laser_config_;


};

}



#endif //SROS_UAM05LP_MODULE_H
