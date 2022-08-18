//
// Created by lfc on 16-11-2.
//

#ifndef SROS_BASE_LASER_MODULE_H
#define SROS_BASE_LASER_MODULE_H

#include <core/msg/laser_scan_msg.hpp>

namespace laser {
enum LidarType {
    UTM30LX = 0,
    LMS151 = 1,
    UST10LX = 2,
    OMD30M = 3,
    LMS511 = 4,
    TIM571 = 5,
    GAZEBOSIM = 6,
    OMDUHD = 7,
    BAGSIM = 8,
    SIMINICSPAVO = 10,
    S300 = 11,
    NANO = 13,
    DUAL_OMD = 20,
    DUAL_OMD_PAVO = 21,
    DUAL_PAVO = 22,
    DUAL_S300 = 23,
    MICRO_SCAN3 = 30,
    DUAL_SCAN3 = 31,
    DUAL_NANO = 32,
    DUAL_TIM571 = 33,
    DUAL_ALL = 34,
    WANJI = 40,
    KELI = 41,
    KELI_270 = 42,
    UAM05LP = 44,
    ORADAR_ORBB = 45,
    NOTUSE = 100,
};

class BaseLaserModule {
public:
    BaseLaserModule(){}

    virtual ~BaseLaserModule(){}

    virtual bool doOpen() = 0;

    virtual void doStart() = 0;

    virtual void doStop() = 0;

    virtual void doClose() = 0;

    virtual bool getScanData(sros::core::LaserScan_ptr& scan_ptr) = 0;

    virtual void setHostIpAddress(std::string host_ip){

    }

    virtual int getDeltaTimestampSize() { return 0;}

    virtual std::string getSerialNO() const {
        return "NA";
    }

    virtual std::string getModelNO() const {
        return "NA";
    }

    virtual std::string getVersionNO() const {
        return "NA";
    }
};

}


#endif //SROS_BASE_LASER_MODULE_H
