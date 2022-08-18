//
// Created by lfc on 16-11-10.
//

#ifndef SROS_OMD30M_MODULE_H
#define SROS_OMD30M_MODULE_H

#include <modules/laser/base_laser_module.h>
#include "include/pepperl_fuchs_r2000/r2000_driver.h"
#include "../standard_laser_module.h"
#include "core/circle_optimizer_set.hpp"
namespace laser {
class Omd30mModule : public BaseLaserModule{
public:

    Omd30mModule(std::string ip = "192.168.1.100");

    Omd30mModule(LidarType type,std::string ip = "192.168.1.100");

    virtual ~Omd30mModule();

    virtual bool doOpen();

    virtual void doStart();

    virtual void doStop();

    virtual void doClose();

    virtual bool getScanData(sros::core::LaserScan_ptr& scan_ptr);

    std::string getSerialNO() const;

    std::string getModelNO() const;

    std::string getVersionNO() const;

    int getDeltaTimestampSize() { return delta_timestamp_.size();}

private:
    //! IP or hostname of laser range finder
    std::string scanner_ip_;

    //! scan_frequency parameter
    int scan_frequency_;

    //! samples_per_scan parameter
    int samples_per_scan_;

    //! Pointer to driver
    std::shared_ptr<pepperl_fuchs::R2000Driver> driver_;

    //! Serial Number
    std::string serial_no_;

    //! Model number
    std::string model_no_;

    //! Version number
    std::string version_no_;

    const int max_stamp_cache_size_ = 200;
    
    int64_t last_lidar_stamp = 0;
    circle::CircleOptimizerArray<int64_t> delta_timestamp_;
};
}

#endif //SROS_OMD30M_MODULE_H
