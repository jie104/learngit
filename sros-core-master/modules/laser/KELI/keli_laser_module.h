//
// Created by lfc on 2020/3/23.
//

#ifndef SROS_KELI_LASER_MODULE_H
#define SROS_KELI_LASER_MODULE_H
#include <modules/laser/base_laser_module.h>

namespace laser{
template <class ScanMsg> class KeliDriver;

class KeliLaserModule: public BaseLaserModule {
 public:
    KeliLaserModule(std::string ip_address,int ip_port);

    virtual ~KeliLaserModule();

    virtual bool doOpen();

    virtual void doStart();

    virtual void doStop();

    virtual void doClose();

    virtual bool getScanData(sros::core::LaserScan_ptr& scan_ptr);

    std::string getSerialNO() const;

    std::string getModelNO() const;

    std::string getVersionNO() const;

 private:
    std::shared_ptr<KeliDriver<sros::core::LaserScanMsg>> keli_driver_;
    std::string ip_address_;
    int ip_port_;
};

}

#endif  // SROS_KELI_LASER_MODULE_H
