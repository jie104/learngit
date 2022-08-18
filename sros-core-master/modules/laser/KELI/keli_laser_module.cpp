//
// Created by lfc on 2020/3/23.
//

#include "keli_laser_module.h"
#include "keli_driver.hpp"
namespace laser{

KeliLaserModule::KeliLaserModule(std::string ip_address,int ip_port):ip_address_(ip_address),ip_port_(ip_port) {
    keli_driver_.reset(new KeliDriver<sros::core::LaserScanMsg>(ip_address, std::to_string(ip_port)));
}
KeliLaserModule::~KeliLaserModule() {

}
bool KeliLaserModule::doOpen() {return keli_driver_->open(); }
void KeliLaserModule::doStart() {
    keli_driver_->start();
}
void KeliLaserModule::doStop() {
    keli_driver_->close();
}
void KeliLaserModule::doClose() { keli_driver_->close(); }
bool KeliLaserModule::getScanData(sros::core::LaserScan_ptr& scan_ptr) {
    if (keli_driver_->ok()) {
        return keli_driver_->getScan(scan_ptr);
    }else{
        LOG(ERROR) << "cannot open keli driver! will reopen!";
        doClose();
        if (doOpen()) {
            LOG(INFO) << "successfully to open the lidar!";
            doStart();
        }else{
            LOG(ERROR) << "cannot open! will sleep for 1s and retry!";
            sleep(1);
        }
    }
}
std::string KeliLaserModule::getSerialNO() const { return "1.1.1"; }
std::string KeliLaserModule::getModelNO() const { return "1.1.1"; }
std::string KeliLaserModule::getVersionNO() const { return "1.1.1"; }
}