//
// Created by lfc on 17-7-13.
//

#include <glog/logging.h>
#include <core/settings.h>
#include "simlaser_module.h"
#include "network/simlaser_device.h"
namespace laser{

bool SimlaserModule::doOpen() {
    if (!laser_device) {
        laser_device.reset(new laser::SimLaserDevice);

    }
    int try_count = 0;
    std::string ip_address = "127.0.0.1";
    int port = 5099;
    int time_out = 1000;
    while (try_count < 20) {
        if (laser_device->connect(ip_address, port, time_out)) {
            LOG(INFO) << "successfully to get the address!";
            return true;
        } else {
            laser_device.reset(new laser::SimLaserDevice);
            try_count++;
        }
    }
    LOG(INFO) << "err to connect the ip:" << ip_address;

    return false;
}

void SimlaserModule::doStart() {
    laser_device->setLaserCallback(boost::bind(&SimlaserModule::scanCallback, this, _1));
}

void SimlaserModule::doStop() {

}

void SimlaserModule::doClose() {
}

bool SimlaserModule::getScanData(sros::core::LaserScan_ptr& scan_ptr) {
        boost::unique_lock<boost::mutex> lock(wakeup_mutex);
        boost::xtime xt;
#if BOOST_VERSION >= 105000
        boost::xtime_get(&xt, boost::TIME_UTC_);
#else
        boost::xtime_get(&xt, boost::TIME_UTC);
#endif
        xt.sec += 4;
        if (condition.timed_wait(lock, xt)) {
            boost::mutex::scoped_lock scope_lock(thread_mutex);
//            scan->topic_ = ""
            scan_ptr = scan;
            return true;
        } else {
            return false;
        }
}

void SimlaserModule::scanCallback(std::shared_ptr<connection::BaseMsg> scan_ptr) {
    boost::mutex::scoped_lock scope_lock(thread_mutex);
    scan.reset(new sros::core::LaserScanMsg);
    connection::LaserScanStampedMsg_ptr laser_scan = std::dynamic_pointer_cast<connection::LaserScanStampedMsg>(
            scan_ptr);
    scan->time_ = sros::core::util::get_time_in_us();
    scan->scan_time = laser_scan->scan_time;
    scan->angle_increment = laser_scan->angle_increment;
    scan->time_increment = laser_scan->time_increment;
    scan->angle_max = laser_scan->angle_max;
    scan->angle_min = laser_scan->angle_min;
    scan->intensities = laser_scan->intensities;
    scan->ranges = laser_scan->ranges;
    scan->range_max = laser_scan->range_max;
    scan->range_min = laser_scan->range_min;
    condition.notify_all();
}

}

