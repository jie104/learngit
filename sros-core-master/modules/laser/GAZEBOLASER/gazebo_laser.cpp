//
// Created by lfc on 17-5-21.
//

#include <glog/logging.h>
#include "gazebo_laser.h"
#include "src/gazebo_laser_device.h"

namespace laser {

GazeboLaserModule::GazeboLaserModule() {

}

void GazeboLaserModule::doClose() {
    gazebo_laser.reset();
}

bool GazeboLaserModule::doOpen() {
    if (!gazebo_laser) {
        gazebo_laser.reset(new src::GazeboLaserDevice);
    }
    int try_count = 0;
    std::string ip_address = "127.0.0.1";
    int port = 5099;
    int time_out = 1000;
    while (try_count < 20) {
        if (gazebo_laser->connect(ip_address, port, time_out)) {
            LOG(INFO) << "successfully to get the address!";
            return true;
        } else {
            gazebo_laser.reset(new src::GazeboLaserDevice);
            try_count++;
        }
    }
    LOG(INFO) << "err to connect the ip:" << ip_address;
    return false;

}

void GazeboLaserModule::doStop() {
    gazebo_laser.reset();
}

bool GazeboLaserModule::getScanData(sros::core::LaserScan_ptr &scan_ptr) {
    boost::unique_lock<boost::mutex> lock(wakeup_mutex);
    boost::xtime xt;
#if BOOST_VERSION >= 105000
    boost::xtime_get(&xt, boost::TIME_UTC_);
#else
    boost::xtime_get(&xt, boost::TIME_UTC);
#endif
    xt.sec += 4;
    if (condition.timed_wait(lock, xt)) {
        scan_ptr = last_scan;
        return true;
    } else {
        LOG(INFO) << "wait time long! will return false!" ;
        return false;
    }
}

GazeboLaserModule::~GazeboLaserModule() {

}

void GazeboLaserModule::getGazeboScan(network::GazeboLaserScanMsg_ptr scan) {
    last_scan.reset(new sros::core::LaserScanMsg);
    last_scan->time_ = sros::core::util::get_time_in_us();
    last_scan->scan_time = scan->scan_time;
    last_scan->angle_increment = scan->angle_increment;
    last_scan->time_increment = scan->time_increment;
    last_scan->angle_max = scan->angle_max;
    last_scan->angle_min = scan->angle_min;
    last_scan->intensities = scan->intensities;
    last_scan->ranges = scan->ranges;
    last_scan->range_max = scan->range_max;
    last_scan->range_min = scan->range_min;
    LOG(INFO) << "get scan!";
    condition.notify_all();

}

void GazeboLaserModule::doStart() {
    gazebo_laser->setLaserCallback(boost::bind(&GazeboLaserModule::getGazeboScan, this, _1));
}
}