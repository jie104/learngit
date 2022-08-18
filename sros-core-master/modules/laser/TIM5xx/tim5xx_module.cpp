//
// Created by lfc on 17-2-11.
//

#include <glog/logging.h>
#include <sick_tim/sick_tim_common_tcp.h>
#include <sick_tim/sick_tim551_2050001_parser.h>
#include <math.h>
#include "tim5xx_module.h"
using namespace sick_tim;
namespace laser {
Tim5xxModule::Tim5xxModule(std::string ip_address,std::string ip_port):ip_add(ip_address),port(ip_port){
//    ip_add = "192.168.1.175";
//    port = "2112";
    time_limit = 5;
    parser = new sick_tim::SickTim5512050001Parser();
}

Tim5xxModule::~Tim5xxModule() {
    tim_571.reset();
}

bool Tim5xxModule::doOpen() {
    bool is_ok = false;
    while (!is_ok) {
        if (tim_571) {
            tim_571.reset();
        }
        tim_571 = std::make_shared<SickTimCommonTcp>(ip_add, port, time_limit, parser);
        int result = tim_571->init();
        if (result == sick_tim::ExitSuccess) {
            is_ok = true;
            return true;
        }else {
            LOG(INFO) << "can not open the lidar! will open it again!";
            sleep(1);
        }
    }
    return false;
}

void Tim5xxModule::doStart() {
    if (!tim_571) {
        doOpen();
    }
}

void Tim5xxModule::doStop() {
    tim_571.reset();
}

void Tim5xxModule::doClose() {
    tim_571.reset();
}

bool Tim5xxModule::getScanData(sros::core::LaserScan_ptr& scan_ptr) {
    if (scan_ptr) {
        int result = tim_571->loopOnce(scan_ptr);
        double range_min = 0.1;
        scan_ptr->range_min = range_min > scan_ptr->range_min ? range_min : scan_ptr->range_min;
        scan_ptr->time_ = sros::core::util::get_time_in_us();
        if (result == sick_tim::ExitSuccess) {
            return true;
        }else {
            return false;
        }
    }else {
        LOG(INFO) << "the scan_ptr is zero! will return false";
        return false;
    }
}

}