//
// Created by lfc on 2019/12/20.
//
#include "micro_scan3_module.h"
int main(){
    auto micro_module = std::make_shared<laser::MicroScan3Module>();
    micro_module->doOpen();
    micro_module->doStart();

    while (1) {
        sros::core::LaserScan_ptr scan_ptr;
        if (micro_module->getScanData(scan_ptr)) {
//            LOG(INFO) << "get scan!"<<1.0/scan_ptr->scan_time<<","<<scan_ptr->angle_increment<<","<<scan_ptr->ranges.size();
//            if (scan_ptr->time_ != 0) {
//                LOG(INFO) << "output!";
//            }
            LOG(INFO) << micro_module->getVersionNO();
        }
    }
    micro_module->doClose();
}