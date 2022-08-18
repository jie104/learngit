//
// Created by lfc on 17-9-1.
//

#ifndef SROS_SCAN_SLAMMSG_HPP
#define SROS_SCAN_SLAMMSG_HPP

#include "base_slammsg_interface.hpp"
namespace slam{
class ScanSlamMsg :public BaseSlamMsgInterface{
public:
    ScanSlamMsg():BaseSlamMsgInterface(SCAN_SLAMMSG) {
        topic = "SCAN";
    }

    virtual ~ScanSlamMsg(){

    }
    float angle_min;
    float angle_max;
    float angle_increment;
    float time_increment;
    float range_min;
    float range_max;
    std::vector<float> ranges;
    std::vector<float> intensities;
    std::string topic;

private:


};

typedef std::shared_ptr<ScanSlamMsg> ScanSlam_Ptr;
}

#endif //SROS_SCAN_SLAMMSG_HPP
