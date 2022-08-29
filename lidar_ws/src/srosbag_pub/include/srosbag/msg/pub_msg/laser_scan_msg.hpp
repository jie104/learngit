//
// Created by lhx on 15-12-23.
//

#ifndef SROS_BAG_LASER_SCAN_H
#define SROS_BAG_LASER_SCAN_H

#include <vector>
#include "core/util/time.h"
#include "base_msg.h"

namespace sros{

namespace bag{

class LaserScanMsg : public BaseMsg {
public:
    LaserScanMsg()
            : BaseMsg("TOPIC_LASER", TYPE_LASER_SCAN_DATA) {

    }
    virtual ~LaserScanMsg() { }

    virtual void getTime() override {
        time_= sros::core::util::get_time_in_us();
        // printf("the scan time is:%lld\n",time_);
    }

    double angle_min;
    double angle_max;
    double angle_increment;
    double time_increment;
    double scan_time;
    double range_min;
    double range_max;
    std::vector<float> ranges;
    std::vector<float> intensities;

private:
    // LaserScanMsg在外部有引用,不能使用util::get_time_us,故在此重复实现

};

typedef std::shared_ptr<LaserScanMsg> LaserScan_ptr;

}
}

#endif //SROS_LASER_SCAN_H
