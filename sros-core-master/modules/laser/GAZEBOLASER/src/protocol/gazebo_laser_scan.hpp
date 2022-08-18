//
// Created by lhx on 16-1-20.
//

#ifndef SRC_SDK_NETWORK_PROTOCOL_GAZEBO_LASER_SCAN_MSG_H
#define SRC_SDK_NETWORK_PROTOCOL_GAZEBO_LASER_SCAN_MSG_H

#include "base_msg.hpp"

#include <assert.h>
#include <vector>

namespace network {

class GazeboLaserScanMsg : public BaseMsg {
public:
    GazeboLaserScanMsg()
            : BaseMsg(MSG_GAZEBO_LASER_SCAN),
              ranges_num_(0),
              intensities_num_(0),
              angle_min(0),
              angle_max(0),
              angle_increment(0),
              time_increment(0),
              scan_time(0),
              range_min(0),
              range_max(0) {

    };

    virtual ~GazeboLaserScanMsg() { };

    virtual bool encodeBody() override {
        if (ranges.empty()) {
            return false;
        }

        encode_field(angle_min);
        encode_field(angle_max);
        encode_field(angle_increment);
        encode_field(time_increment);
        encode_field(scan_time);
        encode_field(range_min);
        encode_field(range_max);

        ranges_num_ = (int) ranges.size();
        encode_field(ranges_num_);

        intensities_num_ = (int) intensities.size();
        encode_field(intensities_num_);

        for (auto item : ranges) {
            encode_field(item);
        }

        for (auto item : intensities) {
            encode_field(item);
        }

        // 需要保证不会溢出data_数组
        return data_offset_ <= MAX_DATA_LENGTH;
    }

    virtual bool decodeBody() override {
        float item;

        decode_field(angle_min);
        decode_field(angle_max);
        decode_field(angle_increment);
        decode_field(time_increment);
        decode_field(scan_time);
        decode_field(range_min);
        decode_field(range_max);

        decode_field(ranges_num_);
        decode_field(intensities_num_);

        if (ranges_num_ <= 0) {
            return false;
        }

        for (int i = 0; i < ranges_num_; i++) {
            decode_field(item);
            ranges.push_back(item);
        }

        for (int i = 0; i < intensities_num_; i++) {
            decode_field(item);
            intensities.push_back(item);
        }

        return true;
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
    int ranges_num_;
    int intensities_num_;

};

typedef std::shared_ptr<GazeboLaserScanMsg> GazeboLaserScanMsg_ptr;

}


#endif //SRC_SDK_NETWORK_PROTOCOL_PATH_MSG_H
