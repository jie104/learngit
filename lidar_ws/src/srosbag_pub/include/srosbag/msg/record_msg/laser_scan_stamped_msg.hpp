//
// Created by lhx on 16-1-20.
//

#ifndef SRC_SDK_NETWORK_PROTOCOL_LASER_SCAN_STAMPED_MSG_H
#define SRC_SDK_NETWORK_PROTOCOL_LASER_SCAN_STAMPED_MSG_H

#include "base_msg.hpp"

#include <assert.h>
#include <vector>

namespace record {

class LaserScanStampedMsg : public BaseMsg {
public:
    LaserScanStampedMsg()
            : BaseMsg(MSG_LASER_SCAN_STAMPED),
              ranges_num_(0),
              intensities_num_(0),
              angle_min(0),
              angle_max(0),
              angle_increment(0),
              time_increment(0),
              scan_time(0),
              range_min(0),
              range_max(0),
              timestamp(0) {

    };
    LaserScanStampedMsg(MSG_TYPE msg_type_)
            : BaseMsg(msg_type_),
              ranges_num_(0),
              intensities_num_(0),
              angle_min(0),
              angle_max(0),
              angle_increment(0),
              time_increment(0),
              scan_time(0),
              range_min(0),
              range_max(0),
              timestamp(0) {

    };

    virtual ~LaserScanStampedMsg() {
    };

    virtual bool encodeBody() override {
        data_offset_ = getHeaderLength();
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
        encode_field(timestamp);

        ranges_num_ = (int) ranges.size();
        encode_field(ranges_num_);

        intensities_num_ = (int) intensities.size();
        encode_field(intensities_num_);

        for (auto item : ranges) {
            encode_field(item);
        }
//
        for (auto item : intensities) {
            encode_field(item);
        }

//        printf("the end length is:%d\n", date_tmp);
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
        decode_field(timestamp);

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
    int64_t timestamp;
    std::vector<float> ranges;
    std::vector<float> intensities;

private:
    int ranges_num_;
    int intensities_num_;

};

typedef std::shared_ptr<LaserScanStampedMsg> LaserScanStampedMsg_ptr;

}


#endif //SRC_SDK_NETWORK_PROTOCOL_PATH_MSG_H
