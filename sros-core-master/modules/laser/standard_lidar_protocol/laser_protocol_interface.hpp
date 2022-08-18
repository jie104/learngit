//
// Created by lfc on 2021/10/13.
//

#ifndef SROS_LASER_PROTOCOL_INTERFACE_HPP
#define SROS_LASER_PROTOCOL_INTERFACE_HPP

#include <boost/circular_buffer.hpp>

#ifdef ROS_NODE

#include <sensor_msgs/LaserScan.h>
#include <chrono>

typedef sensor_msgs::LaserScan ScanMsg;
#else
#include <core/msg/laser_scan_msg.hpp>
typedef sros::core::LaserScanMsg ScanMsg;
#endif

#include "synchronize_time_manager.hpp"

namespace sros {
class LaserProtocolInterface {
 public:
    struct LaserConfig {  //应该会初始化把
        double angle_min = -135.0 / 180.0 * M_PI;
        double angle_max = 135.0 / 180.0 * M_PI;
        double angle_increment = 1 / 3.0 * M_PI / 180.0;
        double time_increment = 1.0 / (15 * 811.0);
        double range_min = 0;
        double range_max = 30.0;
        int scan_size = 811;
        double half_circle_time_in_us = 0.5 / 15.0 * 1.0e6;
    };

    LaserProtocolInterface() {}

    virtual ~LaserProtocolInterface() {}

    virtual void getStartCmd(std::vector<char> &data) {}

    virtual void getStartCmd(std::vector<std::vector<char>> &datas) {}

    virtual int findPackageStart(boost::circular_buffer<char> &ring_buffer) = 0;

    virtual bool resolvePackage(boost::circular_buffer<char> &ring_buffer, int start_index) = 0;

    virtual bool isEndPackage() = 0;

    virtual bool checkScanValid() = 0;

    virtual bool cpToScan(std::shared_ptr<ScanMsg> &scan) = 0;

    template<class Type>
    void cpToDataLittleEndian(Type &value, char *data, int size) {
        //小端模式，低字节在低位
        value = (Type) 0;
        for (int i = 0; i <= size - 1; ++i) {
            value |= (unsigned char) data[i] << (i * 8);
        }
    }

    template<class Type>
    void cpToDataBigEndian(Type &value, char *data, int size) {
        //大端模式
        value = (Type) 0;
        for (int i = size - 1; i >= 0; --i) {
            value |= (unsigned char) data[size - 1 - i] << (i * 8);
        }
    }

    bool readBufferFront(boost::circular_buffer<char> &ring_buffer, char *dst, std::size_t numbytes) {
        if (ring_buffer.size() < numbytes) return false;
        char *pone = ring_buffer.array_one().first;
        std::size_t pone_size = ring_buffer.array_one().second;
        char *ptwo = ring_buffer.array_two().first;
        // std::size_t ptwo_size = ring_buffer_.array_two().second;
        if (pone_size >= numbytes) {
            std::memcpy(dst, pone, numbytes);
        } else {
            std::memcpy(dst, pone, pone_size);
            std::memcpy(dst + pone_size, ptwo, numbytes - pone_size);
        }
        return true;
    }

 protected:
    uint64_t get_time_in_ns() {
        return static_cast<uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count());
    }

    uint64_t get_time_in_us(){
#ifdef ROS_NODE
        return get_time_in_ns() / 1000L;
#else
        return sros::core::util::get_time_in_us();
#endif
    }


    int64_t getSyncTime(const int64_t &system_time, int64_t &origin_scan_time) {
        int64_t delta_sync_time = system_time - origin_scan_time;
        int64_t sync_time = system_time;
        if (syschronize_time_.empty()) {
            syschronize_time_.resize(max_stamp_cache_size_);
        }
        syschronize_time_.push_back(delta_sync_time);
        int64_t min_delta_stamp = syschronize_time_.getMinValue();
        if (min_delta_stamp != 0) {
            sync_time = min_delta_stamp + origin_scan_time;
        }
        if (abs(sync_time - system_time) > 1e5) {
            LOG(INFO) << "delta time is large! will return system time!";
            syschronize_time_.reset();
            return system_time;
        }
        return sync_time;
    }

 private:
    sros::SynchronizeTimeManager<int64_t> syschronize_time_;
    const int max_stamp_cache_size_ = 200;
};

}  // namespace sros

#endif  // SROS_LASER_PROTOCOL_INTERFACE_HPP
