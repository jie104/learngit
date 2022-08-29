//
// Created by cwt on 2021/10/15.
//
#ifndef SROS_KELI_OBSTACLE_LASER_PROTOCOL_HPP
#define SROS_KELI_OBSTACLE_LASER_PROTOCOL_HPP
#include <glog/logging.h>
#include "laser_protocol_interface.hpp"
#include "synchronize_time_manager.hpp"

namespace sros {
class KeliObstacleLaserProtocol : public LaserProtocolInterface {
 public:
    struct KeliPackageData {
        static const int package_size = 1222;  // 1210, 540, 1082, 540
        uint8_t laser_state;
        uint32_t stamp;
        uint32_t circle_no;
        uint16_t package_no;
        uint16_t point_nm;
        std::vector<uint16_t> measures;
    };

    struct KeliScanData {
        LaserConfig config;
        std::vector<std::shared_ptr<KeliPackageData>> package_datas;
        int64_t end_scan_time;
        int64_t end_package_time;
    };

    KeliObstacleLaserProtocol() {
        package_datas_.resize(KeliPackageData::package_size);
        scan_data_.config.angle_min = -135 / 180.0 * M_PI;
        scan_data_.config.angle_max = 135 / 180.0 * M_PI;
        scan_data_.config.angle_increment = (1.0 / 3.0) / 180.0 * M_PI;
        scan_data_.config.time_increment = 0.000040;
        scan_data_.config.range_min = 0.05;
        scan_data_.config.range_max = 10.0;
        scan_data_.config.scan_size = 811;                                  //采样点个数
        scan_data_.config.half_circle_time_in_us = 1.0 / 28.0 * 1.0e6 / 2;  // us
    }

    virtual void getStartCmd(std::vector<char> &data) {
        data.clear();
        data.push_back(0xFA);
        data.push_back(0x5A);
        data.push_back(0xA5);
        data.push_back(0xAA);
        data.push_back(0x00);
        data.push_back(0x02);
        data.push_back(0x01);
        data.push_back(0x01);
    }

    virtual void getStartCmd(std::vector<std::vector<char>> &data) {
        data.emplace_back();
        getStartCmd(data.back());
    }

    virtual ~KeliObstacleLaserProtocol() {}

    virtual int findPackageStart(boost::circular_buffer<char> &ring_buffer) {
        if (ring_buffer.size() < 552) return -1;
        for (std::size_t i = 0; i < ring_buffer.size() - 4; i++) {
            if (((unsigned char)ring_buffer[i]) == 0xFA && ((unsigned char)ring_buffer[i + 1]) == 0x5A &&
                ((unsigned char)ring_buffer[i + 2]) == 0xA5 && ((unsigned char)ring_buffer[i + 3]) == 0xAA) {
                return i;
            }
        }
        return -2;
    }

    virtual bool resolvePackage(boost::circular_buffer<char> &ring_buffer, int start_index) {
        uint16_t data_length;
        uint8_t package_index;
        ring_buffer.erase_begin(start_index);
        const int header_size = 6;
        readBufferFront(ring_buffer, package_datas_.data(), header_size);
        ring_buffer.erase_begin(header_size);
        cpToData(data_length, &package_datas_[4], 2);
        readBufferFront(ring_buffer, package_datas_.data(), data_length);

        bool isFirstPackage = false;
        bool isPackage = false;
        if (package_datas_[1] == 0x01 && package_datas_[4] == 0x04) {
            isPackage = true;
            if (package_datas_[5] == 0x00) {
                isFirstPackage = true;
            }
        }

        if (isPackage) {
            int default_offset = 6;
            int PackageOffset = default_offset;
            if (checkXor(package_datas_.data(), data_length)) {
                std::shared_ptr<KeliPackageData> scan_ptr(new KeliPackageData);
                auto &scan_data = *scan_ptr;
                uint8_t package_no;
                cpToData(package_no, &package_datas_[5], 1);
                scan_data.package_no = (uint16_t)package_no;
                if (isFirstPackage) {
                    scan_data_.package_datas.clear();
                    cpToData(scan_data.laser_state, &package_datas_[68 + default_offset], 1);
                    cpToData(scan_data.stamp, &package_datas_[69 + default_offset], 4);
                    scan_data_.end_scan_time = sros::core::util::get_time_in_us();
                    scan_data_.end_package_time = (int64_t)(scan_data.stamp * 1e3);  // us
                    PackageOffset += 128;
                }
                scan_data.measures.clear();
                for (int i = PackageOffset; i < data_length; i += 2) {
                    scan_data.measures.emplace_back();
                    cpToData(scan_data.measures.back(), &package_datas_[i], 2);
                }
                scan_data_.package_datas.push_back(scan_ptr);
                return true;
            }
        }
        return false;
    }

    template <class Type>
    void cpToData(Type &value, char *data, int size) {
        value = (Type)0;
        for (int i = size - 1; i >= 0; --i) {
            value |= (unsigned char)data[size - 1 - i] << (i * 8);
        }
    }

    virtual bool isEndPackage() {
        auto &last_package = scan_data_.package_datas.back();
        if (last_package->package_no == 3) {
            return true;
        }
        return false;
    }

    virtual bool cpToScan(sros::core::LaserScan_ptr &scan) {
        if (!scan) {
            scan.reset(new sros::core::LaserScanMsg);
        }
        initializeScan(scan_data_, scan);
        auto &ranges = scan->ranges;
        auto &intens = scan->intensities;
        ranges.clear();
        intens.clear();

        for (auto &package : scan_data_.package_datas) {
            if (package->package_no < 2) {
                for (auto &measure : package->measures) {
                    ranges.push_back(float(measure / 1000.0f));
                }
            } else if (package->package_no < 4) {
                for (auto &measure : package->measures) {
                    intens.push_back(float(measure));
                }
            }
        }
        scan_data_.package_datas.clear();
        return true;
    }

    virtual bool checkScanValid() {
        if (scan_data_.package_datas.size() == 4) {
            if (scan_data_.package_datas.back()->package_no == 3) {
                return true;
            }
        }
        LOG(INFO) << "package is wrong! will clear!";
        scan_data_.package_datas.clear();
        return false;
    }

    bool checkXor(char *recvbuf, int recvlen) {
        int i = 0;
        char check = 0;
        char *p = recvbuf;
        for (i = 1; i < recvlen; i++) {
            check += p[i];
        }
        if (check == *recvbuf) {
            return true;
        } else
            return false;
    }

 private:
    void initializeScan(KeliScanData &scan_data, sros::core::LaserScan_ptr &scan) {
        int64_t sync_time = getSyncTime(scan_data.end_scan_time, scan_data.end_package_time);
        auto &config = scan_data.config;
        scan->scan_time = 2 * config.half_circle_time_in_us / 1e6;
        scan->time_ = sync_time + (int64_t)config.half_circle_time_in_us;  //当前第一个包的时间是该圈的开始时间
        scan->angle_increment = config.angle_increment;
        scan->angle_min = config.angle_min;
        scan->angle_max = config.angle_max;
        scan->time_increment = config.time_increment;
        scan->range_min = config.range_min;
        scan->range_max = config.range_max;
    }
    KeliScanData scan_data_;
    std::vector<char> package_datas_;
};

}  // namespace sros

#endif  // SROS_KELI_OBSTACLE_LASER_PROTOCOL_HPP
