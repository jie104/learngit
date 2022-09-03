//
// Created by lfc on 2021/10/13.
//

#ifndef SROS_WANJI_LASER_PROTOCOL_HPP
#define SROS_WANJI_LASER_PROTOCOL_HPP

#include <glog/logging.h>
#include "laser_protocol_interface.hpp"
#include "synchronize_time_manager.hpp"

namespace sros {
    class WanjiLaserProtocol : public LaserProtocolInterface {
    public:
        struct WanjiPackageData {
            static const int package_size = 898;
            uint16_t laser_state;  // 24
            uint32_t stamp;        // 42
            uint32_t circle_no;    // 46
            uint16_t package_no;   // 50
            uint16_t point_nm;
            std::vector<uint16_t> measures;  // 85
        };

        struct WanjiScanData {
            LaserConfig config;
            std::vector<std::shared_ptr<WanjiPackageData>> package_datas;
            int64_t end_scan_time;
            int64_t end_package_time;
        };

        WanjiLaserProtocol() {
            package_datas_.resize(WanjiPackageData::package_size);
            scan_data_.config.angle_min = -135.0 / 180.0 * M_PI;
            scan_data_.config.angle_max = 135.0 / 180.0 * M_PI;
            scan_data_.config.angle_increment = 1 / 3.0 * M_PI / 180.0;
            scan_data_.config.time_increment = 1.0 / (15 * 810.0 * 270 / 360);
            scan_data_.config.range_min = 0;
            scan_data_.config.range_max = 30.0;
            scan_data_.config.scan_size = 810;
            scan_data_.config.half_circle_time_in_us = 1 / 30.0 * 1.0e6;
        }

        virtual ~WanjiLaserProtocol() {}

        virtual int findPackageStart(boost::circular_buffer<char> &ring_buffer) {
            if (ring_buffer.size() < 60) return -1;
            for (std::size_t i = 0; i < ring_buffer.size() - 4; i++) {
                if (((unsigned char) ring_buffer[i]) == 0x02 && ((unsigned char) ring_buffer[i + 1]) == 0x02 &&
                    ((unsigned char) ring_buffer[i + 2]) == 0x02 && ((unsigned char) ring_buffer[i + 3]) == 0x02) {
                    return i;
                }
            }
            return -2;
        }

        virtual bool resolvePackage(boost::circular_buffer<char> &ring_buffer, int start_index) {
            if (ring_buffer.size() < package_datas_.size() + start_index) {  // TODO:修改成正常size
                return false;
            }
            ring_buffer.erase_begin(start_index);
            readBufferFront(ring_buffer, package_datas_.data(), package_datas_.size());
            ring_buffer.erase_begin(package_datas_.size());
            uint32_t data_length;
            cpToData<uint32_t>(data_length, &package_datas_[4], 4);
            if (checkXor(package_datas_.data(), data_length + 9)) {
                std::shared_ptr<WanjiPackageData> scan_ptr(new WanjiPackageData);
                auto &scan_data = *scan_ptr;
                if (package_datas_[8] == 0x73 && package_datas_[9] == 0x53 && package_datas_[10] == 0x4E &&
                    package_datas_[11] == 0x20) {
                    cpToData(scan_data.laser_state, &package_datas_[24], 2);
                    cpToData(scan_data.stamp, &package_datas_[42], 4);
                    cpToData(scan_data.circle_no, &package_datas_[46], 4);
                    cpToData(scan_data.package_no, &package_datas_[50], 2);
                    cpToData(scan_data.point_nm, &package_datas_[83], 2);
                    if (scan_data.package_no == 1) {
#ifdef ROS_NODE
                        scan_data_.end_scan_time = get_time_in_us();
#else
                        scan_data_.end_scan_time = sros::core::util::get_time_in_us();
#endif
                        scan_data_.end_package_time = scan_data.stamp * 1e3;
                    }
                    scan_data.measures.clear();
                    for (int i = 0; i < scan_data.point_nm; ++i) {
                        scan_data.measures.emplace_back();
                        cpToData(scan_data.measures.back(), &package_datas_[i * 2 + 85], 2);
                    }
                }
                scan_data_.package_datas.push_back(scan_ptr);
            }
            return true;
        }

        template<class Type>
        void cpToData(Type &value, char *data, int size) {
            value = (Type) 0;
            for (int i = size - 1; i >= 0; --i) {
                value |= (unsigned char) data[size - 1 - i] << (i * 8);
            }
        }

        virtual bool isEndPackage() {
            auto &last_package = scan_data_.package_datas.back();
            if (last_package->package_no == 6) {
                return true;
            }
            return false;
        }

        virtual bool cpToScan(std::shared_ptr<ScanMsg> &scan) {
            if (!scan) {
                scan.reset(new ScanMsg);
            }
            initializeScan(scan_data_, scan);
            auto &ranges = scan->ranges;
            auto &intens = scan->intensities;
            ranges.clear();
            intens.clear();
            for (auto &package: scan_data_.package_datas) {
                if (package->package_no <= 2) {
                    for (int i = 0; i < package->point_nm; ++i) {
                        ranges.push_back(float(package->measures[i]) / 1000.0f);
                    }
                } else if (package->package_no <= 4) {
                    for (int i = 0; i < package->point_nm; ++i) {
                        intens.push_back(float(package->measures[i]));
                    }
                }
            }
            if (intens.size() < ranges.size()) {
                int delta = ranges.size() - intens.size();
                for (int i = 0; i < delta; ++i) {
                    intens.push_back(0);
                }
            }
            scan_data_.package_datas.clear();
            if (ranges.size() >= scan_data_.config.scan_size && intens.size() >= scan_data_.config.scan_size) {
                ranges.resize(scan_data_.config.scan_size);
                intens.resize(scan_data_.config.scan_size);
                return true;
            } else {
                LOG(INFO) << "range size:" << ranges.size() << "," << intens.size();
            }
            return false;
        }

        virtual bool checkScanValid() {
            if (scan_data_.package_datas.size() == 6) {
                if (scan_data_.package_datas.back()->package_no == 6) {
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
            int len;
            if (*p == (char) 0x02) {
                p = p + 8;
                len = recvlen - 9;
                for (i = 0; i < len; i++) {
                    check ^= *p++;
                }
                if (check == *p) {
                    return true;
                } else
                    return false;
            } else {
                return false;
            }
        }

    private:
        void initializeScan(WanjiScanData &scan_data, std::shared_ptr<ScanMsg> &scan) {
            int64_t sync_time = getSyncTime(scan_data.end_scan_time, scan_data.end_package_time);
            auto &config = scan_data.config;
            scan->scan_time = 2.0 * config.half_circle_time_in_us/1.0e6;
#ifdef ROS_NODE
            scan->header.stamp = ros::Time::now();
#else
            scan->time_ = sync_time - 135.0/180.0 * config.half_circle_time_in_us;
#endif
            scan->angle_increment = config.angle_increment;
            scan->angle_min = config.angle_min;
            scan->angle_max = config.angle_max;
            scan->time_increment = -config.time_increment;
            scan->range_min = config.range_min;
            scan->range_max = config.range_max;
        }

        WanjiScanData scan_data_;
        std::vector<char> package_datas_;
    };

}  // namespace sros

#endif  // SROS_WANJI_LASER_PROTOCOL_HPP
