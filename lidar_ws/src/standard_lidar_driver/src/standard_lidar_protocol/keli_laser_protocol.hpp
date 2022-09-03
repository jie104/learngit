//
// Created by cwt on 2021/10/15.
// Modified by chj on 2022/1/26.
//
#ifndef SROS_KELI_LASER_PROTOCOL_HPP
#define SROS_KELI_LASER_PROTOCOL_HPP

#include <glog/logging.h>
#include "laser_protocol_interface.hpp"
#include "synchronize_time_manager.hpp"

namespace sros {
    class KeliLaserProtocol : public LaserProtocolInterface {
    public:
        struct KeliPackageData {
            uint16_t check_sum;
            uint8_t cmd_type = 0x01;
            uint16_t total_index;
            uint8_t sub_pkg_num;
            uint8_t sub_index;
            uint8_t sub_data_type;
            uint16_t data_len;

            uint32_t stamp;
            std::vector<uint16_t> measures;
        };

        struct KeliScanData {
            LaserConfig config;
            std::vector<std::shared_ptr<KeliPackageData>> package_datas;
            int64_t end_scan_time;
            int64_t end_package_time;
        };

        KeliLaserProtocol(const std::string lidar_model) {
            if (lidar_model == "KLM-2036DE")
                angle_range_deg_ = 360;
            else
                angle_range_deg_ = 276;
            scan_data_.config.angle_min = -angle_range_deg_ / 2 / 180.0 * M_PI;
            scan_data_.config.angle_max = angle_range_deg_ / 2 / 180.0 * M_PI;
            scan_data_.config.angle_increment = 0.12 / 180.0 * M_PI;
            scan_data_.config.time_increment = 1.0 / 25 / (360 / 0.12 );//应该以旋转一圈为计算
            scan_data_.config.range_min = 0.05;
            scan_data_.config.range_max = 30.0;
            scan_data_.config.scan_size = static_cast<int>(angle_range_deg_ / 0.12); //采样点个数
            scan_data_.config.half_circle_time_in_us = 1.0 / 25.0 * 1.0e6 / 2;  // us
        }

        virtual ~KeliLaserProtocol() {}

        virtual int findPackageStart(boost::circular_buffer<char> &ring_buffer) {
            if (ring_buffer.size() < 552) return -1;
            for (std::size_t i = 0; i < ring_buffer.size() - 4; i++) {
                if (((unsigned char) ring_buffer[i]) == 0xFA && ((unsigned char) ring_buffer[i + 1]) == 0x5A &&
                    ((unsigned char) ring_buffer[i + 2]) == 0xA5 && ((unsigned char) ring_buffer[i + 3]) == 0xAA) {
//                    LOG(INFO) << "found!";
                    return i;
                }
            }
            return -2;
        }

        virtual bool resolvePackage(boost::circular_buffer<char> &ring_buffer, int start_index) {
            uint16_t data_length;
            ring_buffer.erase_begin(start_index);
            const int header_size = 6;
            package_datas_.resize(header_size);
            readBufferFront(ring_buffer, package_datas_.data(), header_size);
            ring_buffer.erase_begin(header_size);
            cpToData(data_length, &package_datas_[4], 2);
            package_datas_.resize(data_length);
            readBufferFront(ring_buffer, package_datas_.data(), data_length);

            bool isFirstPackage = false;
            bool isPackage = false;
            if (package_datas_[2] == 0x01) {
                isPackage = true;
                if (package_datas_[6] == 0x00) {
                    isFirstPackage = true;
                }
            }

            if (isPackage) {
                std::shared_ptr<KeliPackageData> scan_ptr(new KeliPackageData);
                auto &scan_data = *scan_ptr;
                cpToData(scan_data.check_sum, &package_datas_[0], 2);
                if (checkSum(package_datas_.data() + 2, data_length - 2, scan_data.check_sum)) {
                    cpToData(scan_data.total_index, &package_datas_[3], 2);
                    cpToData(scan_data.sub_pkg_num, &package_datas_[5], 1);
                    cpToData(scan_data.sub_index, &package_datas_[6], 1);
                    cpToData(scan_data.sub_data_type, &package_datas_[7], 1);
                    cpToData(scan_data.data_len, &package_datas_[8], 2);
                    if (isFirstPackage) {
                        scan_data_.package_datas.clear();
                    }
                    if (scan_data.sub_data_type == 0x03) {
#ifdef ROS_NODE
                        scan_data_.end_scan_time = get_time_in_us();
#else
                        scan_data_.end_scan_time = sros::core::util::get_time_in_us();
#endif
                        cpToDataBigEndian(scan_data.stamp, &package_datas_[107], 4);
                        scan_data_.end_package_time = static_cast<int64_t> (scan_data.stamp * 1e3);
                    }
                    //科力雷达的时间戳是每圈第一个点在雷达时间轴上的发生时间
                    scan_data.measures.clear();
                    for (int i = 0; i < scan_data.data_len / 2; ++i) {
                        scan_data.measures.emplace_back();
                        cpToData(scan_data.measures.back(), &package_datas_[i * 2 + 10], 2);
                    }
                    scan_data_.package_datas.push_back(scan_ptr);
                    return true;
                }
            }
            return false;
        }

        virtual void getStartCmd(std::vector<char> &data) {
            data.clear();
            data.push_back(0xFA);
            data.push_back(0x5A);
            data.push_back(0xA5);
            data.push_back(0xAA);
            data.push_back(0x03);
            data.push_back(0x00);
            data.push_back(0x01);
            data.push_back(0x00);
            data.push_back(0x01);
        }

        virtual void getStartCmd(std::vector<std::vector<char>> &data) {
            data.emplace_back();
            getStartCmd(data.back());
        }

        template<class Type>
        void cpToData(Type &value, char *data, int size) {
            //科力的大部分数据是小端模式，低字节在低位
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

        virtual bool isEndPackage() {
            auto &last_package = scan_data_.package_datas.back();
            if (last_package->sub_data_type == 0x03) {
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
            auto sub_pkg_num = scan_data_.package_datas[0]->sub_pkg_num;
            for (auto &package: scan_data_.package_datas) {
                //int i = 0;
                if (package->sub_data_type == 0x01) {
                    for (auto &measure: package->measures) {
                        ranges.push_back(float(measure) / 500.0f);
                        //i++;
                        //LOG(INFO) << "!!!!!keli - ranges " << i  << ": " << measure;
                    }
                    //LOG(INFO) << "!!!!!keli - ranges package_no" << package->package_no  << ", " << i;
                } else if (package->sub_data_type == 0x02) {
                    for (auto &measure: package->measures) {
                        intens.push_back(float(measure)/1000.0f);
                        //i++;
                        //LOG(INFO) << "!!!!!keli - intens " << i << ": " << measure;
                    }
                    //LOG(INFO) << "!!!!!keli - intens package_no" << package->package_no  << ", " << i;
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
            return true;
        }

        virtual bool checkScanValid() {
            if (scan_data_.package_datas.size() == scan_data_.package_datas[0]->sub_pkg_num) {
                if (scan_data_.package_datas.back()->sub_index == scan_data_.package_datas[0]->sub_pkg_num - 1) {
                    return true;
                }
            }
            LOG(INFO) << "package is wrong! will clear!";
            scan_data_.package_datas.clear();
            return false;
        }

        bool checkSum(char *recvbuf, int recvlen, uint16_t check_sum) {
            unsigned short sum = 0;
            for (auto i = 0; i < recvlen; i++) {
                sum += (unsigned char) *(recvbuf + i);
            }
            if (check_sum == sum) {
                return true;
            } else
                return false;
        }

    private:
        void initializeScan(KeliScanData &scan_data, std::shared_ptr<ScanMsg> &scan) {
            int64_t sync_time = getSyncTime(scan_data.end_scan_time, scan_data.end_package_time);
            auto &config = scan_data.config;
            scan->scan_time = 2.0 * config.half_circle_time_in_us / 1.0e6; // 扫描周期，单位用s，和ros系统一致。
#ifdef ROS_NODE
            scan->header.frame_id = "laser";
            scan->header.stamp = ros::Time::now();
#else
            scan->time_ = sync_time - config.half_circle_time_in_us;    //科力雷达0°在正前方，时间戳反映0°时刻
#endif
            scan->angle_increment = config.angle_increment;
            scan->angle_min = config.angle_min;
            scan->angle_max = config.angle_max;
            scan->time_increment = config.time_increment;
            scan->range_min = config.range_min;
            scan->range_max = config.range_max;
        }

        KeliScanData scan_data_;
        std::vector<char> package_datas_;
        double angle_range_deg_;
    };

}  // namespace sros

#endif  // SROS_KELI_LASER_PROTOCOL_HPP
