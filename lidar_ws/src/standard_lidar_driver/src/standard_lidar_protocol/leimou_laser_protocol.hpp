//
// Created by cwt on 2021/10/15.
// Modified by chj on 2022/1/26.
//
#ifndef SROS_LEIMOU_LASER_PROTOCOL_HPP
#define SROS_LEIMOU_LASER_PROTOCOL_HPP

#include <glog/logging.h>
#include "laser_protocol_interface.hpp"
#include "synchronize_time_manager.hpp"

namespace sros {
    class LeimouLaserProtocol : public LaserProtocolInterface {
    public:
        struct LeimouPackageData {
            uint16_t measures_size;     //16
            uint16_t angle_inc;
            std::vector<uint16_t> intensities;
            std::vector<uint16_t> measures;
        };

        struct LeimouScanData {
            LaserConfig config;
            std::vector<std::shared_ptr<LeimouPackageData>> package_datas;
        };

        LeimouLaserProtocol() {
            scan_data_.config.angle_min = -angle_range_deg_ / 2 / 180.0 * M_PI;
            scan_data_.config.angle_max = angle_range_deg_ / 2 / 180.0 * M_PI;
            scan_data_.config.angle_increment = 0.25 / 180.0 * M_PI;
            scan_data_.config.time_increment = 1.0 / 15 / (angle_range_deg_ / 0.25 );//应该以旋转一圈为计算
            scan_data_.config.range_min = 0.05;
            scan_data_.config.range_max = 25.0;
            scan_data_.config.scan_size = static_cast<int>(angle_range_deg_ / 0.25); //采样点个数
        }

        virtual ~LeimouLaserProtocol() {}

        virtual int findPackageStart(boost::circular_buffer<char> &ring_buffer) {
            // LOG(INFO) << "ring_buffer.size: " << ring_buffer.size();
            if (ring_buffer.size() < 4200) return -1;
            for (std::size_t i = 0; i < ring_buffer.size() - 4; i++) {
                if (((unsigned char) ring_buffer[i]) == 0xAA && ((unsigned char) ring_buffer[i + 1]) == 0x88 &&
                    ((unsigned char) ring_buffer[i + 2]) == 0x88 && ((unsigned char) ring_buffer[i + 3]) == 0xAA) {
                //    LOG(INFO) << "found!";
                    return i;
                }
            }
            return -2;
        }

        virtual bool resolvePackage(boost::circular_buffer<char> &ring_buffer, int start_index) {
            // if (ring_buffer.size() < package_datas_.size() + start_index) {  // 8660
            if (ring_buffer.size() < package_datas_.size() + start_index) {  
                // LOG(INFO) << "package_datas_.size(): " << package_datas_.size();
                return false;

            }

            uint32_t data_length = 0;
            ring_buffer.erase_begin(start_index);
            const int header_size = 18;
            package_datas_.resize(header_size);
            readBufferFront(ring_buffer, package_datas_.data(), header_size);
            ring_buffer.erase_begin(header_size);


            // ring_buffer.erase_begin(start_index);
            // readBufferFront(ring_buffer, package_datas_.data(), package_datas_.size());
            // ring_buffer.erase_begin(package_datas_.size());
            // uint32_t data_length;
            // cpToDataBigEndian<uint32_t>(data_length, &package_datas_[6], 4);
            // LOG(INFO) << "data_length: " << data_length;
            // if (checkXor(package_datas_.data(), data_length + 9))
            // if (package_datas_[5]==0x03)
            //  {
            //     std::shared_ptr<LeimouPackageData> scan_ptr(new LeimouPackageData);
            //     auto &scan_data = *scan_ptr;
            //     cpToDataBigEndian(scan_data.measures_size, &package_datas_[16], 2);
            //     LOG(INFO) << "measure_size:" << scan_data.measures_size;
            //     package_datas_.clear();



            if (package_datas_[5]==0x03) {
                std::shared_ptr<LeimouPackageData> scan_ptr(new LeimouPackageData);
                auto &scan_data = *scan_ptr;
                cpToDataBigEndian(data_length, &package_datas_[6], 4);
                cpToDataBigEndian(scan_data.angle_inc,&package_datas_[14],2);
                // LOG(INFO) << "angle_inc: " << scan_data.angle_inc;
                cpToDataBigEndian(scan_data.measures_size, &package_datas_[16], 2);
                package_datas_.clear();
                package_datas_.resize(data_length); // 4330


                readBufferFront(ring_buffer, package_datas_.data(), data_length-18); 
                scan_data_.package_datas.clear();

                // readBufferFront(ring_buffer, package_datas_.data(), data_length); 
                // scan_data_.package_datas.clear();

                // scan_data.intensities.clear();
                scan_data.measures.clear();


                for (int i = 0; i < scan_data.measures_size; ++i) { 
                    scan_data.measures.emplace_back();
                    // scan_data.intensities.emplace_back();
                    cpToDataBigEndian(scan_data.measures.back(), &package_datas_[i * 2], 2);
                    // cpToDataBigEndian(scan_data.intensities.back(), &package_datas_[scan_data.measures_size * 2 + 2*i], 2);

                }
                scan_data_.package_datas.push_back(scan_ptr);
                return true;
            
            } 
            return false;
            
        }




    virtual void getStartCmd(std::vector<std::vector<char>>& datas) {
        datas.resize(1);
        // int index = 0;
        for(auto& data:datas) {
            data.clear();
            data = getBody();
        }
    }

    std::vector<char> getBody(){
        std::vector<char> data;
        data.push_back(0xaa);
        data.push_back(0x88);
        data.push_back(0x88);
        data.push_back(0xaa);

        // unsigned char check=0;
        // for (int i=0;i<7;i++)
        // {
        //     data.push_back(PROTOCOL_RESPONSE[i]);
        //     check^=PROTOCOL_RESPONSE[i];
        // }
        
        // data.push_back(check);
        // data.push_back(0x88);
        // data.push_back(0xaa);

        // data.push_back(0xaa);

        // data.push_back(0x88);

        return data;
    }



        template<class Type>
        void cpToData(Type &value, char *data, int size) {
            //大端模式
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
            if (scan_data_.package_datas.size() == 1) {
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
            // auto &intens = scan->intensities;
            ranges.clear();
            // intens.clear();
            for (auto &package: scan_data_.package_datas) {
                for (int i=0;i<package->measures_size;i++)
                {
                    ranges.push_back(float(package->measures[i])*0.001);    //m
                    // intens.push_back(float(package->intensities[i]));  
                    // LOG(INFO) << i << " measure:" << package->measures[i]*0.01;
                }
            }
            // if (intens.size() < ranges.size()) {
            //     int delta = ranges.size() - intens.size();
            //     for (int i = 0; i < delta; ++i) {
            //         intens.push_back(0);
            //     }
            // }
            scan_data_.package_datas.clear();
            // if (ranges.size() >= scan_data_.config.scan_size && intens.size() >= scan_data_.config.scan_size) {
            //     ranges.resize(scan_data_.config.scan_size);
            //     intens.resize(scan_data_.config.scan_size);
            //     return true;
            // } else {
            //     LOG(INFO) << "range size:" << ranges.size() << "," << intens.size();
            // }
            return true;
        }

        virtual bool checkScanValid() {
            if (scan_data_.package_datas.size() == 1) {
                return true;
            }
            LOG(INFO) << "package is wrong! will clear!";
            package_datas_.clear();
            return false;
        }

        bool checkXor(char *recvbuf, int recvlen) {
            int i = 0;
            char check = 0;
            char *p = recvbuf;
            int len;
            if (*p == (char) 0xaa) {
                p = p + 4;
                len = recvlen - 9;
                for (i = 0; i < len-5; i++) {
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
        void initializeScan(LeimouScanData &scan_data, std::shared_ptr<ScanMsg> &scan) {
            auto &config = scan_data.config;
            scan->angle_increment = config.angle_increment;
            scan->angle_min = config.angle_min;
            scan->angle_max = config.angle_max;
            scan->time_increment = config.time_increment;
            scan->range_min = config.range_min;
            scan->range_max = config.range_max;
        }

        LeimouScanData scan_data_;
        std::vector<char> package_datas_;
        double angle_range_deg_=270;
        const unsigned char PROTOCOL_RESPONSE[7]={0xa1,0x02,0x0,0x0,0x0,0x10,0x02};

    };

}  // namespace sros

#endif  // SROS_KELI_LASER_PROTOCOL_HPP
