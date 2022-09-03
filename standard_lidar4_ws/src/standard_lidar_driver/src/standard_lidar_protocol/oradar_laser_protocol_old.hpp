//
// Created by lfc on 2022/5/31.
//

#ifndef STANDARD_LIDAR_DRIVER_ORADAR_LASER_PROTOCOL_HPP
#define STANDARD_LIDAR_DRIVER_ORADAR_LASER_PROTOCOL_HPP
#include "laser_protocol_interface.hpp"
#include <fstream>

namespace sros{
class OradarLaserProtocol: public LaserProtocolInterface {
 public:
    struct OradarPackageBlock{ // oradar传输过来的包结构，包的一个块，6个包块组成一个完整的雷达扫描包
        uint16_t package_size;
        uint8_t freq_type; // 04: 10, 05: 15Hz, 06: 20Hz, 07: 25Hz, 08: 30Hz
        uint8_t freq;
        uint8_t block_no; // 是完整包的哪一块
        uint8_t package_num;
        uint16_t point_num; // 15Hz, 252, 252*6=1512
        uint32_t timestamp; // 块内的雷达点的最后一个扫描点的时间
        std::vector<uint16_t> measures;
        std::vector<uint8_t> intensities;
    };


    OradarLaserProtocol() {
    }

    virtual ~OradarLaserProtocol() {
        sync_time_out.close();
    }

    virtual int findPackageStart(boost::circular_buffer<char> &ring_buffer) {
//        std::cout << "findStart\n，buffer size = " << ring_buffer.size() << std::endl;
        if (ring_buffer.size() < 60) return -1;
        for (std::size_t i = 0; i < ring_buffer.size() - 4; i++) {
            // 奥比改了强度的字节长度之后，包头变了: 04 04 01 00
            if (((unsigned char) ring_buffer[i]) == 0x04 && ((unsigned char) ring_buffer[i + 1]) == 0x04 &&
                ((unsigned char) ring_buffer[i + 2]) == 0x01 && ((unsigned char) ring_buffer[i + 3]) == 0x00
                ) {
                std::cout << "findStart = "  << i << std::endl;
                return i;
            }

//            if (((unsigned char) ring_buffer[i]) == 0x03 && ((unsigned char) ring_buffer[i + 1]) == 0x08 &&
//                ((unsigned char) ring_buffer[i + 2]) == 0x01 && ((unsigned char) ring_buffer[i + 3]) == 0x00) {
//                std::cout << "findStart = "  << i << std::endl;
//                return i;
//            }

//            // 用Wireshark抓包，得到包头
//            if (((unsigned char) ring_buffer[i]) == 0x02 && ((unsigned char) ring_buffer[i + 1]) == 0x54 &&
//                ((unsigned char) ring_buffer[i + 2]) == 0x01 && ((unsigned char) ring_buffer[i + 3]) == 0x00) {
//                std::cout << "findStart = "  << i << std::endl;
//                return i;
//            }


        }

        return -2;
    }

    virtual bool resolvePackage(boost::circular_buffer<char> &ring_buffer, int start_index) {
        // init block;
        ring_buffer.erase_begin(start_index);
        parseConfig(ring_buffer);
        parseScanPoints(ring_buffer);
        package_blocks.push_back(block);
        return true;
    }

    virtual bool isEndPackage() {
        auto &last_block = package_blocks.back();
        if (last_block.block_no == block_num) {
            return true;
        }
        return false;
    }

    virtual bool cpToScan(std::shared_ptr<ScanMsg> &scan) { // convert 2 scan
        scan.reset(new ScanMsg); // 初始化保证清空

        int64_t sync_time = getSyncTime(sys_recv_block_time, sensor_send_block_time); // use first block
        save_sync_times(sync_time);
        scan->scan_time = 2.0 * 1 / block.freq / 1.0e6;
#ifdef ROS_NODE
        scan->header.stamp = ros::Time::now();
#else
        scan->time_ = sync_time + 90 / 360 * scan->scan_time; // middle time
#endif
        int scan_point_num = block_num * block.point_num;
        scan->angle_increment = angle_range / scan_point_num * M_PI / 180.0; // 15Hz 0.17857
//        scan->angle_min = angle_min;
//        scan->angle_max = angle_max;
///!!! 这里必须要用弧度，不能用角度
        scan->angle_min = angle_min* M_PI / 180.0;
        scan->angle_max = angle_max* M_PI / 180.0;

        scan->time_increment = 1.0 / (block.freq * scan_point_num * 360 / angle_range);
        scan->range_min = range_min;
        scan->range_max = range_max;
        int total_num = 0;
        for(auto &blk : package_blocks){
            total_num += blk.point_num;
            for (int i = 0; i < blk.point_num; ++i) {
                scan->ranges.push_back(float(blk.measures[i]) * range_unit);
                scan->intensities.push_back(float(blk.intensities[i]) * intensity_unit);
            }
        }
//        std::cout << " total point number:" << total_num << std::endl;
        package_blocks.clear();
        return checkConvertedScan(scan, scan_point_num);
    }

    virtual bool checkScanValid() {
        if (package_blocks.size() == 6) {
            if (package_blocks.back().block_no == 6) {
                return true;
            }
        }
        return false;
    }

    virtual void getStartCmd(std::vector<std::vector<char>>& datas) {
        datas.resize(2);
        int index = 0;
        for(auto& data:datas) {
            data.clear();
            data = getBody(index);
            index++;
        }
    }

    std::vector<char> getBody(int head = 0){
        std::vector<char> data;
        if (head == 0) {
            data.push_back(0x00);
            data.push_back(0x10);
            data.push_back(0x00);
            data.push_back(0x00);
            data.push_back(0x01);
            data.push_back(0x08);
            data.push_back(0x00);
            data.push_back(0x00);
            data.push_back(0x12);
            data.push_back(0x34);
            data.push_back(0x56);
            data.push_back(0x78);
            data.push_back(0x9b);
            data.push_back(0x0d);
            data.push_back(0x67);
            data.push_back(0x00);
        } else if (head == 1) {
            data.push_back(0x00);
            data.push_back(0x10);
            data.push_back(0x00);
            data.push_back(0x00);
            data.push_back(0x01);
            data.push_back(0x0a);
            data.push_back(0x00);
            data.push_back(0x00);
            data.push_back(0x00);
            data.push_back(0x00);
            data.push_back(0x00);
            data.push_back(0x01);
            data.push_back(0x19);
            data.push_back(0xec);
            data.push_back(0xfd);
            data.push_back(0xc5);
        }
        return data;
    }

 private:
    bool parseConfig(boost::circular_buffer<char>& ring_buffer){
        std::cout << "parseConfig() \n";
        std::vector<char> package_config_bytes(package_config_size);
        if (ring_buffer.size() < package_config_size) {
            return false;
        }
        readBufferFront(ring_buffer, package_config_bytes.data(), package_config_bytes.size());
        ring_buffer.erase_begin(package_config_bytes.size());
        cpToDataBigEndian(block.package_size, &package_config_bytes[0], 2);
        cpToDataBigEndian(block.freq_type, &package_config_bytes[10], 1);
        cpToDataBigEndian(block.block_no, &package_config_bytes[11], 1);
        cpToDataBigEndian(block.package_num, &package_config_bytes[12], 2); // what's this?
        cpToDataBigEndian(block.timestamp, &package_config_bytes[14], 4); //
        block.point_num = (block.package_size - 20) / 3;
        block.freq = (block.freq_type - 2) * 5;
//        std::cout  << " blk no:" << block.block_no << " time:" << block.timestamp <<
//            " freq type:" << block.freq_type << " pack size:" << block.package_size << std::endl;
        if (ring_buffer.size() < block.package_size - package_config_size) {
            return false;
        }

        if(block.block_no == (uint8_t)1){
#ifdef ROS_NODE
            sys_recv_block_time = get_time_in_us();
#else
            sys_recv_block_time = sros::core::util::get_time_in_us();
#endif
            sensor_send_block_time = block.timestamp;
        }
        return true;
    }

    bool parseScanPoints(boost::circular_buffer<char>& ring_buffer){
//        std::cout << "parseScanPoints() \n";
        std::vector<char> package_data_bytes(block.package_size - package_config_size);
        readBufferFront(ring_buffer, package_data_bytes.data(), package_data_bytes.size());

        block.intensities.clear();
        block.measures.clear();
        for (int i=0; i < block.point_num; i++){
            block.measures.emplace_back();
            cpToDataBigEndian(block.measures.back(), &package_data_bytes[i * 3], 2);
            block.intensities.emplace_back();
            cpToDataBigEndian(block.intensities.back(), &package_data_bytes[i * 3 + 2], 1); //2
        }
        return true;
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

    static bool checkConvertedScan(std::shared_ptr<ScanMsg> &scan, int scan_point_num){
        if (scan->intensities.size() < scan->ranges.size()) {
            for (int i = 0; i < scan->ranges.size() - scan->intensities.size(); ++i) {
                scan->intensities.push_back(0);
            }
        }
        if (scan->ranges.size() >= scan_point_num && scan->intensities.size() >= scan_point_num) {
            scan->ranges.resize(scan_point_num);
            scan->intensities.resize(scan_point_num);
            return true;
        } else {
            LOG(INFO) << "range size:" << scan->ranges.size() << "," << scan->intensities.size();
        }
    }

    void open_output_file() {
        sync_time_out.open("/sros/bin/sync_lidar_time.txt", std::ios::out);
        sync_time_out << "(us)system_time " << " scan_time" << " sync_time " << " circle_time" << " ratio" << std::endl;
    }

    void save_sync_times(int64_t sync_time){
        if(save_sync_time && !sync_time_out.is_open()){
            open_output_file();
        }
        if(sync_time_out.is_open()) {
            sync_time_out << sys_recv_block_time << " " << sensor_send_block_time << " " << sync_time << std::endl;
        }
    }

 private:
    const float range_min = 0.01; // 这里错误
    const float range_min = 0.01; // 这里错误

    const float range_max = 40.0;
    const float angle_min = -135.0;
    const float angle_max = 135.0;
    const float angle_range = angle_max - angle_min;
    const float range_unit = 0.002; // 2mm
    const float intensity_unit = 5.0;
    const int package_config_size = 20;
    const int block_num = 6;
    OradarPackageBlock block;
    std::vector<OradarPackageBlock> package_blocks;

    int64_t sys_recv_block_time;
    int64_t sensor_send_block_time;

    std::ofstream sync_time_out;
    bool save_sync_time = false;
};
}

#endif  // STANDARD_LIDAR_DRIVER_ORADAR_LASER_PROTOCOL_HPP