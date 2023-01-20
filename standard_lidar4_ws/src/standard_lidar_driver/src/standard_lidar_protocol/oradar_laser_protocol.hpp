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
        std::vector<uint16_t> intensities;
    };


    OradarLaserProtocol() {
    }

    virtual ~OradarLaserProtocol() {
        sync_time_out.close();
    }

    virtual int findPackageStart(boost::circular_buffer<char> &ring_buffer) {
        if (ring_buffer.size() < 60) return -1;
        for (std::size_t i = 0; i < ring_buffer.size() - 4; i++) {
            if (((unsigned char) ring_buffer[i]) == 0x4D && ((unsigned char) ring_buffer[i + 1]) == 0x53 &&
                ((unsigned char) ring_buffer[i + 2]) == 0x01 && ((unsigned char) ring_buffer[i + 3]) == 0xF4 &&
                ((unsigned char) ring_buffer[i + 4]) == 0xEB && ((unsigned char) ring_buffer[i + 5]) == 0x90) {
                return i;
            }
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
        if (last_block.block_no == BLOCK_NUM) {
            return true;
        }
        return false;
    }

    virtual bool cpToScan(std::shared_ptr<ScanMsg> &scan) { // convert 2 scan
        scan.reset(new ScanMsg); // 初始化保证清空

        int64_t sync_time = getSyncTime(sys_recv_block_time, sensor_send_block_time); // use first block
        save_sync_times(sync_time);
        scan->scan_time = 1.0 / block.freq; // 雷达转360度的时间，ros系统用s为单位，其实可以用条件判断处理。
//#ifdef ROS_NODE
//        scan->header.stamp = ros::Time::now();
//#else
//        scan->time_ = sync_time + 90 / 360 * scan->scan_time * 1e6; // cal middle scan time, unit us, received end scan time of first package block (6 blocks)
//#endif
        int scan_point_num = BLOCK_NUM * block.point_num;
        scan->angle_increment = ANGLE_RANGE_DEG / scan_point_num * M_PI / 180.0; // 15Hz 0.17857
        scan->angle_min = ANGLE_MIN_DEG * M_PI / 180.0;
        scan->angle_max = ANGLE_MAX_DEG * M_PI / 180.0;
        scan->time_increment = 1.0 / (block.freq * scan_point_num * 360 / ANGLE_RANGE_DEG);
        scan->range_min = RANGE_MIN;
        scan->range_max = RANGE_MAX;
        int total_num = 0;
        for(auto &blk : package_blocks){
            total_num += blk.point_num;
            for (int i = 0; i < blk.point_num; ++i) {
                scan->ranges.push_back(float(blk.measures[i]) * RANGE_UNIT);
                scan->intensities.push_back(float(blk.intensities[i]) * INTENSITY_UNIT);
            }
        }
        package_blocks.clear();
        return checkConvertedScan(scan, scan_point_num);
    }

    virtual bool checkScanValid() {
        if (package_blocks.size() == 6) {
            if (package_blocks.back().block_no == 6) {
                return true;
            }
        }
        LOG(INFO) << "package is wrong! will clear!";
        package_blocks.clear();
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
        std::vector<char> package_config_bytes(PACKAGE_CONFIG_SIZE);
        if (ring_buffer.size() < PACKAGE_CONFIG_SIZE) {
            return false;
        }
        readBufferFront(ring_buffer, package_config_bytes.data(), package_config_bytes.size());
        ring_buffer.erase_begin(package_config_bytes.size());
        cpToDataBigEndian(block.package_size, &package_config_bytes[6], 2);
        cpToDataBigEndian(block.freq_type, &package_config_bytes[10], 1);
        cpToDataBigEndian(block.block_no, &package_config_bytes[11], 1);
        cpToDataBigEndian(block.package_num, &package_config_bytes[12], 2); // what's this?
        cpToDataBigEndian(block.timestamp, &package_config_bytes[14], 4); //
        block.point_num = (block.package_size - 20) / 4;
        block.freq = (block.freq_type - 2) * 5;
        if (ring_buffer.size() < block.package_size - PACKAGE_CONFIG_SIZE) {
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
        std::vector<char> package_data_bytes(block.package_size - PACKAGE_CONFIG_SIZE);
        readBufferFront(ring_buffer, package_data_bytes.data(), package_data_bytes.size());

        block.intensities.clear();
        block.measures.clear();
        for (int i=0; i < block.point_num; i++){
            block.measures.emplace_back();
            cpToDataBigEndian(block.measures.back(), &package_data_bytes[i * 4], 2);
            block.intensities.emplace_back();
            cpToDataBigEndian(block.intensities.back(), &package_data_bytes[i * 4 + 2], 2);
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
    const float RANGE_MIN = 0.01;
    const float RANGE_MAX = 40.0;
    const float ANGLE_MIN_DEG = -135.0;
    const float ANGLE_MAX_DEG = 135.0;
    const float ANGLE_RANGE_DEG = ANGLE_MAX_DEG - ANGLE_MIN_DEG;
    const float RANGE_UNIT = 0.002; // 2mm
    const float INTENSITY_UNIT = 1.0;
    const int PACKAGE_CONFIG_SIZE = 20;
    const int BLOCK_NUM = 6;
    OradarPackageBlock block;
    std::vector<OradarPackageBlock> package_blocks;

    int64_t sys_recv_block_time;
    int64_t sensor_send_block_time;

    std::ofstream sync_time_out;
    bool save_sync_time = false;
};
}

#endif  // STANDARD_LIDAR_DRIVER_ORADAR_LASER_PROTOCOL_HPP
