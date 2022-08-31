//
// Created by chj on 2022/1/21.
//

#ifndef STANDARD_LIDAR_DRIVER_SIMINICS_LASER_PROTOCOL_HPP
#define STANDARD_LIDAR_DRIVER_SIMINICS_LASER_PROTOCOL_HPP
#include <glog/logging.h>
#include "laser_protocol_interface.hpp"
#include "synchronize_time_manager.hpp"


namespace sros {
class SiminicsLaserProtocol : public LaserProtocolInterface {
 public:
    struct SiminicsScanPackage {
        uint32_t data_id; // unnecessary
        uint16_t data_type; // unnecessary
        uint16_t angle_resolution;
//        uint16_t angle_start; // 雷达的角度坐标系和我们采用的不一致, 因此不读取该值
//        uint16_t angle_end;
        uint32_t data_length;
        uint16_t point_num;
        uint32_t stamp; // 表示当前TCP包最后一个点云数据发送激光的时间
        std::vector<uint16_t> intensities_data;
        std::vector<uint16_t> ranges_data;
    };

    SiminicsLaserProtocol() {}

    virtual ~SiminicsLaserProtocol() {}

    virtual int findPackageStart(boost::circular_buffer<char>& ring_buffer) {
        if (ring_buffer.size() < 60) return -1;     // why 60?
        for (std::size_t i = 0; i < ring_buffer.size() - 8; i++) {
            if (((unsigned char)ring_buffer[i + 0]) == PROTOCOL_HEAD[0] &&
                ((unsigned char)ring_buffer[i + 1]) == PROTOCOL_HEAD[1] &&
                ((unsigned char)ring_buffer[i + 2]) == PROTOCOL_HEAD[2] &&
                ((unsigned char)ring_buffer[i + 3]) == PROTOCOL_HEAD[3] &&
                ((unsigned char)ring_buffer[i + 4]) == PROTOCOL_HEAD[4] &&
                ((unsigned char)ring_buffer[i + 5]) == PROTOCOL_HEAD[5] &&
                ((unsigned char)ring_buffer[i + 6]) == PROTOCOL_HEAD[6] &&
                ((unsigned char)ring_buffer[i + 7]) == PROTOCOL_HEAD[7]) {
                return i;
            }
        }
        return -2;
    }

    virtual bool resolvePackage(boost::circular_buffer<char>& ring_buffer, int start_index) {
        sys_recv_time = get_time_in_us();
        // 可在转换之前 重置simin_scan
        ring_buffer.erase_begin(start_index);
        if(parseConfig(ring_buffer)){
            if(parseScanData(ring_buffer)){
                sensor_send_time = simin_scan.stamp;
                return true;
            }
        }
        return false;
    }

    // rename to getPackageHeadAnaTail
    virtual void getStartCmd(std::vector<std::vector<char>>& datas) {
        datas.resize(3);
        int index = 0;
        for(auto& data:datas) {
            data.clear();

            for (int i = 0; i < PROTOCOL_HEAD_LEN; ++i) {
                data.push_back(PROTOCOL_HEAD[i]);
            }
            for (int j = 0; j < PROTOCOL_VERSION_RESERVER_LEN; ++j) {
                data.push_back(PROTOCOL_VERDION_RESERVER[j]);
            }
            uint16_t body_size = 8;
            data.push_back(0x00);
            data.push_back(0x00);
            data.push_back(0xff & body_size >> 8);
            data.push_back(0xff & body_size);
            auto body = getBody(index);
            index++;
            for (auto &d : body) {
                data.push_back(d);
            }
            for (int j = 0; j < PROTOCOL_END_LEN; ++j) {
                data.push_back(PROTOCOL_END[j]);
            }
        }
    }

    std::vector<char> getBody(int head = 0){
        std::vector<char> data;
        if (head == 0) {
            data.push_back(0xEB);
            data.push_back(0x90);
            data.push_back(0xA0);
            data.push_back(0xA0);
            data.push_back(0x00);
            data.push_back(0x00);
            data.push_back(0x00);
            data.push_back(0x01);
        } else if (head == 1) {
            data.push_back(0xEB);
            data.push_back(0x90);
            data.push_back(0xA0);
            data.push_back(0xA6);
            data.push_back(0x00);
            data.push_back(0x00);
            data.push_back(0x00);
            data.push_back(0x01);
        } else if (head == 2) {
            data.push_back(0xEB);
            data.push_back(0x90);
            data.push_back(0xA0);
            data.push_back(0xA0);
            data.push_back(0x00);
            data.push_back(0x00);
            data.push_back(0x00);
            data.push_back(0x00);
        }
        return data;
    }

    virtual bool isEndPackage() { // scan in one package, always true.
        return true;
    }

    virtual bool cpToScan(std::shared_ptr<ScanMsg>& scan) {
        if (!scan) {
            scan.reset(new ScanMsg);
        }
        int64_t sync_time = getSyncTime(sys_recv_time, sensor_send_time);
        scan->scan_time = 1 / SCAN_FREQ; // scan period, unit s
#ifdef ROS_NODE
        scan->header.stamp = ros::Time::now();
        scan->header.frame_id = "laser";
#else
        scan->time_ = sync_time - ANGLE_RANGE_DEG / 2 / 360.0 * scan->scan_time * 1e6; // unit us, calculate middle time. received end scan time.
#endif
        scan->angle_increment = simin_scan.angle_resolution * ANGLE_UNIT *M_PI / 180.0; // 0.08, 0.16, 0.32
//        scan->angle_min = simin_scan.angle_start * ANGLE_UNIT * M_PI / 180.0; // -140.0 / 180.0 * M_PI;
//        scan->angle_max = simin_scan.angle_end * ANGLE_UNIT* M_PI / 180.0; // 140.0 / 180.0 * M_PI;
        scan->time_increment = 1.0 / (SCAN_FREQ * simin_scan.point_num * 360 / ANGLE_RANGE_DEG);
        scan->range_min = 0.01;
        scan->range_max = 120;

        scan->ranges.clear();
        scan->intensities.clear();
        for (int i = 0; i < simin_scan.point_num; ++i) {
            scan->intensities.push_back(float(simin_scan.intensities_data[i]));
            scan->ranges.push_back(float(simin_scan.ranges_data[i]) * RANGE_UNIT);
        }
        if (scan->intensities.size() < scan->ranges.size()) { // 可能存在丢失强度的情况？ 从代码逻辑看不会。
            for (int i = 0; i < scan->ranges.size() - scan->intensities.size(); ++i) {
                scan->intensities.push_back(0);
            }
        }
        return true;
    }

    virtual bool checkScanValid() {
        return true;
    }

 private:
    bool parseConfig(boost::circular_buffer<char>& ring_buffer){
        if (ring_buffer.size() < PACKAGE_CONFIG_SIZE) {
            return false;
        }
        std::vector<char> package_config_bytes(PACKAGE_CONFIG_SIZE);
        readBufferFront(ring_buffer, package_config_bytes.data(), package_config_bytes.size());
        ring_buffer.erase_begin(package_config_bytes.size());

        // parse config
        cpToDataBigEndian<uint32_t>(simin_scan.data_length, &package_config_bytes[12], 4);
        simin_scan.point_num = (simin_scan.data_length - 24) / 4;
        cpToDataBigEndian(simin_scan.data_id, &package_config_bytes[20], 4);
        cpToDataBigEndian(simin_scan.data_type, &package_config_bytes[28], 2);
        cpToDataBigEndian(simin_scan.angle_resolution, &package_config_bytes[30], 2);
        LOG(INFO) << "angle_resolution: " << simin_scan.angle_resolution << std::endl;
//        cpToDataBigEndian(simin_scan.angle_start, &package_config_bytes[32], 2);
//        cpToDataBigEndian(simin_scan.angle_end, &package_config_bytes[34], 2);

        return true;
    }

    bool parseScanData(boost::circular_buffer<char>& ring_buffer){
        if (ring_buffer.size() < simin_scan.point_num * 4 + 8) {
            return false;
        }
        std::vector<char> package_data_bytes(simin_scan.point_num * 4 + 4);
        readBufferFront(ring_buffer, package_data_bytes.data(), package_data_bytes.size());
        ring_buffer.erase_begin(package_data_bytes.size());
        cpToDataBigEndian(simin_scan.stamp, &package_data_bytes[package_data_bytes.size() - 4], 4);
        if (checkXor(package_data_bytes.data(), package_data_bytes.size())){
            simin_scan.intensities_data.clear();
            for (int i = 0; i < simin_scan.point_num; ++i) {
                simin_scan.intensities_data.emplace_back();
                cpToDataBigEndian(simin_scan.intensities_data.back(), &package_data_bytes[i * 4], 2);
            }
            simin_scan.ranges_data.clear();
            for (int i = 0; i < simin_scan.point_num; ++i) {
                simin_scan.ranges_data.emplace_back();
                cpToDataBigEndian(simin_scan.ranges_data.back(), &package_data_bytes[i * 4 + 2], 2);
            }
            return true;
        }else{
            return false;
        }
    }

    bool checkXor(char* recvbuf, int recvlen) {
        return true;
    }

 private:
    const float RANGE_UNIT = 0.002; // 2mm
    const float ANGLE_UNIT = 0.01;
    const float ANGLE_RANGE_DEG = 280.0;
    const int PACKAGE_CONFIG_SIZE = 36;
    const float SCAN_FREQ = 50;
    SiminicsScanPackage simin_scan; // parse a received tcp package to this siminics scan.
    int64_t sys_recv_time;
    int64_t sensor_send_time;

    // tcp protocol fixed head and tail
    const unsigned char PROTOCOL_HEAD[8] = { 0xFF,0x53,0x4D,0x49,0x4E,0x49,0x43,0x53 };
    const int PROTOCOL_HEAD_LEN = 8;
    const unsigned char PROTOCOL_VERDION_RESERVER[4] = { 0x01,0x0,0x0,0x0 };
    const int PROTOCOL_VERSION_RESERVER_LEN = 4;
    const int PROTOCOL_DATA_LEN = 4;
    const unsigned char PROTOCOL_END[4] = { 0xFE,0xFE,0xFE,0xFE };
    const int PROTOCOL_END_LEN = 4;
};

}  // namespace sros

#endif //STANDARD_LIDAR_DRIVER_SIMINICS_LASER_PROTOCOL_HPP
