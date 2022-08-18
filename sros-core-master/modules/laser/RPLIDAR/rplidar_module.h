//
// Created by jin on 2020/4/9.
//

#ifndef RPLIDAR_ROS_RPLIDAR_MODULE_H
#define RPLIDAR_ROS_RPLIDAR_MODULE_H
#include <string>
//#include <sensor_msgs/LaserScan.h>
#include "sdk/include/rplidar.h"
#include <modules/laser/base_laser_module.h>

namespace laser{
    using namespace rp::standalone::rplidar;
    class RPLaserModule : public BaseLaserModule{
    public:
        RPLaserModule();

        virtual ~RPLaserModule();

        virtual bool doOpen();

        virtual void doStart();

        virtual void doStop();

        virtual void doClose();

        virtual bool getScanData(sros::core::LaserScan_ptr& scan_ptr);

        std::string getSerialNO() const;

        std::string getModelNO() const;

        std::string getVersionNO() const;

    private:
        rp::standalone::rplidar::RPlidarDriver *drv_;
        bool connected_ = false;
        bool started_ = false;
//        std::string channel_type;
//        std::string tcp_ip;
        std::string serial_port_;
//        int tcp_port = 20108;
        int serial_baudrate_ = 256000;
        RplidarScanMode current_scan_mode_;
        std::string frame_id_;
        bool inverted_ = false;
        bool angle_compensate = true;
        float max_distance_ = 8.0;
        int angle_compensate_multiple_ = 1;//it stand of angle compensate at per 1 degree
        std::string scan_mode_;
        static float getAngle(const rplidar_response_measurement_node_hq_t& node)
        {
            return node.angle_z_q14 * 90.f / 16384.f;
        }
        void wrapScan(sros::core::LaserScan_ptr& scan_msg, rplidar_response_measurement_node_hq_t *nodes,
                      size_t node_count, uint64_t start,
                      double scan_time,
                      float angle_min, float angle_max);
    };

}
#endif //RPLIDAR_ROS_RPLIDAR_MODULE_H
