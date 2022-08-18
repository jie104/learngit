//
// Created by jin on 2020/4/16.
//

#ifndef FITXXX_MINGZHI_LASER_MODULE_H
#define FITXXX_MINGZHI_LASER_MODULE_H
#include <modules/laser/base_laser_module.h>
#include "modules/laser/MINGZHI/include/FITXXX/FITXXX.h"
//#include <sensor_msgs/LaserScan.h>
#include <thread>
#include "core/circle_stamp_array.hpp"

namespace laser {
    class MingzhiLaserModule : public BaseLaserModule{
    private:
        // Params
        double scan_range_min_;
        double scan_range_max_;
        double angle_min_;
        double angle_max_;
        int port_;
        int scan_seq_;

        std::string host_ip_;
        std::string frame_;
        std::string frame_id_;

        // Laser status
        bool connect_status_;
        bool get_config_;
        bool scanParamInitialized_;
        FITXXX laser;

        // Laser config
        ULDINI_Type uld_config_;
        // Laser message
        sros::core::LaserScanMsg scan_msg_;

        uint64_t HB_time_;//当前时间，以s为单位

        //心跳线程
        std::thread heart_beat_thread_;
        bool heart_beat_flag_;

        //时间校准
        CircleStampArray<int64_t> delta_timestamp_;
        int64_t last_lidar_stamp = 0;

        // Get Laser config.
        bool getConfig();

        // load from laser config
        void initialize_scan_param();

        // Start continous laser scan msgs
        void startMesure();

        // Send Heart Beat to laser.
        void sendHB2Lidar();


    public:
        MingzhiLaserModule();

        virtual ~MingzhiLaserModule();

        virtual bool doOpen();

        virtual void doStart();

        virtual void doStop();

        virtual void doClose();

        virtual bool getScanData(sros::core::LaserScan_ptr& scan);
    };
}
#endif //FITXXX_MINGZHI_LASER_MODULE_H
