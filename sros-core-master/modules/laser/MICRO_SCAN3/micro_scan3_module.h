//
// Created by lfc on 16-11-10.
//

#ifndef SROS_MICRO_SCAN3_MODULE_H
#define SROS_MICRO_SCAN3_MODULE_H

#include <modules/laser/base_laser_module.h>
//#include "sick_safetyscanners/SickSafetyscanners.h"
#include "sick_safetyscanners/datastructure/CommSettings.h"
#include "sick_safetyscanners/datastructure/Data.h"
#include "../standard_laser_module.h"
#include "core/circle_optimizer_set.hpp"
#include <queue>
namespace sick{
class SickSafetyscanners;
}

namespace laser {
class MicroScan3Module : public BaseLaserModule {
 public:
    MicroScan3Module(std::string ip_address,int ip_port);

    virtual ~MicroScan3Module();

    virtual bool doOpen();

    virtual void doStart();

    virtual void doStop();

    virtual void doClose();

    virtual bool getScanData(sros::core::LaserScan_ptr& scan_ptr);

    std::string getSerialNO() const;

    std::string getModelNO() const;

    std::string getVersionNO() const;

    virtual void setHostIpAddress(std::string host_ip);

 private:
    int64_t getSyncStamp(int64_t lidar_time,int64_t system_time);

    void receivedUDPPacket(const sick::datastructure::Data& data);

    bool buildScan(const int64_t stamp,const sick::datastructure::Data& data,sros::core::LaserScan_ptr& scan_ptr);

    sros::core::LaserScan_ptr popScan();

    void pushScan(const sros::core::LaserScan_ptr& scan);

    inline float radToDeg(float rad)
    {
        return rad * 180.0f / M_PI;
    }

    inline float degToRad(float deg)
    {
        return deg * M_PI / 180.0f;
    }
    //! IP or hostname of laser range finder
    std::string scanner_ip_;

    int ip_port_ = 0;

    //! scan_frequency parameter
    int scan_frequency_;

    //! samples_per_scan parameter
    int samples_per_scan_;

    //! Serial Number
    std::string serial_no_ = "1";

    //! Model number
    std::string model_no_ = "1";

    //! Version number
    std::string version_no_="1";
    bool have_get_version_ = false;

    circle::CircleOptimizerArray<int64_t> delta_timestamp_;

    float range_max_;
    float range_min_;
    const float angle_offset_ = -90.0f;

    std::mutex condition_mutex_;
    std::mutex read_write_mutex_;
    std::condition_variable_any condition_;
    std::queue<sros::core::LaserScan_ptr> scan_queue_;

    std::shared_ptr<sick::SickSafetyscanners> device_;
    sick::datastructure::CommSettings communication_settings_;
};
}  // namespace laser

#endif  // SROS_OMD30M_MODULE_H
