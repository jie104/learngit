//
// Created by jin on 20-4-1.
//

#ifndef WJ_716_LIDAR_WANJI_LIDAR_MODULE_H
#define WJ_716_LIDAR_WANJI_LIDAR_MODULE_H

#include <modules/laser/base_laser_module.h>
#include <mutex>
#include <queue>
#include <thread>
#include "../standard_lidar_protocol/wanji_laser_protocol.hpp"
#include "../standard_lidar_protocol/tcp_scan_data_receiver.hpp"
#include "../standard_lidar_protocol/udp_scan_data_receiver.hpp"

namespace laser {
class GeneralLaserModule : public BaseLaserModule {
 public:
    GeneralLaserModule(const std::string& ip_address, int& port,std::string lidar_type);
    virtual ~GeneralLaserModule();

    //建立tcp连接
    virtual bool doOpen();
    //建立新线程,不断接收数据保存到队列中
    virtual void doStart();
    //关闭接收数据的线程
    virtual void doStop();
    //关闭socket
    virtual void doClose();
    //从queue里获取单帧scan
    virtual bool getScanData(sros::core::LaserScan_ptr& scan);

    std::string getSerialNO() const;

    std::string getModelNO() const;

    std::string getVersionNO() const;

protected:

    // 是否有扫描数据
    virtual bool hasScanData();

 private:
    std::string ip_;
    int port_;
    bool is_ok_ = false;
    std::string lidar_type_;
    std::shared_ptr<sros::LaserProtocolInterface> lidar_protocol_;
    std::shared_ptr<sros::DataReceiverInterface> data_receiver_;

};

}  // namespace laser
#endif  // WJ_716_LIDAR_WANJI_LIDAR_MODULE_H
