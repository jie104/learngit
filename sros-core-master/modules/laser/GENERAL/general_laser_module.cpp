//
// Created by jin on 20-4-3.
//

//
// Created by jin on 20-4-1.
//

#include "general_laser_module.h"
#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>
#include "../standard_lidar_protocol/keli_laser_protocol.hpp"
#include "../standard_lidar_protocol/siminics_laser_protocol.hpp"
#include "../standard_lidar_protocol/oradar_laser_protocol.hpp"
namespace laser {
GeneralLaserModule::GeneralLaserModule(const std::string &ip_address, int &port,std::string lidar_type) : ip_(ip_address), port_(port),lidar_type_(lidar_type) {
    LOG(ERROR) << "will creat:" << lidar_type_;
    LOG(ERROR) << "successfully!";
}

GeneralLaserModule::~GeneralLaserModule() {}

//建立网络连接, tcp or udp
bool GeneralLaserModule::doOpen() {
    LOG(INFO) << "Start open process!";
    LOG(ERROR) << "port:" << ip_ << "," << port_;
    bool is_connect = false;
    while (!is_connect) {
        if(lidar_type_ == "WANJI"){
            lidar_protocol_.reset(new sros::WanjiLaserProtocol);
            data_receiver_.reset(new sros::TcpScanDataReceiver(lidar_protocol_, ip_, port_));
        }else if(lidar_type_ == "SIMINICSPAVO"){
            lidar_protocol_.reset(new sros::SiminicsLaserProtocol);
            data_receiver_.reset(new sros::TcpScanDataReceiver(lidar_protocol_, ip_, port_));
        }else if(lidar_type_ == "KELI"){
            lidar_protocol_.reset(new sros::KeliLaserProtocol("KLM-2036DE"));
            data_receiver_.reset(new sros::UdpScanDataReceiver(lidar_protocol_, ip_, port_));
        }else if(lidar_type_ == "KELI_270"){
            lidar_protocol_.reset(new sros::KeliLaserProtocol("KLM-2027DE"));
            data_receiver_.reset(new sros::UdpScanDataReceiver(lidar_protocol_, ip_, port_));
        }else if(lidar_type_ == "ORADAR_ORBB"){
            lidar_protocol_.reset(new sros::OradarLaserProtocol);
            data_receiver_.reset(new sros::UdpScanDataReceiver(lidar_protocol_, ip_, port_));
        }

        // 增加对网口数据的判断，保证设备初始化完成
        if (data_receiver_->isConnected()) {
            if (hasScanData()) {
                is_connect = true;
            } else {
                is_connect = false;
                doClose();
            }
        }
    }
    if (is_connect) {
        is_ok_ = true;
    }
    return is_ok_;
}

void GeneralLaserModule::doStart() {
    LOG(INFO) << "Do start";
    return;
}

//关闭接收数据的线程
void GeneralLaserModule::doStop() {
    if (is_ok_) {
        is_ok_ = false;
        LOG(INFO) << "Please close socket!";
    } else {
        LOG(INFO) << "Stopped already!";
    }
}

//关闭socket
void GeneralLaserModule::doClose() {
    if (data_receiver_) {
        data_receiver_->disconnect();
    }
}

//从queue里获取单帧scan
bool GeneralLaserModule::getScanData(sros::core::LaserScan_ptr &scan) {  // 1s内获取不到scan返回false;
    if (!data_receiver_->isConnected()) {
        doOpen();
    }
    if(data_receiver_->getScan(scan)){
        scan->angle_max = scan->angle_min + (float)scan->ranges.size() * scan->angle_increment - scan->angle_increment;
        return true;
    }
    return false;
}

std::string GeneralLaserModule::getSerialNO() const { return "1.1.1"; }
std::string GeneralLaserModule::getModelNO() const { return "1.1.1"; }
std::string GeneralLaserModule::getVersionNO() const { return "1.1.1"; }

// 是否有扫描数据【开机上电后初始化慢，导致网口能连上但无数据，表现为开机雷达报故障】
bool GeneralLaserModule::hasScanData() {
    auto laser_scan = std::make_shared<sros::core::LaserScanMsg>();
    return getScanData(laser_scan);
}

}  // namespace laser
