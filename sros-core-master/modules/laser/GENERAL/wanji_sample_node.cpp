//
// Created by lfc on 2021/10/13.
//
#include "../standard_lidar_protocol/wanji_laser_protocol.hpp"
#include "../standard_lidar_protocol/tcp_scan_data_receiver.hpp"

int main(int argc, char* argv[]) {
    std::shared_ptr<sros::WanjiLaserProtocol> wanji(new sros::WanjiLaserProtocol);
    sros::TcpScanDataReceiver receiver(wanji,"192.168.1.100",2110);
    while(1){
        sros::core::LaserScan_ptr scan;
        if(receiver.getScan(scan)){
            LOG(INFO) << "scan:" << scan->time_ << "," << scan->ranges.size() << ","
                      << (sros::core::util::get_time_in_us() - scan->time_) / 1.0e6;
        }
//        usleep(4e4);
    }
}