//
// Created by jin on 2020/4/9.
//
#include "rplidar_module.h"
#include "glog/logging.h"

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define DEG2RAD(x) ((x)*M_PI/180.)
namespace laser{
    using namespace rp::standalone::rplidar;

    RPLaserModule::RPLaserModule():drv_(nullptr),scan_mode_(""){
        drv_ = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
        if (!drv_) {
            LOG(INFO) << "Create Driver fail, exit";
        }
        serial_port_ = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0";
//        serial_port_ = "/dev/ttyUSB0";
        serial_baudrate_ = 256000;
        frame_id_ = "laser";
        scan_mode_ = "Sensitivity";// Stability for 10k, Sensitivity for 16k.
        LOG(INFO) << "Driver is created!";
    };
    RPLaserModule::~RPLaserModule(){
        if(drv_){
            RPlidarDriver::DisposeDriver(drv_);
        }
    }
    bool RPLaserModule::doOpen() {
        if (IS_FAIL(drv_->connect(serial_port_.c_str(), (_u32)serial_baudrate_))) {
            LOG(INFO) << "Error, cannot bind to the specified serial port " << serial_port_.c_str()<<","<<serial_baudrate_;
            RPlidarDriver::DisposeDriver(drv_);
            return false;
        }else{
            LOG(INFO) << "Open successfully!" << "," << serial_port_.c_str() << "," << serial_baudrate_;
            if (drv_->isConnected()) {
                LOG(INFO) << "connected rplidar!";
            }

            u_result     op_result;
            op_result = drv_->startMotor();//返回值只有ok
            //根据需要的模式启动scan
            if (scan_mode_.empty()) {
                op_result = drv_->startScan(false /* not force scan */, true /* use typical scan mode */, 0, &current_scan_mode_);
            } else {
                std::vector<RplidarScanMode> allSupportedScanModes;
                op_result = drv_->getAllSupportedScanModes(allSupportedScanModes);
                if (IS_OK(op_result)) {
                    _u16 selectedScanMode = _u16(-1);
                    for (std::vector<RplidarScanMode>::iterator iter = allSupportedScanModes.begin(); iter != allSupportedScanModes.end(); iter++) {
                        if (std::string(iter->scan_mode) == scan_mode_) {
                            selectedScanMode = iter->id;
                            break;
                        }
                    }
                    if (selectedScanMode == _u16(-1)) {
                        LOG(INFO) << "scan mode " << scan_mode_.c_str() << "is not supported by lidar, supported modes:";
                        for (std::vector<RplidarScanMode>::iterator iter = allSupportedScanModes.begin(); iter != allSupportedScanModes.end(); iter++) {
                            LOG(INFO) << iter->scan_mode << "\t: max_distance: " << iter->max_distance << " m, Point number: " << (1000/iter->us_per_sample) << "K";
                        }
                        op_result = RESULT_OPERATION_FAIL;
                    } else {//根据mode id启动
                        op_result = drv_->startScanExpress(false /* not force scan */, selectedScanMode, 0, &current_scan_mode_);
                    }
                }
            }
            if (IS_OK(op_result)) {
                connected_ = true;
                //default frequent is 10 hz (by motor pwm value),  current_scan_mode.us_per_sample is the number of scan point per us
                angle_compensate_multiple_ = (int) (1000 * 1000 / current_scan_mode_.us_per_sample / 10.0 / 360.0);
                if (angle_compensate_multiple_ < 1)
                    angle_compensate_multiple_ = 1;
                max_distance_ = current_scan_mode_.max_distance;
                LOG(INFO) << "current scan mode: " << current_scan_mode_.scan_mode << ", max_distance: "
                          << current_scan_mode_.max_distance << " m, Point number: "
                          << (1000 / current_scan_mode_.us_per_sample) << "K, angle_compensate: "
                          << angle_compensate_multiple_;
                return true;
            } else {
                LOG(INFO) << "Can not start scan!";
                return false;
            }
        }
    }

    void RPLaserModule::doStart() {
        if(!drv_){
            LOG(INFO) << "Driver is null while start!";
        }
        if(connected_){
            started_ = true;
        }else{
            LOG(INFO) << "Start failed cause disconnected!";
        }
    }

    void RPLaserModule::doStop() {
        drv_->stopMotor();
        drv_->stop();
        started_ = false;
        LOG(INFO) << "Lidar has been stopped!";
    }

    void RPLaserModule::doClose() {
        connected_ = false;
        RPlidarDriver::DisposeDriver(drv_);//断开连接,析构driver
        LOG(INFO) << "Disconnected!";
    }

    bool RPLaserModule::getScanData(sros::core::LaserScan_ptr &scan_ptr) {
        LOG(INFO) << "getScanData";
        rplidar_response_measurement_node_hq_t nodes[360*8];
        size_t   count = _countof(nodes);
        if(!connected_ || !started_){
//            LOG(INFO) << "Fail while getScanData, connected status: " << connected_ << ", started status: " << started_;
            return false;
        }else{
            LOG(INFO) << "connected status: " << connected_ << ", started status: " << started_;
        }
        auto start_scan_time = sros::core::util::get_time_in_s();
        u_result op_result;
        op_result = drv_->grabScanDataHq(nodes, count);
        auto end_scan_time = sros::core::util::get_time_in_s();
        double scan_duration = end_scan_time - start_scan_time;

        if (op_result == RESULT_OK) {
            LOG(INFO) << "grab ok";
            op_result = drv_->ascendScanData(nodes, count);
            float angle_min = DEG2RAD(0.0f);
            float angle_max = DEG2RAD(359.0f);
            if (op_result == RESULT_OK) {
                if (angle_compensate) {
                    //const int angle_compensate_multiple = 1;
                    const int angle_compensate_nodes_count = 360*angle_compensate_multiple_;
                    int angle_compensate_offset = 0;
                    rplidar_response_measurement_node_hq_t angle_compensate_nodes[angle_compensate_nodes_count];
                    memset(angle_compensate_nodes, 0, angle_compensate_nodes_count*sizeof(rplidar_response_measurement_node_hq_t));

                    int i = 0, j = 0;
                    for( ; i < count; i++ ) {
                        if (nodes[i].dist_mm_q2 != 0) {
                            float angle = getAngle(nodes[i]);
                            int angle_value = (int)(angle * angle_compensate_multiple_);
                            if ((angle_value - angle_compensate_offset) < 0) angle_compensate_offset = angle_value;
                            for (j = 0; j < angle_compensate_multiple_; j++) {

                                int angle_compensate_nodes_index = angle_value-angle_compensate_offset+j;
                                if(angle_compensate_nodes_index >= angle_compensate_nodes_count)
                                    angle_compensate_nodes_index = angle_compensate_nodes_count-1;
                                angle_compensate_nodes[angle_compensate_nodes_index] = nodes[i];
                            }
                        }
                    }
                    wrapScan(scan_ptr, angle_compensate_nodes, angle_compensate_nodes_count,
                                 start_scan_time, scan_duration,
                                 angle_min, angle_max);
                } else {
                    int start_node = 0, end_node = 0;
                    int i = 0;
                    // find the first valid node and last valid node
                    while (nodes[i++].dist_mm_q2 == 0);
                    start_node = i-1;
                    i = count -1;
                    while (nodes[i--].dist_mm_q2 == 0);
                    end_node = i+1;

                    angle_min = DEG2RAD(getAngle(nodes[start_node]));
                    angle_max = DEG2RAD(getAngle(nodes[end_node]));

                    wrapScan(scan_ptr, &nodes[start_node], end_node-start_node +1,
                                 start_scan_time, scan_duration,
                                 angle_min, angle_max);
                }
            } else if (op_result == RESULT_OPERATION_FAIL) {
                // All the data is invalid, just publish them
                float angle_min = DEG2RAD(0.0f);
                float angle_max = DEG2RAD(359.0f);
                wrapScan(scan_ptr, nodes, count,
                             start_scan_time, scan_duration,
                             angle_min, angle_max);
            }
            return true;
        }else{
            LOG(INFO) << "Grab failed!";
            return false;
        }
    }

    std::string RPLaserModule::getSerialNO()const{return "";}

    std::string RPLaserModule::getModelNO()const {return "";}

    std::string RPLaserModule::getVersionNO()const{return "";}

    void RPLaserModule::wrapScan(sros::core::LaserScan_ptr &scan_msg, rplidar_response_measurement_node_hq_t *nodes,
                                 size_t node_count, uint64_t start,
                                 double scan_time,
                                 float angle_min, float angle_max) {
        scan_msg->time_ = sros::core::util::get_time_in_us();
//        scan_msg->angle_min = -3.12414;
//        scan_msg->angle_max = 3.14159;
        bool reversed = (angle_max > angle_min);
        if ( reversed ) {
            scan_msg->angle_min =  M_PI - angle_max;
            scan_msg->angle_max =  M_PI - angle_min;
//            LOG(INFO) << "REversed:" << scan_msg->angle_min << ", " << scan_msg->angle_max;

        } else {
            scan_msg->angle_min =  M_PI - angle_min;
            scan_msg->angle_max =  M_PI - angle_max;
        }
        scan_msg->angle_increment =
                (scan_msg->angle_max - scan_msg->angle_min) / (double)(node_count-1);

        scan_msg->scan_time = scan_time;
        scan_msg->time_increment = scan_time / (double)(node_count-1);
        scan_msg->range_min = 0.15;
        scan_msg->range_max = max_distance_;//8.0;

        scan_msg->intensities.resize(node_count);
        scan_msg->ranges.resize(node_count);
//        bool reverse_data = (!inverted_ && reversed) || (inverted_ && !reversed);
//        if (!reverse_data) {
//            for (size_t i = 0; i < node_count; i++) {
//                float read_value = (float) nodes[i].dist_mm_q2/4.0f/1000;
//                if (read_value == 0.0)
//                    scan_msg->ranges[i] = std::numeric_limits<float>::infinity();
//                else
//                    scan_msg->ranges[i] = read_value;
//                scan_msg->intensities[i] = (float) (nodes[i].quality >> 2);
//            }
//        } else {
        for (size_t i = 0; i < node_count; i++) {
            float read_value = (float)nodes[i].dist_mm_q2/4.0f/1000.0f;
            if (read_value == 0.0)
                scan_msg->ranges[node_count-1-i] = 0.0;
            else
                scan_msg->ranges[node_count-1-i] = read_value;
            scan_msg->intensities[node_count-1-i] = (float) (nodes[i].quality >> 2);
        }
    }
}