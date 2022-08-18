//
// Created by lfc on 2020/11/23.
//

#ifndef SERIAL_READ_IMU_SERIAL_READ_HPP
#define SERIAL_READ_IMU_SERIAL_READ_HPP
#include <glog/logging.h>
#include <boost/asio.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/function.hpp>
#include <memory>
#include "imu_protocol_interface.hpp"
#include <fstream>
#include <iostream>
namespace imu {
typedef boost::function<void(const std::shared_ptr<ImuInfo>)> SerialCallbackFunc;

class ImuSerialRead {
 public:
    ImuSerialRead(const std::string serial_port, int baud_rate, std::shared_ptr<ImuProtocolInterface> imu_protocol)
        : serial_port_(serial_port), baud_rate_(baud_rate), imu_protocol_(imu_protocol), rings_(1024) {}

    virtual ~ImuSerialRead() {
        origin_imu_out_.close();
    }

    bool open() {
        if (!is_open_ && init()) {
            boost::thread(boost::bind(&ImuSerialRead::readLoop, this));
            origin_imu_out_.open("/sros/bin/imu_origin.txt", std::ios::out);
            origin_imu_out_ << "time_in_sensor " << " acc_x(g) " << "acc_y " << "acc_z " << "gyro_x(deg/s) " 
                            << "gyro_y " << "gyro_z " << "yaw(deg) " << "roll " << "pitch " << std::endl;
        }
        return is_open_;
    }

    void setDataReceiveCallback(const SerialCallbackFunc &callback) { dataCallback_ = callback; }

    void close() { is_open_ = false; }

    void readLoop() {
        LOG(INFO) << "begin to read imu info!";
        while (is_open_ || need_wait_) {
            try {
                auto data_length = imu_protocol_->maxDataLength();
                std::vector<unsigned char> datas(data_length, 0);
                if (readData(datas, data_length)) {
                    // LOG(INFO) << "read!" << data_length;
                    datas.resize(data_length);
                    resolve(datas);
                    need_wait_ = false;
                } else {
                    LOG(INFO) << "err to read anything!";
                }
            } catch (std::exception &e) {
                LOG(INFO) << "serial response an exception!" << e.what();
                need_wait_ = true;
                init();
            }
        }
    }

    bool readData(std::vector<unsigned char> &data_read, int &data_size) {
        int read_size = serial_->read_some(boost::asio::buffer(data_read, data_size));
        data_size = read_size;
        return data_size != 0;
    }

    void resolve(const std::vector<unsigned char> &datas) {
        ImuErrorCode check_result;
        for (auto &data : datas) {
            //            std::cout << (int)data << ",";
            rings_.push_back(data);
        }
        int rings_size = rings_.size();
        int index = 0;
        while (index < rings_size) {
            rings_size = rings_.size();
            if (imu_protocol_->getHeader(rings_, index)) {
                auto data_length = imu_protocol_->imuDataLength();
                if(rings_size < data_length){
                    break;
                }
                int curr_rest_size = rings_size - index;
                if (curr_rest_size >= data_length) {
                    std::vector<unsigned char> imu_datas;
                    for (int j = 0; j < data_length; ++j) {
                        imu_datas.push_back(rings_[index + j]);
                    }
                    if (imu_protocol_->xorCheck(imu_datas)) {
                        std::shared_ptr<ImuInfo> imu_info(new ImuInfo);
                        imu_protocol_->resolveData(imu_datas, imu_info);
                        check_result = imu_protocol_->imuSensorCheck();
                        if (checkGyroz(check_result)) {
                            LOG(INFO) << "IMU 所给航向不可信！";
                        } else {
                            if (dataCallback_) {
                                dataCallback_(imu_info);
                            }
                        }
//                        LOG(INFO) << "Q: " << imu_info->quaternion.w() << " " << imu_info->quaternion.x() << " " << imu_info->quaternion.y() << " " << imu_info->quaternion.z();
//                        LOG(INFO) << "yaw: " << imu_info->yaw*180/3.14 << " roll: " << imu_info->roll*180/3.14 << " pitch " << imu_info->pitch*180/3.14;

//                        origin_imu_out_ << imu_info->time_in_sensor << " " << imu_info->acc[0]/9.7887 << " " << imu_info->acc[1]/9.7887 << " " << imu_info->acc[2]/9.7887
//                                        << " " << imu_info->gyro[0]/0.01745329 << " " << imu_info->gyro[1]/0.01745329 << " " << imu_info->gyro[2]/0.01745329
//                                        << " " << imu_info->yaw*180.0/3.14 << " " << imu_info->pitch*180.0/3.14 << " " << imu_info->roll*180.0/3.14 << std::endl;
                    }
                    rings_.erase_begin(index + data_length);
                }
            }
            index++;
        }
    }

    bool checkGyroz(const ImuErrorCode& result) {
        uint8_t mask = 0x01;
        if ((result.single_error.overTemp & mask) || 
            (result.single_error.leap_gyro & 0x07) ||
            (result.single_error.constValue_gyro & mask << 2) ||
            (result.single_error.overRange_gyro & mask << 2) ||
            (result.single_error.overRange_acc) ||
            (result.single_error.nan_inf_gyro & 0x07))
            return true;    // 航向不可信 
        else
            return false;
    }

 private:
    bool init() {
        is_open_ = false;
        static uint8_t i = 0; 
        static bool port_change = false;
        while (!is_open_) {
            try {
                std::string precommand {"chmod 777 "};
                std::string command = precommand + serial_port_;
                execShell(command);
                LOG(INFO) << "begin to read:" << serial_port_ << "," << baud_rate_;
                serial_ = std::make_shared<boost::asio::serial_port>(io_service_, serial_port_);
                serial_->set_option(boost::asio::serial_port::baud_rate(baud_rate_));
                serial_->set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
                if (serial_->is_open()) {
                    imu_protocol_->startProcess(serial_);
                    usleep(10000);
                    LOG(INFO) << "successfully to read!";
                    is_open_ = true;
                    break;
                } else {
                    LOG(INFO) << "cannot open serial:" << serial_port_;
                }
            } catch (...) {
                LOG(INFO) << "cannot open the serial! please make sure the cmd right!";
                LOG(INFO) << "will sleep for 1s and try again!";
                sleep(1);
                i++;
                if(i > 2) {
                    i = 0;
                    LOG(INFO) << "serial port time out!!!";
                    return false;
                }
                continue;
            }
        }
        return is_open_;
    }

    std::string serial_port_;
    int baud_rate_;
    SerialCallbackFunc dataCallback_;

    boost::circular_buffer<unsigned char> rings_;
    std::shared_ptr<ImuProtocolInterface> imu_protocol_;
    std::shared_ptr<boost::asio::serial_port> serial_ = NULL;
    boost::asio::io_service io_service_;
    bool is_open_ = false;
    bool need_wait_ = false;
    std::ofstream origin_imu_out_;
};

}  // namespace imu

#endif  // SERIAL_READ_PGV_SERIAL_READ_HPP
