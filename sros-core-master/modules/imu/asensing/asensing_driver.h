/**
 * @file asensing_driver.h
 * @author zmy (626670628@qq.com)
 * @brief 导远imu驱动接口
 * @version 0.1
 * @date 2021-04-06
 * 
 * 
 */

#ifndef ODOM_IMU_DIRVERS_ASENSING_DIRVER_H
#define ODOM_IMU_DIRVERS_ASENSING_DIRVER_H

#include "../driver_base.h"
#include "../transform.hpp"
#include "3rd/asensing_imu_reader/imu_info.h"
#include "3rd/asensing_imu_reader/pgv_serial_read.hpp"
#include <Eigen/Dense>
#include <future>

namespace imu
{
    namespace driver
    {
        class AsensingDriver : public DriverBase
        {

        public:
            AsensingDriver(std::string device_name, const int type, const std::string imu_dev = "/dev/ttyUSB0");

            ~AsensingDriver() = default;
            virtual bool open() override;
            virtual void close() override;

        private:
            void onDataReceiver(const std::vector<unsigned char> datas);
            bool xorCheck(const std::vector<uint8_t> &data);
            void resolveImuData(const std::vector<unsigned char> &datas);
            template <class Type>
            void rollCopy(Type &value, const std::vector<unsigned char> &datas,
                          int &index)
            {
                mempcpy(&value, &datas[index], sizeof(value));
                index += sizeof(value);
            }

        private:
            pgv::PgvSerialRead serial_read_;
            std::future<void> read_thread_;
            boost::circular_buffer<unsigned char> rings_;
            std::string imu_dev_;
            bool first_process = true;
            Eigen::Quaterniond rotate_q;
            Eigen::Vector3d last_w;
            int imu_type_; //0:180D使用   1:250量程无yaw_incre  2:带有yaw_incre
        };

    } // namespace imu::driver
}

#endif
