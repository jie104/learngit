/**
 * @file atom_driver.h
 * @author zmy (626670628@qq.com)
 * @brief 子imu驱动
 * @version 0.1
 * @date 2021-03-26
 * 
 * 
 */

#ifndef ODOM_IMU_DIRVERS_ATOM_DIRVER_H
#define ODOM_IMU_DIRVERS_ATOM_DIRVER_H

#include "../driver_base.h"
#include "3rd/atom_imu_reader/imu_receive.hpp"
#include <bitset>
#include <future>
#include <string>

namespace imu
{
    namespace driver
    {
        class AtomDriver : public DriverBase
        {
        public:
           AtomDriver(std::string device_name, const std::string imu_dev = "/dev/ttyUSB0");
           virtual ~AtomDriver() = default;

            bool open() override;
            void close() override;

        private:
            bool read();
            int saberDataCallback(void *data, u16 length, uint16_t type);

        private:
            ImuReceive receive_;
            std::string imu_dev_;
            std::future<bool> read_thread_;

            SaberData_RAW_HandleType *acc_raw_data;
            SaberData_RAW_HandleType *gyro_raw_data;
            SaberData_Quaternion_HandleType *quat;
            SaberData_Euler_HandleType *euler;
            SaberData_OS_Time_ms_HandleType *tick;
            std::bitset<5> flag_;
            ImuData raw_data_;
        };
    }
}

#endif