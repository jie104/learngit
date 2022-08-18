/**
 * @file lins_driver.h
 * @author zmy (626670628@qq.com)
 * @brief lins IMU 驱动
 * @version 0.1
 * @date 2021-03-29
 * 
 * 
 */

#ifndef ODOM_IMU_DIRVERS_LINS_DIRVER_H
#define ODOM_IMU_DIRVERS_LINS_DIRVER_H

#include "../driver_base.h"
#include "3rd/lins_imu_reader/sr_ls_imu.h"
#include <bitset>
#include <functional>

namespace imu
{
    namespace driver
    {
        class LinsDriver : public DriverBase
        {
        public:
            LinsDriver(std::string device_name, const std::string imu_dev = "/dev/ttyUSB0");;
            ~LinsDriver() = default;

            bool open() override;
            void close() override;

        private:
            int32_t onDataReceive(lsimu_data_t *data);

        private:
            lsimu_data_t imu_data_ = {0};
            ImuData raw_data_;
            const std::string imu_dev_;
        };
    }
}

#endif