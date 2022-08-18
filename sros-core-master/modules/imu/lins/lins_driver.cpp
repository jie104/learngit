/**
 * @file lins_driver.cpp
 * @author zmy (626670628@qq.com)
 * @brief lins IMU 驱动 实现
 * @version 0.1
 * @date 2021-03-29
 * 
 * 
 */
#include "lins_driver.h"
#include "core/util/time.h"
#include <Eigen/Geometry>

namespace imu
{
    namespace driver
    {
        static std::function<int32_t(lsimu_data_t *data)> interface_function;

        static int32_t on_data_receive(lsimu_data_t *data)
        {
            if (interface_function)
            {
                return interface_function(data);
            }
            return 0;
        }

        LinsDriver::LinsDriver(std::string device_name, const std::string imu_dev): DriverBase(device_name),imu_dev_(imu_dev)
        {
        }

        bool LinsDriver::open()
        {
            memset(&imu_data_, 0, sizeof(imu_data_));
            interface_function = std::bind(&LinsDriver::onDataReceive, this, std::placeholders::_1);
            auto ret = imu_open(imu_dev_.c_str(), on_data_receive);
            return ret == 0;
        }

        int32_t LinsDriver::onDataReceive(lsimu_data_t *data)
        {
            raw_data_.header.sync_stamp = sros::core::util::get_time_in_us();
            memcpy(&imu_data_, data, sizeof(lsimu_data_t));
            raw_data_.linear_acceleration = Eigen::Vector3f(imu_data_.Ax, imu_data_.Ay, imu_data_.Az);
            raw_data_.angular_velocity = Eigen::Vector3f(imu_data_.Gx, imu_data_.Gy, imu_data_.Gz);

            raw_data_.orientation = Eigen::Quaternionf(Eigen::AngleAxisf(imu_data_.Roll, Eigen::Vector3f::UnitX()) *
                                                       Eigen::AngleAxisf(imu_data_.Pitch, Eigen::Vector3f::UnitY()) *
                                                       Eigen::AngleAxisf(imu_data_.Yaw, Eigen::Vector3f::UnitZ()))
                                        .normalized();
            raw_data_.header.stamp = static_cast<int64_t>(imu_data_.raw_timestamp) * 1e3; // to us
            // LOG(INFO) << "new imu timestamp: " << raw_data_.timestamp;
            handleData(raw_data_);
            return 0;
        }

        void LinsDriver::close()
        {
            imu_close();
        }

    } // namespace imu::driver
}
