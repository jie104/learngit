/**
 * @file atom_driver.cpp
 * @author zmy (626670628@qq.com)
 * @brief  原子imu驱动
 * @version 0.1
 * @date 2021-03-26
 * 
 * 
 */

#include "atom_driver.h"
#include "core/util/time.h"

#define IMU_BAUDRATE 115200
#define IMU_CID 0x06
#define IMU_MID 0x81

namespace imu
{
    namespace driver
    {
    AtomDriver::AtomDriver(std::string device_name, const std::string imu_dev)
        : DriverBase(device_name), imu_dev_(imu_dev) {

    }

    bool AtomDriver::open()
        {
            // zmy FIXME:进入死循环
            if(receive_.open(imu_dev_.c_str(), 115200))
            {
                read_thread_ = std::async(std::launch::async, &AtomDriver::read, this);
                return true;
            }
            else
            {
                return false;
            }
        }

        bool AtomDriver::read()
        {
            // zmy TODO: 确认会等待结束还是直接跳过
            return receive_.creatReadLoop(std::bind(&AtomDriver::saberDataCallback, this, std::placeholders::_1, std::placeholders::_2,
                                                    std::placeholders::_3),
                                          IMU_CID, IMU_MID);
        }

        int AtomDriver::saberDataCallback(void *data, u16 length, uint16_t type)
        {
            switch (type)
            {
            case SESSION_NAME_RAW_ACC:
                *acc_raw_data = *((SaberData_RAW_HandleType *)data);
                raw_data_.linear_acceleration = Eigen::Vector3f(acc_raw_data->accX, acc_raw_data->accY, acc_raw_data->accZ);
                flag_[0] = true;
                break;
            case SESSION_NAME_RAW_GYRO:
                *gyro_raw_data = *((SaberData_RAW_HandleType *)data);
                raw_data_.angular_velocity = Eigen::Vector3f(acc_raw_data->gyroX, acc_raw_data->gyroY, acc_raw_data->gyroZ);
                flag_[1] = true;
                break;
            case SESSION_NAME_QUAT:
                *quat = *((SaberData_Quaternion_HandleType *)data);
                flag_[2] = true;
                break;
            case SESSION_NAME_EULER:
                *euler = *((SaberData_Euler_HandleType *)data);
                // raw_data_.euler = Eigen::Vector3f(euler->roll, euler->pitch, euler->yaw);
                raw_data_.orientation = Eigen::Quaternionf(Eigen::AngleAxisf(euler->roll, Eigen::Vector3f::UnitX()) *
                                                           Eigen::AngleAxisf(euler->pitch, Eigen::Vector3f::UnitY()) *
                                                           Eigen::AngleAxisf(euler->yaw, Eigen::Vector3f::UnitZ()))
                                            .normalized();
                flag_[3] = true;
                break;
            case SESSION_NAME_OS_TIME:
                raw_data_.header.sync_stamp = sros::core::util::get_time_in_us();
                *tick = *((SaberData_OS_Time_ms_HandleType *)data);
                raw_data_.header.stamp = static_cast<int64_t>(tick->OS_Time_ms) * 1e3; // to us
                LOG(INFO) << "new imu timestamp: " << tick->OS_Time_ms;
                flag_[4] = true;
                break;
            default:
                break;
            }
            if (flag_.all())
            {
                handleData(raw_data_);
                flag_.reset();
            }
            return 0;
        }

        void AtomDriver::close()
        {
            receive_.close();
        }

    }
}
