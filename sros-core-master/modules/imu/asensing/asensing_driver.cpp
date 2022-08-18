/**
 * @file asensing_driver.cpp
 * @author zmy (626670628@qq.com)
 * @brief 导远imu驱动实现
 * @version 0.1
 * @date 2021-04-06
 * 
 * 
 */

#include "asensing_driver.h"
#include <boost/filesystem.hpp>
#include <stdio.h>

#include "core/util/time.h"

template <class Quaternion>
static void QuaternionToEuler(const Quaternion &quaternion, double &yaw, double &roll, double &pitch)
{
    auto w = quaternion.w();
    auto x = quaternion.x();
    auto y = quaternion.y();
    auto z = quaternion.z();
    //    yaw = atan2(2*(w * z + x * y), 1 - 2 * (z * z + x * x));
    //    roll = asin(2*(w * x - y * z));
    //    pitch = atan2(w * y + z * x, 1 - 2 * (x * x + y * y));
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (w * y - z * x);
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

namespace imu
{
    namespace driver
    {
        AsensingDriver::AsensingDriver(std::string device_name, const int type, const std::string imu_dev)
            : DriverBase(device_name), rings_(1024), imu_dev_(imu_dev), imu_type_(type), serial_read_(type)
        {
        }

        bool AsensingDriver::open()
        {
            if (serial_read_.open(imu_dev_, 230400))
            {
                serial_read_.setDataReceiveCallback([this](const std::vector<unsigned char> datas)
                                                    { onDataReceiver(datas); });
                read_thread_ = std::async(std::launch::async, [this]()
                                          { serial_read_.readLoop(); });
                setStateOK();
                return true;
            }
            else
            {
                setStateOpenFailed();
                return false;
            }
        }

        void AsensingDriver::close()
        {
            setStateOff();
            serial_read_.close();
        }

        void AsensingDriver::onDataReceiver(const std::vector<unsigned char> datas)
        {
            for (auto &data : datas)
            {
                rings_.push_back(data);
            }

            int imu_data_size = 54;
            if (imu_type_ == 2)
                imu_data_size = 58; // 有yaw_inc的imu使用

            while (rings_.size() >= imu_data_size)
            {
                std::vector<unsigned char> imu_datas;
                int search_size = rings_.size() - 2;
                int rings_size = search_size + 2;
                int i = 0;
                for (i = 0; i < search_size; ++i)
                {
                    //zmy TODO:确定不同型号是否更改校验值
                    if (rings_[i] == 0xbd)
                    {
                        if (rings_[i + 1] == 0xdb)
                        {
                            if (rings_[i + 2] == 0x04)
                            {
                                int curr_rest_size = rings_size - i;
                                if (curr_rest_size >= imu_data_size)
                                {
                                    for (int j = 0; j < imu_data_size; ++j)
                                    {
                                        imu_datas.push_back(rings_[i + j]);
                                    }
                                    resolveImuData(imu_datas);
                                    rings_.erase_begin(i + imu_data_size);
                                    break;
                                }
                            }
                        }
                    }
                }
                if (i == search_size)
                {
                    break;
                }
            }
        }

        bool AsensingDriver::xorCheck(const std::vector<uint8_t> &data)
        {
            int length = data.size() - 1;
            uint8_t xor_byte = data[length];
            uint8_t check_xor = 0;
            for (int i = 0; i < length; ++i)
            {
                check_xor ^= data[i];
            }
            if (check_xor == xor_byte)
            {
                return true;
            }
            LOG(INFO) << "the length is:" << length << "," << (int)check_xor << ","
                      << (int)xor_byte;
            LOG(INFO) << "the xor check is wrong! please check imu type";
            return false;
        }

        void AsensingDriver::resolveImuData(const std::vector<unsigned char> &datas)
        {

            if (xorCheck(datas))
            {
                int index = 3;
                ImuInfo imu_info{};
                rollCopy(imu_info.qx, datas, index);
                rollCopy(imu_info.qy, datas, index);
                rollCopy(imu_info.qz, datas, index);
                rollCopy(imu_info.qw, datas, index);
                // 有yaw_inc的imu使用
                if (imu_type_ == 2)
                    rollCopy(imu_info.yaw_inc, datas, index);

                rollCopy(imu_info.gx_f, datas, index);
                rollCopy(imu_info.gy_f, datas, index);
                rollCopy(imu_info.gz_f, datas, index);
                rollCopy(imu_info.ax_f, datas, index);
                rollCopy(imu_info.ay_f, datas, index);
                rollCopy(imu_info.az_f, datas, index);
                rollCopy(imu_info.gx_n, datas, index);
                rollCopy(imu_info.gy_n, datas, index);
                rollCopy(imu_info.gz_n, datas, index);
                rollCopy(imu_info.ax_n, datas, index);
                rollCopy(imu_info.ay_n, datas, index);
                rollCopy(imu_info.az_n, datas, index);
                rollCopy(imu_info.tmpt, datas, index);
                rollCopy(imu_info.time, datas, index);
                imu_info.convertToDoubleValue();

                ImuData imu_data;

                imu_data.header.frame_id = "/imu";
                imu_data.header.stamp = static_cast<uint64_t>(imu_info.time) * 1000; //us
                imu_data.header.sync_stamp = sros::core::util::get_time_in_us();

                Eigen::Quaterniond rotate_q = imu_info.quaternion();
                imu_data.orientation.w() = rotate_q.w();
                imu_data.orientation.x() = rotate_q.x();
                imu_data.orientation.y() = rotate_q.y();
                imu_data.orientation.z() = rotate_q.z();

                imu_data.angular_velocity.x() = imu_info.gx_d;
                imu_data.angular_velocity.y() = imu_info.gy_d;
                imu_data.angular_velocity.z() = imu_info.gz_d;

                imu_data.linear_acceleration.x() = imu_info.ax_d;
                imu_data.linear_acceleration.y() = imu_info.ay_d;
                imu_data.linear_acceleration.z() = imu_info.az_d;

                handleData(imu_data);
                keepOK(); //如果调用keepAlive，频率太频繁，可以用计数的方法降低频率。
            }
        }
    } // namespace imu::driver
}
