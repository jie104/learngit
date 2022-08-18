/**
 * @file imu_info.h
 * @author zmy (626670628@qq.com)
 * @brief 
 * @version 0.1
 * @date 2021-04-06
 * 
 * 
 */

#include "../../../transform.hpp"

namespace imu
{
    namespace driver
    {
        struct ImuInfo
        {
            int16_t roll, pitch, yaw, db_0, db_1, gx, gy, gz, qw, qx, qy,
                qz, tmpt;
            uint16_t gx_n, gy_n, gz_n, ax_n, ay_n, az_n;
            uint32_t time;
            float gx_f, gy_f, gz_f, ax_f, ay_f, az_f,yaw_inc;
            void convertToDoubleValue()
            {
                double angle_scale = M_PI / 32768.0;
                double gyro_scale = 250 * M_PI / 180.0 / 32767.0;
                double float_gyro_scale = M_PI / 180.0;
                double acc_scale = 4 * 9.7887 / 32767.0;
                double quaternion_scale = 1.0 / 32767.0;
                qw_d = qw * quaternion_scale;
                qx_d = qx * quaternion_scale;
                qy_d = qy * quaternion_scale;
                qz_d = qz * quaternion_scale;
                tf::QuaternionToEuler(quaternion(), yaw_d, roll_d, pitch_d);
                gx_d = gx_f * float_gyro_scale;
                gy_d = gy_f * float_gyro_scale;
                gz_d = gz_f * float_gyro_scale;
                ax_d = ax_f;
                ay_d = ay_f;
                az_d = az_f;
            }
            const Eigen::Quaterniond quaternion()
            {
                Eigen::Quaterniond quat;
                quat.x() = qx_d;
                quat.y() = qy_d;
                quat.z() = qz_d;
                quat.w() = qw_d;
                return quat;
            }

            double roll_d, pitch_d, yaw_d, gx_d, gy_d, gz_d, ax_d, ay_d, az_d, qw_d, qx_d, qy_d, qz_d;
        };
    } // namespace imu::driver
}
