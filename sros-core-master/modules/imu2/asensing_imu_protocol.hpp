//
// Created by lfc on 2021/11/8.
//

#ifndef IMU_DRIVER_ASENSING_IMU_PROTOCOL_HPP
#define IMU_DRIVER_ASENSING_IMU_PROTOCOL_HPP
#include "imu_protocol_interface.hpp"

namespace imu{
//所有通信协议均继承于协议接口基类（ImuProtocolInterface），该类中所有纯虚函数都需要在子类中实现
class AsensingImuProtocol: public ImuProtocolInterface {
 public:
    AsensingImuProtocol(ImuType asensing_type, float imu_frequency):ImuProtocolInterface(asensing_type, imu_frequency){
        if (asensing_type == Asensing_Imu_2) {
            data_length_ = 58;
        }else{
            data_length_ = 54;
        }
    }

    virtual ~AsensingImuProtocol(){

    }

    struct AsensingImuInfo {
        int16_t  qw, qx, qy, qz, tmpt;
        uint16_t gx_n, gy_n, gz_n, ax_n, ay_n, az_n;
        uint32_t time;
        float gx_f, gy_f, gz_f, ax_f, ay_f, az_f, yaw_inc;
    };

    virtual bool startProcess(std::shared_ptr<boost::asio::serial_port> serial) {
        std::vector<uint8_t> cmd_datas;
        cmd_datas.clear();
        cmd_datas.push_back(0xBD);
        cmd_datas.push_back(0xDB);
//        cmd_datas.push_back(0x54);
//        cmd_datas.push_back(0x00);
        cmd_datas.push_back(0x52);
        cmd_datas.push_back(0x20);
        serial->write_some(boost::asio::buffer(cmd_datas));

        cmd_datas.clear();
        cmd_datas.push_back(0xBD);
        cmd_datas.push_back(0xDB);
        cmd_datas.push_back(0x04);
        serial->write_some(boost::asio::buffer(cmd_datas));
        return true;
    }

    // virtual bool getOptionCmd(std::vector<uint8_t>& option_datas){
    //     option_datas.clear();
    //     option_datas.push_back(0xBD);
    //     option_datas.push_back(0xDB);
    //     option_datas.push_back(0x04);
    //     return true;
    // }

    virtual int imuDataLength(){ return data_length_; }

    virtual int maxDataLength(){ return data_length_; }

    virtual bool xorCheck(const std::vector<uint8_t> &data){
        if (data.size() == data_length_) {
            int length = data.size() - 1;
            uint8_t xor_byte = data[length];
            return xorCheckByLength(data, xor_byte, length);
        }
        return false;
    }

    virtual void resolveData(const std::vector<uint8_t>& datas,std::shared_ptr<ImuInfo>& imu_info){
        if (!imu_info) {
            imu_info.reset(new ImuInfo);
        }
        int index = 3;
        // AsensingImuInfo asensing_imu_info{};
        asensing_imu_info_ = {};
        rollCopy(asensing_imu_info_.qx, datas, index);
        rollCopy(asensing_imu_info_.qy, datas, index);
        rollCopy(asensing_imu_info_.qz, datas, index);
        rollCopy(asensing_imu_info_.qw, datas, index);
        // 有yaw_inc的imu使用
        if (imu_type_ == Asensing_Imu_2)
            rollCopy(asensing_imu_info_.yaw_inc, datas, index);
        rollCopy(asensing_imu_info_.gx_f, datas, index);
        rollCopy(asensing_imu_info_.gy_f, datas, index);
        rollCopy(asensing_imu_info_.gz_f, datas, index);
        rollCopy(asensing_imu_info_.ax_f, datas, index);
        rollCopy(asensing_imu_info_.ay_f, datas, index);
        rollCopy(asensing_imu_info_.az_f, datas, index);
        rollCopy(asensing_imu_info_.gx_n, datas, index);
        rollCopy(asensing_imu_info_.gy_n, datas, index);
        rollCopy(asensing_imu_info_.gz_n, datas, index);
        rollCopy(asensing_imu_info_.ax_n, datas, index);
        rollCopy(asensing_imu_info_.ay_n, datas, index);
        rollCopy(asensing_imu_info_.az_n, datas, index);
        rollCopy(asensing_imu_info_.tmpt, datas, index);
        rollCopy(asensing_imu_info_.time, datas, index);
        imu_info->time_in_sensor = asensing_imu_info_.time;
        imu_info->time_in_sensor *= 1e3;//转换成us
        imu_info->receive_time_in_local = getSensorSyncTime(get_time_in_us(), imu_info->time_in_sensor);
        convertToImuInfo(asensing_imu_info_, imu_info);
        imu_check_buffer_.push_back(*imu_info);
        if (imu_check_buffer_.size() > imu_check_buffer_size_)
            imu_check_buffer_.erase_begin(1);
    }

    virtual bool getHeader(boost::circular_buffer<unsigned char> &rings,int index){
        if (rings.size() >= index + 2) {
            if(rings[index] == 0xbd){
                if (rings[index + 1] == 0xdb) {
                    if (rings[index + 2] == 0x04) {
                        return true;
                    }
                }
            }
        }
        return false;
    }

    virtual void imuOverRange () {
        const float acc_range = 3.6 * 9.7887; // Asensing511 量程4g，取90%, m/s^2
        const float gyro_range = 225.0; // Asensing511 量程250deg，取90%, deg/s

        if (abs(asensing_imu_info_.ax_f) > acc_range) {
            LOG(INFO) << "IMU 数据错误：加表1出现超量程！！";
            imu_error_table_.single_error.overRange_acc |= 0x01;
        }
        if (abs(asensing_imu_info_.ay_f) > acc_range) {
            LOG(INFO) << "IMU 数据错误：加表2出现超量程！！";
            imu_error_table_.single_error.overRange_acc |= 0x02;
        }
        if (abs(asensing_imu_info_.az_f) > acc_range) {
            LOG(INFO) << "IMU 数据错误：加表3出现超量程！！";
            imu_error_table_.single_error.overRange_acc |= 0x04;
        }

        if (abs(asensing_imu_info_.gx_f) > gyro_range) {
            LOG(INFO) << "IMU 数据错误：陀螺1出现超量程acc_range！！";
            imu_error_table_.single_error.overRange_gyro |= 0x01;
        }
        if (abs(asensing_imu_info_.gy_f) > gyro_range) {
            LOG(INFO) << "IMU 数据错误：陀螺2出现超量程！！";
            imu_error_table_.single_error.overRange_gyro |= 0x02;
        }
        if (abs(asensing_imu_info_.gz_f) > gyro_range) {
            LOG(INFO) << "IMU 数据错误：陀螺3出现超量程！！";
            imu_error_table_.single_error.overRange_gyro |= 0x04;
        }
        return;
    }

 private:
    void convertToImuInfo(const AsensingImuInfo& asensing_info,std::shared_ptr<ImuInfo>& imu_info){
        const double angle_scale = M_PI / 32768.0;
        const double gyro_scale = 250 * M_PI / 180.0 / 32767.0;
        const double float_gyro_scale = M_PI / 180.0;
        const double acc_scale = 4 * 9.7887 / 32767.0;
        const double quaternion_scale = 1.0 / 32767.0;
        imu_info->quaternion.w() = asensing_info.qw * quaternion_scale;
        imu_info->quaternion.x() = asensing_info.qx * quaternion_scale;
        imu_info->quaternion.y() = asensing_info.qy * quaternion_scale;
        imu_info->quaternion.z() = asensing_info.qz * quaternion_scale;
        QuaternionToEuler(imu_info->quaternion, imu_info->yaw, imu_info->roll, imu_info->pitch);
        imu_info->gyro[0] = asensing_info.gx_f * float_gyro_scale;
        imu_info->gyro[1] = asensing_info.gy_f * float_gyro_scale;
        imu_info->gyro[2] = asensing_info.gz_f * float_gyro_scale;
        imu_info->acc[0] = asensing_info.ax_f;
        imu_info->acc[1] = asensing_info.ay_f;
        imu_info->acc[2] = asensing_info.az_f;
    }

    int data_length_;
    AsensingImuInfo asensing_imu_info_;
};

}

#endif  // IMU_DRIVER_ASENSING_IMU_PROTOCOL_HPP
