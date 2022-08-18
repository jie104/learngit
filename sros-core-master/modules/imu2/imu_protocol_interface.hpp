//
// Created by lfc on 2021/11/8.
//

#ifndef IMU_DRIVER_BASE_IMU_PROTOCOL_HPP
#define IMU_DRIVER_BASE_IMU_PROTOCOL_HPP
#include <Eigen/Dense>
#include <boost/circular_buffer.hpp>
#include <chrono>
#include <termios.h>
#include <dirent.h>
#include "core/settings.h"
#include "core/circle_optimizer_set.hpp"
namespace imu {
struct ImuInfo {
    int64_t time_in_sensor = 0;
    int64_t receive_time_in_local = 0;
    double roll = 0;
    double pitch = 0;
    double yaw = 0;

    Eigen::Vector3f gyro;
    Eigen::Vector3f acc;
    Eigen::Quaternionf quaternion;
};

union ImuErrorCode {
    uint64_t whole_error;
    struct {
        uint64_t overTemp         : 1;

        uint64_t overRange_gyro   : 3;
        uint64_t overRange_acc    : 3;

        uint64_t constValue_gyro  : 3;
        uint64_t constValue_acc   : 3;

        uint64_t leap_gyro        : 3;

        uint64_t nan_inf_acc      : 3;
        uint64_t nan_inf_gyro     : 3;
    }single_error;
};

class ImuProtocolInterface {
 public:
    enum ImuType {
        None = 0,
        Record = 1,
        Asensing_Imu = 2,
        Asensing_Imu_2 = 3,
        Hipnuc_Imu = 4
    };
    const uint8_t mask = 0x01;

    ImuProtocolInterface(ImuType imu_type, float imu_frequency) : sync_time_array_(200), imu_frequency_(imu_frequency), 
                        imu_check_buffer_size_(4), imu_check_buffer_(imu_check_buffer_size_), port_character_(""){}

    virtual ~ImuProtocolInterface() {}

    /// 获取当前imu的启动指令，如果不需要开启，则返回false
    /// \param cmd_datas 指令数据
    /// \return 如果包含开启指令，则返回true，否则返回false
    virtual bool startProcess(std::shared_ptr<boost::asio::serial_port> sp) = 0;

    /// 获取当前imu的通信模式，如果不需要选择模式，则返回false
    /// \param option_datas 模式数据
    /// \return 如果包含模式，则返回true，否则返回false
    // virtual bool getOptionCmd(std::vector<uint8_t> &option_datas) = 0;

    /// 单帧imu数据长度
    /// \return 返回imu单帧数据长度
    virtual int imuDataLength() = 0;

    virtual int maxDataLength() = 0;

    /// 异或校验
    /// \param data 需要异或校验的数据
    /// \return 校验成功返回true，否则返回false
    virtual bool xorCheck(const std::vector<uint8_t> &data) = 0;

    /// 解析读取的串口数据
    /// \param datas 输入校验之后的串口数据
    /// \param imu_info 输出解析后的imu数据。所有imu数据都是标准格式，如果数据内容不够，后期可以添加
    virtual void resolveData(const std::vector<uint8_t> &datas, std::shared_ptr<ImuInfo> &imu_info) = 0;

    /// imu解析头，函数中注意判断循环数组是否越界
    /// \param rings 用于存放读取数据的循环数组
    /// \param index 当前index值
    /// \return 如果获取到头，则返回true，否则返回错误
    virtual bool getHeader(boost::circular_buffer<unsigned char> &rings, int index) = 0;

    std::string readPort() {
        std::string imu_id{};
        std::string predir{"/dev/serial/by-id/"};
        std::string suffix = port_character_; 
        DIR *dir = opendir(predir.c_str());
        if (dir == nullptr) 
            LOG(INFO) << "cannot access /dev/serial/by-id";
        else {
            dirent *name = nullptr;
            while ((name = readdir(dir)) != nullptr) {
                std::string name_s {name->d_name};
                if (name_s == "." || name_s == "..")
                    continue;
                LOG(INFO) << name->d_name;
                if (name_s.find(suffix) != std::string::npos) {
                    //closedir(dir);
                    imu_id = predir + name_s;
                    break;
                }
            }
        }
        closedir(dir);
        
        auto &s = sros::core::Settings::getInstance();
        if (imu_id.data() != nullptr)
            s.setValue<std::string>("imu.imu_dev", imu_id+";");
        else
            s.setValue<std::string>("imu.imu_dev", "\\NA;");
        return imu_id;
    }

    const ImuType imuType() { return imu_type_; }

    //！ imu单传感器级数据检查;
    /*！
        \返回值：数据正常返回true，反之返回false;
    */
    ImuErrorCode imuSensorCheck () {   
        memset(&imu_error_table_, 0, sizeof(ImuErrorCode));
        imuOverTemperatureCheck();
        if (imu_error_table_.whole_error != 0) return imu_error_table_;
        imuNanInfCheck();
        if (imu_error_table_.whole_error != 0) return imu_error_table_;
        // imuLeapCheck();
        // if (imu_error_table_.whole_error != 0) return imu_error_table_;
        // imuConstValueCheck();
        // if (imu_error_table_.whole_error != 0) return imu_error_table_;
        imuOverRange();
        return imu_error_table_;
    }

    virtual void imuNanInfCheck() {
        
        if (imu_check_buffer_.size() >= imu_check_buffer_size_) {
            for (int i = 0; i < 3 ; i++) {
                if (isnanf(imu_check_buffer_.back().acc[i]) || std::isinf(imu_check_buffer_.back().acc[i])) {
                    imu_error_table_.single_error.nan_inf_acc |= (mask << i); 
                    LOG(INFO) << "加表" << i << "出现无穷大或无意义值！！";
                }
                if (isnanf(imu_check_buffer_.back().gyro[i]) || std::isinf(imu_check_buffer_.back().gyro[i])) {
                    imu_error_table_.single_error.nan_inf_gyro |= (mask << i);
                    LOG(INFO) << "陀螺" << i << "出现无穷大或无意义值！！";
                }
            }
            if (imu_error_table_.single_error.nan_inf_gyro & 0x07)
            {
                LOG(INFO) << "因为陀螺出现无穷大或无意义值，丢弃buffer中当前拍！";
                imu_check_buffer_.pop_back();
            } 
        }
    }

    //! imu过温检查
    /*!
        \注意：若该型号imu带有温度计，需自行定义温度检查
        \返回值：默认imu温度正常,未过温，返回false;
    */
    virtual void imuOverTemperatureCheck() {return;}

    //！ imu常值检测
    /*!
        \注意：此为默认，也可在传感器驱动中，自定义实现；
        \返回值：若imu数据正常，无常值现象，返回false
    */
    virtual void imuConstValueCheck() { 
        if (imu_check_buffer_.size() >= imu_check_buffer_size_) {
            int count_acc[3] = {0};
            int count_gyro[3] = {0};
            for (int i = 0; i < imu_check_buffer_size_ - 1; i++) {
                for(int j = 0; j < 3; j++) {
                    if (std::abs(imu_check_buffer_[i+1].acc[j] - imu_check_buffer_[i].acc[j]) < 1e-9) {
                        count_acc[j]++;
                        LOG(INFO) << "加表" << j << "前后拍相差小: " << std::abs(imu_check_buffer_[i+1].acc[j] - imu_check_buffer_[i].acc[j]);
                        LOG(INFO) << "加表" << j << "数据： " << imu_check_buffer_[i+1].acc[j] << " , " << imu_check_buffer_[i].acc[j];
                    }
                        
                    if (std::abs(imu_check_buffer_[i+1].gyro[j] - imu_check_buffer_[i].gyro[j]) < 1e-9) {
                        count_gyro[j]++;
                        LOG(INFO) << "陀螺" << j << "前后拍相差小: " << std::abs(imu_check_buffer_[i+1].gyro[j] - imu_check_buffer_[i].gyro[j]);
                        LOG(INFO) << "陀螺" << j << "数据： " << imu_check_buffer_[i+1].gyro[j] << " , " << imu_check_buffer_[i].gyro[j];
                    }
                        
                }
            }
            for (int j = 0; j < 3; j++) {
                if (count_acc[j] >= imu_check_buffer_size_ - 1) {
                    LOG(INFO) << "IMU 数据错误：加表" << j << "出现常值！！";
                    imu_error_table_.single_error.constValue_acc |= (mask << j);
                }
                if (count_gyro[j] >= imu_check_buffer_size_ - 1) {
                    LOG(INFO) << "IMU 数据错误：陀螺" << j << "出现常值！！";
                    imu_error_table_.single_error.constValue_gyro |= (mask << j);
                }
            }
        }   
        return;
    }

    //！ imu超量程检测
    /*!
        \注意：每款传感器量程不同，需自定义函数实现；
        \返回值：若imu数据正常，无超量程现象，返回false，反之返回true；
    */
    virtual void imuOverRange() = 0;

    //！ imu跳变检测
    /*!
        \注意：此为默认，也可在传感器驱动中，自定义实现；
        \返回值：若imu数据正常，无数据跳变，返回false，反之返回true；
    */
    virtual void imuLeapCheck() {
        if (imu_check_buffer_.size() >= imu_check_buffer_size_) {
            LOG(INFO) << "imu buffer size :" << imu_check_buffer_.size();
            for (int i = 0; i < 3; i++) {
                float rate = std::abs(imu_check_buffer_[imu_check_buffer_size_- 2].gyro[i] - imu_check_buffer_[imu_check_buffer_size_- 3].gyro[i]) / 
                             std::abs(imu_check_buffer_[imu_check_buffer_size_- 3].gyro[i] - imu_check_buffer_[imu_check_buffer_size_- 4].gyro[i]);
                LOG(INFO) << "rate: " << rate;
                if (std::abs(imu_check_buffer_[imu_check_buffer_size_- 1].gyro[i] - 
                    imu_check_buffer_[imu_check_buffer_size_-2].gyro[i]) > (2000.0 * M_PI/180 / imu_frequency_)) {
                    LOG(INFO) << "IMU 数据错误：陀螺" << i << "数值突变！！ 角加速度： " << std::abs(imu_check_buffer_[imu_check_buffer_size_-1].gyro[i] - 
                    imu_check_buffer_[imu_check_buffer_size_-2].gyro[i]) * 180/M_PI << " 未抛弃角速度： " << 
                    imu_check_buffer_[imu_check_buffer_size_-2].gyro[i] * 180/M_PI  << " 将抛弃的加速度：" << 
                    imu_check_buffer_[imu_check_buffer_size_-1].gyro[i] * 180/M_PI;
                    imu_error_table_.single_error.leap_gyro |= (mask << i);
                    LOG(INFO) << "突变时rate: " << rate;
                }
            }
            if (imu_error_table_.single_error.leap_gyro & 0x07)
            {
                LOG(INFO) << "因为陀螺突变，丢弃buffer中突变值！";
                imu_check_buffer_.pop_back();
            } 
        }  
        return;
    }

 protected:
    template <class Type>
    void rollCopy(Type &value, const std::vector<unsigned char> &datas, int &index) {
        mempcpy(&value, &datas[index], sizeof(value));
        index += sizeof(value);
    }

    /// 获取当前传感器同步时间
    /// \param time_in_local 获取时本地时间
    /// \param time_in_sensor 该数据传感器时间
    /// \return 同步后传感器数据在本地时间
    int64_t getSensorSyncTime(const int64_t time_in_local, const int64_t time_in_sensor) {
        int64_t delta_time = time_in_local - time_in_sensor;
        sync_time_array_.push_back(delta_time);
        int64_t curr_time = sync_time_array_.getMinValue() + time_in_sensor;
        if (std::abs(curr_time - time_in_local) > 1e5) {
            LOG(INFO) << "delta time is wrong!";
            sync_time_array_.reset();
            curr_time = time_in_local;
        }
        return curr_time;
    }

    uint64_t get_time_in_ns() {
        return static_cast<uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count());
    }

    uint64_t get_time_in_us() { return get_time_in_ns() / 1000L; }

    /// 异或校验实现
    /// \param data 输入的待校验数据
    /// \param xor_byte 校验值
    /// \param length 校验数据长度
    /// \return
    bool xorCheckByLength(const std::vector<uint8_t> &data, uint8_t xor_byte, int length) {
        uint8_t check_xor = 0;
        for (int i = 0; i < length; ++i) {
            check_xor ^= data[i];
        }
        if (check_xor == xor_byte) {
            return true;
        }
        return false;
    }

    template <class Quaternion>
    void QuaternionToEuler(const Quaternion &quaternion, double &yaw, double &roll, double &pitch) {
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
            pitch = std::copysign(M_PI / 2, sinp);  // use 90 degrees if out of range
        else
            pitch = std::asin(sinp);
        // yaw (z-axis rotation)
        double siny_cosp = 2 * (w * z + x * y);
        double cosy_cosp = 1 - 2 * (y * y + z * z);
        yaw = std::atan2(siny_cosp, cosy_cosp);
    }

    int data_length_ = 0;

    ImuType imu_type_;

 protected:
    int imu_check_buffer_size_;
    boost::circular_buffer<ImuInfo> imu_check_buffer_;
    ImuErrorCode imu_error_table_;
    float imu_frequency_;
    std::string port_character_;
    
 private:
    circle::CircleOptimizerArray<int64_t> sync_time_array_;
};

}  // namespace imu

#endif  // IMU_DRIVER_BASE_IMU_PROTOCOL_HPP
