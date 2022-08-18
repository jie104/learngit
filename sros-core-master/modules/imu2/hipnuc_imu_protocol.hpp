//
// Created by huajie on 2021/11/17.
//

#ifndef IMU_DRIVER_HIPNUC_IMU_PROTOCOL_HPP
#define IMU_DRIVER_HIPNUC_IMU_PROTOCOL_HPP
#include "imu_protocol_interface.hpp"

namespace imu {
//所有通信协议均继承于协议接口基类（ImuProtocolInterface），该类中所有纯虚函数都需要在子类中实现
class HipnucImuProtocol : public ImuProtocolInterface {
 public:
    HipnucImuProtocol(ImuType hipnuc_type, float imu_frequency) : ImuProtocolInterface(hipnuc_type, imu_frequency) { 
        data_length_ = 6 + 76; 
        cal_param_num_ = 3;
        cal_param_ = {};
        port_character_ = "usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller";
    }

    virtual ~HipnucImuProtocol() {}

    struct HipnucImuInfo {
        uint8_t id;                                /* user defined ID       */
        float acc[3]; /* acceleration          */  //顺序为：XYZ
        float gyr[3]; /* angular velocity      */  //顺序为：XYZ
        float mag[3]; /* magnetic field        */  //顺序为：XYZ
        float eul[3];                              /* attitude: eular angle */
        //节点欧拉角 顺序为：横滚角(Roll)，俯仰角(Pitch)，航向角(Yaw)
        float quat[4];  /* attitude: quaternion  */
                        //节点四元数集合,顺序为WXYZ
        float pressure; /* air pressure          */
        uint32_t timestamp;
    };
    
    virtual bool startProcess(std::shared_ptr<boost::asio::serial_port> serial) { 
        cal_param_.clear();
        calParamRead(serial, cal_param_num_, cal_param_);
        if (paramFault(cal_param_num_, cal_param_)) {
            cal_param_.clear();
            LOG(INFO) << "故障时参数个数： " << cal_param_.size();
            for (int i=0; i<cal_param_num_; i++) {
                cal_param_.push_back(1.0);
            }
        }
        return true;
    }
    
    void calParamRead(std::shared_ptr<boost::asio::serial_port> sp, const int pnum, std::vector<float>& param) {

        if (pnum <= 0 || pnum > 16) {
            LOG(WARNING) << "IMU 所要读取参数数量错误！！";
            return;
        }

        sp->write_some(boost::asio::buffer("AT+EOUT=0\r\n"));
        sleep(2.0);  // 大于50ms 小于20Hz 
        tcflush(sp->lowest_layer().native_handle(), TCIOFLUSH);

        std::string command = "AT+CUSTRD=0,";
        command += std::to_string(pnum);
        command += "\r\n";
        sp->write_some(boost::asio::buffer(command));
        tcflush(sp->lowest_layer().native_handle(), TCOFLUSH);
        sleep(0.1);  // 大于50ms 小于20Hz

        sp->write_some(boost::asio::buffer("AT+RST\r\n"));
        tcflush(sp->lowest_layer().native_handle(), TCOFLUSH);
        sleep(0.1);

        paramParse(sp, pnum, param);
        sleep(0.1); 

        return;
    }

    bool paramFault(const int pnum, std::vector<float>& param){
        int num_parse = param.size();
        if (num_parse != pnum) {
            LOG(WARNING) << "IMU 读取标定参数有误！！！";
            LOG(WARNING) << "IMU 预设参数个数： " << pnum << " 个，实际解析出： " << num_parse << "个！！";
            return true;
        }

        for(int i=0; i <pnum; i++) {
            if (param[i] > 1.2 || param[i] < 0.8) {              // 当前只打算使用陀螺刻度系数
                LOG(WARNING) << "IMU未标定或标定有误！！！";
                return true;
            }
        }
        return false;
    }

    void paramParse(std::shared_ptr<boost::asio::serial_port> sp, const int pnum, std::vector<float>& param) {
        const uint8_t times_limit = 3;
        uint8_t times = 0;
        char buff[200] = {0}; 
        size_t num = 0;

        do
        {
            num = sp->read_some(boost::asio::buffer(buff, pnum*12));
            times++;
        } while (num != pnum*12 && times < times_limit);

        LOG(INFO) << "buffer打印：\n" << buff;

        if (num == pnum*12)
        {
            for (int i=0; i<pnum; i++)
            {
                if(buff[i*12]=='0' && buff[i*12+1]=='x')
                {
                    LOG(INFO) << "进入解码！！！" << i;
                    int data_dec = 0;
                    int data_temp = 0;
                    int order = 1;
                    float result = 0.0;
                    for(int j=9; j>1; j--)
                    {
                        data_temp = buff[i*12+j]>=65 ? buff[i*12+j]-'A'+10 : buff[i*12+j] - '0';
                        data_dec += data_temp * order;
                        order *= 16;
                    }

                    bool sign = int(data_dec * 1e-7);
                    int integ = int(data_dec * 1e-5) % 100;
                    int frac = data_dec % 100000;
                    result = integ + frac*1e-5;
                    result = sign? -result : result;
                    LOG(INFO) << "参数结果 " << i << ": " << result;
                    param.push_back(result);
                }
                else
                {
                    LOG(WARNING) << "IMU Parameters error!!!";
                    return;
                }
            }
        }
        return;
    }

    virtual int imuDataLength() { return data_length_; }

    virtual int maxDataLength() { return data_length_; }

    virtual bool xorCheck(const std::vector<uint8_t>& data) {
        // vector转数组
        const uint8_t* buffer = data.data();
        // 16位CRC 校验
        uint16_t crc = 0;
        crc16Update(&crc, buffer, 4);
        crc16Update(&crc, buffer + 6, 76);

        uint16_t u;
        memcpy(&u, buffer + 4, 2);

        if (crc != u) {
            return false;
        }
        return true;
    }

    virtual void resolveData(const std::vector<uint8_t>& datas, std::shared_ptr<ImuInfo>& imu_info) {
        uint8_t trash;
        HipnucImuInfo temp_imu_info;
        static HipnucImuInfo pre_imu_info = HipnucImuInfo {};
        static bool first_flag = true;
        if (!imu_info) {
            imu_info.reset(new ImuInfo);
        }
        int index = 6;
        // HipnucImuInfo hipnuc_imu_info{};
        hipnuc_imu_info_ = {};
        rollCopy(trash, datas, index);
        rollCopy(hipnuc_imu_info_.id, datas, index);
        rollCopy(trash, datas, index);
        rollCopy(trash, datas, index);
        rollCopy(hipnuc_imu_info_.pressure, datas, index);
        rollCopy(hipnuc_imu_info_.timestamp, datas, index);
        rollCopy(hipnuc_imu_info_.acc[0], datas, index);
        rollCopy(hipnuc_imu_info_.acc[1], datas, index);
        rollCopy(hipnuc_imu_info_.acc[2], datas, index);
        rollCopy(hipnuc_imu_info_.gyr[0], datas, index);
        rollCopy(hipnuc_imu_info_.gyr[1], datas, index);
        rollCopy(hipnuc_imu_info_.gyr[2], datas, index);
        rollCopy(hipnuc_imu_info_.mag[0], datas, index);
        rollCopy(hipnuc_imu_info_.mag[1], datas, index);
        rollCopy(hipnuc_imu_info_.mag[2], datas, index);
        rollCopy(hipnuc_imu_info_.eul[0], datas, index);
        rollCopy(hipnuc_imu_info_.eul[1], datas, index);
        rollCopy(hipnuc_imu_info_.eul[2], datas, index);
        rollCopy(hipnuc_imu_info_.quat[0], datas, index);
        rollCopy(hipnuc_imu_info_.quat[1], datas, index);
        rollCopy(hipnuc_imu_info_.quat[2], datas, index);
        rollCopy(hipnuc_imu_info_.quat[3], datas, index);
        imu_info->time_in_sensor = hipnuc_imu_info_.timestamp;
        imu_info->time_in_sensor *= 1e3;//转换成us
        imu_info->receive_time_in_local = getSensorSyncTime(get_time_in_us(), imu_info->time_in_sensor);

        if (imu_frequency_ == 200) {
            if (first_flag == true) {
                pre_imu_info = hipnuc_imu_info_;
                first_flag = false;
            } else {
                temp_imu_info = hipnuc_imu_info_;
                // hipnuc_imu_info_.gyr[0] = 0.5 * pre_imu_info.gyr[0] + 0.5 * hipnuc_imu_info_.gyr[0];
                // hipnuc_imu_info_.gyr[1] = 0.5 * pre_imu_info.gyr[1] + 0.5 * hipnuc_imu_info_.gyr[1];
                // hipnuc_imu_info_.gyr[2] = 0.5 * pre_imu_info.gyr[2] + 0.5 * hipnuc_imu_info_.gyr[2];

                // hipnuc_imu_info_.acc[0] = 0.5 * pre_imu_info.acc[0] + 0.5 * hipnuc_imu_info_.acc[0];
                // hipnuc_imu_info_.acc[1] = 0.5 * pre_imu_info.acc[1] + 0.5 * hipnuc_imu_info_.acc[1];
                // hipnuc_imu_info_.acc[2] = 0.5 * pre_imu_info.acc[2] + 0.5 * hipnuc_imu_info_.acc[2];

                temp_imu_info.gyr[0] = 0.8 * pre_imu_info.gyr[0] + 0.2 * hipnuc_imu_info_.gyr[0];
                temp_imu_info.gyr[1] = 0.8 * pre_imu_info.gyr[1] + 0.2 * hipnuc_imu_info_.gyr[1];
                temp_imu_info.gyr[2] = 0.8 * pre_imu_info.gyr[2] + 0.2 * hipnuc_imu_info_.gyr[2];

                temp_imu_info.acc[0] = 0.8 * pre_imu_info.acc[0] + 0.2 * hipnuc_imu_info_.acc[0];
                temp_imu_info.acc[1] = 0.8 * pre_imu_info.acc[1] + 0.2 * hipnuc_imu_info_.acc[1];
                temp_imu_info.acc[2] = 0.8 * pre_imu_info.acc[2] + 0.2 * hipnuc_imu_info_.acc[2];

                pre_imu_info = hipnuc_imu_info_;
                hipnuc_imu_info_ = temp_imu_info;
            }
        }

        convertToImuInfo(hipnuc_imu_info_, imu_info);
        imu_check_buffer_.push_back(*imu_info);
        if (imu_check_buffer_.size() > imu_check_buffer_size_)
            imu_check_buffer_.erase_begin(1);
    }

    virtual bool getHeader(boost::circular_buffer<unsigned char>& rings, int index) {
        if (rings.size() >= index + 2) {
            if (rings[index] == 0x5a) {
                if (rings[index + 1] == 0xa5) {
                    return true;
                }
            }
        }
        return false;
    }

    virtual void imuOverRange () {
        const float acc_range = 7.2; // CH110 量程8g，取90%, g 
        const float gyro_range = 450.0; // CH110 量程500deg，取90%, deg/s
        const uint8_t mask = 0x01;

        for (int i = 0; i < 3; i++) {
            if (abs(hipnuc_imu_info_.acc[i]) > acc_range) {
                LOG(INFO) << "IMU 数据错误：加表" << i << "出现超量程！！";
                imu_error_table_.single_error.overRange_acc |= (mask << i);
            }
            if (abs(hipnuc_imu_info_.gyr[i]) > gyro_range) {
                LOG(INFO) << "IMU 数据错误：陀螺" << i << "出现超量程！！";
                imu_error_table_.single_error.overRange_gyro |= (mask << i);
            }
        }
        return;
            
    }

 private:
    ///
    /// \param currect_crc previous crc value, set 0 if it's first section
    /// \param src source stream data
    /// \param len lengthcrc16Update
    void crc16Update(uint16_t* currect_crc, const uint8_t* src, uint32_t len) const {
        uint32_t crc = *currect_crc;
        uint32_t j;
        for (j = 0; j < len; ++j) {
            uint32_t i;
            uint32_t byte = src[j];
            crc ^= byte << 8;
            for (i = 0; i < 8; ++i) {
                uint32_t temp = crc << 1;
                if (crc & 0x8000) {
                    temp ^= 0x1021;
                }
                crc = temp;
            }
        }
        *currect_crc = crc;
    }

    void convertToImuInfo(const HipnucImuInfo& hipnuc_info, std::shared_ptr<ImuInfo>& imu_info) {
        const double gra_acc = 9.7887;
        const double deg_to_rad = 0.01745329;
        imu_info->quaternion.w() = hipnuc_info.quat[0];
        imu_info->quaternion.x() = hipnuc_info.quat[1];
        imu_info->quaternion.y() = hipnuc_info.quat[2];
        imu_info->quaternion.z() = hipnuc_info.quat[3];
        QuaternionToEuler(imu_info->quaternion, imu_info->yaw, imu_info->roll, imu_info->pitch);

        if (cal_param_.empty()) {
            for (int i=0; i<cal_param_num_; i++) {
                cal_param_.push_back(1.0);
            }
        }
        for (int i = 0; i<3; i++) {
            imu_info->gyro[i] = hipnuc_info.gyr[i] * deg_to_rad * cal_param_[i];
            imu_info->acc[i] = hipnuc_info.acc[i] * gra_acc;
        }
    }

    int data_length_;
    HipnucImuInfo hipnuc_imu_info_;
    std::vector<float> cal_param_;
    int cal_param_num_;
};

}  // namespace imu

#endif  // IMU_DRIVER_HIPNUC_IMU_PROTOCOL_HPP
