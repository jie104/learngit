/**
 * @file device.h
 *
 * @author lhx
 * @date 2018年1月31日
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef CORE_DEVICE_DEVICE_H_
#define CORE_DEVICE_DEVICE_H_

#include <atomic>
#include <boost/thread/shared_mutex.hpp>
#include <memory>
#include <string>
#include "core/util/time.h"
#include "device_id.h"
#include <mutex>

namespace sros {
namespace device {

enum DeviceState {
    DEVICE_NONE = 0x00,  // 状态不可用

    // 0x01 ~ 0x0F

    DEVICE_OK = 0x01,  // 工作正常

    // 0x10 ~ 0x1F

    DEVICE_INITIALIZING = 0x10,  // 初始化中

    // 0x20 ~ 0x2F

    DEVICE_OFF = 0x40,  // 设备正常关闭

    // 0x30 ~ 0x7F
    // RESERVED

    // 0x80 ~ 0xFF

    DEVICE_ERROR = 0x80,              // 其他ERROR
    DEVICE_ERROR_OPEN_FAILED = 0x81,  // 设备打开失败
    DEVICE_ERROR_TIMEOUT = 0x82,      // 通信超时
    DEVICE_ERROR_INITIAL = 0x93,      // 初始化出错
    DEVICE_OPENING = 0x83,
};

// 设备通信接口类型，即与vc300/vc400如何连接的
enum DeviceCommInterfaceType {
    DEVICE_COMM_INTERFACE_TYPE_NONE = 0x00,

    DEVICE_COMM_INTERFACE_TYPE_ETH_1 = 0x11,
    DEVICE_COMM_INTERFACE_TYPE_ETH_2 = 0x12,

    DEVICE_COMM_INTERFACE_TYPE_CAN_1 = 0x21,
    DEVICE_COMM_INTERFACE_TYPE_CAN_2 = 0x22,

    DEVICE_COMM_INTERFACE_TYPE_RS232_1 = 0x31,
    DEVICE_COMM_INTERFACE_TYPE_RS232_2 = 0x32,
    DEVICE_COMM_INTERFACE_TYPE_RS232_3 = 0x33,
    DEVICE_COMM_INTERFACE_TYPE_RS232_4 = 0x34,
    DEVICE_COMM_INTERFACE_TYPE_RS232_5 = 0x35,

    DEVICE_COMM_INTERFACE_TYPE_RS485_1 = 0x41,
    DEVICE_COMM_INTERFACE_TYPE_RS485_2 = 0x42,

    DEVICE_COMM_INTERFACE_TYPE_USB_0 = 0x50,
    DEVICE_COMM_INTERFACE_TYPE_USB_1 = 0x51,
    DEVICE_COMM_INTERFACE_TYPE_USB_2 = 0x52,
};

// 设备类型
enum DeviceMountType {
    DEVICE_MOUNT_NONE = 0x00,
    DEVICE_MOUNT_SROS = 0x01,      // 挂载在sros上的设备，可以配置
    DEVICE_MOUNT_HARDWARE = 0x02,  // 挂载在硬件上的设备，不允许配置
    DEVICE_MOUNT_SRC = 0x03,       // 挂载在src上的设备，不能修改参数
    DEVICE_MOUNT_VIRTUAL = 0x04,   // 虚拟设备
};

/** 每个设备要用唯一的设备名*/
const char DEVICE_EU100_TIM312_1[] = "EU100_TIM312_LEFT";
const char DEVICE_EU100_TIM312_2[] = "EU100_TIM312_RIGHT";
const char DEVICE_EU100_TIM312_BACK[] = "EU100_TIM312_BACK";
const char DEVICE_R2100[] = "R2100";
const char DEVICE_LC100[] = "LC100";
const char DEVICE_LC100_2[] = "LC100_2";
const char DEVICE_LC100_3[] = "LC100_3";
const char DEVICE_MOTOR_1[] = "MC_MOTOR_1";
const char DEVICE_MOTOR_2[] = "MC_MOTOR_2";
const char DEVICE_MOTOR_3[] = "AC_MOTOR_1";
const char DEVICE_MOTOR_4[] = "AC_MOTOR_2";

//编码器
const char DEVICE_WALK_1[] = "ENCODER_WALK_1";
const char DEVICE_WALK_2[] = "ENCODER_WALK_2";
const char DEVICE_WALK_3[] = "ENCODER_WALK_3";
const char DEVICE_ROTATE_1[] = "ENCODER_ROTATE_1";
const char DEVICE_ROTATE_2[] = "ENCODER_ROTATE_2";
const char DEVICE_ROTATE_3[] = "ENCODER_ROTATE_3";
const char DEVICE_PULL_ROPE_1[] = "ENCODER_PULL_ROPE_1";
const char DEVICE_PULL_ROPE_2[] = "ENCODER_PULL_ROPE_2";
const char DEVICE_PULL_SWAY_1[] = "ENCODER_PULL_SWAY_1";
const char DEVICE_PULL_SWAY_2[] = "ENCODER_PULL_SWAY_1";

//IO拓展模块
const char DEVICE_IOEXTEND_1353[] = "IOEXTEND_1353";

const char DEVICE_SPEAKER[] = "speaker";
const char DEVICE_EAC[] = "EAC";
const char DEVICE_IMU[] = "IMU";
const char DEVICE_PGV_UP[] = "PGV_UP";
const char DEVICE_PGV_DOWN[] = "PGV_DOWN";
const char DEVICE_VSC[] = "VSC";
const char DEVICE_SPU[] = "SPU";
const char DEVICE_PMU[] = "PMU";
const char DEVICE_BU100[] = "BU100";
const char DEVICE_BATTERY[] = "BATTERY";
const char DEVICE_SH100_TOF_1[] = "SH100_TOF_1";
const char DEVICE_SH100_TOF_2[] = "SH100_TOF_2";
const char DEVICE_SH100_TOF_3[] = "SH100_TOF_3";
const char DEVICE_TOF[] = "TOF";
const char DEVICE_SVC100_UP[] = "SVC100_UP";
const char DEVICE_SVC100_DOWN[] = "SVC100_DOWN";
const char DEVICE_SVC200_UP[] = "svc200_up";
const char DEVICE_SVC200_DOWN[] = "svc200_down";
const char DEVICE_SVC200_BACK[] = "svc200_back";
const char DEVICE_CAMERA_BACKWARD[] = "CAMERA_BACKWARD";//TODO 添加fm
const char DEVICE_CAMERA_FORWARD[] = "CAMERA_FORWARD";
const char DEVICE_CAMERA_BACK[] = "CAMERA_BACK";
const char DEVICE_CAMERA_LEFT[] = "CAMERA_LEFT";
const char DEVICE_CAMERA_RIGHT[] = "CAMERA_RIGHT";
const char DEVICE_CAMERA_DOWN[] = "CAMERA_DOWN";
const char DEVICE_CAMERA_UP[] = "CAMERA_UP";
const char DEVICE_CAMERA_D435[] = "CAMERA_D435";
const char DEVICE_LIDAR_LIVOX[] = "LIDAR_LIVOX";
const char DEVICE_CAMERA_O3D303[] = "CAMERA_O3D303";
const char DEVICE_CAMERA_MV_CE013[] = "CAMERA_MV_CE013";
const char DEVICE_CAMERA_D435_2[] = "CAMERA_D435_2";
const char DEVICE_CAMERA_DEPTH[] = "CAMERA_DEPTH";
const char DEVICE_CAMERA_DEPTH_2[] = "CAMERA_DEPTH_2";
const char DEVICE_CAMERA_D435_3[] = "CAMERA_D435_3";
const char DEVICE_CAMERA_D435_4[] = "CAMERA_D435_4";

const char DEVICE_LIDAR[] = "LIDAR";
const char DEVICE_LIDAR_1[] = "LIDAR_1";
const char DEVICE_LIDAR_2[] = "LIDAR_2";
const char DEVICE_SRC[] = "SRC";
const char DEVICE_SCREEN[] = "SCREEN";
const char DEVICE_UST_LIDAR_FORWARD[] = "FORWARD_UST_LIDAR";
const char DEVICE_UST_LIDAR_BACK[] = "BACK_UST_LIDAR";
const char DEVICE_UST_LIDAR_LEFT[] = "LEFT_UST_LIDAR";
const char DEVICE_UST_LIDAR_RIGHT[] = "RIGHT_UST_LIDAR";

const char DEVICE_ETH0[] = "ETH0";
const char DEVICE_ENP3S0[] = "ENP3S0";
const char DEVICE_CAN[] = "CAN";

// 标识唯一的设备ID，由三位十位数组成,这个用于统一故障码，编码需要参见文档：SROS统一错误码

enum DeviceFault {
    DEVICE_FAULT_OPEN = 401,     // 打开失败
    DEVICE_FAULT_INITIAL = 402,  // 初始化失败
    DEVICE_FAULT_TIMEOUT = 403,  // 通信超时
    DEVICE_FAULT_OPENING = 404,  //首次建立连接中

    DEVICE_FAULT_OTHER = 499,  // 位置的故障码都定义为99
};

const uint32_t DEFAULT_TIMEOUT_TIME = 5 * 1000;  // 默认超时时间

class Device {
 public:
    /**
     * @brief 构建一个设备
     * @param name 由用户保证唯一
     * @param device_id 有用户保证唯一
     * @param device_comm_interface_type
     * @param device_type
     */
    explicit Device(const std::string &name, DeviceID device_id, DeviceCommInterfaceType device_comm_interface_type,
                    DeviceMountType device_type);

    virtual ~Device() = default;

    std::string getInterfaceName();

    void keepAlive();
    void checkAlive();

    std::string getName() const { return name_; }
    void rename(const std::string &name) {name_ = name;}
    DeviceID getID() const { return device_id_; }

    DeviceState getState() const { return state_; }
    bool isOk() const { return state_ == DEVICE_OK || state_ == DEVICE_OFF || state_ == DEVICE_NONE; }  // 设备是否正常
    void setStateNone();
    void setStateInitialization();
    void setStateOK();
    void setStateOff();
    void setStateOpenFailed();
    void setStateInitialFailed();
    void setStateTimeout(uint32_t raw_fault_code = 0);
    void setStateOpening();//有一些设备打开失败一直会尝试连接，没有超时限制
    void setState(DeviceState state, uint32_t raw_fault_code);

    //设置unicorn_fault_code_,修改此状态需先从faultcenter移除历史码，再新增当前码，否则会导致故障码错报
    void setUnicornFaultCode(uint32_t old_unicorn_fault_code,uint32_t cur_unicorn_fault_code);

    bool isRawFault() const { return raw_fault_code_ != 0; }  // 是否有设备上报故障
    void setStateError(uint32_t raw_fault_code);              // 设置设备原始的错误码
    uint32_t getRawFaultCode() const { return raw_fault_code_; }
    bool isFault() const {
        return unicorn_fault_code_ != 0 && state_ != DEVICE_NONE && state_ != DEVICE_OFF;
    }  // 是否有故障，包括无法连接，超时等.设备为空或关闭是错误码还在，我们要剔除这样的情况
    uint32_t getUnicornFaultCode() const { return unicorn_fault_code_; }

    void updateAliveTime() { time_ = core::util::get_time_in_ms(); }

    void setSerialNo(const std::string &str) {
        boost::lock_guard<boost::shared_mutex> look(mutex_);
        serial_no_ = str;
    }
    std::string getSerialNo() const {
        boost::shared_lock<boost::shared_mutex> lock(mutex_);
        return serial_no_;
    }

    void setModelNo(const std::string &str) {
        boost::lock_guard<boost::shared_mutex> look(mutex_);
        model_no_ = str;
    }
    std::string getModelNo() const {
        boost::shared_lock<boost::shared_mutex> lock(mutex_);
        return model_no_;
    }

    void setVersionNo(const std::string &str) {
        boost::lock_guard<boost::shared_mutex> look(mutex_);
        version_no_ = str;
    }
    std::string getVersionNo() const {
        boost::shared_lock<boost::shared_mutex> lock(mutex_);
        return version_no_;
    }

    void setInfo(const std::string &str) {
        boost::lock_guard<boost::shared_mutex> look(mutex_);
        info_ = str;
    }
    std::string getInfo() const {
        boost::shared_lock<boost::shared_mutex> lock(mutex_);
        return info_;
    }

    void setTimeoutTime(uint32_t ms);

 protected:
    // 错误码映射，将原始的错误码映射成唯一的错误码，注意是3位，第一位表示故障等级，第二位和第三位表示设备故障编码
    virtual uint32_t faultCodeMapping(uint32_t raw_fault_code) { return DEVICE_FAULT_OTHER; }

 private:
    void cleanFault();

    void updateSrosEmergencySrc(bool trigger);

    std::string name_;

    DeviceID device_id_ = DEVICE_ID_UNDEFINED;  // 需要通过次ID来确定唯一的设备

    DeviceState state_ = DEVICE_NONE;

    std::atomic_uint_fast64_t time_;

    uint32_t raw_fault_code_ = 0;      // 设备原始的故障码，设备发上来多少就记录多少
    uint32_t unicorn_fault_code_ = 0;  // 统一后的故障码，包括设备ID、错误等级、映射后的错误码等
    std::mutex variable_mutex;         // 锁住设备相关变量，以解决多线程操作变量不同步的问题

    uint32_t hardware_error_code_ = 0;

    mutable boost::shared_mutex mutex_;  // 用读写锁锁住当前所有的string类型数据，防止崩溃

    // 注意一下接口不要经常写，因为需要加读写锁，占用资源
    std::string info_;  // 一些设备的信息

    std::string serial_no_;

    std::string model_no_;

    std::string version_no_;

    DeviceCommInterfaceType comm_interface_type_ = DEVICE_COMM_INTERFACE_TYPE_NONE;

    DeviceMountType mount_type_ = DEVICE_MOUNT_NONE;  // 设备类型，标记是否可以修改参数等

    uint32_t timeout_time_ = DEFAULT_TIMEOUT_TIME;  // 设备超时时间(ms)
};

typedef std::shared_ptr<Device> Device_ptr;

}  // namespace device
}  // namespace sros

#endif  // CORE_DEVICE_DEVICE_H_
