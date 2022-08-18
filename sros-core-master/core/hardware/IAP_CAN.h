/**
 * 
 * @copyright   : Copyright (C) 2019 Standard-robots, Inc
 * @file        : IAP_CAN.h
 * @description : 
 * @author      : EHL (linenhui@standard-robots.com / enhuilyn@qq.com)
 * @date        : 2022/04/27
 * @brief       : V1.0.0 
 */

#ifndef CORE_HARDWARE_IAP_CAN_H_
#define CORE_HARDWARE_IAP_CAN_H_

// #include "core/device/IODevice.h"

#include "core/device/SRIODevice.h"

namespace sros {
namespace device {

// 不能修改这里的值,将作为数组/向量的索引
typedef enum IapHostCmdID {
    IAP_HOST_SCAN = 0,
    IAP_HOST_REQUEST,
    IAP_HOST_SEND_FILE_INFO,
    IAP_HOST_SEND_DATA_INFO,
    IAP_HOST_SEND_DATA,
    IAP_HOST_SEND_TEST,
    IAP_HOST_RESPONSE_DEV_DATA,
    IAP_HOST_RESPONSE_DEV_INFO,
    IAP_HOST_GET_DEV_RESULT,
    ID_TABLE_SIZE,
}IAP_CAN_COMMAND_t;

enum IapDeviceState {
    IAP_DEV_BOOTLOADER = 0x00,
    IAP_DEV_APP_MODE = 0x01, // The host is in normal running mode
    IAP_DEV_ACK = 0xf4,
    IAP_DEV_NACK = 0xf5,
    IAP_DEV_UTEST = 0xf6,
    IAP_DEV_REQ_DATA = 0xf7,
    IAP_DEV_DISCON = 0xaf
};

// 设备扫描号 / 设备类型
enum IapDeviceType {
    IAP_DEV_EU200 = 0x01,
    IAP_DEV_SH100 = 0x02,
    IAP_DEV_SPU100 = 0x03,
    IAP_DEV_LC200 = 0x04,
    IAP_DEV_VSC300 = 0x05,
    IAP_DEV_BU100 = 0x06
};

// 设备默认ID
enum IapDeviceDefaultID {
    DEV_ID_EU200 = 0x01,
    DEV_ID_SH100 = 0x02,
    DEV_ID_SPU100 = 0x320,
    DEV_ID_LC200 = 0x04,
    DEV_ID_VSC300 = 0x05,
    DEV_ID_BU100 = 0x320
};

// 设备偏移
enum IapDeviceIdOffset {
    IAP_DEV_ID_OFFSET_EU200 = 0x00,
    IAP_DEV_ID_OFFSET_SH100 = 0x20,
    IAP_DEV_ID_OFFSET_SPU100 = 0x40,
    IAP_DEV_ID_OFFSET_LC200 = 0x60,
    IAP_DEV_ID_OFFSET_VSC300 = 0x80,
    IAP_DEV_ID_OFFSET_BU100 = 0xA0
};

// CAN ID 定义
#define  IAP_CAN_BASE_ID_SCAN      0x18FEFF00   // 扫描设备 CAN ID基地址
#define  IAP_CAN_BASE_ID_CMD       0x18FF0000   // 请求命令 CAN ID基地址
#define  IAP_CAN_BASE_ID_RESPONSE  0x18060000   // 设备回复 CAN ID基地址
#define  IAP_CAN_BASE_ID_STATUS    0x18FFF000   // 不真正用于发送,只响应

// 请求命令ID
#define  IAP_CMD_SCAN              0x18FEFF00         // 扫描设备
#define  IAP_CMD_ENTER_REQ         0x18FF0000         // 请求进入IAP模式
#define  IAP_CMD_FILE_INFO         0x18FF0001         // 发送固件信息        
#define  IAP_CMD_DATA_INFO         0x18FF0002         // 发送数据包信息
#define  IAP_CMD_DATA              0x18FF0003         // 发数据包数据
#define  IAP_CMD_TEST              0x18FF0004         // 发送更新测试

#define  IAP_CMD_DEV_REQ_DATA      0x18FFF004         // 上位机响应设备的主动请求数据包数据
#define  IAP_CMD_DEV_REQ_INFO      0x18FFF005         // 上位机响应设备的主动请求数据包信息
#define  IAP_CMD_DEV_STATUS        0x18FFF00A         // 上位机响应设备的主动上报状态/升级结果

// 设备回复命令ID
#define  IAP_DEV_ECHO_SCAN         0x18060000         // 设备回复扫描命令
#define  IAP_DEV_READY             0x18060001         // 设备进入IAP模式
#define  IAP_DEV_FILE_INFO         0x18060002         // 设备收到固件信息后回复
#define  IAP_DEV_DATA_INFO         0x18060003         // 设备收到数据包信息
#define  IAP_DEV_REQUEST_PKG_DATA  0x18060004         // 设备主动请求获取数据包
#define  IAP_DEV_REQUEST_PKG_INFO  0x18060005         // 设备请求获取数据包信息,校验失败时
#define  IAP_DEV_DATA              0x18060006         // 设备正常收到数据包数据后回复
#define  IAP_DEV_TEST              0x18060008         // 设备回复测试结果
#define  IAP_DEV_STATUS            0x1806000A         // 设备回复升级结果

#define IAP_CAN_DATA_PKG_SIZE_MAX    512

/**
  扫描设备命令: CAN发送ID = IAP_CAN_BASE_ID_SCAN + 设备扫描号   回复:回复命令ID + 设备偏移
  其它请求命令: CAN发送ID = CAN基地址 + 设备偏移 + 请求命令编号      回复:回复命令ID + 设备偏移
*/

// 设备编号 和 设备偏移,  映射表
const uint32_t IAP_DEV[][3] = {
    {IAP_DEV_EU200,     IAP_DEV_ID_OFFSET_EU200,   DEV_ID_EU200},
    {IAP_DEV_SH100,     IAP_DEV_ID_OFFSET_SH100,   DEV_ID_SH100},
    {IAP_DEV_SPU100,    IAP_DEV_ID_OFFSET_SPU100,  DEV_ID_SPU100},
    {IAP_DEV_LC200,     IAP_DEV_ID_OFFSET_LC200,   DEV_ID_LC200},
    {IAP_DEV_VSC300,    IAP_DEV_ID_OFFSET_VSC300, DEV_ID_VSC300},
    {IAP_DEV_BU100,     IAP_DEV_ID_OFFSET_BU100,  DEV_ID_BU100}
};

// 命令状态, can_id, response_id 的射表
// 实际设备ID == CAN_ID_TABLE + 设备编号 或者 设备偏移
const uint32_t CAN_ID_TABLE[ID_TABLE_SIZE][3] = {
   {IAP_HOST_SCAN,              IAP_CMD_SCAN,          IAP_DEV_ECHO_SCAN},
   {IAP_HOST_REQUEST,           IAP_CMD_ENTER_REQ,     IAP_DEV_READY},
   {IAP_HOST_SEND_FILE_INFO,    IAP_CMD_FILE_INFO,     IAP_DEV_FILE_INFO},
   {IAP_HOST_SEND_DATA_INFO,    IAP_CMD_DATA_INFO,     IAP_DEV_DATA_INFO},
   {IAP_HOST_SEND_DATA,         IAP_CMD_DATA,          IAP_DEV_DATA},
   {IAP_HOST_SEND_TEST,         IAP_CMD_TEST,          IAP_DEV_TEST},
   {IAP_HOST_RESPONSE_DEV_DATA, IAP_CMD_DEV_REQ_DATA,  IAP_DEV_REQUEST_PKG_DATA},
   {IAP_HOST_RESPONSE_DEV_INFO, IAP_CMD_DEV_REQ_INFO,  IAP_DEV_REQUEST_PKG_INFO},
   {IAP_HOST_GET_DEV_RESULT,    IAP_CMD_DEV_STATUS,    IAP_DEV_STATUS},
};

const std::string IAP_NAME[ID_TABLE_SIZE] = {
    "SCAN",
    "REQ",
    "FILE",
    "INFO",
    "DATA",
    "TEST",
    "DATA2",
    "INFO2",
    "RESULT"
};

using IapCanDataCallback = std::function<void(int)>;


class IapCan : public SRIODevice {
 public:
    IapCan(const std::string &name, DeviceID device_id, DeviceCommInterfaceType device_comm_interface_type,
                       std::shared_ptr<IOInterface> io_interface);

    void init(const uint32_t dev_type, const uint32_t dev_offset, const uint step);
    bool asyncRequestData();
    bool asyncRequestData(const std::vector<uint8_t> &data);

    int scanDevice(const uint32_t dev_type);
    int requestDeviceEnterIAP();
    bool sendData(const std::vector<uint8_t> &data);

    int sendFileInfo(uint32_t packetSize);
    int sendDataPacketInfo(uint32_t pkgIndex,  const uint32_t pkgDataSize, uint32_t crc);
    bool sendIAPdata(const std::vector<uint8_t> &data);

    uint32_t getDevVersionNo();
    std::string getDevVersionStr();

    void setIapDataCallback(IapCanDataCallback func) { iap_data_callback_ = func; }

 protected:
    void onDataRecive(const std::vector<uint8_t> &data) final;  // 处理收到的数据，各个设备需要重写此函数

 private:

    uint32_t version_num_;
    std::string version_str_;

    uint32_t iap_cmd_step_;
    uint32_t iap_dev_type_;
    uint32_t iap_dev_offset_;
    uint32_t dev_default_id_;

    IapCanDataCallback iap_data_callback_ = nullptr;
    AsyncConditionVariable<int> data_index_variable_;
    AsyncConditionVariable<int> dev_response_;

};

typedef std::shared_ptr<IapCan> IapCan_ptr;

}  // namespace device
}  // namespace sros

#endif  // CORE_HARDWARE_IAP_CAN_H_
