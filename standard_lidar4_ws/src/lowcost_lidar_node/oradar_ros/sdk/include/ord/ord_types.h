#ifndef ORD_SDK_ORD_TYPES_H_
#define ORD_SDK_ORD_TYPES_H_

#include <vector>
#include <cstdint>
#include <memory>

namespace ord_sdk
{

class ScanFrameData
{
public:
  class FrameData
  {
  public:
    std::vector<uint16_t> ranges;
    std::vector<uint16_t> intensities;
  };

  uint32_t timestamp;
  std::vector<FrameData> layers;
};

class ScanBlockData
{
public:
  class BlockData
  {
  public:
    std::vector<uint16_t> ranges;
    std::vector<uint16_t> intensities;
  };

  uint8_t block_id;
  uint32_t timestamp;
  uint8_t sync_mode;
  uint16_t frame_cnt;
  std::vector<BlockData> layers;
};


enum commadn_byte{
    /*ctrl type
              控制信息类型：
              00：无效
              01：配置
              02：查询
              03：保留
    */    
    /*ctrl code
              配置操作码：
              00：无效
              01：IP地址配置 
              02：端口号配置
              03：MAC地址配置
              04：子网掩码配置
              05：旋转频率配置
              06：点云滤波配置
              07：时间戳配置
              08：设备连接操作
              09：设备开关操作
              10：点云数据传输开关操作
              11：工作状态配置
              12：设备序列号配置
              13：配置信息保存
              14：设备重启
              15：产品型号配置
              其他：保留
              查询操作码：
              00：无效
              01：IP地址查询
              02：端口号查询
              03：MAC地址查询查询
              04：子网掩码查询
              05：旋转频率查询
              06：点云滤波参数查询
              07：时间戳查询
              08：瞬时旋转频率查询
              09：内部监测温度查询
              10：探测器偏置高压查询
              11：工作状态查询
              12：外同步状态查询
              13：硬件版本信息查询
              14：固件版本信息查询
              15：设备序列号查询
              16：产品型号查询
              其他：保留
    */

    command_start = 0,
    set_invalid = 0x0100,
    set_lidar_ip = 0x0101,
    set_lidar_port = 0x0102,
    set_scan_speed = 0x0105,
    set_filter_level = 0x0106,
    set_timestamp = 0x0107,
    set_track_connect = 0x0108,
    set_measure_onoff = 0x0109,
    set_stream_onoff = 0x010a,
    set_parameter_save = 0x010d,
    set_device_reboot = 0x010e,
    set_scan_direction = 0x0115,


    get_invalid = 0x0200,
    get_lidar_ip = 0x0201,
    get_lidar_port = 0x0202,
    get_scan_speed = 0x0205,
    get_filter_level = 0x0206,
    get_timestamp = 0x0207,
    get_motor_speed = 0x0208,
    get_temperature = 0x0209,
    get_high_voltage = 0x020a,
    get_sync_status = 0x020c,
    get_hardware_version = 0x020d,
    get_firmware_version = 0x020e,
    get_scan_direction = 0x0212,
    command_end
};

enum error_t {
  no_error = 0,
  operation_failure,
  timed_out,
  address_in_use,
};

const int LASER_SCAN_BLOCK_LENGTH_10HZ = 378;
const int LASER_SCAN_BLOCK_LENGTH_15HZ = 252;
const int LASER_SCAN_BLOCK_LENGTH_20HZ = 192;
const int LASER_SCAN_BLOCK_LENGTH_25HZ = 150;
const int LASER_SCAN_BLOCK_LENGTH_30HZ = 126;

const int LASER_SCAN_BLOCK_TYPE_10HZ = 0x04;
const int LASER_SCAN_BLOCK_TYPE_15HZ = 0x05;
const int LASER_SCAN_BLOCK_TYPE_20HZ = 0x06;
const int LASER_SCAN_BLOCK_TYPE_25HZ = 0x07;
const int LASER_SCAN_BLOCK_TYPE_30HZ = 0x08;

#pragma pack(push, 1)
struct PointCloudPacket
{
  uint16_t  frame_length;
  uint8_t   data_type;
  uint8_t   block_num;
  uint16_t  frame_count;
  uint32_t  timestamp;
  uint8_t   sync_mode;
  uint8_t   reserved;
  uint8_t   pc_data[LASER_SCAN_BLOCK_LENGTH_10HZ*3];
};

struct ordFrameHead
{
    uint16_t framelenth;
    uint8_t frametype;
    uint8_t frametypeh;
    union {
            uint16_t ctrlcomm;
            struct {
              uint8_t type;
              uint8_t code;
            }ctrl_str;
    }ctrl_un;  
    union {
            uint16_t respcomm;
            uint16_t frame_len;
            struct {
              uint8_t type;
              uint8_t code;
            }resp_str;
    }resp_un;
};
#pragma pack(pop)

}

#endif
