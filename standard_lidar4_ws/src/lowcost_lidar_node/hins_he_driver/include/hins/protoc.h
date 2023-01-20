/*********************************************************************
*
* Software License Agreement ()
*
*  Copyright (c) 2020, HINS, Inc.
*  All rights reserved.
*
* Author: Hu Liankui
* Create Date: 8/9/2020
*********************************************************************/

#pragma once

#include <array>
#include <vector>

namespace hins {

const std::array<unsigned char, 8> kStartCapture {0x52, 0x41, 0x75, 0x74, 0x6f, 0x01, 0x87, 0x80};

struct ScanData
{
  // 距离数据
  std::vector<std::uint32_t> distance_data;
  
  // 强度数据
  std::vector<std::uint32_t> amplitude_data;

  // 时间差
  unsigned short int time_increment;
};

struct ShadowsFilterParam
{
  float min_angle;
  float max_angle;
  int neighbors;            // 删除窗口
  int window;               // 搜索计算窗口
  int shadows_filter_level; 
  int traverse_step;          // 遍历步长
};

struct XingSongLaserParam
{
  bool change_flag;                           // 是否改变雷达参数 true为改变，false为不改变
  std::string run_state;                      // 运行状态
  int spin_frequency_Hz;                      // 转速频率
  std::string sampling_size_per_position;     
  int noise_filter_level;                     // 过滤等级
  std::string angle_increment;                // 角度分辨率
  bool block_enable;                        // true为发送避障帧，false为不发送
};

#pragma pack(1)

struct XSPackageHeader
{
  char head[4];
  uint16_t start_angle;      // 起始角度
  uint16_t end_angle;        // 终止角度
  uint16_t data_size;        // 这个包里面总的测量点数
  uint16_t data_position;    // 当前帧测量点位置
  uint16_t measure_size;     // 当前帧测量点数量
  uint16_t time;             // 时间标志（未启用）
};

struct HSGetAreaDataPackage 
{
  short int mode;                 // 传感器工作模式
  short int channel;            // 通道值
  short int angle;
  short int speed;
  short int channel_group;
};
struct HSAreaDataPackage 
{
  char head[5];
  uint8_t channel;                     // 通道值
  uint8_t input_status;           // 输入端口状态，未启用
  uint8_t output;                 // 输出端口状态
  uint8_t led_status;          // led状态，未启用
  uint16_t error_status;      // 传感器故障状态
  uint16_t check;                // 校验码
};

struct DisturbFilterParam
{
  uint32_t threshold;
  int range;
  bool disturb_filter_enable;
};

const uint16_t kXSPackageHeadSize = sizeof(XSPackageHeader);
const uint16_t kHSAreaDataPackageSize = sizeof(HSAreaDataPackage);
const uint16_t kMaxDistance = 36000;
const uint16_t kMaxIntensity = 30000;


#pragma pack()


}

