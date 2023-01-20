#include "ord/ord_driver.h"
#include <algorithm>
#include <cstring>
#include <iostream>

namespace ord_sdk
{

  OrdDriver::OrdDriver(const SocketAddress &sensor)
      : impl_(new Impl())
  {
    if (typeid(sensor) == typeid(LidarAddress))
      sensor_ = std::unique_ptr<LidarAddress>(new LidarAddress((const LidarAddress &)sensor));

    FilterLevel_ = 0;
    MotorSpeed_ = 0;
    FirstGetInfo_ = 0;
  }

  OrdDriver::~OrdDriver()
  {
    if (impl_ && impl_->isOpened())
      impl_->close();
  }

  error_t OrdDriver::open()
  {
    return impl_->open(*sensor_);
  }

  bool OrdDriver::isOpened() const
  {
    return impl_->isOpened();
  }

  void OrdDriver::close()
  {
    impl_->close();
  }

  void OrdDriver::setTimeout(int timeout)
  {
    impl_->setTimeout(timeout);
  }

  error_t OrdDriver::setLidarIPAddress(in_addr_t address)
  {
    error_t result = no_error;
    std::vector<uint8_t> request, response, content;
    content.resize(4);
    *(uint32_t *)(content.data()) = htonl(address);
    impl_->GnerateProtocolFrame(set_lidar_ip, request, content);
    result = impl_->DealProtocolData(std::move(request), response);
    return result;
  }

  error_t OrdDriver::setLidarNetPort(in_port_t port)
  {
    error_t result = no_error;
    std::vector<uint8_t> request, response, content;
    content.resize(4);
    *(uint16_t *)(content.data()) = htons(port);
    impl_->GnerateProtocolFrame(set_lidar_port, request, content);
    result = impl_->DealProtocolData(std::move(request), response);
    return result;
  }

  error_t OrdDriver::setTailFilterLevel(uint32_t level)
  {
    error_t result = no_error;
    std::vector<uint8_t> request, response, content;
    content.resize(4);
    *(uint32_t *)(content.data()) = htonl(level);
    impl_->GnerateProtocolFrame(set_filter_level, request, content);
    result = impl_->DealProtocolData(std::move(request), response);
    if (result == no_error)
    {
      FilterLevel_ = level;
    }
    return result;
  }

  error_t OrdDriver::setTimestamp(uint32_t timestamp)
  {
    error_t result = no_error;
    std::vector<uint8_t> request, response, content;
    content.resize(4);
    *(uint32_t *)(content.data()) = htonl(timestamp);
    impl_->GnerateProtocolFrame(set_timestamp, request, content);
    result = impl_->DealProtocolData(std::move(request), response);
    return result;
  }

  error_t OrdDriver::setScanSpeed(uint32_t speed)
  {
    error_t result = no_error;
    std::vector<uint8_t> request, response, content;
    content.resize(4);
    *(uint32_t *)(content.data()) = htonl(speed);
    impl_->GnerateProtocolFrame(set_scan_speed, request, content);
    result = impl_->DealProtocolData(std::move(request), response);
    if (result == no_error)
    {
      MotorSpeed_ = speed;
    }
    return result;
  }

  error_t OrdDriver::trackConnect()
  {
    error_t result = no_error;
    std::vector<uint8_t> request, response, content;
    uint32_t CmdCode = 0x12345678;
    content.resize(4);
    *(uint32_t *)content.data() = htonl(CmdCode);
    impl_->GnerateProtocolFrame(set_track_connect, request, content);

    result = impl_->DealProtocolData(std::move(request), response);
    return result;
  }

  error_t OrdDriver::enableMeasure()
  {
    error_t result = no_error;
    std::vector<uint8_t> request, response, content;
    uint32_t CmdCode = 0x01;
    content.resize(4);
    *(uint32_t *)content.data() = htonl(CmdCode);
    impl_->GnerateProtocolFrame(set_measure_onoff, request, content);

    result = impl_->DealProtocolData(std::move(request), response);
    return result;
  }

  error_t OrdDriver::disableMeasure()
  {
    error_t result = no_error;
    std::vector<uint8_t> request, response, content;
    uint32_t CmdCode = 0x00;
    content.resize(4);
    *(uint32_t *)content.data() = htonl(CmdCode);
    impl_->GnerateProtocolFrame(set_measure_onoff, request, content);

    result = impl_->DealProtocolData(std::move(request), response);
    return result;
  }

  error_t OrdDriver::enabelDataStream()
  {
    error_t result = no_error;
    std::vector<uint8_t> request, response, content;
    uint32_t CmdCode = 0x01;
    content.resize(4);
    *(uint32_t *)content.data() = htonl(CmdCode);
    impl_->GnerateProtocolFrame(set_stream_onoff, request, content);
    result = impl_->DealProtocolData(std::move(request), response);
    return result;
  }

  error_t OrdDriver::disableDataStream()
  {
    error_t result = no_error;
    std::vector<uint8_t> request, response, content;
    uint32_t CmdCode = 0x00;
    content.resize(4);
    *(uint32_t *)content.data() = htonl(CmdCode);
    impl_->GnerateProtocolFrame(set_stream_onoff, request, content);
    result = impl_->DealProtocolData(std::move(request), response);
    return result;
  }

  error_t OrdDriver::setScanDirection(uint32_t direction)
  {
    error_t result = no_error;
    std::vector<uint8_t> request, response, content;
    content.resize(4);
    *(uint32_t *)(content.data()) = htonl(direction);
    impl_->GnerateProtocolFrame(set_scan_direction, request, content);
    result = impl_->DealProtocolData(std::move(request), response);
    return result;
  }

  error_t OrdDriver::applyConfigs()
  {
    error_t result = no_error;
    std::vector<uint8_t> request, response, content;
    uint32_t CmdCode = 0x00;
    content.resize(4);
    *(uint32_t *)content.data() = htonl(CmdCode);
    impl_->GnerateProtocolFrame(set_parameter_save, request, content);
    result = impl_->DealProtocolData(std::move(request), response);
    return result;
  }

  error_t OrdDriver::deviceReboot()
  {
    error_t result = no_error;
    std::vector<uint8_t> request, response, content;
    uint32_t CmdCode = 0x00;
    content.resize(4);
    *(uint32_t *)content.data() = htonl(CmdCode);
    impl_->GnerateProtocolFrame(set_device_reboot, request, content);
    result = impl_->DealProtocolData(std::move(request), response);
    return result;
  }

  error_t OrdDriver::getLidarIPAddress(in_addr_t &address)
  {
    error_t result = no_error;
    std::vector<uint8_t> request, response, content;

    impl_->GnerateProtocolFrame(get_lidar_ip, request, content);

    result = impl_->DealProtocolData(std::move(request), response);
    if (result == no_error)
    {
      address = ntohl(*(uint32_t *)response.data());
    }
    return result;
  }

  error_t OrdDriver::getLidarNetPort(in_port_t &port)
  {
    error_t result = no_error;
    std::vector<uint8_t> request, response, content;

    impl_->GnerateProtocolFrame(get_lidar_port, request, content);

    result = impl_->DealProtocolData(std::move(request), response);
    if (result == no_error)
    {
      port = ntohs(*(uint16_t *)response.data());
    }
    return result;
  }

  error_t OrdDriver::getScanSpeed(uint32_t &speed)
  {
    error_t result = no_error;
    std::vector<uint8_t> request, response, content;

    impl_->GnerateProtocolFrame(get_scan_speed, request, content);

    result = impl_->DealProtocolData(std::move(request), response);
    if (result == no_error)
    {
      speed = ntohl(*(uint32_t *)response.data());
      MotorSpeed_ = speed;
    }
    return result;
  }

  error_t OrdDriver::getTailFilterLevel(uint32_t &level)
  {
    error_t result = no_error;
    std::vector<uint8_t> request, response, content;

    impl_->GnerateProtocolFrame(get_filter_level, request, content);

    result = impl_->DealProtocolData(std::move(request), response);
    if (result == no_error)
    {
      level = ntohl(*(uint32_t *)response.data());
      FilterLevel_ = level;
    }
    return result;
  }

  error_t OrdDriver::getTimestamp(uint32_t &timestamp)
  {
    error_t result = no_error;
    std::vector<uint8_t> request, response, content;

    impl_->GnerateProtocolFrame(get_timestamp, request, content);

    result = impl_->DealProtocolData(std::move(request), response);
    if (result == no_error)
    {
      timestamp = ntohl(*(uint32_t *)response.data());
    }
    return result;
  }

  error_t OrdDriver::getMotorSpeed(uint32_t &motor_speed)
  {
    error_t result = no_error;
    std::vector<uint8_t> request, response, content;

    impl_->GnerateProtocolFrame(get_motor_speed, request, content);

    result = impl_->DealProtocolData(std::move(request), response);
    if (result == no_error)
    {
      motor_speed = ntohl(*(uint32_t *)response.data());
    }
    return result;
  }

  error_t OrdDriver::getInternalTemperature(float &inter_temp)
  {
    error_t result = no_error;
    std::vector<uint8_t> request, response, content;

    impl_->GnerateProtocolFrame(get_temperature, request, content);

    result = impl_->DealProtocolData(std::move(request), response);
    if (result == no_error)
    {
      inter_temp = *(float *)response.data();
    }
    return result;
  }

  error_t OrdDriver::getSyncStatus(uint32_t &sync_status)
  {
    error_t result = no_error;
    std::vector<uint8_t> request, response, content;
    impl_->GnerateProtocolFrame(get_sync_status, request, content);

    result = impl_->DealProtocolData(std::move(request), response);
    if (result == no_error)
    {
      sync_status = ntohl(*(uint32_t *)response.data());
    }
    return result;
  }

  error_t OrdDriver::getFirmwareVersion(std::string &firmware_version)
  {
    error_t result = no_error;
    std::vector<uint8_t> request, response, content;

    impl_->GnerateProtocolFrame(get_firmware_version, request, content);

    result = impl_->DealProtocolData(std::move(request), response);
    if (result == no_error)
    {
      firmware_version.assign((char *)response.data(), 16);
    }
    return result;
  }

  error_t OrdDriver::getHardwareVersion(std::string &hardware_version)
  {
    error_t result = no_error;
    std::vector<uint8_t> request, response, content;

    impl_->GnerateProtocolFrame(get_hardware_version, request, content);

    result = impl_->DealProtocolData(std::move(request), response);
    if (result == no_error)
    {
      hardware_version.assign((char *)response.data(), 16);
    }
    return result;
  }

  error_t OrdDriver::getSDKVersion(std::string &sdk_version)
  {

    error_t result = no_error;

    sdk_version = "v1.0.5";

    return result;
  }

  error_t OrdDriver::getScanDirection(uint32_t &direction)
  {
    error_t result = no_error;
    std::vector<uint8_t> request, response, content;

    impl_->GnerateProtocolFrame(get_scan_direction, request, content);

    result = impl_->DealProtocolData(std::move(request), response);
    if (result == no_error)
    {
      direction = ntohl(*(uint32_t *)response.data());
    }
    return result;
  }

  error_t OrdDriver::getScanFrameData(ScanFrameData &scan_frame_data)
  {
    ScanBlockData scan_block_data;
    uint8_t _id[6] = {0};
    uint8_t begin = 0;
    while (1)
    {
      error_t result = getScanBlockData(scan_block_data);
      if (result != error_t::no_error)
        return result;

      uint8_t _index = scan_block_data.block_id - 1;
      if (_index < 6)
      {
        _id[_index] = 1;
        int count = scan_block_data.layers[0].ranges.size();
        if (begin == 0)
        {
          begin = 1;
          scan_frame_data.timestamp = scan_block_data.timestamp;
          scan_frame_data.layers.resize(1);
          scan_frame_data.layers[0].ranges.resize(count * 6);
          scan_frame_data.layers[0].intensities.resize(count * 6);
        }

        for (int i = 0; i < count; i++)
        {
          scan_frame_data.layers[0].ranges[_index * count + i] = scan_block_data.layers[0].ranges[i];
          scan_frame_data.layers[0].intensities[_index * count + i] = scan_block_data.layers[0].intensities[i];
        }
      }
      if ((_index == 0) || (_index == 5))
      {
        uint8_t temp = 0;
        for (uint8_t i = 0; i < 6; i++)
        {
          temp += _id[i];
        }
        if (temp == 6)
        {
          break;
        }
      }
    }

    if(FirstGetInfo_ == 0)
    {
      uint32_t temp;
      error_t ret = getTailFilterLevel(temp);
      if(ret == error_t::no_error)
      {
        ret = getScanSpeed(temp);
        {
          if(ret == error_t::no_error)
          {
            FirstGetInfo_ = 1;
          }
        }
      } 
    }

    point_cloud_filter(FilterLevel_, MotorSpeed_, scan_frame_data.layers[0].ranges);
    
    return error_t::no_error;
  }

  error_t OrdDriver::getScanBlockData(ScanBlockData &scan_block_data)
  {

    std::vector<uint8_t> block_data;
    error_t result = impl_->waitScanData(block_data);
    if (result == error_t::no_error)
    {
      if (block_data.size() > 0)
      {
        const PointCloudPacket *pc_packet = reinterpret_cast<const PointCloudPacket *>(block_data.data() + sizeof(ordFrameHead));
        uint8_t block_scan_type = pc_packet->data_type;
        uint32_t block_length = 0;
        switch (block_scan_type)
        {
        case LASER_SCAN_BLOCK_TYPE_10HZ:
          block_length = LASER_SCAN_BLOCK_LENGTH_10HZ;
          break;
        case LASER_SCAN_BLOCK_TYPE_15HZ:
          block_length = LASER_SCAN_BLOCK_LENGTH_15HZ;
          break;
        case LASER_SCAN_BLOCK_TYPE_20HZ:
          block_length = LASER_SCAN_BLOCK_LENGTH_20HZ;
          break;
        case LASER_SCAN_BLOCK_TYPE_25HZ:
          block_length = LASER_SCAN_BLOCK_LENGTH_25HZ;
          break;
        case LASER_SCAN_BLOCK_TYPE_30HZ:
          block_length = LASER_SCAN_BLOCK_LENGTH_30HZ;
          break;
        }
        scan_block_data.block_id = pc_packet->block_num;
        scan_block_data.timestamp = ntohl(pc_packet->timestamp);
        scan_block_data.frame_cnt = pc_packet->frame_count;
        scan_block_data.sync_mode = pc_packet->sync_mode;
        scan_block_data.layers.resize(1);
        scan_block_data.layers[0].ranges.resize(block_length);
        scan_block_data.layers[0].intensities.resize(block_length);
        for (int i = 0; i < block_length; i++)
        {
          scan_block_data.layers[0].ranges[i] = ntohs(*(uint16_t *)(pc_packet->pc_data + 4 * i));
          scan_block_data.layers[0].intensities[i] = ntohs(*(uint16_t *)(pc_packet->pc_data + 4 * i + 2));
        }
      }
    }
    return result;
  }

  void OrdDriver::point_cloud_filter(uint32_t FilterLevel, uint32_t MotorSpeed, std::vector<uint16_t>& distances)
  {
    uint16_t SumLeftAbnormal = 0;      //滑窗左侧点云夹角异常数量
    uint16_t SumRightAbnormal = 0;     //滑窗右侧点云夹角异常数量
    int32_t Neighbors_ = 0;            //相邻滤除处理点个数
    int32_t PointNumThreshold = 0;     //临近点云超过阈值的数量门限
    float TanAngleThreshold = 0;       //点云滤波角度判决阈值，依据临近点云夹角
    float AngleResSin = 0;             //角度间隔对应的正弦值
    float AngleResCos = 0;             //角度间隔对应的余弦值
    double TanAngle = 0;               //夹角的正切值
    uint16_t abnormal_point_diff = 25; //与左右两边的点 进行 预判断  10： 15 3cm  15： 30 4cm   10： 25 5cm   10： 35 7cm   10： 40 8cm
    uint16_t PointDistanceCmp = 0;     //点云距离比较量暂存值
    uint16_t cur_distance = 0;
    uint16_t diff_1, diff_2;
    int count = distances.size(); //一圈的点数
    
    std::vector<uint16_t> temp_distance_data;

    if (count < FILTER_WINDOW_SIZE)
    {
      return;
    }

    if (FilterLevel == 0)
    {
      return;
    }
    if ((MotorSpeed != 10) && (MotorSpeed != 15) && (MotorSpeed != 20) && (MotorSpeed != 25) && (MotorSpeed != 30))
    {
      return;
    }

    //std::cout << "FilterLevel:" << FilterLevel << "MotorSpeed:" << MotorSpeed << "count:" << count << std::endl;
    temp_distance_data.resize(count);
    std::copy(distances.begin(), distances.end(), temp_distance_data.begin());

    switch (FilterLevel)
    {
    case 1:
    {
      // tan(3°),即拖尾、噪点角度范围在±4°以内,剔除相邻点个数为1,判断一侧至少3点符合
      TanAngleThreshold = 0.03492076949175; //  0.03492076949175   0.052407779283041
      Neighbors_ = 0;
      PointNumThreshold = 4;
      break;
    }
    case 2:
    {
      // tan(3°),即拖尾、噪点角度范围在±4°以内,剔除相邻点个数为1，判断一侧至少2点符合
      TanAngleThreshold = 0.03492076949175; // 0.03492076949175      0.052407779283041
      Neighbors_ = 0;
      PointNumThreshold = 3;
      break;
    }
    case 3:
    {
      // tan(5.5°),即拖尾、噪点角度范围在±7°以内,剔除相邻点个数为1,判断一侧至少2点符合
      TanAngleThreshold = 0.052407779283041; // 0.052407779283041     0.096289048197539
      Neighbors_ = 0;
      PointNumThreshold = 3;
      break;
    }
    case 4: 
    {
      // tan(9°),即拖尾、噪点角度范围在±9°以内,剔除相邻点个数为1，判断一侧至少2点符合
      TanAngleThreshold = 0.052407779283041; // 0.052407779283041    0.158384440324536
      Neighbors_ = 1;
      PointNumThreshold = 3;
      break;
    }
    case 5:
    {
      // tan(9°),即拖尾、噪点角度范围在±9°以内,剔除相邻点个数为1,判断一侧至少1点符合
      TanAngleThreshold = 0.096289048197539;  // 0.096289048197539     0.158384440324536
      Neighbors_ = 1;
      PointNumThreshold = 3;
      break;
    }
    default:
    {
      TanAngleThreshold = 0;
      break;
    }
    }

    switch (MotorSpeed)
    {
    case 10:
    {
      AngleResSin = -0.002073449665673;
      AngleResCos = 0.999997850400932;
      abnormal_point_diff = 15;
      break;
    }
    case 15:
    {
      AngleResSin = -0.003110171712830;
      AngleResCos = 0.999995163404262;
      abnormal_point_diff = 30;
      break;
    }
    case 20:
    {
      AngleResSin = -0.004146890417175;
      AngleResCos = 0.999991401612968;
      abnormal_point_diff = 25;
      break;
    }
    case 25:
    {
      AngleResSin = -0.005183604664443;
      AngleResCos = 0.999986565031092;
      abnormal_point_diff = 35;
      break;
    }
    case 30:
    {
      AngleResSin = -0.006220313340373;
      AngleResCos = 0.999980653663833;
      abnormal_point_diff = 40;
      break;
    }
    default:
    {
      break;
    }
    }

    for (int i = (FILTER_WINDOW_SIZE / 2); i < (count - (FILTER_WINDOW_SIZE / 2)); i++)
    {
      cur_distance = temp_distance_data[i];
      if (cur_distance > temp_distance_data[i + 1])
      {
        diff_1 = cur_distance - temp_distance_data[i + 1];
      }
      else
      {
        diff_1 = temp_distance_data[i + 1] - cur_distance;
      }

      if (cur_distance > temp_distance_data[i - 1])
      {
        diff_2 = cur_distance - temp_distance_data[i - 1];
      }
      else
      {
        diff_2 = temp_distance_data[i - 1] - cur_distance;
      }

      if ((diff_1 < abnormal_point_diff) && (diff_2 < abnormal_point_diff))
      {
        if (cur_distance > 500)
        {
          continue;
        }
      }

      int half_window = FILTER_WINDOW_SIZE / 2;
      for (int pos = (i - half_window); pos < i + half_window + 1; pos++)
      {
        //避免待检测点云与自身对比，当前待检测点云距离为0时不做滤波
        if (pos == i || cur_distance == 0)
        {
          continue;
        }

        PointDistanceCmp = temp_distance_data[pos];
        TanAngle = (cur_distance * AngleResSin) / (PointDistanceCmp - cur_distance * AngleResCos); //计算当前最新点云与左右6个相邻点云间的夹角的正切值
        if ((TanAngle < TanAngleThreshold) && (TanAngle > ((-1) * TanAngleThreshold)))
        {
          if (pos < i)
            SumLeftAbnormal++; //累计滑窗左侧夹角超过阈值的点云数
          else
            SumRightAbnormal++; //累计滑窗右侧夹角超过阈值的点云数
        }

        if ((SumLeftAbnormal >= PointNumThreshold) || (SumRightAbnormal >= PointNumThreshold)) //如果左侧和右侧有一侧有夹角超过阈值，则认为是拖尾/噪点
        {
          for (int num = ((-1) * Neighbors_); num <= Neighbors_; num++)
          {
            distances[i + num] = 0; //认为为拖尾点或噪点，滤除该点及临近Neighbors_个点云
          }

          break;
        }
      }

      SumLeftAbnormal = 0;
      SumRightAbnormal = 0;
    }
  }

}
