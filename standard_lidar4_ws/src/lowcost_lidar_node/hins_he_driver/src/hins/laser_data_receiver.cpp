#include "hins/laser_data_receiver.h"
#include "hins/protoc.h"
#include "utils.h"
#include <iostream>
#include <fstream>

// #define DEBUG
namespace hins {

LaserDataReceiver::LaserDataReceiver(const ConnectionAddress& conn_info)
  : conn_info_(conn_info)
  , is_connected_(false)
  , inbuf_(4096)
  , instream_(&inbuf_)
  , tcp_socket_ptr_(0)
  , ring_buffer_(65536)
  , scan_data_()
  , last_data_time_(hins::Now())
  , scan_all_point_num_(0)
  , laser_steady_time(500) 
  , have_block_(0)
  , _now_channel_(-1)
  , scan_all_point_num_init_flag(true)
{
  std::cout << "Connecting to TCP data channel at hostname: " << conn_info.GetAddress()
          << " tcp_port:" << conn_info.GetPort() << std::endl;

  if(Connect())
    std::cout << "Lidar connect success." << std::endl;
  else
    std::cout << "Lidar connect failed." << std::endl;

}

LaserDataReceiver::~LaserDataReceiver()
{
  Disconnect();

  if(tcp_socket_ptr_)
    delete tcp_socket_ptr_;
}

short int LaserDataReceiver::GetLaserSteadyTime()
{
  return laser_steady_time;
}

bool LaserDataReceiver::IsConnected()
{
  return is_connected_;
}

void LaserDataReceiver::Disconnect()
{
  is_connected_ = false;
  try
  {
    if( tcp_socket_ptr_ != nullptr )
      tcp_socket_ptr_->close();

    io_service_.stop();
    if( boost::this_thread::get_id() != io_service_thread_.get_id() )
      io_service_thread_.join();
  }
  catch (std::exception& e)
  {
    std::cout << "Exception:" << e.what() << std::endl;
  }
  std::cout << "Disconnect." << std::endl;
}

bool LaserDataReceiver::CheckConnection()
{
  if( !IsConnected() )
    return false;
  if( (hins::Now() - last_data_time_) > 1000 ) // 1s 断线
  {
    Disconnect();
    std::cout << "超时断线，超时时间：" << (hins::Now() - last_data_time_)/1000.0 << std::endl;
    return false;
  }
  return true;
}

ScanData LaserDataReceiver::GetFullScan()
{
  std::unique_lock<std::mutex> lock(scan_mutex_);
  while( CheckConnection() && scan_data_.size() < 2 )
    data_notifier_.wait_for(lock, std::chrono::seconds(1));


  ScanData data;
  if( scan_data_.size() >= 2 && IsConnected())
  {
    data = ScanData(std::move(scan_data_.front()));
    scan_data_.pop_front();
  }
  else
    std::cout << "null data" << std::endl;

  #ifdef DEBUG
  std::cout << "scan_all_point_num_:" << scan_all_point_num_
            << "     data.distance_data.size():" << data.distance_data.size()
            << std::endl;
  #endif

  // if(data.distance_data.size() != scan_all_point_num_)
  //   data.distance_data.clear();

  return data;
}

void LaserDataReceiver::HandleSocketRead(const boost::system::error_code &error)
{
  if(!error)
  {
    // 1. 将所有数据推送到缓冲区
    instream_.clear();
    while(!instream_.eof())
    {
      char buf[4096];
      instream_.read(buf, 4096);
      int bytes_read = instream_.gcount();
      WriteBufferBack(buf, bytes_read);       // 将 buf 的数据写到 ring_buffer_ 里
    }

    // 2. 继续读取数据，直到数据被读取完
    while(HandleNextPacket()) {}

    // 3. 继续异步读取数据
    boost::asio::async_read(*tcp_socket_ptr_, inbuf_, boost::bind(&LaserDataReceiver::HandleSocketRead, this, boost::asio::placeholders::error));
  }
  else
  {
    if( error.value() != 995 )
      std::cout << "ERROR: data connection error:"
                << error.message()
                << " "
                << error.value()
                << std::endl;
    Disconnect();
  }
}

int LaserDataReceiver::SyncWrite()
{
  boost::system::error_code ec;

  int ret = tcp_socket_ptr_->write_some(boost::asio::buffer(kStartCapture), ec);
  if(ec)
  {
    std::cout << "write failed：%" << boost::system::system_error(ec).what() << std::endl;
    ret = -1;
  }
  #ifdef DEBUG
  std::cout << "SyncWrite kStartCapture done" << std::endl;
  #endif
  return ret;
}


int LaserDataReceiver::SyncWrite(const std::array<unsigned char, 12> command)
{
  boost::system::error_code ec;
  int ret = tcp_socket_ptr_->write_some(boost::asio::buffer(command), ec);
  if(ec)
  {
    std::cout << "write failed：" << boost::system::system_error(ec).what() << std::endl;
    ret = -1;
  }
  #ifdef DEBUG
  std::cout << "SyncWrite command done" << std::endl;
  #endif
  
  return ret;
}

int LaserDataReceiver::SyncWrite(const std::array<char, 18> command)
{
  boost::system::error_code ec;
  int ret = tcp_socket_ptr_->write_some(boost::asio::buffer(command), ec);
  if(ec)
  {
    std::cout << "write failed：" << boost::system::system_error(ec).what() << std::endl;
    ret = -1;
  }
  #ifdef DEBUG
  std::cout << "SyncWrite command done" << std::endl;
  #endif
  
  return ret;
}


// CRC循环校验码-12位命令
// 修改command的最后两位
// 返回循环校验码的10进制数字
unsigned int CRC_Verify_len12(std::array<unsigned char, 12> *command)
{
  unsigned int i, j;
  unsigned int wCrc = 0xffff;
  unsigned int wPolynom = 0xA001;
  /*-------------------------------------*/
  for (i = 0; i < 12 - 2; i++) // 后两位为校验位
  {
    wCrc ^= command->at(i);
    for (j = 0; j < 8; j++)
    {
      if (wCrc & 0x0001)
      {
        wCrc = (wCrc >> 1) ^ wPolynom;
      }
      else
      {
        wCrc = wCrc >> 1;
      }
    }
  }
  unsigned int tmp1 = (wCrc & 0xff00) >> 8;           // 高位
  unsigned int tmp2 = (wCrc & 0x00ff);                // 低位
  command->at(10) = tmp2;
  command->at(11) = tmp1;
  return wCrc;
}

// 生成雷达控制命令
std::array<unsigned char, 12> LaserDataReceiver::GenerateParamCommand(XingSongLaserParam param)
{
  string run_state = param.run_state;

  int spin_frequency_Hz = param.spin_frequency_Hz;
  string angle_increment = param.angle_increment;
  int noise_filter_level = param.noise_filter_level;

  std::array<unsigned char, 12> set_param_command;
  set_param_command.fill(0x00);                       // 命令初始化全为0

  //----------设置数据帧头----------//
  set_param_command.at(0) = 'S';
  set_param_command.at(1) = 'C';
  set_param_command.at(2) = 't';
  set_param_command.at(3) = 'r';
  set_param_command.at(4) = 'l';

  //----------设置休眠控制----------//
  if (run_state == "run")
    set_param_command.at(5) = 0x00;
  else if (run_state == "stop")
    set_param_command.at(5) = 0x01;
  else
    std::cout << "run_state error!" << std::endl;   // 若输入非法,则为run状态

  //------------尚未配置------------//
  set_param_command.at(6) = 0x00;

  //----------设置扫描频率----------//
  if (spin_frequency_Hz == 10)
    set_param_command.at(7) = 0x00;
  else if (spin_frequency_Hz == 20)
    set_param_command.at(7) = 0x01;
  else                                               // 若输入非法,则为10kHz
  {
    set_param_command.at(7) = 0x00;
    std::cout << "spin_frequency_Hz error!" << std::endl;
  }

  //----------设置角度分辨率----------//
  if(angle_increment=="0.025")
  {
    set_param_command.at(7) = 0x00;
    set_param_command.at(8) = 0x00;
  }
  else
    if(angle_increment=="0.050")
    {
      set_param_command.at(8) = 0x01;
    }
    else
      if(angle_increment=="0.100")
      {
        set_param_command.at(8) = 0x02;
      }
      else
        if(angle_increment=="0.250")
        {
          set_param_command.at(8) = 0x03;
        }
        else
          if(angle_increment=="0.500")
          {
            set_param_command.at(8) = 0x04;
          }
          else                        // 都不符合则调到0.025/10hz
          {
            set_param_command.at(7) = 0x00;
            set_param_command.at(8) = 0x00;
          }
  //----------设置过滤等级----------//
  if ( noise_filter_level < 0 || noise_filter_level > 3)   // 输入非法，则默认为1
  {
    std::cout << "noise_filter_level error!" << std::endl;
    noise_filter_level = 1;
    set_param_command.at(9) = (unsigned char)noise_filter_level;
  }
  else
    set_param_command.at(9) = (unsigned char)noise_filter_level;

  //----------设置校验位----------//
  CRC_Verify_len12(&set_param_command);

  #ifdef DEBUG
  std::cout << "set_param_command:";
  for (auto val = set_param_command.begin(); val < set_param_command.end(); val++)
  {
      cout  << (int)*val << " ";
  }
  std::cout << std::endl;
  #endif

  return set_param_command;
}

bool LaserDataReceiver::Connect()
{
  try
  {
    // 创建hostname/ip
    boost::asio::ip::tcp::resolver resolver(io_service_);
    boost::asio::ip::tcp::resolver::query query(conn_info_.GetAddress(), std::to_string(conn_info_.GetPort()));
    boost::asio::ip::tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
    boost::asio::ip::tcp::resolver::iterator end;

    if(nullptr == tcp_socket_ptr_)
    {
      tcp_socket_ptr_ = new boost::asio::ip::tcp::socket(io_service_);
    }
    else
    {
      delete tcp_socket_ptr_;
      tcp_socket_ptr_ = new boost::asio::ip::tcp::socket(io_service_);
    }

    boost::system::error_code error = boost::asio::error::host_not_found;

    // 迭代端点并建立连接
    while (error && endpoint_iterator != end)
    {
      // tcp_socket_ptr_->close();
      tcp_socket_ptr_->connect(*endpoint_iterator++, error);
    }
    if (error)
    {
      std::cout << "Connect failed." << error.message().c_str() << std::endl;
      return false;
    }

    // 开始异步读取数据
    boost::asio::async_read(*tcp_socket_ptr_, inbuf_, boost::bind(&LaserDataReceiver::HandleSocketRead, this, boost::asio::placeholders::error));
    io_service_thread_ = boost::thread(boost::bind(&boost::asio::io_service::run, &io_service_));
    is_connected_ = true;
  }
  catch (std::exception& e)
  {
    std::cout << "Exception:" << e.what() << std::endl;
    is_connected_ = false;
    return false;
  }
  return true;
}

// 根据协议寻找数据帧头
int16_t LaserDataReceiver::FindPacketStart()
{
  if(ring_buffer_.size() < kXSPackageHeadSize)
    return -1;

  // 兴颂雷达数据的帧头
  for(size_t i = 0; i < ring_buffer_.size() - 4; i++)
  {
    if(0x48 == ((unsigned char)ring_buffer_[i])   &&
       0x49 == ((unsigned char)ring_buffer_[i+1]) &&
       0x53 == ((unsigned char)ring_buffer_[i+2]) &&
       0x4e == ((unsigned char)ring_buffer_[i+3]))
     {
      area_data_flag_ = false;
      return i;
    }
      else if(
      0x57 == ((unsigned char)ring_buffer_[i])   &&
       0x53 == ((unsigned char)ring_buffer_[i+1]) &&
       0x69 == ((unsigned char)ring_buffer_[i+2]) &&
       0x6d == ((unsigned char)ring_buffer_[i+3]) &&
       0x75 == ((unsigned char)ring_buffer_[i+4])
    )
    {
      // std::cout << "区域数据帧头" << std::endl;
      area_data_flag_ = true;
      return i;
    }
  }
  return -2;
}

// 将src数据写到 ring_buffer_ 里
void LaserDataReceiver::WriteBufferBack(char *src, std::size_t num_bytes)
{
  if(ring_buffer_.size() + num_bytes > ring_buffer_.capacity())
    throw std::exception();

  ring_buffer_.resize(ring_buffer_.size() + num_bytes);           // 修改 ring_buffer_ 的大小
  char* pone = ring_buffer_.array_one().first;                    // ring_buffer_ 的 array_one 的头指针
  std::size_t pone_size = ring_buffer_.array_one().second;        // ring_buffer_ 的 array_one 的大小
  char* ptwo = ring_buffer_.array_two().first;                    // ring_buffer_ 的 array_two 的头指针
  std::size_t ptwo_size = ring_buffer_.array_two().second;        // ring_buffer_ 的 array_two 的大小

  // 将src数据写到 ring_buffer_ 里
  if(ptwo_size >= num_bytes)
  {
    std::memcpy(ptwo + ptwo_size - num_bytes, src, num_bytes);
  }
  else
  {
    std::memcpy(pone + pone_size + ptwo_size - num_bytes, src, num_bytes - ptwo_size);
    std::memcpy(ptwo, src + num_bytes - ptwo_size, ptwo_size);
  }
}

bool LaserDataReceiver::HandleNextPacket()
{
  if(scan_data_.empty())
  {
    scan_data_.emplace_back();
  }

  if(RetrivePacket())
  {
    return true;
  }
  else
  {
    return false;
  }

}


bool LaserDataReceiver::RetrivePacket()
{
  bool ret = false;
  int16_t head_index = FindPacketStart();                     // 寻找帧头
  if(head_index < 0)
    return ret;

  if(area_data_flag_)
  {
    return GetAreaData(head_index);
  }
  else
    return GetRangeData(head_index);
}

bool LaserDataReceiver::GetAreaData(int16_t head_index)
{
  bool ret = false;
  // 寻找帧头并处理数据
if(ring_buffer_.size() - head_index >= kHSAreaDataPackageSize)
  {
    ring_buffer_.erase_begin(head_index);                     // 删除 帧头前的数据
    head_index = 0;

    // 获取帧
    char data_buf[kHSAreaDataPackageSize];
    ReadBufferFront(data_buf, kHSAreaDataPackageSize);
    have_block_ = int(data_buf[7]);
    _now_channel_ = int(data_buf[5]);

    #ifdef DEBUG
    // if(data_buf[7]==0x00)           // 00为检测到物体
    //   have_block_ = true;
    // else
    //   have_block_ = false;

    // std::cout << "当前通道值：" << int(data_buf[5]) << " "
    //                    << "输出状态：" << int(data_buf[7]) << " "
    //                    << "have_block_:" << have_block_ << std::endl;

    

    #endif


    
    ret = true;
    ring_buffer_.erase_begin(head_index + kHSAreaDataPackageSize);       // 删除 ring_buffer_ 的数据
    return ret;
  }
  return ret;    
}

/**
 * @brief  对雷达数据帧进行解析
 */
bool LaserDataReceiver::GetRangeData(int16_t head_index)
{
  bool ret = false;
  // 寻找帧头并处理数据
  if(ring_buffer_.size() - head_index >= kXSPackageHeadSize)
  {
    ring_buffer_.erase_begin(head_index);                     // 删除 帧头前的数据
    head_index = 0;

    // 1. 解析数据帧帧头
    char head_buf[kXSPackageHeadSize];
    ReadBufferFront(head_buf, kXSPackageHeadSize);

    XSPackageHeader header;
    header.start_angle = (unsigned char)head_buf[4] << 8;     // 起始角度
    header.start_angle |= (unsigned char)head_buf[5];
    header.end_angle = (unsigned char)head_buf[6] << 8;       // 终止角度
    header.end_angle |= (unsigned char)head_buf[7];
    header.data_size = (unsigned char)head_buf[8] << 8;       // 测量点总数
    header.data_size |= (unsigned char)head_buf[9];
    header.data_position = (unsigned char)head_buf[10] << 8;  // 当前帧测量点位置
    header.data_position |= (unsigned char)head_buf[11];
    header.measure_size = (unsigned char)head_buf[12] << 8;   // 当前帧测量点数量
    header.measure_size |= (unsigned char)head_buf[13];
    header.time = (unsigned char)head_buf[14] << 8;           // 当前帧测量时间戳
    header.time |= (unsigned char)head_buf[15];
    if(header.data_size > header.measure_size)
    {
      header.data_size = header.measure_size;
    }

    if(header.start_angle==20 && laser_steady_time > 0 || scan_all_point_num_init_flag)         // 前几帧数据异常
    {
      // 计算角度分辨率
      angle_increment_ = double(header.end_angle - header.start_angle)/header.measure_size;
      scan_all_point_num_ = 320.0f/angle_increment_;
      scan_all_point_num_init_flag = false;
      // rec_begin_flag_ += 1;
      laser_steady_time--;
      // std::cout << angle_increment_ << "    " << scan_all_point_num_;
    }
    // std::cout << "angle_increment_:" << angle_increment_ << "  scan_all_point_num:" << scan_all_point_num_ << std::endl;
    std::unique_lock<std::mutex> lock(scan_mutex_);

    // 2. 解析帧内容
    ScanData& scan_data = scan_data_.back();
    scan_data.distance_data.resize(scan_all_point_num_);
    scan_data.amplitude_data.resize(scan_all_point_num_);
    
    uint16_t body_size = kXSPackageHeadSize + header.data_size * 4;
    if((ring_buffer_.size() - head_index) >= body_size)
    {
      #ifdef DEBUG
      std::cout << "begin:" << header.start_angle 
                << "  end:" << header.end_angle 
                << "  data_size:" << header.data_size
                << "  data_position:" << header.data_position
                << "  measure_size:" << header.measure_size
                << "  time:" << header.time
                << "  laser_steady_time:" << laser_steady_time
                << std::endl;
      #endif

      char* body_buf = new char[body_size];
      ReadBufferFront(body_buf, body_size);                   // 将当前帧的数据复制到 body_buf 中

      ring_buffer_.erase_begin(head_index + body_size);       // 删除 ring_buffer_ 的数据


      short int begin_point_index = 0;

      // 计算本帧第一点的索引值
      begin_point_index = (header.start_angle-20)/angle_increment_;
      if(scan_all_point_num_ < begin_point_index+header.data_size)
      {
        scan_data.distance_data.resize(begin_point_index+header.data_size);
        scan_data.amplitude_data.resize(begin_point_index+header.data_size);
      }
      for(int i = 0; i < header.data_size; i++)
      {
        unsigned short int distance;
        unsigned short int intensity;
        distance = (unsigned char)body_buf[i*4 + 17] * 256;
        distance |= (unsigned char)body_buf[i*4 + 16];
        intensity = (unsigned char)body_buf[i*4 + 19] * 256;
        intensity |= (unsigned char)body_buf[i*4 + 18];

        if(distance > kMaxDistance)
        {
          distance = kMaxDistance+10000;
        }
        if(intensity > kMaxIntensity)
        {
          intensity = kMaxIntensity;
        }

        scan_data.distance_data.at(begin_point_index+i) = distance;
        scan_data.amplitude_data.at(begin_point_index+i) = intensity;
      }

      delete [] body_buf;

      if(header.start_angle == 20)                             // 第一帧的时间戳存到scan_data.time_increment里
        scan_data.time_increment=float(header.time);

      // 接收完一圈数据
      else if(header.end_angle == 340 && header.data_position == header.measure_size)
      {
        if(header.time - scan_data.time_increment < 0)        // 将一圈的时间放进scan_data.time_increment里
          scan_data.time_increment = 65535 - scan_data.time_increment + header.time;
        else
          scan_data.time_increment = header.time - scan_data.time_increment;

        scan_data_.emplace_back();
        if(scan_data_.size() > 5)
        {
          scan_data_.pop_front();
          std::cout << "buffer data too many, drops it" << std::endl;
        }
        data_notifier_.notify_one();
        last_data_time_ = hins::Now();
        ret = true;
      }

      if(FindPacketStart() >= 0)
      {
        ret = true;
      }
    }
  }

  return ret;
}

// 将ring_buffer_前num_bytes的数据拷贝到dst
void LaserDataReceiver::ReadBufferFront(char *dst, const uint16_t &num_bytes)
{
  if( ring_buffer_.size() < num_bytes )
    throw std::exception();

  char* pone = ring_buffer_.array_one().first;                  // 指向环形缓冲区ring_buffer_的array_one的开始地址
  std::size_t pone_size = ring_buffer_.array_one().second;      // ring_buffer_的array_one的大小
  char* ptwo = ring_buffer_.array_two().first;                  // 指向环形缓冲区ring_buffer_的array_two的开始地址

  if( pone_size >= num_bytes )                                  // ring_buffer_的array_one的大小大于num_bytes
  {
    std::memcpy(dst, pone, num_bytes);                          // 复制环形缓冲区的数据到dst中
  }
  else
  {
    std::memcpy(dst, pone, pone_size);                          // 复制环形缓冲区的array_one数据到dst中
    std::memcpy(dst + pone_size, ptwo, num_bytes - pone_size);  // 复制环形缓冲区的array_two数据到dst中
  }
}

}
