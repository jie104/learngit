#include "hins/xingsong_driver.h"
#include <ros/ros.h>
// #define DEBUG

namespace hins {

XingSongDriver::XingSongDriver(const ConnectionAddress &conn_info,hins::XingSongLaserParam param)
  : conn_info_(conn_info)
  , data_receiver_ptr_(nullptr)
  , laser_param(param)
  , have_block_(false)
{
  guard_thread_ = std::thread(&XingSongDriver::RunMain, this);
  shadows_filter_param_.max_angle = -1.0;
}

XingSongDriver::XingSongDriver(const ConnectionAddress &conn_info, XingSongLaserParam param, ShadowsFilterParam shadows_filter_param, DisturbFilterParam disturb_filter_param)
  : conn_info_(conn_info)
  , data_receiver_ptr_(nullptr)
  , laser_param(param)
  , shadows_filter_param_(shadows_filter_param)
  , have_block_(0)
  , now_channel_(-1)
  , disturb_filter_param_(disturb_filter_param)
{ 
  HSGetAreaDataPackage command;
  command.angle = 0;
  command.channel = 1;
  command.channel_group = 2;      // 智能模式的通道
  command.speed = 0;
  command.mode = 1;
  block_enable_ = param.block_enable;
  SetAreaCommand(command);
  guard_thread_ = std::thread(&XingSongDriver::RunMain, this);
}

XingSongDriver::~XingSongDriver()
{
  if(data_receiver_ptr_)
  {
    delete data_receiver_ptr_;
  }
}

bool XingSongDriver::StartCapturingTCP()
{
  if(data_receiver_ptr_)
  {
    delete data_receiver_ptr_;
  }

  data_receiver_ptr_ = new LaserDataReceiver(conn_info_);
  if(!data_receiver_ptr_->IsConnected())
  {
    return false;
  }
  if(CheckLaserParam(laser_param))
  {
    data_receiver_ptr_->SyncWrite(data_receiver_ptr_->GenerateParamCommand(laser_param));
    // this_thread::sleep_for(chrono::milliseconds(3000));                                             //睡眠1秒
  }
  
  if(data_receiver_ptr_->SyncWrite() > 0)
  {
    return true;
  }
  return false;
}

/**
 * @brief  检查laser_param是否有问题
 * @param[in] laser_param 雷达配置参数  
 * @return 参数有问题则返回false，否则返回true
 */
bool XingSongDriver::CheckLaserParam(XingSongLaserParam& laser_param)
{
  if(!laser_param.change_flag) // 如果change_flag为false，则返回false
    return false;
  if(laser_param.spin_frequency_Hz == 10 || laser_param.spin_frequency_Hz == 20)
    if(laser_param.noise_filter_level >= 0 && laser_param.noise_filter_level <= 3)
      if(laser_param.angle_increment == "0.025" || laser_param.angle_increment == "0.050"
          || laser_param.angle_increment == "0.100" || laser_param.angle_increment == "0.250"
          || laser_param.angle_increment == "0.500")
      {
        laser_param.change_flag = true;
        return true;
      }
  laser_param.change_flag = false;
  ROS_WARN("laser_param error!");
  return false;
}

/**
 * @brief  拖尾过滤参数初始化
 * @param[in] scan_num 激光点数
 * @return null
 */
void XingSongDriver::ShadowsFilterInit(int scan_num)
{
  shadows_filter_threshold_max_.clear();
  shadows_filter_threshold_min_.clear();

  float max, min, min_angle, max_angle, angle_increment;
  angle_increment = 0.888889*2*M_PI/(float)scan_num;
  int window;

  switch (shadows_filter_param_.shadows_filter_level)
  {
    case 1:                                         // 快速，筛选角度大,搜索窗口小
      min_angle = M_PI*5.0f/180.0f;                 // 转换为弧度制
      max_angle = M_PI*175.0f/180.0f;               // 转换为弧度制
      window = 1;
      shadows_filter_param_.window = window;
      break;

    case 2:                                         // 较慢，筛选角度大，搜索窗口适中
      min_angle = M_PI*5.0f/180.0f;                 // 转换为弧度制
      max_angle = M_PI*175.0f/180.0f;               // 转换为弧度制
      window = 3;
      shadows_filter_param_.window = window;
      break;

    case 3:                                         // 较慢，筛选角度较小,搜索窗口大
      min_angle = M_PI*15.0f/180.0f;                // 转换为弧度制
      max_angle = M_PI*165.0f/180.0f;               // 转换为弧度制
      window = 5;
      shadows_filter_param_.window = window;
      break;
    
    default:                                                           // 按照shadows_filter_param_配置
      min_angle = M_PI*shadows_filter_param_.min_angle/180.0f;         // 转换为弧度制
      max_angle = M_PI*shadows_filter_param_.max_angle/180.0f;         // 转换为弧度制
      window = shadows_filter_param_.window;

      break;
  }
#ifdef DEBUG
  std::cout << "shadows_filter_level:" << shadows_filter_param_.shadows_filter_level
      << "  min_angle:" << shadows_filter_param_.min_angle
      << "  max_angle:" << shadows_filter_param_.max_angle
      << std::endl;
  std::cout << "max:";
#endif
  for(int i=0;i < window;i++)
  {
    max = sin(max_angle)/sin(M_PI-max_angle-angle_increment*(i+1));
    min = sin(min_angle)/sin(M_PI-min_angle-angle_increment*(i+1));

    shadows_filter_threshold_max_.push_back(max);
    shadows_filter_threshold_min_.push_back(min);
    #ifdef DEBUG
    std::cout << shadows_filter_threshold_max_.back() << "  ";
    #endif
  }
}


/**
 * @brief 串扰过滤算法
 * @param[in] scan_data 雷达数据
 * @param[in] disturb_param 串扰过滤参数
 * @return null
 */
void XingSongDriver::DisturbFilter(ScanData& scan_data, DisturbFilterParam disturb_param)
{
  if(!disturb_param.disturb_filter_enable)  // 判断是否需要过滤
    return;
  int scan_num = int(scan_data.distance_data.size());
  int index_max = int(scan_data.distance_data.size()) - disturb_param.range-2;
  disturb_del_index_.clear();
  
  int begin_index = 0;
  int end_index = 0;
  uint32_t sub_res = 0;
  uint32_t deviation = disturb_param.threshold; //偏差
  int point_num =  disturb_param.range; // 点数
  // std::cout<< "point_num:" << point_num;
   std::vector<int> disturb_del_vector;
  for(int i =0; i < scan_num-1; ++i)   // 遍历每个点
  {
    if(scan_data.distance_data[i] > scan_data.distance_data[i+1])
      sub_res = scan_data.distance_data[i] - scan_data.distance_data[i+1];
    else
      sub_res = scan_data.distance_data[i+1] - scan_data.distance_data[i];
    
    // 大于偏差则检查
    if(sub_res > deviation)
    {
       if(i - begin_index > point_num)     // 正常
        ; // 不操作
      else
      {
        for(int tmp = begin_index; tmp <= i+1; ++tmp)
          {
            disturb_del_index_.insert(tmp);                   // 将begin_index到end_index的点设为最大值
            disturb_del_vector.push_back(tmp);
          }
      }
      begin_index = i; // 更新
    }
  }

  if(disturb_del_vector.size()==0)
    return;
  for(int i = 0; i<disturb_del_vector.size(); ++i)
  {
    scan_data.distance_data[disturb_del_vector[i]] = kMaxDistance + 1;
  }

}


/**
 * @brief  拖尾过滤算法
 * @param[in] scan_data 激光雷达数据
 * @param[in] scan_num 雷达数据点数
 * @return null
 */
void XingSongDriver::ShadowsFilter(ScanData& scan_data, int scan_num)
{
  #ifdef DEBUG
  int del_num = 0;
  #endif
  if(GetLaserSteadyTime() > 0)
  {
    ShadowsFilterInit(scan_num);
  }
  int search_index, search_index_tmp, del_index, del_index_tmp;
  float a_b_rate;

  shadows_del_index.clear();
  // 每traverse_step根激光计算一次
  for (int i = 0; i < scan_num; i += shadows_filter_param_.traverse_step)
  {
    if(scan_data.distance_data[i] > kMaxDistance)                        // 如果search_index_tmp激光超出范围，则跳过
      continue;

    for(search_index = i+1;                                              // 搜索[i,i+window]内是否存在拖影现象
        search_index <= i + shadows_filter_param_.window;
        search_index++)
    {
      search_index_tmp = search_index;  

      // 环形索引优化                                                         
      if(search_index_tmp < 0)                                                                        
        search_index_tmp = 0;
      else
        if(search_index_tmp >= scan_num)
          search_index_tmp = scan_num - 1;

      if( (scan_data.distance_data[search_index_tmp] > kMaxDistance) ||                               // 如果search_index_tmp激光超出范围，则跳过
          (scan_data.distance_data[i] > kMaxDistance)                  )
        continue;

      a_b_rate = (float)scan_data.distance_data[i]/(float)scan_data.distance_data[search_index_tmp];  // i与search_index激光比较
      if( (a_b_rate < shadows_filter_threshold_min_[abs(search_index_tmp-i-1)]) ||                      // 如果存在拖影现象
          (a_b_rate > shadows_filter_threshold_max_[abs(search_index_tmp-i-1)])   )        
        if (scan_data.distance_data[i] < scan_data.distance_data[search_index_tmp])
        {
          #ifdef DEBUG
          del_num++;
          #endif
          shadows_del_index.insert(search_index_tmp);
        }
        else
        {
          #ifdef DEBUG
          del_num++;
          #endif
          shadows_del_index.insert(i);
        }
    }
  }
  for (auto iter = shadows_del_index.begin(); iter != shadows_del_index.end(); ++iter) 
  {
    // cout << *iter << "\t"<< endl;
    scan_data.distance_data[*iter] = kMaxDistance + 1;
  }
  #ifdef DEBUG
  std::cout << "del_num:" << del_num << std::endl;
  #endif
}

ScanData XingSongDriver::GetFullScan()
{
  if (data_receiver_ptr_)
  {
    if (shadows_filter_param_.max_angle < 0 || shadows_filter_param_.shadows_filter_level == 0)
      return data_receiver_ptr_->GetFullScan();
    ScanData fullscan = data_receiver_ptr_->GetFullScan();
    int scan_num = fullscan.distance_data.size();
    ShadowsFilter(fullscan, scan_num);                      // 过滤拖尾点
    have_block_ = GetBlockValue();
    RefreshChannel();
    block1_ = have_block_&0x01;
    block2_ = have_block_&0x02;
    block3_ = have_block_&0x04;
    #ifdef DEBUG
    std::cout << "block1: " << block1_ << " "
                       << "block2: " << block2_ << " "
                       << "block3:" << block3_ << " "
                       << "have_block_:" << have_block_ << " "
                       << std::endl;
    #endif
    return fullscan;
  }
  else
    return ScanData();
}


/**
 * @brief  根据输入参数设置获取避障数据命令, 改变get_area_data_command_
 * @param[in] command 激光雷达数据
 * @return null
 */
void XingSongDriver::SetAreaCommand(HSGetAreaDataPackage command)
{  
  /**********帧头**********/
  get_area_data_command_.at(0) = 0x57;
  get_area_data_command_.at(1) = 0x53;
  get_area_data_command_.at(2) = 0x69;
  get_area_data_command_.at(3) = 0x6d;
  get_area_data_command_.at(4) = 0x75;

 
  /**********传感器工作模式**********/
  get_area_data_command_.at(5) = command.mode;

  /**********传感器通道值**********/
    if(command.channel > 63 | command.channel < 0)
    command.channel = 0;
  get_area_data_command_.at(6) = command.channel;

  /**********通道角度**********/
  if(command.angle > 180 | command.angle < -180)
    command.angle = 0;
  get_area_data_command_.at(7) = (command.angle& 0xff00) >> 8;       // 高位在前
  get_area_data_command_.at(8) = (command.angle& 0x00ff);                // 低位在后

  /**********通道速度**********/
    if(command.speed > 300 | command.speed < -300)
    command.speed = 0;
  get_area_data_command_.at(9) = (command.speed& 0xff00) >> 8;       // 高位在前
  get_area_data_command_.at(10) = (command.speed& 0x00ff);                // 低位在后

  /**********传感器通道组号**********/
  if(command.channel_group > 4 | command.channel_group < 0)
    command.channel_group = 0;
  get_area_data_command_.at(11) = (command.channel_group& 0xff00) >> 8;       // 高位在前
  get_area_data_command_.at(12) = (command.channel_group& 0x00ff);                // 低位在后

  /**********预留位**********/
  get_area_data_command_.at(13) = 0x00;
  get_area_data_command_.at(14) = 0x00;
  get_area_data_command_.at(15) = 0x00;

  short int check = CRCVerify(&get_area_data_command_,18);
  /**********校验位**********/
  get_area_data_command_.at(16) = (check& 0x00ff);                // 低位在前
  get_area_data_command_.at(17) = (check& 0xff00) >> 8;       // 高位在后
}

/**
 * @brief  对命令进行crc校验
 * @param[in] command 激光雷达数据
 * @param[in] len 命令总长度(含校验位)
 * @return null
 */
short int XingSongDriver::CRCVerify(std::array<char,18> *command, int len)
{
  int i, j;
  unsigned int wCrc = 0xffff;
  unsigned int wPolynom = 0xA001;
  /*-------------------------------------*/
  for (i = 0; i < len - 2; i++) // 后两位为校验位
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
  // unsigned int tmp1 = (wCrc & 0xff00) >> 8;           // 高位
  // unsigned int tmp2 = (wCrc & 0x00ff);                // 低位
  // command->at(10) = tmp2;
  // command->at(11) = tmp1;
  return wCrc;
}

void XingSongDriver::RunMain()
{
  if(StartCapturingTCP())
  {
    std::cout << "兴颂雷达启动成功." << std::endl;
  }
  else
  {
    std::cout << "兴颂雷达启动失败，请检查网络连接." << std::endl;
  }

  while(ros::ok()) // @todo: stop loop condition need to modified
  {
    if(IsConnected())
    {
      if(block_enable_)
        SendAreaCommand();
      #ifdef DEBUG
      std::cout << "send command: ";
      
      int i ;
      for (i = 0; i < 18; i++)
        std::cout << int(get_area_data_command_.at(i)) << " ";
      std::cout << std::endl;
      #endif
    }
    else
    {
      while(!IsConnected()){
        std::cout << "激光雷达断线，正在重新连接 ... " << std::endl;
        if(StartCapturingTCP()){
          std::cout << "激光雷达重新连接成功." << std::endl;
        }
        usleep(1000 * 50); // 50ms
      }
    }

    usleep(1000*25); // 25ms
  }

  Disconnect();
}

}
