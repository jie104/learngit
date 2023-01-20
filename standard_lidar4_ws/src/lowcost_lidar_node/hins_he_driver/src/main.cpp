#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "hins/xingsong_driver.h"
#include "hins_he_driver/hins_srv.h"
#include <time.h>
#include <sys/time.h>

unsigned long long last_system_time_stamp = 0;
using namespace hins;

HSGetAreaDataPackage Command;     // 全局变量，获取区域数据指令
bool Area1, Area2,Area3;                   // 三个区域的bool值
int NowChannel;
bool y_direction;
// 声明函数
sensor_msgs::LaserScanPtr ToLaserScan(const ScanData &data, float start_angle, float end_angle, std::string frame_name);
void ParamInit(ros::NodeHandle &nh_private ,ConnectionAddress &laser_conn_info, XingSongLaserParam &laser_param, ShadowsFilterParam &shadows_filter_param,  DisturbFilterParam &disturb_filter_param);
bool HinsAreaDataCallback(hins_he_driver::hins_srv::Request  &req, hins_he_driver::hins_srv::Response &res);

int main(int argc, char** argv){

  // 1. ROS 初始化
  ros::init(argc, argv, "hins_driver_laser");
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private("~");

  float start_angle, end_angle;
  std::string topic_name, frame_name, service_name;
  // 角度过滤参数
  nh_private.param<float>("start_angle", start_angle, 0.);
  nh_private.param<float>("end_angle", end_angle, 2 * M_PI);
  nh_private.param<std::string>("topic_name", topic_name, std::string("scan1"));
  nh_private.param<std::string>("frame_name", frame_name, std::string("laser1"));
  nh_private.param<std::string>("service_name", service_name, std::string("hins_he_area_data"));
  nh_private.param<bool>("y_direction", y_direction, true);

  ROS_INFO("advertise: %s", topic_name.c_str());
  ros::Publisher scan_pub = nh_.advertise<sensor_msgs::LaserScan>(topic_name, 3, true);

  float start_angle_rad, end_angle_rad;     // 角度过滤参数-弧度制
  start_angle_rad = start_angle*M_PI /180.0f;
  end_angle_rad = end_angle*M_PI /180.0f;
  // 2. 雷达驱动
  ConnectionAddress laser_conn_info;
  XingSongLaserParam laser_param;
  ShadowsFilterParam shadows_filter_param;
  DisturbFilterParam disturb_filter_param;
  // 雷达参数
  laser_param.run_state = "run";
  // 初始化参数
  ParamInit(nh_private, laser_conn_info, laser_param, shadows_filter_param, disturb_filter_param);

  // 启动雷达
  XingSongDriverHdr driver_hdr = std::make_shared<XingSongDriver>(laser_conn_info,laser_param,shadows_filter_param, disturb_filter_param);
  
  // 创建服务器
  ros::ServiceServer hins_service = nh_.advertiseService(service_name, HinsAreaDataCallback);
  // 区域数据
  
  Command.angle = 0;
  Command.channel = 0;                     // 通道指定模式使用  
  Command.channel_group = 2;      // 智能通道选择模式使用
  Command.speed = 0;
  Command.mode = 1;

  
  // 3. ros 消息发布
  ros::Rate r(50);
  while(ros::ok()){
    ScanData data = driver_hdr->GetFullScan();
    driver_hdr->SetAreaCommand(Command);          // 设置指定通道
    sensor_msgs::LaserScanPtr scan_msg_ptr = ToLaserScan(data, start_angle_rad, end_angle_rad, frame_name);
    
    if(scan_msg_ptr->ranges.size() != 0)            // 如果数据长度有误，则不发布
    {  
      // 发布雷达数据
      scan_pub.publish(scan_msg_ptr);
    }
    // 获取避障数据
    Area1 = driver_hdr->GetBlock1Value();
    Area2 = driver_hdr->GetBlock2Value();
    Area3 = driver_hdr->GetBlock3Value();
    NowChannel = driver_hdr->GetResponseChannel();
    // std::cout  << "Area1: " << Area1 << " "
    //                     << "Area2: " << Area2 << " "
    //                     << "Area3: " << Area3 << " "
    //                     << std::endl;
    ros::spinOnce();
    r.sleep();
  }
}

sensor_msgs::LaserScanPtr ToLaserScan(const ScanData &data, float start_angle, float end_angle, std::string frame_name)
{
  sensor_msgs::LaserScanPtr ret = boost::make_shared<sensor_msgs::LaserScan>();

  // ros帧头
  ret->header.frame_id = frame_name;
  // ret->header.stamp = ros::Time::now() - ros::Duration(data.time_increment*1e-6);
  ret->range_min = 0.05;
  ret->range_max = 35.0;

  if(y_direction)    // y轴向前
  {  
    ret->angle_min = -0.3888888889 * M_PI;
    ret->angle_max = 1.3888888889 * M_PI;

  }
  else              // x轴向前
  {
    ret->angle_min = -0.888888889 * M_PI;
    ret->angle_max = 0.888888889 * M_PI;
  }
      // ret->time_increment = (float(data.time_increment)/1000000) / data.distance_data.size();
    // ret->time_increment = (float(data.time_increment)*1.e-6) / data.distance_data.size();
  // 系统时间
  unsigned long sec, nsec;  
  unsigned long long system_time_stamp;
  struct timeval timeofday;
  gettimeofday(&timeofday,NULL);
  sec  = timeofday.tv_sec;
  nsec = timeofday.tv_usec * 1000;
  system_time_stamp = (unsigned long long)sec*1000000000ll + (unsigned long long)nsec;// - data.time_increment*1000;
  // ret->header.stamp.fromNSec((uint64_t) (system_time_stamp-(system_time_stamp - last_system_time_stamp)*320.0f/360.0f)); //*320.0f/360.0f
  ret->header.stamp = ros::Time::now() - ros::Duration((system_time_stamp - last_system_time_stamp)*320.0f*1.e-9/360.0f);

  // 雷达测量一周所需的时间/测量点数  
  ret->time_increment = static_cast<float>(system_time_stamp - last_system_time_stamp)*1.e-9*320.0f/ 360.0f / static_cast<float>(data.distance_data.size()-1);
  last_system_time_stamp = system_time_stamp;
    // std::cout //<< "data.time_increment :" << (float(data.time_increment) / data.distance_data.size())*1.e-6 << "    "
    //                  << "ret->time_increment:" << ret->time_increment << "\t"
    //                << "data.time_increment(ms):" << data.time_increment*1e-3 << "\t"
    //               //  << "data.distance_data.size:" <<data.distance_data.size() << "   "
    //                << "systime_delta(ms):" << (system_time_stamp - last_system_time_stamp)*1.e-6 << "\t"
    //                << std::endl;

  ret->angle_increment = 1.777778 * M_PI/float(data.distance_data.size());

  ret->ranges.resize(data.distance_data.size(), 0);
  ret->intensities.resize(data.amplitude_data.size(), 0);
  
  for( std::size_t i = 0; i < data.distance_data.size(); i++ )
  {
    float angle = 20*M_PI/180.0f + i * ret->angle_increment;
  
    float dis = float(data.distance_data[i])/1000.0f;
    
    if(angle < start_angle || angle > end_angle)
      ret->ranges[i]  = 1024.0;
      // ret->ranges[i]  = std::numeric_limits<float>::infinity();
    else
      ret->ranges[i] = dis;
    ret->intensities[i] = data.amplitude_data[i];
  }

  return ret;
}

/**
 * @brief 读取launch文件参数并将其赋值到输入参数中
 * @param[in] nh_private ros node_handle
 * @param[in] laser_conn_info 雷达连接参数
 * @param[in] laser_param 雷达配置参数
 * @param[in] shadows_filter_param 拖尾过滤参数
 * @param[in] disturb_filter_param  串扰过滤参数
 * @return null
 */
void ParamInit(ros::NodeHandle &nh_private ,ConnectionAddress &laser_conn_info, XingSongLaserParam &laser_param, ShadowsFilterParam &shadows_filter_param,  DisturbFilterParam &disturb_filter_param)
{
  std::string server_address, angle_increment;
  int port, measure_frequency, shadows_filter_neighbors, shadows_filter_window, shadows_filter_level, shadows_traverse_step, noise_filter_level, spin_frequency;
  float start_angle, end_angle, shadows_filter_max_angle, shadows_filter_min_angle;
  bool change_flag, block_enable, disturb_filter_enable;
  int disturb_filter_threshold, disturb_filter_point_num;
  
  // 雷达连接参数
  nh_private.param("laser_ip", server_address, std::string("192.168.1.88"));
  nh_private.param<int>("laser_port", port, 8080);

  // 雷达扫描参数
  //nh_private.param<int>("measure_frequency_kHz", measure_frequency, int("-1"));
  nh_private.param<int>("spin_frequency_Hz", spin_frequency, -1);
  nh_private.param<std::string>("angle_increment", angle_increment, std::string("-1"));
  nh_private.param<int>("noise_filter_level", noise_filter_level, -1);
  nh_private.param<bool>("change_param", change_flag, false);
  nh_private.param<bool>("block_enable", block_enable, false);
  //防拖尾参数
  nh_private.param<float>("shadows_filter_max_angle", shadows_filter_max_angle, float(175.0));
  nh_private.param<float>("shadows_filter_min_angle", shadows_filter_min_angle, float(5.0));
  nh_private.param<int>("shadows_filter_neighbors", shadows_filter_neighbors, 1);
  nh_private.param<int>("shadows_filter_window", shadows_filter_window, 2);
  nh_private.param<int>("shadows_traverse_step", shadows_traverse_step, 1);
  nh_private.param<int>("shadows_filter_level", shadows_filter_level, -1);

  // 串扰过滤参数
  nh_private.param<int>("disturb_filter_threshold", disturb_filter_threshold, 80);       //juli
  nh_private.param<int>("disturb_filter_point_num", disturb_filter_point_num, 8);
  nh_private.param<bool>("disturb_filter_enable", disturb_filter_enable, false);

  // 雷达连接参数
  laser_conn_info.SetAddress(server_address);
  laser_conn_info.SetPort(port);

  // 雷达扫描参数
  laser_param.change_flag = change_flag;
  laser_param.spin_frequency_Hz = spin_frequency;
  laser_param.angle_increment = angle_increment;
  laser_param.noise_filter_level = noise_filter_level;

  // 防拖尾参数
  shadows_filter_param.max_angle = shadows_filter_max_angle;
  shadows_filter_param.min_angle = shadows_filter_min_angle;
  shadows_filter_param.neighbors = shadows_filter_neighbors;
  shadows_filter_param.window = shadows_filter_window;
  shadows_filter_param.traverse_step = shadows_traverse_step;
  shadows_filter_param.shadows_filter_level = shadows_filter_level;

  // 串扰过滤参数
  disturb_filter_param.threshold = disturb_filter_threshold;
  disturb_filter_param.range = disturb_filter_point_num;
  disturb_filter_param.disturb_filter_enable = disturb_filter_enable;
}


bool HinsAreaDataCallback(hins_he_driver::hins_srv::Request  &req, hins_he_driver::hins_srv::Response &res)
{
  Command.channel = req.channel;
  if(NowChannel == req.channel)                 //  判断请求的通道与现在的通道是否相同，是则success为true
    res.success = true;
  else
    res.success = false;
  
  res.area1 = Area1;
  res.area2 = Area2;
  res.area3 = Area3;
  return true;
}