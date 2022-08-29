#ifndef WJ_716_LIDAR_PROTOCOL_H
#define WJ_716_LIDAR_PROTOCOL_H
#include <iostream>
#include "string.h"
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/asio/placeholders.hpp>
#include <boost/system/error_code.hpp>
#include <boost/bind/bind.hpp>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>
#include <dynamic_reconfigure/server.h>
#include <wj_716_lidar/wj_716_lidarConfig.h>
using namespace std ;
namespace wj_lidar
{
  #define MAX_LENGTH_DATA_PROCESS 200000
  typedef struct TagDataCache
  {
    unsigned char m_acdata[MAX_LENGTH_DATA_PROCESS];
    unsigned int m_u32in;
    unsigned int m_u32out;
  }DataCache;

  class wj_716_lidar_protocol
  {
  public:
    wj_716_lidar_protocol();
    bool dataProcess(unsigned char *data,const int reclen);
    bool protocl(unsigned char *data,const int len);
    bool OnRecvProcess(unsigned char *data, int len);
    bool checkXor(unsigned char *recvbuf, int recvlen);
    void send_scan(const char *data,const int len);
    ros::NodeHandle nh;
    ros::Publisher marker_pub;
    ros::Publisher twoecho_;
    sensor_msgs::LaserScan scan;
    sensor_msgs::LaserScan scan_TwoEcho;
    bool setConfig(wj_716_lidar::wj_716_lidarConfig &new_config,uint32_t level);
    string DeviceMAC;
    bool heartstate;

  private:
    unsigned char        data_[MAX_LENGTH_DATA_PROCESS];
    DataCache   m_sdata;
    wj_716_lidar::wj_716_lidarConfig config_;
    unsigned int g_u32PreFrameNo;
    float scandata[811];
    float scandata_te[811];
    float scaninden[811];
    int resizeNum;
    int index_start;
    int index_end;
  };

}
#endif // WJ_716_LIDAR_PROTOCOL_H
