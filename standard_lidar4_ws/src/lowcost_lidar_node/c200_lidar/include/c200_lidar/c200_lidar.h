#ifndef C200_LIDAR_H_
#define C200_LIDAR_H_

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <ros/ros.h>
#include <string>
#include <stdint.h>
#include <c200_lidar/c200_structs.h>
#include <sensor_msgs/LaserScan.h>
#include <deque>

#define START_FLAG             0x02020202
#define RSP_MEASUREMENT_DATA   0x1232

using namespace std;

using namespace free_optics;

class C200
{
public:
    C200();
    virtual ~C200();

    void connect(std::string host, int port = 2111);

    void disconnect();

    bool isConnected();

    void login();

    void getDeviceID();

    void getSerialNumber();

    void getFirmwareVersion();

    status_t getDeviceState();

    void getScanAngle();

    void scanContinous(uint8_t state);

    bool getScanData();

    void saveConfig();
	void filterHandle(std::vector<float>, std::vector<float>, int);

    void SetTimeStamp(bool);
    
    //add by zhurui 2022/10/10
    void SetReflectSwitch(uint8_t reflectflag);

    bool GetNORSwitchFlag();

    std::string host_;
    int port_;
    bool filterswitch_;
    bool NTPswitch_;
    //add by zhurui 2022/10/10
    bool NORswitch_;

protected:
    bool connected_;
    int socket_fd_;
    uint8_t buffer[65536];
    std::deque<uint8_t> data;

private:
    uint16_t byteExchange(uint16_t value);

    ros::NodeHandle nh_, nh_private_;
    ros::Publisher laser_scan_publisher_;
    sensor_msgs::LaserScan laser_scan;
    sensor_msgs::LaserScan last_scan;
    bool flag_is_first = false;
    std::string frame_id_;
    double angle_min_;
    double angle_max_;
    double range_min_;
    double range_max_;
    int offset_angle_;
};

#endif
