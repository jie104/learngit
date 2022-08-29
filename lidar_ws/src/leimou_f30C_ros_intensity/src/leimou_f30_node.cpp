/*********************************************************
 * leimou_f30_node.cpp
 * Author: wuji228@163.com
 * Date:2018.10
 **********************************************************/

#include "leimou_f30_driver.h"

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <limits>
#include <string>

#define DEG2RAD(x) ((x)*M_PI/180.)
int start_angle, stop_angle;

void publish_scan(ros::Publisher *pub,
				  const ScanCfg& scan_cfg, 
                  const ScanData& scan_data,
                  ros::Time start, 
                  double scan_time, 
                  bool inverted,
                  float angle_min, 
                  float angle_max,
                  float range_min,
                  float range_max,
                  std::string frame_id)
{
    static int scan_count = 0;
	int angle_index = 0;
    float read_value = 0;
    // laser msgs.
    sensor_msgs::LaserScan scan_msg;

    scan_msg.header.stamp = start;
    scan_msg.header.frame_id = frame_id;
    scan_count++;

    bool reversed = (angle_min > angle_max);
    if (reversed)   
    {
        scan_msg.angle_min =  angle_max - M_PI/2.0;   // M_PI*3.0/4.0
        scan_msg.angle_max =  angle_min - M_PI/2.0;
    } 
    else // regular angle
    {
        scan_msg.angle_min =  angle_min - M_PI/2.0;
        scan_msg.angle_max =  angle_max - M_PI/2.0;
    }

    scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (double)(scan_data.num_values-1);

    scan_msg.scan_time = scan_time;
    scan_msg.time_increment = scan_time / (double)(scan_data.num_values-1);
    /*****************************/
    scan_msg.range_min = range_min;
    scan_msg.range_max = range_max;

    scan_msg.ranges.resize(scan_data.num_values);
    scan_msg.intensities.resize(scan_data.num_values);
	angle_index = (stop_angle - start_angle) / scan_cfg.angle_resolution + 1;
	
    bool reverse_data = (!inverted && reversed) || (inverted && !reversed);
    if (!reverse_data)  // regular angle.
    {
        for (size_t i = 0; i < scan_data.num_values; i++) 
        {
            if(scan_cfg.protocol_type == 0)
            {
                if(scan_cfg.lidar_type == 0)
                {
                    read_value = (float) scan_data.ranges[i]/100.0;
                }
                else
                {
                    read_value = (float) scan_data.ranges[i]/1000.0;
                }
            }
            else if(scan_cfg.protocol_type == 1)
            {
                read_value = (float) scan_data.ranges[i]/1000.0;
            }
            else if(scan_cfg.protocol_type == 2)
            {
                read_value = (float) scan_data.ranges[i]/1000.0;
            }
            
            if ((i >= 0) && (i < angle_index))
            {
                if (read_value == 0.0 || read_value < scan_msg.range_min)
                {
                    scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
                }
                else
                {
                    scan_msg.ranges[i] = read_value;
                }
                scan_msg.intensities[i] = (float) scan_data.intensities[i];
//                std::cout << "index :" << i << " range :" << scan_msg.ranges[i] << "intensities :" << scan_msg.intensities[i] << std::endl;
            }
        }
//        std::cout << "one pack finish !!" << std::endl;
    } 
    else  // inverse angle.
    {
        for (size_t i = 0; i < scan_data.num_values; i++) 
        {
            if(scan_cfg.protocol_type == 0)
            {
                read_value = (float) scan_data.ranges[i]/100.0;
            }
            else if(scan_cfg.protocol_type == 1)
            {
                read_value = (float) scan_data.ranges[i]/1000.0;
            }
            else if(scan_cfg.protocol_type == 2)
            {
                read_value = (float) scan_data.ranges[i]/1000.0;
            }

            if ((i >= 0) && (i < angle_index))
            {
                if (read_value == 0.0 || read_value < scan_msg.range_min)
                {    
                    scan_msg.ranges[scan_data.num_values-1-i] = std::numeric_limits<float>::infinity();                
                }
                else
                {
                    scan_msg.ranges[scan_data.num_values-1-i] = read_value;
                }
                scan_msg.intensities[scan_data.num_values-1-i] = (float) scan_data.intensities[i];
//                std::cout << "intensities num " << i << ": " << scan_msg.intensities[i] << std::endl;
            }
        }
//        std::cout << "one pack finish !!" << std::endl;
    }

    pub->publish(scan_msg);
}

int main(int argc, char ** argv) 
{
    // laser data
    Intelly laser;
    ScanCfg scan_cfg; // laser scan config.
    ScanData scan_data; // laser scan data.
    // sensor_msgs::LaserScan scan_msg;

    // parameters
    std::string host_ip;
    std::string frame_id;   // laser frame id.
    // int start_angle, stop_angle;
    float range_min, range_max; // laser range min&max, in meters.
    // functional parameter.
    bool inverted ;  // invert scan angle.

    ros::init(argc, argv, "leimou_f30_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1000);
    
    // intelly laser scan config.
    // supported value types: bool, int, float, double, string.
    //从leimou_f30.launch中读取，当该文件中不存在该项时，取后面的默认值，否则以该文件中的值为准
    nh_private.param<std::string>("host_ip", scan_cfg.host_ip, "192.168.0.111");   // laser ip.
    nh_private.param<std::string>("frame_id", frame_id, "laser_leimou_f30");   
    nh_private.param<int>("data_port", scan_cfg.data_port, 4001); // laser port.
    nh_private.param<int>("debug_port", scan_cfg.debug_port, 4002); // laser port.
    nh_private.param<int>("start_angle", scan_cfg.start_angle, 0);  // laser scan start angle, in degree.
    nh_private.param<int>("stop_angle", scan_cfg.stop_angle, 180);  // laser scan start angle, in degree.
    nh_private.param<int>("reporting_interval", scan_cfg.reporting_interval, 66);  // scan data reporting interval, in ms. 
    nh_private.param<float>("range_min", range_min, 0.0);   // laser range min, in meters.
    nh_private.param<float>("range_max", range_max, 80.0);  // laser range max, in meters.
     /***Not used now.***/ 
    nh_private.param<int>("protocol_type", scan_cfg.protocol_type, 0);  			// 0-INTELLY 1-SICK_BIN 2-SICK_ASCII
    nh_private.param<int>("transmission_mode", scan_cfg.transmission_mode, 2);  	// 0-主动传输模式， 1-被动传输模式， 2-能量强度模式
    nh_private.param<bool>("enable_timestamp", scan_cfg.enable_timestamp, false); 	// 0-不使能， 1-使能
    nh_private.param<float>("angle_resolution", scan_cfg.angle_resolution, 0.25);  	// 0.33 degree.
    nh_private.param<int>("lidar_type", scan_cfg.lidar_type, 1);  	// 0-F30， 1-F30-C
    // publish config.
	nh_private.param<bool>("inverted", inverted, false);  // invert scan angle.
	{
        std::cout << " laser scan config is: \n"
                  << "--- Protocol Type: " 		<< scan_cfg.protocol_type 		<< " ---\n"
                  << "--- Transmission Mode: " 	<< scan_cfg.transmission_mode 	<< " ---\n"
                  << "--- Reporting Interval: " << scan_cfg.reporting_interval 	<< " ---\n"
                  << "--- Enable Timestamp: " 	<< scan_cfg.enable_timestamp 	<< " ---\n"
                  << "--- Angel Resolution: "   << scan_cfg.angle_resolution    << " ---\n"
                  << "--- IP Address: " 		<< scan_cfg.host_ip 			<< " ---\n"
                  << "--- Data Port: " 			<< scan_cfg.data_port 			<< " ---\n"
                  << "--- Mac Address: " 		<< scan_cfg.mac_addr 			<< " ---\n"
                  << "--- Start angle: " 		<< scan_cfg.start_angle 		<< " ---\n"
                  << "--- Stop angle: " 		<< scan_cfg.stop_angle 			<< " ---\n"
                  << "--- Lidar_type: " 		<< scan_cfg.lidar_type 			<< " ---" << std::endl;
    }   
    start_angle = scan_cfg.start_angle; 
	stop_angle  = scan_cfg.stop_angle;

    // intelly scan from 0 degree to 270 degree. full angle.
    scan_cfg.start_angle = scan_cfg.start_angle*100;   // intelly scan 
    scan_cfg.stop_angle = scan_cfg.stop_angle*100;  // 
    

    printf("Intelly laser running on ROS package intelly_node\n");

    // make connection...
    ROS_INFO_STREAM("Connecting to laser at " << scan_cfg.host_ip);
    // step 1 init.
    if (laser.init(scan_cfg))
        std::cout << "Intelly intialization succeeded!" << std::endl;
    else
    {
        std::cerr << "Intelly intialization error!" << std::endl;
        return -1;
    }
    // step 2 start scan.
    if (laser.StartScan())
        std::cout << "Start laser scan succeeded!" << std::endl;
    else
    {
        std::cerr << "Start laser scan error!" << std::endl;
        return -1;
    }
    ros::Duration(3).sleep(); // sleep for a while.

    ros::Time start_scan_time;
    ros::Time end_scan_time;
    double scan_duration;

    float angle_min = DEG2RAD((float)scan_cfg.start_angle/100.0);
    float angle_max = DEG2RAD((float)scan_cfg.stop_angle/100.0);
    
    bool flag;
    int error_cnt = 0;
    int t1, t2, d1, d2;
    ros::Rate loop_rate(1000/scan_cfg.reporting_interval);  // loop rate  HZ

    while (ros::ok()) 
    {
        start_scan_time = ros::Time::now();
        flag = laser.GrabScanData(scan_data);
        end_scan_time = ros::Time::now();
        scan_duration = (end_scan_time - start_scan_time).toSec();

        if (flag)
        {
            ROS_INFO_STREAM_ONCE("---Received laser data!---"); // This message will only print once
            publish_scan(&scan_pub, scan_cfg, scan_data,
                         start_scan_time, scan_duration, inverted,
                         angle_min, angle_max,
                         range_min, range_max,
                         frame_id);
			if(error_cnt > 0)
			{
				error_cnt = 0;
			}
        }
        else
        {
            error_cnt++;
			if(error_cnt > 3)
			{
				 ROS_ERROR("Waiting laser data...\n");
			}
           
            if (error_cnt >= (1000/scan_cfg.reporting_interval))
            {
                ROS_ERROR("---Grab Laser Data Error, Exit!---");
                return -1;
            }
        }
        
        ros::spinOnce();  // for callback function.
        loop_rate.sleep();
    }

    // done!
    laser.StopScan();
    ros::shutdown();
    return 0;
  
}
