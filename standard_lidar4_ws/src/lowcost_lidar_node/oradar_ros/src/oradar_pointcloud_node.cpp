#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud.h>
#include <arpa/inet.h> 
#include "ord/ord_driver.h"

#define Degree2Rad(X) ((X)*M_PI / 180.)

//using namespace ord_sdk;
std::unique_ptr<ord_sdk::OrdDriver> device = NULL;

void publish_msg(ros::Publisher *pub, ord_sdk::ScanFrameData& scan_frame, ros::Time start,
                 double scan_time, std::string frame_id, bool inverted,
                 double angle_min, double angle_max, double min_range, double max_range) 
{
  sensor_msgs::PointCloud pcMsg;
  int node_count = scan_frame.layers[0].ranges.size();
  int counts = node_count * ((angle_max - angle_min) / 270.0f);
  int angle_start = 135 + angle_min;
  int node_start = node_count * (angle_start / 270.0f);
  float range = 0.0;
  float intensity = 0.0;
  double angle_increment = (angle_max - angle_min) / (double)counts; 
  double angle = 0.0, rad = 0.0;

  //ROS_INFO("get lidar frame count = %d, %d, %d, %d ", node_count, counts, angle_start, node_start);

  pcMsg.header.stamp = ros::Time::now();
  pcMsg.header.frame_id = frame_id;
  pcMsg.points.resize(counts);

  //add an intensity channel to the cloud
  pcMsg.channels.resize(1);
  pcMsg.channels[0].name = "intensities";
  pcMsg.channels[0].values.resize(counts);

  for (int i = 0; i < counts; i++) 
  {
    range = scan_frame.layers[0].ranges[node_start] * 0.002;
    intensity = scan_frame.layers[0].intensities[node_start];
    angle = angle_increment * i + angle_min;
    rad = Degree2Rad(angle);

    if((range >= min_range) && (range <= max_range))
    {	
    	if (!inverted) 
    	{
        	pcMsg.points[i].x = range * cos(rad); 
		pcMsg.points[i].y = range * sin(rad);
		pcMsg.points[i].z = 0.0;
		pcMsg.channels[0].values[i] = intensity;
        	//node_start = node_start + 1;
    	} 
    	else 
    	{
        	pcMsg.points[counts - 1 - i].x = range * cos(rad);
		pcMsg.points[counts - 1 - i].y = range * sin(rad);
		pcMsg.points[counts - 1 - i].z = 0.0;
		pcMsg.channels[0].values[counts - 1 - i] = intensity;
        	//node_start = node_start + 1;
    	}
    }

    node_start = node_start + 1;

  }

  pub->publish(pcMsg);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "point_cloud_publisher");
  std::string frame_id, cloud_topic, lidar_ip;
  int motor_speed, lidar_port ,filter_size ,motor_dir;
  unsigned int motor_speed_get,filter_size_get ,motor_dir_get;
  double angle_min, angle_max;
  double min_range, max_range;
  bool inverted = false;
  bool save_config = false;

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
 
  nh_private.param<double>("angle_max", angle_max, 135.00);
  nh_private.param<double>("angle_min", angle_min, -135.00);
  nh_private.param<double>("range_max", max_range, 30.0);
  nh_private.param<double>("range_min", min_range, 0.05);
  nh_private.param<bool>("inverted", inverted, false);
  nh_private.param<int>("motor_speed", motor_speed, 20);
  nh_private.param<int>("filter_size", filter_size, 1);
  nh_private.param<int>("motor_dir", motor_dir,1);
  nh_private.param<std::string>("lidar_ip", lidar_ip, "192.168.1.100");
  nh_private.param<int>("lidar_port", lidar_port, 2007);

  nh_private.param<std::string>("frame_id", frame_id, "pc_frame");
  nh_private.param<std::string>("cloud_topic",cloud_topic,"point_cloud");
  ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud>(cloud_topic, 50);


  std::string address_str = lidar_ip;
  std::string port_str = "2007";

  in_addr_t address = htonl(INADDR_NONE);
  in_port_t port = 0;
  try {
    address = inet_addr(address_str.c_str());
    if (address == htonl(INADDR_NONE))
      throw std::exception();
    port = htons(std::stoi(port_str));
  }
  catch (...) {
    ROS_ERROR("Invalid device address: %s", lidar_ip.c_str());
    exit(-1);
  }

  ord_sdk::LidarAddress location(address, port);
  device = std::unique_ptr<ord_sdk::OrdDriver>(new ord_sdk::OrdDriver(location));

  if (motor_speed < 10) {
    motor_speed = 10; 
  }
  
  if (motor_speed > 30) {
    motor_speed = 30;
  }

  while(1)
  {
    if (device->isOpened() != true)
    {
      if (device->open() == ord_sdk::no_error)
      	ROS_INFO("Device open success");
      else
        ROS_INFO("Device open fail");
    }

    if (device->trackConnect() != ord_sdk::no_error)
    {
      ROS_INFO("Device connecting ...");
      sleep(1);
      continue;
    }
    else
    {
      ROS_INFO("Device connect success");
      break;
    }
  }
	
  std::string firmware_version;
  if (device->getFirmwareVersion(firmware_version) == ord_sdk::no_error) {
    ROS_INFO("get Firmware version: %s ", firmware_version.c_str());
  }
  else
    ROS_WARN("fail: Unable to query device firmware version");

  if (device->getScanSpeed(motor_speed_get) == ord_sdk::no_error)
  {
    ROS_INFO("get motor speed: %d HZ", motor_speed_get);
  }
  else
  {
    ROS_WARN("fail: false to get motor speed");
    return -1;
  }

  if (motor_speed != motor_speed_get)
  {
    save_config = true;
    if (device->setScanSpeed(motor_speed) == ord_sdk::no_error)
    {
      ROS_INFO("success to set motor speed: %d HZ", motor_speed);
      sleep(1);
    } 
    else
    {
      ROS_ERROR("fail: false to set motor speed!");
      return -1;
    }
  }
  if(device->getTailFilterLevel(filter_size_get) == ord_sdk::no_error)
  {
		ROS_INFO("get filter size %d", filter_size_get);
  }
  if(filter_size != filter_size_get)
	{
    save_config = true;
		if(device->setTailFilterLevel(filter_size) == ord_sdk::no_error)
		{
			ROS_INFO("success to set filter size: %d ", filter_size);
			sleep(1);
		}
		else
		{
			ROS_ERROR("fail: false to set filter size!");
			return -1;
		}
	}
   if(device->getScanDirection(motor_dir_get) == ord_sdk::no_error)
	{
		ROS_INFO("get motor scan dir : %d ", motor_dir_get);
	}
   if(motor_dir_get != motor_dir)
	{
    save_config = true;
	  if(device->setScanDirection(motor_dir) == ord_sdk::no_error)
		{
			ROS_INFO("success to set motor dir : %d ", motor_dir);
      		sleep(1);
		}
		else
		{
			ROS_ERROR("fail: false to set motor dir !");
			return -1;
		}
	}

  if(save_config)
  {
    error_t ret = device->applyConfigs();
    if (ret == ord_sdk::no_error)
    {
      ROS_INFO("success to save config");
      sleep(1);
    }
    else
    {
      ROS_ERROR("save config fail!");
    }
  }
  
  ord_sdk::ScanFrameData scan_frame_data;
  ros::Time start_scan_time;
  ros::Time end_scan_time;
  double scan_duration;
  int count = 0;

  device->enableMeasure();
  device->enabelDataStream();
  ros::Rate rate(motor_speed);

  ROS_INFO("get lidar pointcloud data");

  while (ros::ok()) 
  {
    start_scan_time = ros::Time::now();

    if(device->getScanFrameData(scan_frame_data) != ord_sdk::no_error)
    {
       ROS_INFO("getScanFrameData fail ");
       throw std::exception();
    }

    end_scan_time = ros::Time::now();
    scan_duration = (end_scan_time - start_scan_time).toSec();
    //count = scan_frame_data.layers[0].ranges.size();
    //ROS_INFO("get lidar frame count = %d ", count);
    
    publish_msg(&cloud_pub, scan_frame_data, start_scan_time, scan_duration, frame_id,
                inverted, angle_min, angle_max, min_range, max_range);

    ros::spinOnce();
   // rate.sleep();
  }

  device->disableDataStream();
  device->close();

  return 0;

}
