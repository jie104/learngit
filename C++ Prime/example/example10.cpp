// laser_geometry



#include <iostream>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

class Costmap
{
public:
	
    Costmap();
    ~Costmap()
    {

    }
	void pub_costmap();
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);

private:
	ros::NodeHandle n;
	ros::NodeHandle n_private;
	ros::Publisher cloud_pub;
	ros::Subscriber scan_sub;

	std::string subTopicName;
    std::string cloudTopicName;
	
    laser_geometry::LaserProjection projector_;
	tf::TransformListener tfListener_; 
};

Costmap::Costmap():n(ros::NodeHandle()),n_private(ros::NodeHandle("~"))
{
	n_private.param<std::string>("laserTopic",this->subTopicName,"/scan");
    n_private.param<std::string>("cloudTopic", this->cloudTopicName, "/cloud_pcl");
    
    scan_sub = n.subscribe(this->subTopicName,10,&Costmap::scanCallback,this);
    cloud_pub = n.advertise<sensor_msgs::PointCloud2>(this->cloudTopicName, 1);

    //此处的tf是 laser_geometry 要用到的
    tfListener_.setExtrapolationLimit(ros::Duration(0.1));

}

void Costmap::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  sensor_msgs::PointCloud2 cloud;

  projector_.transformLaserScanToPointCloud("velodyne", *scan_msg, cloud, tfListener_);
 
    // sensor_msgs::PointCloud2 ---> pcl::PointCloud<pcl::PointXYZ>
    //pcl::PointCloud<pcl::PointXYZ> cloud_out;
    //pcl::fromROSMsg(cloud, cloud_out);

  cloud_pub.publish(cloud);
}

