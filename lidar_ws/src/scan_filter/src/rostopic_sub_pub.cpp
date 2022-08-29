#include<ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <iostream>
#include <fstream>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
// #include <boost/bind.hpp> 
// #include <stdio.h>
 
using namespace std;
 
class point_time
{
	//定义这个类的私有成员
	private:
		ros::NodeHandle n; //声明用于ROS系统和通信的节点句柄
 
 
		double t;
		double t_past;
		double t_now;
		int flag_past;
 
		//定义订阅者
		ros::Subscriber point_info_sub;
 
		//定义发布者
		ros::Publisher point_info_pub;
		geometry_msgs::PointStamped ekf_point;
 
		//写入文件流(写入时间的)
		ofstream fout_time;
 
		//写入文件流(写入坐标的)--用统一的文件流就好
		// ofstream fout_point;
 
 
	//公有成员
	public:
		//定义其构造函数
		point_time()
		{
			t_past = 0;
			t_now = 0;
			flag_past = 1;
			t = 0;
			
 
			fout_time.open("time_odom_multi_location_ekf_xyz.txt");	//打开一个这样的文件
 
			fout_time << "x" << "\t" << "y" << "\t"<< "z" << "\t"<< "time" << endl;
 
			//订阅来自/odometry/filtered的话题
			//消息类型为nav_msgs::Odometry
			//通过函数pointCallback来调回
			point_info_sub = n.subscribe<nav_msgs::Odometry>("/odometry/filtered", 1000, &point_time::pointCallback, this);
			
			//发布一个名为/location_ekf_info的话题
			//消息类型为geometry_msgs::PointStamped
			point_info_pub=n.advertise<geometry_msgs::PointStamped>("/location_ekf_info", 1000);
		}
 
		void pointCallback(const nav_msgs::Odometry::ConstPtr &msgPointStamped)
		{
			nav_msgs::Odometry msgPoseStamped = *msgPointStamped;
			//接收到消息后，给到发布者（发布者是私有成员）
			ekf_point.point.x = msgPoseStamped.pose.pose.position.x;
			ekf_point.point.y = msgPoseStamped.pose.pose.position.y;
			ekf_point.point.z = msgPoseStamped.pose.pose.position.z;
 
			//给发布者的加入frameid和时间戳
			ekf_point.header.frame_id ="map";
			ekf_point.header.stamp = ros::Time::now();
			//在终端显示定位结果与时间
			ROS_INFO("(%.5f, %.5f, %.5f)", msgPoseStamped.pose.pose.position.x*100, msgPoseStamped.pose.pose.position.y*100, msgPoseStamped.pose.pose.position.z*100);
			cout << "coordinate_out_time:\t" << msgPoseStamped.header.stamp << "secs" << endl;
			
			//保存到文件中。
			fout_time << msgPoseStamped.pose.pose.position.x*100 <<"\t"<< msgPoseStamped.pose.pose.position.y*100 <<"\t"<< msgPoseStamped.pose.pose.position.z*100 << "\t";
			point_info_pub.publish(ekf_point);//发布出去
 
			//将时间差输出出来
			if(flag_past == 1)
			{
				t_past = msgPoseStamped.header.stamp.toSec();
				flag_past = 0;
				// fout_time << t << endl;
			}
			else
			{
				t_now = msgPoseStamped.header.stamp.toSec();
				flag_past = 1;
			}
 
			if (t_now != 0)
			{
				t = t_now - t_past;
				if (t_now > t_past)
				{
					t = t;
				}
				else
				{
					t = -t;
				}
				cout << "t_now - t_past = " << t << endl;
				fout_time << t << endl;
			}
			else 
			{
				t=0;
				cout << "t_now - t_past = " << t << endl;
				fout_time << t << endl;
			}
		}
 
		//构析函数
		~point_time()
		{
			fout_time.close();
			cout << "close file success!" << endl;
		}
 
};
 
int main(int argc, char **argv)
{
	ros::init(argc, argv, "ekf_translater");//初始化当前节点的名称（这个名字应该需要跟cmake.list设置的一样）
	// ros::NodeHandle n;
 
	point_time recordT;//定义了一个类
 
	// ros::Subscriber point_info_sub = n.subscribe<geometry_msgs::PointStamped>("/location_info", 1000, boost::bind(pointCallback,_1, t_past, t_now, flag_past));
	ros::spin();
	return 0;
}
 