#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"    //发布的数据类型，ros里封装好的数据类型
#include <sstream>

/* 
    发布方实现：
        1.包含头文件：
          ROS中的文本类型 ---> std_msgs/String.h
        2.初始化ros节点
        3.创建节点句柄
        4.创建发布者对象
        5.编写发布逻辑并发布数据
*/


int main(int argc,char*argv[])
{
    //解决中文乱码问题
    setlocale(LC_ALL,"");
    //2.初始化 ROS节点：
    ros::init(argc,argv,"erGouzi");
    //3.创建节点句柄：
    ros::NodeHandle nh;
    //4.创建发布者对象,10为缓存的数据长度，超过的数据将被除掉，例如由于网络阻塞原因导致信息发不出去，将被放到缓存里面，等网络畅通再发布出去
    ros::Publisher pub=nh.advertise<std_msgs::String>("fang",10);
    //5.编写发布逻辑并发布数据
    //要求以10HZ的频率发布数据，并且文本后加编号
    //先创建被发布的消息
    std_msgs::String msg;
    //发布频率,设置为10HZ
    ros::Rate rate(1);
    //设置编号
    int count=0;
    //编写循环，循环中发布数据
    ros::Duration(3).sleep();   //休眠3s，延迟数据发送，等master注册完毕后，再发送数据，避免订阅不到前几条数据
    while (ros::ok())
    {
        count++;
        //实现字符串拼接数字
        std::stringstream ss;
        ss << "hello ---> " << count;

        msg.data=ss.str();  //将流中的数据提取为字符串
        pub.publish(msg);   //正式发布数据
        //添加日志：
        ROS_INFO("发布的数据是：%s",ss.str().c_str());

        rate.sleep();   //睡眠0.1s
        ros::spinOnce();    //回调函数处理，此处无回调函数，不需要
    }

    return 0;
}