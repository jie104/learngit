#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h" //创建坐标点
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"    //坐标点转换过程需要涉及
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Twist.h"    //发布位姿消息头文件

/*

    需求1：换算出 turtle1相对于 turtle2的关系
    需求2：根据距离计算角速度和线速度



*/

int main(int argc,char* argv[])
{
    // 2.编码、初始化、NodeHandle;
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"tfs_sub");
    ros::NodeHandle nh;

    // 3.创建订阅对象
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener sub(buffer);

    //A.创建发布对象
    ros::Publisher pub=nh.advertise<geometry_msgs::Twist>("turtle2/cmd_vel",100);

    // 4.编写解析逻辑；


    ros::Rate rate(10);
    while (ros::ok())
    {
        //核心
        try{
            //1.计算turtle1 到 计算turtle2 的相对关系,ros::Time(0)表示查找两个坐标系时间戳最近时间的两个坐标系相对关系
            /*
                参数1：目标坐标系(子坐标系)
                参数2：源坐标系（根坐标系）
                参数3：ros::Time(0),取间隔最短的两个坐标关系帧
                返回值：geometry_msgs::TransformStamped 源相对于目标的相对关系
            */
            geometry_msgs::TransformStamped Son1ToSon2= buffer.lookupTransform("turtle2","turtle1",ros::Time(0));
            ROS_INFO("turtle1相对于turtle2 的信息: 父级名称：%s,子级：%s 偏移量(%.2f,%.2f,%.2f)",
                    Son1ToSon2.header.frame_id.c_str(), //turtle2
                    Son1ToSon2.child_frame_id.c_str(),  //turtle1
                    Son1ToSon2.transform.translation.x,
                    Son1ToSon2.transform.translation.y,
                    Son1ToSon2.transform.translation.z
                    );  //turtle1

            //B.根据相对关系，计算并组织速度消息
            geometry_msgs::Twist twist;
            /*
                组织速度，只需要设置线速度的x 和 角速度的 z
                x=系数*sqrt(y^2+x^2)
                z=系数*arctan(y/x)

            */
            twist.linear.x=4*sqrt(pow(Son1ToSon2.transform.translation.x,2))+
                        pow(Son1ToSon2.transform.translation.y,2);
            twist.angular.z=4*atan2(Son1ToSon2.transform.translation.y,Son1ToSon2.transform.translation.x);

            //C.发布
            pub.publish(twist);

        }catch(const std::exception& e){
            ROS_INFO("错误提示：%s",e.what());
        }

        rate.sleep();
        ros::spinOnce();
    }
    // 5.spin()
}