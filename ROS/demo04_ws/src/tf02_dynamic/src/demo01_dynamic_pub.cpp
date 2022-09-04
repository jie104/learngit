#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "tf2_ros/transform_broadcaster.h"  //发布动态坐标关系
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"

/*
    发布方：需要订阅乌龟的位姿信息，转换成相对于窗体的坐标关系，并发布
    准备：  
        话题：  /turtle1/pose
        消息：  /turtlesim/Pose
    流程：
        1.包含头文件
        2.设置编码问题、初始化、NodeHandle
        3.创建订阅对象：订阅 /turtle1/pose
        4.回调函数处理订阅的消息：将位姿态信息转换成坐标相对关系并发布
        5.spin()


*/

void doPose(const turtlesim::Pose::ConstPtr& pose){
    //·获取位姿信息，转换成坐标系相对关系（核心），发布
    //a.创建发布对象;
    static tf2_ros::TransformBroadcaster pub;   //使用静态，避免每次回调函数都创造订阅对象
    //b.组织发布的信息;
    geometry_msgs::TransformStamped ts; 
    ts.header.frame_id="world";
    ts.header.stamp=ros::Time::now();
    ts.child_frame_id="turtle1";

    //坐标系偏移量
    ts.transform.translation.x=pose->x;
    ts.transform.translation.y=pose->y;
    ts.transform.translation.z=0;

    //四元数的设置
    /*
        位姿信息没有四元数，但有个偏航角度，又已知乌龟是2D，没有俯仰、翻滚角度，所以可以得出
        乌龟欧拉角： 0 0 theta

    */

    tf2::Quaternion qtn;
    qtn.setRPY(0,0,pose->theta);

    //转化为四元数
    ts.transform.rotation.x=qtn.getX();
    ts.transform.rotation.y=qtn.getY();
    ts.transform.rotation.z=qtn.getZ();
    ts.transform.rotation.w=qtn.getW();

    //c.发布
    pub.sendTransform(ts);

}



int main(int argc, char* argv[])
{
    // 2.设置编码问题、初始化、NodeHandle;
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"dynamic_pub");
    ros::NodeHandle nh;
    // 3.创建订阅对象：订阅 /turtle1/pose
    ros::Subscriber sub=nh.subscribe("/turtle1/pose",100,doPose);
    // 4.回调函数处理订阅的消息：将位姿态信息转换成坐标相对关系并发布
    ros::spin();
    return 0;

}