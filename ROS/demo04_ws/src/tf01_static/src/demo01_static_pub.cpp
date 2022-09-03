#include "ros/ros.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"


/*
    需求：该节点需要发布两个坐标系之间的相对关系(静态坐标变换)

    流程：
        1.包含头文件
        2.设置编码，节点初始化  NodeHandle;
        3.创建发布对象；
        4.组织被发布的消息；
        5.发布数据
        6.spin();
*/

// ****不同传感器的该模块实现高度类似，所以可考虑通过指令实现****
// rosrun tf2_ros static_transform_publisher x偏移量 y偏移量 z偏移量 z偏航角 y俯仰角 x翻滚角 父坐标系 子坐标系

int main(int argc,char* argv[])
{
    // 2.设置编码，节点初始化  NodeHandle;
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"static_pub");   
    ros::NodeHandle nh;
    // 3.创建发布对象；
    tf2_ros::StaticTransformBroadcaster pub;
    // 4.组织被发布的消息；
    geometry_msgs::TransformStamped tfs;
    tfs.header.stamp=ros::Time::now();  //获取时间戳
    tfs.header.frame_id= "base_link";    //相对坐标系关系被参考的那个
    tfs.child_frame_id="laser";
        //雷达相对小车主体的偏移量
    tfs.transform.translation.x=0.2;
    tfs.transform.translation.y=0.0;
    tfs.transform.translation.z=0.5;
        //需要根据欧拉角转换
    tf2::Quaternion qtn;    //创建四元数对象
        //向该对象设置欧拉角，可以将欧拉角转化为四元数
    qtn.setRPY(0,0,0);  //单位弧度，雷达欧拉角

    tfs.transform.rotation.x=qtn.getX();
    tfs.transform.rotation.y=qtn.getY();
    tfs.transform.rotation.z=qtn.getZ();
    tfs.transform.rotation.w=qtn.getW();


    // 5.发布数据
    pub.sendTransform(tfs);

    //6.spin()
    ros::spin();

    return 0;
}
