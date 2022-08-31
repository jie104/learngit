#include "ros/ros.h"
#include "plumbing_pub_sub/Person.h"

/*
    发布方：发布人消息
        1.包含头文件；
            #include "plumbing_pub_sub/Person.h"
        2.初始化ros节点；
        3.创建ros节点句柄；
        4.创建发布者对象；
        5.编写发布逻辑；

*/
int main(int argc,char* argv[])
{
    setlocale(LC_ALL,"");
    ROS_INFO("这是消息发布方");
    //2.初始化ros节点；
    ros::init(argc,argv,"banZhuRen");
    //3.创建节点句柄；
    ros::NodeHandle nh;
    //4.创建发布者对象；
    ros::Publisher pub=nh.advertise<plumbing_pub_sub::Person>("liaotian",10);
    //5.编写发布逻辑；
    //5-1.创建发布数据
    plumbing_pub_sub::Person person;
    person.name="张三";
    person.age=1;
    person.height=1.73;
    
    //5-2.设置发布频率,1HZ
    ros::Rate rate(10);
    //5-3.循环发布数据
    while (ros::ok()){
        //修改数据
        person.age+=1;
        //休眠
        rate.sleep();
        //核心：发布数据
        pub.publish(person);
        ROS_INFO("发布的消息：%s,%d,%.2f",person.name.c_str(),person.age,person.height);
        //建议
        ros::spinOnce();

    }
    return 0;
}