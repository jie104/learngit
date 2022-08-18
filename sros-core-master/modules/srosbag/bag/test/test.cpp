// #include "type_container.hpp"
#include "bag/message_bag.h"
#include <array>
#include <functional>
#include <iostream>
#include <map>
#include <typeinfo>
#include <vector>

#include "factory/my_factory.h"

using namespace std;

int main()
{
    auto bag = bag::MsgBag::Create("/home/zmy/sbag/");
    std::vector<std::string> names;
    names.emplace_back("imuBag");
    names.emplace_back("my_imu");
    names.emplace_back("odometryBag");
    // bag->startRecord(names);

    // for (int i = 0; i < 1000; ++i)
    // {
    //     Imu imu;
    //     imu.header.stamp = sros::core::util::get_time_in_us();
    //     std::cout << "ddd:" << imu.header.stamp << std::endl;
    //     imu.orientation = geometry_msgs::Quaternion(1, 2, 3, 4);
    //     imu.angular_velocity = geometry_msgs::Vector3(4, 5, 6);
    //     imu.linear_acceleration = geometry_msgs::Vector3(4, 5, 6);
    //     bag->dumpMsg<Imu>(imu);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(10));

    //     Imu my_imu;
    //     my_imu.header.stamp = sros::core::util::get_time_in_us();
    //     // std::cout << "ddd:" << my_imu.header.stamp << std::endl;
    //     my_imu.orientation = geometry_msgs::Quaternion(1, 2, 3, 4);
    //     my_imu.angular_velocity = geometry_msgs::Vector3(4, 5, 6);
    //     my_imu.linear_acceleration = geometry_msgs::Vector3(4, 5, 6);
    //     bag->dumpMsg<Imu>(my_imu, "my_imu");
    //     std::this_thread::sleep_for(std::chrono::milliseconds(10));

    //     nav_msgs::Odometry odom;
    //     odom.header.stamp = sros::core::util::get_time_in_us();
    //     odom.pose = geometry_msgs::Pose{Eigen::Vector3f(3, 4, 5), Eigen::Quaternionf(3, 7, 6, 5)};
    //     odom.twist = geometry_msgs::Twist{Eigen::Vector3f(3, 6, 5), Eigen::Vector3f(3, 4, 5)};

    //     std::this_thread::sleep_for(std::chrono::milliseconds(10));
    //     bag->dumpMsg<nav_msgs::Odometry>(odom);
    // }

    bag->stopRecord();

    bag->setMsgHandle<sros::core::PoseStampedMsg>([](const sros::core::PoseStampedMsg &imu)
                                                  { std::cout << "my imu stamp:" << imu.topic_ << std::endl; },
                                                  "TOPIC_MATCHPOSE");
    bag->playBack("2022-05-17-16-02-50.sbag", true,true,false);
    bag->closePlay();

    bag->setMsgHandle<sros::core::PoseStampedMsg>([](const sros::core::PoseStampedMsg &imu)
                                                  { std::cout << "my imu stamp:" << imu.topic_ << std::endl; },
                                                  "TOPIC_MATCHPOSE");
    bag->playBack("2022-05-19-15-45-54.sbag", true, true, false);

    return 0;
}