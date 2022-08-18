/**
 * @file bag_module.cpp
 * @author zmy (626670628@qq.com)
 * @brief sros bag 模块实现
 * @version 0.1
 * @date 2021-05-27
 *
 *
 */

#include "sros_bag_module.h"
#include "core/msg/base_msg.h"
#include "core/msg/image_msg.hpp"
#include "core/msg/laser_scan_msg.hpp"
#include "core/settings.h"

namespace srosbag
{
    std::vector<std::string> splitStr(const std::string &str, const char delim)
    {
        std::vector<std::string> splited;
        std::stringstream sstr(str);
        std::string splited_str;
        while (getline(sstr, splited_str, delim))
        {
            splited.emplace_back(splited_str);
        }
        return splited;
    }

    BagModule::BagModule() : Module("BagModule"), start_record_(false)
    {
    }

    void BagModule::run()
    {
        LOG(INFO) << "begin to run the srosbag module";
        auto bag_dir = sros::core::Settings::getInstance().getValue<std::string>("srosbag.bag_dir", "/sros/bag/");
        bool is_compress_bag = sros::core::Settings::getInstance().getValue<bool>("srosbag.compress_bag", false);
        msg_bag_ = bag::MsgBag::Create(bag_dir);
        msg_bag_->setCompressBag(is_compress_bag);

        bool use_srosbag = sros::core::Settings::getInstance().getValue<bool>("srosbag.use_srosbag", false);
        if (!use_srosbag)
        {
            msg_bag_->stopRecord();
            LOG(INFO) << " quit the srosbag module run";
            return;
        }

        subscribeTopic("TIMER_200MS", [this](sros::core::base_msg_ptr msg)
                       { getStartParam(); });

        subscribeSensorMsg();

        //         test_ = std::thread(&BagModule::for_test, this);
        //         test_.detach();
        dispatch();
    }

    void BagModule::getStartParam()
    {
        auto start = sros::core::Settings::getInstance().getValue<bool>("srosbag.start_record", 0);
        if (start && !start_record_)
        {
            auto messages_str = sros::core::Settings::getInstance().getValue<std::string>("srosbag.messages", "");
            auto record_messages = splitStr(messages_str, ';');
            if (!record_messages.empty())
            {
                msg_bag_->startRecord(record_messages);
                start_record_ = true;
            }
        }
        else if (!start)
        {
            msg_bag_->stopRecord();
            if (start_record_)
                LOG(INFO) << "Stop to record message!";
            start_record_ = false;
        }

        return;
    }

    void BagModule::subscribeSensorMsg()
    {

        subscribeTopic("TOPIC_LASER", CALLBACK(&BagModule::msgCallback<sros::core::LaserScanMsg>));
        subscribeTopic("FIRST_SCAN", CALLBACK(&BagModule::msgCallback<sros::core::LaserScanMsg>));
        subscribeTopic("SECOND_SCAN", CALLBACK(&BagModule::msgCallback<sros::core::LaserScanMsg>));

        subscribeTopic("TOPIC_D435_COLOR", CALLBACK(&BagModule::msgCallback<sros::core::ImageMsg>));

        subscribeTopic("TOPIC_D435_DEPTH", CALLBACK(&BagModule::msgCallback<sros::core::ImageMsg>));

        subscribeTopic("TOPIC_COLOR", CALLBACK(&BagModule::msgCallback<sros::core::ImageMsg>));

        subscribeTopic("LIVOX_MID70_MSG", CALLBACK(&BagModule::msgCallback<sros::core::LivoxPointsMsg>));

        subscribeTopic("TOPIC_MATCHPOSE", CALLBACK(&BagModule::msgCallback<sros::core::PoseStampedMsg>));

        subscribeTopic("IFM3D_IMG", CALLBACK(&BagModule::msgCallback<sros::core::ImageMsg>));
    }   

    void BagModule::for_test()
    {
        // std::vector<std::string> names;
        // names.emplace_back("imuBag");
        // names.emplace_back("odometryBag");
        // msg_bag_->startRecord(names);
        // // while (true)
        // for (int i = 0; i < 10; ++i)
        // {
        //     Imu imu;
        //     imu.header.stamp = sros::core::util::get_time_in_us();
        //     imu.orientation = geometry_msgs::Quaternion(1, 2, 3, 4);
        //     imu.angular_velocity = geometry_msgs::Vector3(4, 5, 6);
        //     imu.linear_acceleration = geometry_msgs::Vector3(4, 5, 6);
        //     msg_bag_->dumpMsg<Imu>(imu, "imuBag");
        //     // std::this_thread::sleep_for(std::chrono::milliseconds(20));
        //     nav_msgs::Odometry odom;
        //     odom.header.stamp = sros::core::util::get_time_in_us();
        //     odom.pose = geometry_msgs::Pose{Eigen::Vector3f(3, 4, 5), Eigen::Quaternionf(3, 7, 6, 5)};
        //     odom.twist = geometry_msgs::Twist{Eigen::Vector3f(3, 6, 5), Eigen::Vector3f(3, 4, 5)};

        //     msg_bag_->dumpMsg<nav_msgs::Odometry>(odom, "odometryBag");
            // std::this_thread::sleep_for(std::chrono::milliseconds(20));
        // }
    }

} // namespace sros
