//
// Created by lfc on 16-7-22.
//

#include <iostream>
// #include <glog/logging.h>
#include "bag_record.h"
#include "../msg/pub_msg/laser_scan_msg.hpp"
#include "../msg/pub_msg/PoseStampedMsg.h"
#include "../msg/pub_msg/slam_info_msg.hpp"
#define MAX_RANGE_SIZE 1081
#define UHD_RANGE_SIZE 3601
using namespace std;
namespace bag {
BagRecord::BagRecord() : running(false), queue_size_(0), first_record_scan(true),bag_beg_time(0),bag_end_time(0) {
//    laser_msg = make_shared<src::LaserScanStampedMsg>();
//    laser_msg->ranges.resize(MAX_RANGE_SIZE);
//    laser_msg->intensities.resize(MAX_RANGE_SIZE);
}

BagRecord::~BagRecord() {

}

bool BagRecord::open(const char *path, std::ios::openmode mode) {
    if (bag.openBag(path, mode)) {
        auto bagtime = sros::core::util::get_time_in_us();
        string bagversion = "V1.0. Copyright @ 2016 Standardrobots,inc. All rights reserved";
        srosoutstream << "the start  time is:" << bagtime << endl;
        bag_beg_time = bagtime;
        bag.wirteBaginfo(bagversion, bagtime);
        return true;
    }
    return false;

}

void BagRecord::close() {
    boost::mutex::scoped_lock lock(queue_mutex_);//临时变量，随时可以解锁,防止多线程时，同时访问或读写，造成数据出错
    recordStop();
    auto bagtime = sros::core::util::get_time_in_us();
    bag_end_time = bagtime;
    bag.writeEnd(bagtime);
    bag.closeBag();
}

void BagRecord::writeMsg(record::BaseMsg_ptr msg_ptr) {
    msg_ptr->encode();
    auto headerlength = msg_ptr->getHeaderLength();
    int bodylength = (int) msg_ptr->getBodyLength();
    bag.writeHeaderlength(headerlength);
    bag.writeHead((char *) msg_ptr->data(), headerlength);
    bag.writeBody((char *) (msg_ptr->bodyData()), bodylength);
}

bool BagRecord::doRecord(std::string filename) {
    bagfilename = filename;
    string path = bagfilename + ".bag";
    if (!open(path.c_str(), ios::out)) {
        srosoutstream << "error to create the bag file:" << bagfilename << endl;
        close();
        return false;
    }
    if (!ok()) {
        running = true;
    }
    return true;

}

void BagRecord::scanCallback(sros::bag::base_msg_ptr base_ptr) {
    if (ok()) {
        sros::bag::LaserScan_ptr msg = std::dynamic_pointer_cast<sros::bag::LaserScanMsg>(base_ptr);
        if (first_record_scan) {
            first_record_scan = false;
            int range_size = floorf((msg->angle_max - msg->angle_min) / msg->angle_increment + 0.5 + 1);
            if (range_size != msg->ranges.size()) {
                //LOG(INFO) << "!!!err the size is err!" << "the compute size is:" << range_size << ",the range size is:"
                // << msg->ranges.size();
                return;
            }
            if(msg->ranges.size()>MAX_RANGE_SIZE) {
                if (msg->ranges.size() < UHD_RANGE_SIZE) {
                    laser_msg = std::make_shared<record::LaserScanStampedMsg>(record::PF_LASER_SCAN_STAMPED);
                }else{
                    laser_msg = std::make_shared<record::LaserScanStampedMsg>(record::UHD_LASER_SCAN_STAMPED);
                }
            }
            else {
                laser_msg = std::make_shared<record::LaserScanStampedMsg>();
            }
        }
        laser_msg->angle_min = msg->angle_min;
        laser_msg->angle_max = msg->angle_max;
        laser_msg->angle_increment = msg->angle_increment;
        laser_msg->time_increment = msg->time_increment;
        laser_msg->scan_time = msg->scan_time;
        laser_msg->range_min = msg->range_min;
        laser_msg->range_max = msg->range_max;
        laser_msg->ranges.swap(msg->ranges);
        laser_msg->intensities.swap(msg->intensities);

        laser_msg->timestamp = msg->time_;
        recordMsg(laser_msg, msg->topic_);

//        boost::mutex::scoped_lock lock(queue_mutex_);//临时变量，随时可以解锁
//        queue_.push(laser_msg);
//        queue_topic.push(msg->topic_);
//        queue_condition_.notify_all();
    }else{
        //LOG(INFO) << "err to get the laser msg!";
    }
}

void BagRecord::poseCallback(sros::bag::base_msg_ptr base_ptr) {
    if (ok()) {
        sros::bag::PoseStamped_ptr msg = std::dynamic_pointer_cast<sros::bag::PoseStampedMsg>(base_ptr);
        record::PoseStampedMsg_ptr pose_msg = make_shared<record::PoseStampedMsg>();
        pose_msg->setX(msg->pose.x());
        pose_msg->setY(msg->pose.y());
        pose_msg->setZ(msg->pose.z());
        pose_msg->setYaw(msg->pose.yaw());
        pose_msg->setRoll(msg->pose.roll());
        pose_msg->setPitch(msg->pose.pitch());
        pose_msg->setTimestamp(msg->time_);
        recordMsg(pose_msg, msg->topic_);
//        boost::mutex::scoped_lock lock(queue_mutex_);//临时变量，随时可以解锁
//        queue_.push(pose_msg);
//        queue_topic.push(msg->topic_);
//        queue_condition_.notify_all();
    }
}


bool BagRecord::ok() {
    return running;
}

void BagRecord::recordStop() {
    first_record_scan = true;
    running = false;
}

void BagRecord::recordMsg(record::BaseMsg_ptr baseMsg_ptr, std::string topic_name) {
    boost::mutex::scoped_lock lock(queue_mutex_);//临时变量，随时可以解锁,防止并发进入该函数
    int64_t record_time = sros::core::util::get_time_in_us();
    bag.writeStamp(record_time);
    int msgcount = 1;
    bag.writeMsgcount(msgcount);
    bag.writeTopicname(topic_name);
    writeMsg(baseMsg_ptr);
    bag.writeFlush();
}

void BagRecord::recordMsg(sros::bag::base_msg_ptr base_ptr) {
    switch (base_ptr->type_) {
        case sros::bag::TYPE_LASER_SCAN_DATA:
            scanCallback(base_ptr);
            break;
        case sros::bag::TYPE_POSE_STAMPED_DATA:
            poseCallback(base_ptr);
            break;
        case sros::bag::TYPE_SLAM_INFO_DATA:
            slamInfoCallback(base_ptr);
            break;
        default:
            //LOG(INFO) << "err to get the msg:" << base_ptr->type_;
            break;
    }

}

void BagRecord::slamInfoCallback(sros::bag::base_msg_ptr base_ptr) {
    sros::bag::SlamInfoMsg_ptr msg = std::dynamic_pointer_cast<sros::bag::SlamInfoMsg>(base_ptr);
    record::SlamInfoMsg_ptr info_msg(new record::SlamInfoMsg);
    info_msg->angle_max = msg->angle_max;
    info_msg->angle_min = msg->angle_min;
    info_msg->coord_x = msg->coord_x;
    info_msg->coord_y = msg->coord_y;
    info_msg->coord_yaw = msg->coord_yaw;
    info_msg->intensity = msg->intensity;
    info_msg->range_max = msg->range_max;
    info_msg->range_min = msg->range_min;
    recordMsg(info_msg, msg->topic_);
}

bool BagRecord::removeFile(std::string filename) {
    string path = filename + ".bag";

    if (remove(path.c_str())!=-1) {
        //LOG(INFO) << "succ to remove file:" << path;
        return true;
    }
    //LOG(INFO) << "err to remove file:" << path;
    return false;
}

int64_t BagRecord::getRecordPosition() {
    if(ok()) {
        return bag.getWritePosition();
    }else {
        //LOG(INFO) << "play is stop! will return -1";
        return -1;
    }
}
}