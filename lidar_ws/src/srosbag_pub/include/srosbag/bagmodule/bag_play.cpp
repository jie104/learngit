//
// Created by lfc on 16-7-22.
//

#include <iostream>
#include <unistd.h>
// #include <glog/logging.h>
#include "bag_play.h"
#include "global_bag_config.h"
#include "../msg/record_msg/msg_factory.hpp"
#include "../msg/pub_msg/PoseStampedMsg.h"
#include "../msg/pub_msg/laser_scan_msg.hpp"

using namespace std;
namespace bag {

bool BagPlay::open(const char *path, std::ios::openmode mode) {
    if (bag.openBag(path, mode)) {
        std::string bagversion;
        int64_t createtime;
        if (bag.readBaginfo(bagversion, createtime)) {
            bag_beg_time = createtime;
            last_bagtime = createtime;
            last_walltime = sros::core::util::get_time_in_us();
            srosoutstream << "the bag version is:" << bagversion.c_str() << endl;
            srosoutstream << "the create time is:" << createtime << endl;
            return true;
        }
    }
    return false;
}

void BagPlay::close() {
    bag.closeBag();
}

bool BagPlay::doPlay(std::string filename, bool publish_shcedule) {
    is_publish_shcedule = publish_shcedule;
    bagfilename = filename;
    string path = bagfilename + ".bag";
    std::fstream bagfile;
    bagfile.open(path.c_str(), ios::in | ios::ate);
    if (bagfile.is_open()) {
        bagfile.seekg(0, bagfile.end);
        int length = bagfile.tellg();
        bagfile.seekg(length - 5, bagfile.beg);
        int count;
        char buffer_count[sizeof(count)];
        bagfile.read(buffer_count, sizeof(count));
        memcpy(&count, buffer_count, sizeof(count));
        if (count == -1) {
            bagfile.seekg(length - 14, bagfile.beg);
            int64_t bagstamp;
            char buffer_time[sizeof(bagstamp)];
            bagfile.read(buffer_time, sizeof(bagstamp));
//            bagfile.get();
            memcpy(&bagstamp, buffer_time, sizeof(bagstamp));
            bag_end_time = bagstamp;
            printf("the bagstamp is:%ld\n", bagstamp);
        } else {
            //LOG(INFO) << "error to get the end!!!";
//            return false;
        }
    } else {
        //LOG(INFO) << "error to open the bag!!!!" << bagfilename;
        return false;
    }
    bagfile.close();
    //LOG(INFO) << "the path is:" << path;
    if (open(path.c_str(), ios::in)) {
        first_send_scan = true;
        is_stop = false;//初始化标志位
        bag::BagCommandMsg_Ptr syscommand(new BagCommandMsg);
        if (bag::globalbagconfig.handleContinuePlay) {
            bag::globalbagconfig.handleContinuePlay(syscommand);//防止出现死机情况
        }
        boost::thread(boost::bind(&BagPlay::playThread, this));
        //LOG(INFO) << "successfully to get the bag!";
        return true;
    } else {
        //LOG(INFO) << "cannot open the file! will return\n";
        return false;
    }

    return false;
}

record::BaseMsg_ptr BagPlay::readMsg() {
    using namespace record;
    int headerlength;
    bag.readHeaderlength(headerlength);
//    BaseMsg_ptr tmpmsg_ptr;
    char data[headerlength];
    bag.readHead((char *) data, headerlength);



//    header_msg.decodeHeader();
    MSG_TYPE msg_type = (MSG_TYPE)data[0];
    BaseMsg_ptr msg_ptr = record::MsgFactory::getMsg(msg_type);
    memcpy(msg_ptr->data(), data, headerlength);

    msg_ptr->decodeHeader();
    msg_ptr->setBodyLength(msg_ptr->getBodyLength());
    int bodylength = msg_ptr->getBodyLength();
    bag.readBody((char *) msg_ptr->bodyData(), bodylength);
    msg_ptr->decode();
    return msg_ptr;
}

bool BagPlay::waitforSyntime(int64_t now_time, int64_t bagtime) {
    int64_t pre_walltime =
            (int64_t) ((float) (bagtime - last_bagtime) / bag::globalbagconfig.time_scale) + last_walltime;
    int64_t del_time = pre_walltime - now_time;
    if (del_time > 0) {
        usleep(del_time);
    }
    last_walltime = pre_walltime;
    last_bagtime = bagtime;
    return true;
}

sros::bag::base_msg_ptr BagPlay::convertMsg(record::BaseMsg_ptr src_msg) {
    switch (src_msg->getType()) {

        case record::MSG_LASER_SCAN_STAMPED:
        case record::PF_LASER_SCAN_STAMPED:
        case record::UHD_LASER_SCAN_STAMPED: {
            auto msg = std::dynamic_pointer_cast<record::LaserScanStampedMsg>(src_msg);
            auto laser_msg = std::make_shared<sros::bag::LaserScanMsg>();
            laser_msg->topic_ = "BAG_SCAN";//当前发布默认的scantopic
            laser_msg->time_ = msg->timestamp;
            laser_msg->angle_min = msg->angle_min;
            laser_msg->angle_max = msg->angle_max;
            laser_msg->angle_increment = msg->angle_increment;
            laser_msg->time_increment = msg->time_increment;
            laser_msg->scan_time = msg->scan_time;
            laser_msg->range_min = msg->range_min;
            laser_msg->range_max = msg->range_max;
            laser_msg->ranges.swap(msg->ranges);
            laser_msg->intensities.swap(msg->intensities);//交换过去
            return laser_msg;
//            break;
        }
        case record::MSG_POSE_STAMPED: {
            auto msg = std::dynamic_pointer_cast<record::PoseStampedMsg>(src_msg);
            auto pose_msg = std::make_shared<sros::bag::PoseStampedMsg>("BAG_POSE");
            pose_msg->pose.x() = msg->getX();
            pose_msg->pose.y() = msg->getY();
            pose_msg->pose.yaw() = msg->getYaw();
            pose_msg->pose.z() = msg->getZ();
            pose_msg->pose.roll() = msg->getRoll();
            pose_msg->pose.pitch() = msg->getPitch();
            pose_msg->time_ = msg->getTimestamp();
//            //LOG(INFO) << "the roll is:" << pose_msg->pose.roll();
            //msg = std::make_shared<PoseStampedMsg>();
            return pose_msg;
//            break;
        }
        case record::MSG_GPS: {
            auto pose_msg = std::make_shared<sros::bag::PoseStampedMsg>("GPS_TOPIC");
            return pose_msg;
        }
        case record::MSG_SLAM_INFO: {
            record::SlamInfoMsg_ptr msg = std::dynamic_pointer_cast<record::SlamInfoMsg>(src_msg);
            //LOG(INFO) << "angle_min: " << msg->angle_min;
            //LOG(INFO) << "angle_max: " << msg->angle_max;
            //LOG(INFO) << "range_min: " << msg->range_min;
            //LOG(INFO) << "range_max: " << msg->range_max;
            //LOG(INFO) << "intensity: " << msg->intensity;
            //LOG(INFO) << "coord_x: " << msg->coord_x;
            //LOG(INFO) << "coord_y: " << msg->coord_y;
            //LOG(INFO) << "coord_yaw: " << msg->coord_yaw;
            break;
        }
        default:
            //LOG(INFO) << "error to decode the type!!!!";
            break;

    }
    return 0;


}

void BagPlay::sendMsg(sros::bag::base_msg_ptr obj_msg) {
//TODO:填充完整
    if (sendmsg_callback_func)
        sendmsg_callback_func(obj_msg);
    else
        srosoutstream << "error to get the func! cannot publish the msg!" << endl;
}

BagPlay::BagPlay() : is_pause(false), is_stop(false) {
    options_.time_scale = 1.0;

}

BagPlay::~BagPlay() {

}

//BagPlay::BagPlay(PlayerOptions options) : options_(options), is_pause(false), is_stop(false) {
//
//}

void BagPlay::playPause() {
    is_pause = true;

}

void BagPlay::playContinue() {
    is_pause = false;
    pause_condition_.notify_all();
}

void BagPlay::playStop() {
    if (bag.isopen()) {
        bag.closeBag();
    }
    is_stop = true;
}

void BagPlay::setMsgCallbackFunc(MsgCallbackFunc callback) {
//    std::cout << "set!!!" << endl;
    sendmsg_callback_func = callback;
}

void BagPlay::setScale(float time_scale) {
    options_.time_scale = time_scale;
    globalbagconfig.time_scale = time_scale;
}

float BagPlay::getScale() {
    return globalbagconfig.time_scale;
}

void BagPlay::playThread() {
    while (!is_stop) {
        if (is_pause) {
            while (is_pause) {
                boost::unique_lock<boost::mutex> lock(pause_mutex_);
                boost::xtime xt;
#if BOOST_VERSION >= 105000
                boost::xtime_get(&xt, boost::TIME_UTC_);
#else
                boost::xtime_get(&xt, boost::TIME_UTC);
#endif
                xt.sec += 4;
                if (pause_condition_.timed_wait(lock, xt)) {
                    srosoutstream << "will continue" << endl;
                } else {
//                    sros::bag::SlamCommandMsg syscommand;
//                    bag::globalbagconfig.handleContinuePlay(syscommand);//防止出现死机情况
                    srosoutstream << "still stop！ will force to continue" << endl;
                }
            }
            last_walltime = sros::core::util::get_time_in_us() + 100000;
        }
        playOnce();
    }
}

void BagPlay::setRecordMsgCallback(RecordMsgCallbackFunc callback) {
    record_msg_callback = callback;

}

int64_t BagPlay::getReadPosition() {
    if(!is_stop) {
        return bag.getReadPosition();
    }else {
        //LOG(INFO) << "play is stop! will return -1";
        return -1;
    }
}

bool BagPlay::openToPlay(std::string filename) {
    bagfilename = filename;
    string path = bagfilename + ".bag";
    if (open(path.c_str(), ios::in)) {
        first_send_scan = true;
        is_stop = false;//初始化标志位
        //LOG(INFO) << "successfully to get the bag!";
        return true;
    } else {
        //LOG(INFO) << "cannot open the file! will return\n" << filename;
        return false;
    }
}

void BagPlay::playOnce() {
    int64_t bag_time;
    int topic_count;
    bag.readStamp(bag_time);
    bag.readMsgcount(topic_count);//当前只有一个
    if (topic_count == -1) {
        srosoutstream << "get the bag end! will return!" << endl;
        bag::BagCommandMsg_Ptr bag_cmd(new BagCommandMsg);
        if (bag::globalbagconfig.handleStopPlay) {
            bag::globalbagconfig.handleStopPlay(bag_cmd);
        }else {
            if (stopCallback) {
                stopCallback();
            }
        }
        playStop();
        return;
    } else if (topic_count < 0) {
        srosoutstream << "get the errror number! will return false" << endl;
        bag::BagCommandMsg_Ptr bag_cmd(new bag::BagCommandMsg);
        if (bag::globalbagconfig.handleStopPlay) {
            bag::globalbagconfig.handleStopPlay(bag_cmd);
        }else{
            if (stopCallback) {
                stopCallback();
            }
        }
        playStop();
        return;
    }
//        bag::globalbagconfig.time_scale = 3;
    for (int i = 0; i < topic_count; ++i) {
        string topic_name;
        bag.readTopicname(topic_name);
        record::BaseMsg_ptr msg_ptr;
        msg_ptr = readMsg();
        sros::bag::base_msg_ptr topic_msg_ptr;
        topic_msg_ptr = convertMsg(msg_ptr);
//            topic_msg_ptr->topic_ = topic_name;//如果修改topic，在这里修改，发布什么topic，返回什么topic
        int64_t now_time = sros::core::util::get_time_in_us();
        waitforSyntime(now_time, bag_time);
        if (topic_msg_ptr) {
            switch (topic_msg_ptr->type_) {
                case sros::bag::TYPE_LASER_SCAN_DATA:
                    if (sendmsg_callback_func) {
                        sendMsg(topic_msg_ptr);
                    } else if(bag::globalbagconfig.bagscanCallback){
                        bag::globalbagconfig.bagscanCallback(topic_msg_ptr);
                        int64_t progress = (bag_time - bag_beg_time) * 100 / (bag_end_time - bag_beg_time);
                    } else {
                        //LOG(INFO) << "cannot find the scan callback!";
                    }
                    break;
                case sros::bag::TYPE_POSE_STAMPED_DATA:
                    if (sendmsg_callback_func){
                        sendMsg(topic_msg_ptr);
                    } else if(bag::globalbagconfig.posestampedCallback){
                        bag::globalbagconfig.posestampedCallback(topic_msg_ptr);
                    } else{
                        //LOG(INFO) << "cannot find the pose callback!";
                    }
                    break;
            }
        }

        if (topic_name == "TOPIC_LASER") {
            //                srosoutstream << "the shedule is:%" << progress << endl;
        } else if (topic_name == "OdoPoseStamped") {


        } else if(topic_name=="GPS_TOPIC") {
            if (record_msg_callback) {
                record_msg_callback(msg_ptr);
            }
        }
    }
}
}
