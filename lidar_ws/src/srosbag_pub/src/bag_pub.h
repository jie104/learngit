//
// Created by lfc on 16-8-20.
//

#ifndef SROSBAG_PUB_BAGPUB
#define SROSBAG_PUB_BAGPUB

#include <nav_msgs/Path.h>
#include "bag_module.h"
#include "ros/publisher.h"
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "keyboard_manager.h"
//#include
namespace bag{
class BagPub {
public:
    BagPub();

    ~BagPub();

    void bagscanCallback(sros::bag::base_msg_ptr base_msg);

    void bagposeCallback(sros::bag::base_msg_ptr base_msg);

    void setBagName(std::string& bagname);

    void setBagPath(std::string& bagpath);

    void startPlayBag();


    void handlePause();

    void handleContinue();

    void handleKeyCmd(keyboard::KeyCmdType type);

    bool laterThanBeginTime(int64_t curr_time);


private:
    std::shared_ptr<KeyboardManager> keyboard_manager;
    bag::BagCommandMsg_Ptr bag_cmd;
    bag::BagModule bagmodule;
    ros::NodeHandle node_;
    ros::Publisher scan_pub;
    ros::Publisher pose_pub;
    ros::Publisher path_pub;
    nav_msgs::Path path_vec;

    std::string bag_path;
    std::string bag_name;
    float time_scale;
    int64_t begin_play_time = 0;


    tf::TransformBroadcaster tfB_;
    tf::StampedTransform transform_scan_to_base;
    int count;


};

}



#endif //PROJECT_BAGPUB_H
