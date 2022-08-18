/*
 * MyThread.h
 *
 *  Created on: 2015年12月2日
 *      Author: lhx
 */

#ifndef SROSBAG_PUB_BAG_MODULE
#define SROSBAG_PUB_BAG_MODULE

#include "srosbag/bagmodule/bag_play.h"
#include "srosbag/bagmodule/bag_record.h"
#include "srosbag/msg/pub_msg/base_msg.h"
#include "srosbag/bagmodule/global_bag_config.h"

namespace bag {

class BagModule {
public:
    BagModule();

    virtual ~BagModule();

    void scanCallback(sros::bag::base_msg_ptr scan_ptr);

    void poseCallback(sros::bag::base_msg_ptr pose_ptr);

    void syscommandCallback(sros::bag::base_msg_ptr base_ptr);

    bool handleStartRecord(bag::BagCommandMsg_Ptr syscommand);

    bool handleStopRecord(bag::BagCommandMsg_Ptr syscommand);

    bool handleCancelBag(bag::BagCommandMsg_Ptr syscommand);

    bool handleStartPlay(bag::BagCommandMsg_Ptr syscommand);

    bool handlePausePlay(bag::BagCommandMsg_Ptr syscommand);

    bool handleStopPlay(bag::BagCommandMsg_Ptr syscommand);

    bool handleContinuePlay(bag::BagCommandMsg_Ptr syscommand);

    void sendMsg(sros::bag::base_msg_ptr msg_ptr);



private:
    std::shared_ptr<bag::BagRecord> record_ptr;
    std::shared_ptr<bag::BagPlay> play_ptr;
    bag::PlayerOptions play_options;
    bag::RecordOptions record_options;
    std::string recordname;
};

} /* namespace SR */

#endif /* MYTHREAD_H_ */
