/*
 * MyThread.cpp
 *
 *  Created on: 2015年12月2日
 *      Author: lhx
 */

#include <iostream>
#include <memory>

// #include <glog/logging.h>
#include "bag_module.h"

namespace bag {

BagModule::BagModule(){
    record_ptr = std::make_shared<bag::BagRecord>();
    play_ptr = std::make_shared<bag::BagPlay>();
    recordname = "defaultrecord";

    bag::globalbagconfig.sethandleCancelBagCallback(boost::bind(&BagModule::handleCancelBag,this,_1));
    bag::globalbagconfig.sethandleStartPlayCallback(boost::bind(&BagModule::handleStartPlay,this,_1));
    bag::globalbagconfig.sethandleStartRecordCallback(boost::bind(&BagModule::handleStartRecord,this,_1));
    bag::globalbagconfig.sethandleStopPlayCallback(boost::bind(&BagModule::handleStopPlay,this,_1));
    bag::globalbagconfig.sethandleStopRecordCallback(boost::bind(&BagModule::handleStopRecord,this,_1));
    bag::globalbagconfig.sethandlePausePlayCallback(boost::bind(&BagModule::handlePausePlay,this,_1));
    bag::globalbagconfig.sethandleContinuePlayCallback(boost::bind(&BagModule::handleContinuePlay,this,_1));


}

BagModule::~BagModule() {

}

void BagModule::scanCallback(sros::bag::base_msg_ptr scan_ptr) {
    if (bag::globalbagconfig.bag_record) {
        if(record_ptr->ok()){
            record_ptr->scanCallback(scan_ptr);
        }

    }
}

void BagModule::poseCallback(sros::bag::base_msg_ptr pose_ptr) {
    if (bag::globalbagconfig.bag_record) {
        record_ptr->poseCallback(pose_ptr);
    }
}

void BagModule::syscommandCallback(sros::bag::base_msg_ptr base_ptr) {

    //TODO:执行指令
}

bool BagModule::handleStartRecord(bag::BagCommandMsg_Ptr syscommand) {
    srosoutstream << "will start to record" << std::endl;
    srosoutstream << "begin to record" << std::endl;
    recordname = syscommand->getBagPath() + syscommand->getBagName();
    return record_ptr->doRecord(recordname);

}

bool BagModule::handleStopRecord(bag::BagCommandMsg_Ptr syscommand) {
    srosoutstream << "will stop to record\n";
    bag::globalbagconfig.bag_record=false;
    record_ptr->close();
    return true;
}

bool BagModule::handleCancelBag(bag::BagCommandMsg_Ptr syscommand) {
    srosoutstream << "will stop to record\n";
    bag::globalbagconfig.bag_record=false;
    record_ptr->close();
    return true;

}

bool BagModule::handleStartPlay(bag::BagCommandMsg_Ptr syscommand) {
    bag::globalbagconfig.bag_play=true;
    srosoutstream << "will play the record\n";
    recordname = syscommand->getBagPath() + syscommand->getBagName();
    srosoutstream << "will start to play\n";
    printf("the record name is:%s\n", recordname.c_str());
    return play_ptr->doPlay(recordname);
}

bool BagModule::handlePausePlay(bag::BagCommandMsg_Ptr syscommand) {
    srosoutstream << "will pause the play\n";
    play_ptr->playPause();
    return true;

}

bool BagModule::handleStopPlay(bag::BagCommandMsg_Ptr syscommand) {
    srosoutstream << "will stop the play\n";
    play_ptr->playStop();
    bag::globalbagconfig.bag_play = false;//
    return true;
}

bool BagModule::handleContinuePlay(bag::BagCommandMsg_Ptr syscommand) {
    play_ptr->playContinue();//slam供调用
    return true;
}

void BagModule::sendMsg(sros::bag::base_msg_ptr msg_ptr) {

}
} /* namespace SR */

