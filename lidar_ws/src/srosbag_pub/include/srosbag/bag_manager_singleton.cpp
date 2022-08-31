//
// Created by lfc on 17-8-14.
//

// #include <glog/logging.h>
#include "bag_manager_singleton.h"
#include "bagmodule/bag_record.h"
#include "bagmodule/bag_play.h"

namespace bag_manager{

std::shared_ptr<BagManagerSingleton> BagManagerSingleton::bag_module;
boost::mutex BagManagerSingleton::thread_mutex;

BagManagerSingleton::BagManagerSingleton() {

}

std::shared_ptr<BagManagerSingleton> BagManagerSingleton::getInstance() {
    if (!bag_module) {
        boost::mutex::scoped_lock lock(thread_mutex);
        if (!bag_module) {
            bag_module.reset(new BagManagerSingleton());
        }
    }
    return bag_module;
}

bool BagManagerSingleton::startRecord(std::string bag_name) {
    last_write_position = -1;
    if (bag_record) {
        bag_record.reset();
    }
    bag_beg_time = -1;
    bag_end_time = -1;
    bag_record.reset(new bag::BagRecord);
    if(bag_record->doRecord(bag_name)) {
        return true;
    }else {
        //LOG(INFO) << "err to open the file! will return false!";
        bag_record.reset();
        return false;
    }
}

bool BagManagerSingleton::recordMsg(sros::bag::base_msg_ptr base_ptr) {
    if (bag_record && bag_record->ok()) {
        last_write_position = bag_record->getRecordPosition();
        bag_record->recordMsg(base_ptr);

        if (base_ptr->type_ == sros::bag::TYPE_LASER_SCAN_DATA) {
            wakeupPlayThread();
        }
        return true;
    }else{
        return false;
    }
}

void BagManagerSingleton::stopRecord() {
    if (bag_record && bag_record->ok()) {
        bag_record->close();
        bag_beg_time = bag_record->getBegTime();
        bag_end_time = bag_record->getEndTime();
        bag_record.reset();
        //LOG(INFO) << "successfully to stop!";
    }
}

bool BagManagerSingleton::startPlay(std::string bag_name,OptionCallback callback) {
    if (bag_play) {
        bag_play.reset();
    }
    bag_play.reset(new bag::BagPlay);

    play_option.reset(new PlayOption);

    if (callback) {
        callback(play_option);
        bag_play->setMsgCallbackFunc(play_option->msgCallback);
        bag_play->setStopCallback(boost::bind(&BagManagerSingleton::stopPlay,this));
        stopCallback = play_option->stopCallback;
        bag_play->setScale(play_option->time_scale);
    }
    wakeup_state = true;
    if (playOK()) {
        bag_play->openToPlay(bag_name);
    }else {
        //LOG(INFO) << "err to play file name!:" << bag_name;
    }
    thread_manager = boost::thread(boost::bind(&BagManagerSingleton::playThreadLoop, this));
    return true;

//    if(bag_play->doPlay(bag_name)) {
//        return true;
//    }else {
//        //LOG(INFO) << "err to play the bag:" << bag_name;
//        return false;
//    }
}


int64_t BagManagerSingleton::getBagEndTime() {
    return bag_end_time;
}

int64_t BagManagerSingleton::getBagBegTime() {
    return bag_beg_time;
}

BagManagerSingleton::~BagManagerSingleton() {

}

bool BagManagerSingleton::removeFile(std::string filename) {
    if (!bag_record) {
        bag_record.reset(new bag::BagRecord);
    }
    return bag_record->removeFile(filename);
}

void BagManagerSingleton::playThreadLoop() {

    while (playOK()) {
        if (bag_record&&bag_record->ok()) {
            auto read_position = bag_play->getReadPosition();
            if (read_position > last_write_position - position_offset) {
                boost::unique_lock<boost::mutex> lock(wakeup_mutex);
                boost::xtime xt;
#if BOOST_VERSION >= 105000
                boost::xtime_get(&xt, boost::TIME_UTC_);
#else
                boost::xtime_get(&xt, boost::TIME_UTC);
#endif
                xt.nsec += 5e8;
                wakeup_state = true;
                if (condition.timed_wait(lock, xt)) {
                    wakeup_state = false;
                    if (!playOK()) {
                        continue;
                    }
                }
            }
            if (read_position < last_write_position - position_offset) {
                bag_play->playOnce();
            }
        }else {
            bag_play->playOnce();
        }
    }
}

void BagManagerSingleton::stopPlay() {
//    thread_manager.join();
    if (stopCallback) {
        stopCallback();
    } else {
        //LOG(INFO) << "err to call stop callback!";
    }
}

bool BagManagerSingleton::cancelPlay(std::string file_name) {
    if (bag_play) {
        bag_play->playStop();
    }
    thread_manager.join();
    removeFile(file_name);
    return true;
}

void BagManagerSingleton::wakeupPlayThread() {

    if (wakeup_state) {
        condition.notify_all();
    }
}

bool BagManagerSingleton::playOK() {
    if (bag_play) {
        return bag_play->ok();
    } else {
        return false;
    }
}
}