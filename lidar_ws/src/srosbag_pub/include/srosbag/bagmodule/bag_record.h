//
// Created by lfc on 16-7-22.
//

#ifndef TESTBIN_RECORD_H
#define TESTBIN_RECORD_H

#include <boost/thread/pthread/mutex.hpp>
#include <boost/thread/pthread/condition_variable.hpp>
#include <queue>
#include "../msg/record_msg/all_msg.h"
#include "../msg/pub_msg/base_msg.h"
#include "bag.h"

namespace bag {
struct RecordOptions {
    std::string bag_name;
};

class BagRecord {
public:
    BagRecord();

    ~BagRecord();

    bool open(const char *path, std::ios::openmode mode);

    void close();

    void writeMsg(record::BaseMsg_ptr msg_ptr);

    bool doRecord(std::string filename);

    bool removeFile(std::string filename);

    void scanCallback(sros::bag::base_msg_ptr base_ptr);

    void poseCallback(sros::bag::base_msg_ptr base_ptr);
    
    void slamInfoCallback(sros::bag::base_msg_ptr base_ptr);

    bool ok();


    void recordMsg(sros::bag::base_msg_ptr base_ptr);

    void recordMsg(record::BaseMsg_ptr baseMsg_ptr, std::string topic_name);

    int64_t getRecordPosition();

    int64_t getBegTime() {
        return bag_beg_time;
    }

    int64_t getEndTime() {
        return bag_end_time;
    }


    //TODO:check size
private:

    void recordStop();
    Bag bag;
    std::string bagfilename;
    boost::mutex queue_mutex_;
    boost::condition_variable_any queue_condition_;
    std::queue<record::BaseMsg_ptr> queue_;
    std::queue<std::string> queue_topic;
    bool running;
    int queue_size_;
    std::shared_ptr<record::LaserScanStampedMsg> laser_msg;
    bool first_record_scan;

    int64_t bag_beg_time;
    int64_t bag_end_time;

};
}


#endif //TESTBIN_RECORD_H
