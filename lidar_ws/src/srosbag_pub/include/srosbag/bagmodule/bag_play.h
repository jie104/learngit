//
// Created by lfc on 16-7-22.
//

#ifndef TESTBIN_PLAY_H
#define TESTBIN_PLAY_H

#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include "bag.h"
#include "bag_record.h"
#include "../msg/record_msg/all_msg.h"
#include "play_option.hpp"
typedef boost::function<void(record::BaseMsg_ptr)> RecordMsgCallbackFunc;

typedef boost::function<void(sros::bag::base_msg_ptr)> MsgCallbackFunc;
namespace bag {
struct PlayerOptions {
    float time_scale;//该值是一种对时间尺度的测量。0.5表示，bag数据在播放时，减慢成原来的0.5倍；2表示，增快了2倍
};

typedef boost::function<void()> CmdCallbackFunc;

class BagPlay {
public:
    BagPlay();

    ~BagPlay();

//    BagPlay(PlayerOptions options);

    bool open(const char *path, std::ios::openmode mode);

    void close();

    bool doPlay(std::string filename,bool publish_shcedule = true);

    bool openToPlay(std::string filename);

    void playOnce();

    record::BaseMsg_ptr readMsg();

    sros::bag::base_msg_ptr convertMsg(record::BaseMsg_ptr src_msg);

    bool waitforSyntime(int64_t now_time, int64_t bagtime);

    void sendMsg(sros::bag::base_msg_ptr obj_msg);

    void playPause();

    void playContinue();

    void playStop();

    void setMsgCallbackFunc(MsgCallbackFunc callback);

    void setRecordMsgCallback(RecordMsgCallbackFunc callback);

    void setScale(float time_scale);

    float getScale();

    void playThread();

    void setStopCallback(CmdCallbackFunc func){
        stopCallback = func;
    }

    int64_t getBegTime() {
        return bag_beg_time;
    }

    int64_t getEndTime() {
        return bag_end_time;
    }

    bool ok() {
        return !is_stop;
    }

    int64_t getReadPosition();
private:
    Bag bag;
    std::string bagfilename;
    record::PoseStampedMsg header_msg;
    boost::mutex pause_mutex_;
    boost::condition_variable_any pause_condition_;
    //在暂停时，使用
    int64_t last_bagtime;
    int64_t last_walltime;
    PlayerOptions options_;
    bool is_pause;
    bool is_stop;
    bool is_publish_shcedule;
    MsgCallbackFunc sendmsg_callback_func;

    RecordMsgCallbackFunc record_msg_callback;

    CmdCallbackFunc stopCallback;
    bool first_send_scan;
    int64_t bag_end_time;
    int64_t bag_beg_time;
};
}


#endif //TESTBIN_PLAY_H
