//
// Created by lfc on 17-8-14.
//

#ifndef SROS_BAG_MANAGER_SINGLETON_H
#define SROS_BAG_MANAGER_SINGLETON_H

#include <memory>
#include <boost/thread.hpp>
#include <boost/function.hpp>
#include "msg/pub_msg/base_msg.h"
namespace bag{
class BagPlay;
class BagRecord;
}

namespace bag_manager {
typedef boost::function<void(sros::bag::base_msg_ptr)> MsgCallbackFunc;
typedef boost::function<void()> CmdCallbackFunc;
class PlayOption{
public:
    float time_scale = 1.0;
    MsgCallbackFunc msgCallback;
    CmdCallbackFunc stopCallback;
};
typedef std::shared_ptr<PlayOption> PlayOption_Ptr;

typedef boost::function<void(PlayOption_Ptr)> OptionCallback;


class BagManagerSingleton {
public:
    BagManagerSingleton();

    static std::shared_ptr<BagManagerSingleton> getInstance();

    bool startRecord(std::string bag_name);

    bool recordMsg(sros::bag::base_msg_ptr base_ptr);

    void stopRecord();


    bool startPlay(std::string bag_name,OptionCallback callback);

    bool cancelPlay(std::string file_name);

    int64_t getBagEndTime();

    int64_t getBagBegTime();

    virtual ~BagManagerSingleton();

    void stopPlay();

private:
    bool removeFile(std::string filename);

    void playThreadLoop();

    void wakeupPlayThread();

    bool playOK();

    static std::shared_ptr<BagManagerSingleton> bag_module;
    static boost::mutex thread_mutex;

    std::shared_ptr<bag::BagPlay> bag_play;
    std::shared_ptr<bag::BagRecord> bag_record;

    boost::thread thread_manager;

    int64_t bag_beg_time;
    int64_t bag_end_time;

    int64_t last_write_position;
    const int64_t position_offset = 100;
    PlayOption_Ptr play_option;
    CmdCallbackFunc stopCallback;

    bool wakeup_state;
    boost::mutex wakeup_mutex;
    boost::condition_variable_any condition;
};
}


#endif //SROS_BAG_MANAGER_SINGLETON_H
