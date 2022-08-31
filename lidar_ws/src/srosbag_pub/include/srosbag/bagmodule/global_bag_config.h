//
// Created by lfc on 16-8-6.
//

#ifndef SROS_GLOBALBAGVALUE_H
#define SROS_GLOBALBAGVALUE_H

#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include "../msg/pub_msg/base_msg.h"
namespace bag {
class BagCommandMsg{
public:
    BagCommandMsg() : is_stop(false), is_pause(false) {

    }

    void setBagName(std::string name) {
        bag_name = name;
    }

    std::string getBagName() {
        return bag_name;
    }
    ~BagCommandMsg(){

    }

    void setBagPath(std::string name) {
        bag_path = name;
    }

    std::string getBagPath() {
        return bag_path;
    }
    std::string bag_path;
    std::string bag_name;
private:
    bool is_stop;
    bool is_pause;

};
typedef std::shared_ptr<bag::BagCommandMsg> BagCommandMsg_Ptr;
typedef boost::function<bool(BagCommandMsg_Ptr)> CommandCallbackFunc;
typedef boost::function<void(sros::bag::base_msg_ptr)> msgCallbackFunc;

class GlobalBagConfig {
public:
    GlobalBagConfig();

    ~GlobalBagConfig();

    void sethandleStartRecordCallback(CommandCallbackFunc func);

    void sethandleStopRecordCallback(CommandCallbackFunc func);

    void sethandleCancelBagCallback(CommandCallbackFunc func);

    void sethandleStartPlayCallback(CommandCallbackFunc func);

    void sethandleStopPlayCallback(CommandCallbackFunc func);

    void sethandlePausePlayCallback(CommandCallbackFunc func);

    void sethandleContinuePlayCallback(CommandCallbackFunc func);

    void setbagscanCallback(msgCallbackFunc func);

    void setposestampedCallback(msgCallbackFunc func);


    bool bag_record;
    bool bag_pause;
    bool bag_play;
    float time_scale;
    CommandCallbackFunc handleStartRecord;
    CommandCallbackFunc handleStopRecord;
    CommandCallbackFunc handleCancelBag;
    CommandCallbackFunc handleStartPlay;
    CommandCallbackFunc handleStopPlay;
    CommandCallbackFunc handlePausePlay;
    CommandCallbackFunc handleContinuePlay;
    msgCallbackFunc bagscanCallback;
    msgCallbackFunc posestampedCallback;
private:


};

extern GlobalBagConfig globalbagconfig;
}

#endif //SROS_GLOBALBAGVALUE_H
