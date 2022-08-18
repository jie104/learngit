//
// Created by lfc on 16-8-6.
//

#ifndef SROS_GLOBALBAGVALUE_H
#define SROS_GLOBALBAGVALUE_H

#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <core/msg/SlamCommandMsg.h>

namespace bag {
typedef boost::function<bool(sros::core::SlamCommandMsg &)> CommandCallbackFunc;
typedef boost::function<void(sros::core::base_msg_ptr)> msgCallbackFunc;

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
