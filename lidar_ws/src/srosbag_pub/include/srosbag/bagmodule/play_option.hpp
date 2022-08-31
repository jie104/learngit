//
// Created by lfc on 17-9-1.
//

#ifndef SRC_PLATFORM_PLAY_OPTION_HPP
#define SRC_PLATFORM_PLAY_OPTION_HPP


#include <memory>
#include <boost/function.hpp>
#include "../msg/pub_msg/base_msg.h"
namespace bag{
typedef boost::function<void(sros::bag::base_msg_ptr)> MsgCallbackFunc;
typedef boost::function<void()> CmdCallbackFunc;
class PlayOption {
public:
    float time_scale = 1.0;//该值是一种对时间尺度的测量。0.5表示，bag数据在播放时，减慢成原来的0.5倍；2表示，增快了2倍
    MsgCallbackFunc poseMsgCallback;
    MsgCallbackFunc scanMsgCallback;
    CmdCallbackFunc stopCmdCallback;
};

typedef std::shared_ptr<PlayOption> PlayOption_Ptr;

}

#endif //SRC_PLATFORM_PLAY_OPTION_HPP
