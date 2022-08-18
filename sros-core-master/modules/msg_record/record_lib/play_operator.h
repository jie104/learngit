//
// Created by lfc on 17-10-27.
//

#ifndef SROS_PLAY_OPERATOR_H
#define SROS_PLAY_OPERATOR_H

#include "record_msg/base_record_msg.hpp"
#include <boost/function.hpp>
namespace record{

typedef boost::function<void()> StopCallbackFunc;
typedef boost::function<void(BaseRecordMsg_Ptr)> MsgCallbackFunc;

struct PlayOption{
    StopCallbackFunc playStop;
    MsgCallbackFunc msgCallback;
};


class PlayOperator {
public:

    PlayOperator(PlayOption option);

    virtual ~PlayOperator();

    bool open(const char *path);

    void close();

    bool isOpen();


private:
    std::string getPath(const char *path);

    bool readLine(std::stringstream &data);

    bool decodeMsg(MsgMap& msg_map);

    void playThread();

    PlayOperator(){

    }

    PlayOption play;

    std::fstream op_file;

    const std::string end_tag = "END";

};

}


#endif //SROS_PLAY_OPERATOR_H
