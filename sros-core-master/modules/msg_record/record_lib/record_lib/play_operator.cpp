//
// Created by lfc on 17-10-27.
//

#include "play_operator.h"
#include <dirent.h>
#include <sys/stat.h>
#include <boost/thread.hpp>
#include "record_msg/record_msg_factory.h"
namespace record{

PlayOperator::PlayOperator(PlayOption option) {
    play = option;

}

PlayOperator::~PlayOperator() {

}

bool PlayOperator::open(const char *path) {
    std::string path_=getPath(path);
    if (opendir(path_.c_str()) == NULL) {
        LOG(INFO) << "no dir! will create!" << path;
        if (mkdir(path_.c_str(), 0777) < 0) {
            LOG(INFO) << "error to create!";
            return false;
        }
    }
    op_file.open(path, std::ios::in);

    boost::thread(boost::bind(&PlayOperator::playThread, this));
    return op_file.is_open();
}

void PlayOperator::close() {
    op_file.close();
}

bool PlayOperator::isOpen() {
    return op_file.is_open();
}

void PlayOperator::playThread() {
    while (isOpen()) {
        MsgMap msg_map;
        if (decodeMsg(msg_map)) {
            auto type_iter= msg_map.find(convertToStr(type));
            if (type_iter != msg_map.end()) {
                auto msg = RecordMsgFactory::getRecordMsg(RecordMsgType(type_iter->second[0]));
                if (msg) {
                    msg->decode(msg_map);
                    play.msgCallback(msg);//发布msg
                }
            }else {
                LOG(INFO) << "err to get the msg!";
            }

        }else {
            LOG(INFO) << "get the stop signal!";
            play.playStop();//stopmsg
            close();
            return;
        }
    }

}

std::string PlayOperator::getPath(const char *path) {
    std::string file_name(path);
    int str_length = file_name.find_last_of('/');
    return file_name.substr(0, str_length);
}

bool PlayOperator::readLine(std::stringstream &data) {
    if (!op_file.eof()) {
        std::string tmp_str;
        std::getline(op_file, tmp_str);
        data << tmp_str;
        return true;
    }else {
        LOG(INFO) << "get the end! will return!";
        return false;
    }
}

bool PlayOperator::decodeMsg(MsgMap& msg_map) {
    std::stringstream stream_;
    if (readLine(stream_)) {
        std::string value_name;
        stream_ >> value_name;
        while (value_name != end_tag){

            double value;
            while (stream_ >> value) {
                msg_map[value_name].push_back(value);
            }
            stream_.clear();
            stream_.str("");
            if (!readLine(stream_)) {
                LOG(INFO) << "read file end! will return!";
                return false;
            }
            stream_ >> value_name;
        }
        return true;
    }
    LOG(INFO) << "read file end! will return!";
    return false;
}
}