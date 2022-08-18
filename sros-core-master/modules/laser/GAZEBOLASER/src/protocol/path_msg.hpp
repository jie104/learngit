//
// Created by lhx on 16-1-20.
//

#ifndef SRC_SDK_NETWORK_PROTOCOL_PATH_MSG_H
#define SRC_SDK_NETWORK_PROTOCOL_PATH_MSG_H

#include "base_msg.hpp"

#include <assert.h>
#include <vector>

namespace network {

class PathMsg : public BaseMsg {
public:
    PathMsg() : BaseMsg(MSG_PATH), path_num_(0) {
        assert(sizeof(PATH_t) == 44);
    };
    virtual ~PathMsg() { };

    virtual bool encodeBody() override {
        if (!path_num_) {
            return false;
        }

        encode_field(path_num_);

        for(auto item : paths_) {
            encode_field(item);
        }

        // 需要保证不会溢出data_数组
        return data_offset_ <= MAX_DATA_LENGTH;
    }

    virtual bool decodeBody() override {
        PATH_t path;

        decode_field(path_num_);

        if (path_num_ <= 0) {
            return false;
        }

        for (int i = 0; i < path_num_; i++) {
            decode_field(path);
            paths_.push_back(path);
        }

        return true;
    }

    const std::vector<PATH_t> &getPaths() const {
        return paths_;
    }

    bool setPaths(const std::vector<PATH_t> &paths) {
        if (paths.size() <= MAX_PATH_NUM && !paths.empty()) {
            paths_ = paths;
            path_num_ = (PATH_NUM_t) paths_.size();
            return true;
        } else {
            return false;
        }
    }

    const static unsigned long MAX_PATH_NUM = 9; // 最多可容纳的path个数

private:
    typedef unsigned char PATH_NUM_t;
    PATH_NUM_t path_num_;
    std::vector<PATH_t> paths_;

};

typedef std::shared_ptr<PathMsg> PathMsg_ptr;

}


#endif //SRC_SDK_NETWORK_PROTOCOL_PATH_MSG_H
