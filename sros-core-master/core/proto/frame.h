//
// Created by lhx on 17-1-9.
//
// Copyright 2018 Standard Robots
//

#ifndef SROS_MODULES_NETWORK_PROTO_FRAME_H_
#define SROS_MODULES_NETWORK_PROTO_FRAME_H_


#include <memory>

#include "./main.pb.h"

namespace proto {

typedef std::shared_ptr<Message> Message_ptr;

/**
 *
 *
 * |-------------------------------|
 * | FRAME_SIZE |       DATA       |
 * |-------------------------------|
 * |             |    FRAME_SIZE   |
 * |     HEADER  |       BODY      |
 *
 *
 *
 */
class Frame;

typedef std::shared_ptr<Frame> Frame_ptr;

class Frame {
public:

    static const int FRAME_HEADER_LEN = 4;

    static const int MAX_SIZE = 256 * 1024;

    uint8_t data[MAX_SIZE];

    size_t total_size;

    uint8_t* get_frame_data() {
        return data + FRAME_HEADER_LEN;
    }

    void clear();

    static size_t getFrameBodySize(const Frame &frame);

    static Message_ptr buildMessage(Frame &frame);

    static Frame_ptr buildFrame(Message_ptr);
};



} // namespace proto

#endif // SROS_MODULES_NETWORK_PROTO_FRAME_H_
