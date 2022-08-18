//
// Created by lhx on 17-1-9.
//
// Copyright 2018 Standard Robots
//

#include "frame.h"

#include <iostream>
#include <memory>

namespace proto {

void Frame::clear() {
    total_size = 0;

    memset(data, 0, MAX_SIZE);
}

size_t Frame::getFrameBodySize(const Frame &frame) {

    size_t frame_size = 0;
    frame_size += frame.data[0] << 24;
    frame_size += frame.data[1] << 16;
    frame_size += frame.data[2] << 8;
    frame_size += frame.data[3] << 0;

    if (frame_size > Frame::MAX_SIZE - Frame::FRAME_HEADER_LEN) {
        std::cout << "wrong frame size = " << frame_size<<std::endl;
        return 0;
    } else {
        return frame_size;
    }
}

Message_ptr Frame::buildMessage(Frame &frame) {
    Message_ptr msg = std::make_shared<Message>();

    if (msg) {
        msg->ParseFromArray(frame.data + Frame::FRAME_HEADER_LEN,
                            static_cast<int>(frame.total_size - Frame::FRAME_HEADER_LEN));
    }

    return msg;
}

Frame_ptr Frame::buildFrame(Message_ptr msg) {
    Frame_ptr frame = std::make_shared<Frame>();

    size_t frame_body_size = msg->ByteSizeLong();
    frame->total_size = Frame::FRAME_HEADER_LEN + frame_body_size;

    // 确保不会溢出
    assert(frame->total_size <= Frame::MAX_SIZE);

    frame->data[0] = (uint8_t) (frame_body_size >> 24);
    frame->data[1] = (uint8_t) (frame_body_size >> 16);
    frame->data[2] = (uint8_t) (frame_body_size >> 8);
    frame->data[3] = (uint8_t) (frame_body_size >> 0);

    msg->SerializeToArray(frame->data + Frame::FRAME_HEADER_LEN, Frame::MAX_SIZE - Frame::FRAME_HEADER_LEN);

    return frame;
}

} // namespace proto
