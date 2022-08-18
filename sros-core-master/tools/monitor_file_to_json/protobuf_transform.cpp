//
// Created by huangwuxian on 19-6-18.
//

#include "protobuf_transform.h"

ProtobufTransform::ProtobufTransform() {

}

ProtobufTransform::~ProtobufTransform() {

}

bool ProtobufTransform::toJsonString(const Message &msg, string *output) {
    util::JsonOptions opt;
    util::Status status = util::MessageToJsonString(msg, output, opt);
    if (status.ok()) {
        return true;
    }

    return false;
}

bool ProtobufTransform::fromJsonString(const string &input, Message *msg) {
    util::JsonParseOptions opt;
    util::Status status = util::JsonStringToMessage(input, msg, opt);
    if (status.ok()) {
        return true;
    }
    return false;
}
