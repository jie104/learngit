//
// Created by huangwuxian on 19-6-18.
//

#ifndef SROS_PROTOBUF_TRANSFORM_H
#define SROS_PROTOBUF_TRANSFORM_H

#include <google/protobuf/message.h>
#include <google/protobuf/util/json_util.h>

using namespace google::protobuf;

class ProtobufTransform {
public:
    explicit ProtobufTransform();
    ~ProtobufTransform();

    // Protobuf转Json
    static bool toJsonString(const Message &msg, string *output);

    // Json转Protobuf
    static bool fromJsonString(const string &input, Message *msg);

private:
};

#endif //SROS_PROTOBUF_TRANSFORM_H
