//
// Created by huangwuxian on 19-6-18.
//

#ifndef SROS_MONITOR_PARSE_H
#define SROS_MONITOR_PARSE_H

#include <iostream>
#include "core/util/json.h"

using nloJson = nlohmann::json;

//typedef ::google::protobuf::Message         ProtobufMsg;
//typedef ::google::protobuf::Reflection      ProtobufReflection;
//typedef ::google::protobuf::FieldDescriptor ProtobufFieldDescriptor;
//typedef ::google::protobuf::Descriptor      ProtobufDescriptor;

class Parse {
public:
    static bool parseMonitorFile(const std::string &file_path, std::string &json);

private:
    static size_t parseModuleLen(std::ifstream &file);
    // https://blog.csdn.net/susser43/article/details/84399187
//    static void PbMsg2Json(const ProtobufMsg& src, nloJson& dst, bool enum2str = false);
//    static void RepeatedMessage2Json(const ProtobufMsg& message,
//                                     const ProtobufFieldDescriptor* field,
//                                     const ProtobufReflection* reflection,
//                                     nloJson& json, bool enum2str);
};


#endif //SROS_MONITOR_PARSE_H
