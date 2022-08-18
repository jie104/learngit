//
// Created by lfc on 17-10-27.
//
#include "base_record_msg.hpp"

namespace record{
std::shared_ptr<RecordMsgMap> RecordMsgMap::record_msg_module;

std::string record::RecordMsgMap::getNameByType(record::RecordMsgType type) {
    if (!record_msg_module) {
        RecordMsgMap::getInstance();
    }

    return record_msg_module->getName(type);
}

std::shared_ptr<RecordMsgMap> record::RecordMsgMap::getInstance() {
    if (!record_msg_module) {
        record_msg_module.reset(new RecordMsgMap());
    }
    return record_msg_module;
}

int RecordMsgMap::getTypeByName(std::string name) {
    if (!record_msg_module) {
        RecordMsgMap::getInstance();
    }
    return record_msg_module->getType(name);
}
}
