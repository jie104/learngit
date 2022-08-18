//
// Created by lfc on 17-10-27.
//

#include "record_operator.h"
namespace record{

RecordOperator::RecordOperator() {

}

RecordOperator::~RecordOperator() {
    close();
}

bool RecordOperator::open(const char *path, bool add_file) {
    if (add_file) {
        record_stream.open(path, std::ios::app|std::ios::out);
    } else {
        record_stream.open(path, std::ios::out);
    }
    if (record_stream.is_open()) {
        file_name = path;
        is_open = true;
        record_stream.width(20);
//            record_stream.precision(10);
        record_stream.setf(std::ios::fixed, std::ios::floatfield);
        record_stream.setf(record_stream.left);
        return true;
    }
    LOG(INFO) << "cannot open the file!";
    is_open = false;
    return false;
}

void RecordOperator::close() {
    is_open = false;
    record_stream.close();
}

bool RecordOperator::isOpen() {
    return is_open;
}

void RecordOperator::recordMsg(BaseRecordMsg_Ptr base_msg) {
    if (is_open) {
        base_msg->encode(record_stream);
    }else {
        LOG(INFO) << "the file is not open!";
    }
}
}