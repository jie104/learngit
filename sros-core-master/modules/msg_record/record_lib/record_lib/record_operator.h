//
// Created by lfc on 17-10-27.
//

#ifndef SROS_RECORD_OPERATOR_H
#define SROS_RECORD_OPERATOR_H

#include "record_msg/base_record_msg.hpp"
namespace record{
class RecordOperator {
public:
    RecordOperator();

    virtual ~RecordOperator();

    bool open(const char *path, bool add_file = false);

    void close();

    bool isOpen();

    void recordMsg(BaseRecordMsg_Ptr base_msg);


private:
    std::fstream record_stream;
    bool is_open;
    std::string file_name;

};

typedef std::shared_ptr<RecordOperator> RecordOperator_Ptr;

}


#endif //SROS_RECORD_OPERATOR_H
