//
// Created by lfc on 17-10-27.
//

#ifndef SROS_RECORD_MODULE_H
#define SROS_RECORD_MODULE_H
#include "core/module.h"
namespace record{
class RecordOperator;

class RecordModule :public sros::core::Module{
public:
    RecordModule();

    virtual ~RecordModule();

    virtual void run();
private:
    bool enable_record_;
    void obstacleCallback(sros::core::base_msg_ptr base_ptr);

    bool creatRecordFile();

    std::shared_ptr<RecordOperator> record_op;

    const std::string file_path = "/sros/record/";

    static void systemExitTrace(int signum);


};

}


#endif //SROS_RECORD_MODULE_H
