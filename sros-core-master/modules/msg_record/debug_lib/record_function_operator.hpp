//
// Created by lfc on 17-11-24.
//

#ifndef SROS_RECORD_FUNCTION_OPERATOR_HPP
#define SROS_RECORD_FUNCTION_OPERATOR_HPP

#include "base_record_function.h"

#ifndef func_to_str
#define func_to_str(value_name) #value_name
#endif
namespace debug{
class RecordFunctionOperator {
public:
    RecordFunctionOperator(std::string record_stack_name) {
        recorder = BaseRecordFunction::getInstance();
        func_stack = recorder->registerFuncsStack(record_stack_name,20);
    }

    void pushBackFunc(std::string func_name) {
        func_stack->pushBack(func_name);
    }

    ~RecordFunctionOperator(){

    }


private:
    RecordFunctionOperator(){

    }

    std::shared_ptr<BaseRecordFunction> recorder;
    CircleFuncStr func_stack;

};

typedef std::shared_ptr<RecordFunctionOperator> RecordFunctionOperator_Ptr;

}


#endif //SROS_RECORD_FUNCTION_OPERATOR_HPP
