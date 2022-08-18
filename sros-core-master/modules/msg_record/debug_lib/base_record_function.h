//
// Created by lfc on 17-11-24.
//

#ifndef SROS_BASE_RECORD_FUNCTION_H
#define SROS_BASE_RECORD_FUNCTION_H

#include "circle_func_array.hpp"
#include <memory>
#include <string>
#include <map>
#include <boost/thread.hpp>
#include <fstream>

namespace debug{

typedef std::shared_ptr<CircleFuncArray<std::string>> CircleFuncStr;
typedef std::map<std::string,CircleFuncStr> AllFuncStr;
class BaseRecordFunction {//TODO:一定要注意,重启sros不能作为LOG记录,或者LOG应记录sros的重启记录.而且,LOG应在文件足够大的时候,自动清理
public:
    static std::shared_ptr<BaseRecordFunction> getInstance();

    CircleFuncStr registerFuncsStack(std::string stack_name,int num = 20);

    CircleFuncStr getFuncStack(std::string stack_name);

    void outputAllFuncs();

    ~BaseRecordFunction(){

    }
private:
    BaseRecordFunction();

    static std::shared_ptr<BaseRecordFunction> record_function;
    static boost::mutex thread_mutex;

    AllFuncStr all_funcs;
    std::fstream record_stream;
    const std::string file_name = "/sros/restart_record.log";

};

}


#endif //SROS_BASE_RECORD_FUNCTION_H
