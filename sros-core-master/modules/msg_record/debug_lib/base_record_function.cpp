//
// Created by lfc on 17-11-24.
//

#include "base_record_function.h"

namespace debug{

std::shared_ptr<BaseRecordFunction> BaseRecordFunction::record_function;
boost::mutex BaseRecordFunction::thread_mutex;

std::shared_ptr<BaseRecordFunction> BaseRecordFunction::getInstance() {
    if (!record_function) {
        boost::mutex::scoped_lock lock(thread_mutex);
        if (!record_function) {
            record_function.reset(new BaseRecordFunction);
        }
    }
    return record_function;
}

CircleFuncStr BaseRecordFunction::registerFuncsStack(std::string stack_name, int num) {
    auto funcs_iter = all_funcs.find(stack_name);
    if (funcs_iter == all_funcs.end()) {
        //如果没有注册过,可以直接使用.最怕是注册过,一般注册过的,是在一个线程执行的,将函数均保存到该内容中.
        CircleFuncStr funcs(new CircleFuncArray<std::string>(num));
        all_funcs[stack_name] = funcs;
        return funcs;
    } else {
        LOG(INFO) << "have exist! will return the ptr!";
        return funcs_iter->second;
    }
}

CircleFuncStr BaseRecordFunction::getFuncStack(std::string stack_name) {
    auto funcs_iter = all_funcs.find(stack_name);
    if (funcs_iter == all_funcs.end()) {
        //如果没有注册过,可以直接使用.最怕是注册过,一般注册过的,是在一个线程执行的,将函数均保存到该内容中.
        LOG(INFO) << "cannot get the funcs stack! will return 0!";
        return 0;
    }else {
        return funcs_iter->second;
    }
}

void BaseRecordFunction::outputAllFuncs() {
    record_stream.open(file_name, std::ios::app|std::ios::out);

    timespec time;
    clock_gettime(CLOCK_REALTIME, &time);  //获取相对于1970到现在的秒数
    tm nowtime;
    localtime_r(&time.tv_sec, &nowtime);
    std::stringstream time_str;
    std::string header = "The sros have stopped! maybe core dumped! some information will be recorded!\n";
    time_str << header;
    time_str<<"the stop time is:\n"<<"year:"<<(nowtime.tm_year+1900)<<"\nmonth:"<<nowtime.tm_mon+1<<"\nday:"<<nowtime.tm_mday
    <<"\ntime:"<<nowtime.tm_hour<<":"<<nowtime.tm_min<<"\n";
    time_str<<"all functions in recorded stack is:\n";
    for (auto funcs_iter:all_funcs) {
        auto &funcs = funcs_iter.second;
        time_str << "in the stack:" << funcs_iter.first << "\n";
        int size = funcs->getSize();
        for (int i = 0; i < size; ++i) {
            if (funcs->array(i).size()) {
                time_str << "func:" << funcs->array(i) << "\n";
            }
        }
    }
    time_str << "err record complete!\n";
    time_str << "\n";
    LOG(INFO) << time_str.str();
    record_stream << time_str.str();
    record_stream << std::endl;
    record_stream.close();
}

BaseRecordFunction::BaseRecordFunction() {

}
}