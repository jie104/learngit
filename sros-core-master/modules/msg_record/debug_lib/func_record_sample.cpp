//
// Created by lfc on 17-11-25.
//

//该模块用于演示如何使用
#include "record_function_operator.hpp"

int main(int argc, char* argv[]) {
    using namespace debug;
    RecordFunctionOperator_Ptr func_record(new RecordFunctionOperator("module name"));//注意这里一定要添加module name!必须确保每个线程的module name 不一样.这里，需要把其定义为成员变量

    func_record->pushBackFunc("func name");//可以存储function name
    func_record->pushBackFunc("important info");//可以存储一些重要的信息
    func_record->pushBackFunc("warn info");//可以存储warning的信息

    //注意不需要clear!,func_record内部是一个固定大小的数组,该数组最终输出的形式类似于栈,栈顶为最新的function或者info
    auto record_output = BaseRecordFunction::getInstance();
    record_output->outputAllFuncs();//该部分在异常回调函数内实现,不需要每个模块自己去调用!具体的实现机制可以参考"/modules/msg_record/record_module.cpp"文件下,systemExitTrace函数

}