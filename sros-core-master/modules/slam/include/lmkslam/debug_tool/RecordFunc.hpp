//
// Created by lfc on 17-7-17.
//

#ifndef SROS_RECORDFUNC_HPP
#define SROS_RECORDFUNC_HPP
#include <string>
#include <iostream>
#define func_to_str(value_name) #value_name
namespace debug{
class RecordFunc {
public:
    RecordFunc();

    virtual ~RecordFunc(){

    }

    static void Record(std::string func_name);

    static std::string getStr();

    static void clear();
private:

};


}



#endif //SROS_RECORDFUNC_HPP
