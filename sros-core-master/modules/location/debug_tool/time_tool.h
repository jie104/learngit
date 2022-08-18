//
// Created by lfc on 17-7-6.
//

#ifndef PROJECT_TIME_TOOL_H
#define PROJECT_TIME_TOOL_H
//#include <srosbag/bagmodule/sros_time.h>
#include "core/util/time.h"
namespace tool{
class TimeTool {
public:
    void startTiming() {
        start_time = sros::core::util::get_time_in_us();
    }
    void endTiming() {
        end_time = sros::core::util::get_time_in_us();
    }
    double getDurTimeInSec(){
        double delta_time = (double) (end_time - start_time) / (double) 1e6;
        return delta_time;
    }

private:
    int64_t start_time;
    int64_t end_time;
};
}



#endif //PROJECT_TIME_TOOL_H
