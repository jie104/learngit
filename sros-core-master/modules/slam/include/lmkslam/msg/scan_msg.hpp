//
// Created by lfc on 17-5-24.
//

#ifndef LMKSLAM_SCAN_MSG_HPP
#define LMKSLAM_SCAN_MSG_HPP

#include "base_msg_interface.hpp"
#include <vector>
namespace slam {
class ScanMsg: public BaseMsgInterface {
public:
    ScanMsg() : BaseMsgInterface(SCAN_MSG) {

    }

    virtual ~ScanMsg() { }

    float angle_min;
    float angle_max;
    float angle_increment;
    float time_increment;
    float range_min;
    float range_max;
    std::vector<float> ranges;
    std::vector<float> intensities;
private:


};
typedef std::shared_ptr<ScanMsg> ScanMsg_Ptr;
//typedef std::vector<ScanMsg> ScanMsg_Vector;
//typedef std::map<ScanMsg> ScanMsg_Map;
}


#endif //LMKSLAM_SCAN_MSG_HPP
