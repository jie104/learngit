//
// Created by lfc on 19-7-8.
//

#ifndef SROS_LMK_MATCHINFO_MSG_HPP
#define SROS_LMK_MATCHINFO_MSG_HPP

#include "base_msg.h"
namespace sros{
namespace core{
enum LmkMatchType{
    TYPE_FLAT = 1,
    TYPE_CYLINDER = 2,
};

struct LmkMatchInfo{
    float pose_x; // m
    float pose_y;
    float pose_yaw; // 弧度
    LmkMatchType  lmk_type;
    bool is_matched = false;
};

class LmkMatchinfoMsg :public BaseMsg{
public:
    LmkMatchinfoMsg():BaseMsg("LMK_MATCH_INFO",TYPE_LMKMATCH_DATA){
        getTime();
    }

    virtual void getTime(){
        time_ = sros::core::util::get_time_in_us();
    }
    std::vector<LmkMatchInfo> lmk_infos;
private:


};

typedef std::shared_ptr<LmkMatchinfoMsg> LmkMatchinfoMsg_ptr;
}

}


#endif //SROS_LMK_MATCHINFO_MSG_HPP
