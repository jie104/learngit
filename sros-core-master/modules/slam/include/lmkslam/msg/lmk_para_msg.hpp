//
// Created by lfc on 17-11-15.
//

#ifndef SROS_LMK_PARA_MSG_HPP
#define SROS_LMK_PARA_MSG_HPP

#include "base_msg_interface.hpp"
namespace slam{


class LmkParaMsg :public BaseMsgInterface{
public:

    enum LMK_EXTRACT_TYPE{
        FLAT_LMK = 1,
        CYLINDER_LMK = 2,
    };

    LmkParaMsg() : BaseMsgInterface(LMK_PARA_MSG){

    }

    LMK_EXTRACT_TYPE extract_type;


private:


};
typedef std::shared_ptr<LmkParaMsg> LmkParaMsg_Ptr;

}


#endif //SROS_LMK_PARA_MSG_HPP
