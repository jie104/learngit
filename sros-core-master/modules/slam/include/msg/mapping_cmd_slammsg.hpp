//
// Created by lfc on 17-9-1.
//

#ifndef SROS_MAPPING_CMD_SLAMMSG_HPP
#define SROS_MAPPING_CMD_SLAMMSG_HPP

#include "base_slammsg_interface.hpp"
namespace slam{

enum MAPPINGSLAMCMD{
    START_MAPPING_SLAMCMD,    //绘制地图
    STOP_MAPPING_SLAMCMD,    //保存地图,进入IDLE状态
    CANCEL_MAPPING_SLAMCMD,    //不保存地图,进入IDLE状态
};
class MappingCmdSlamMsg :public BaseSlamMsgInterface{
public:
    MappingCmdSlamMsg():BaseSlamMsgInterface(MAPPING_CMD_SLAMMSG){

    }

    virtual ~MappingCmdSlamMsg(){

    }
    MAPPINGSLAMCMD cmd;

    std::string map_name;
    std::string map_path;

private:

};
typedef std::shared_ptr<MappingCmdSlamMsg> MappingCmd_Ptr;

}

#endif //SROS_MAPPING_CMD_SLAMMSG_HPP
