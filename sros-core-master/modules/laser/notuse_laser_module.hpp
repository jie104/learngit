//
// Created by lfc on 17-7-18.
//

#ifndef SROS_NOTUSE_LASER_MODULE_HPP
#define SROS_NOTUSE_LASER_MODULE_HPP

#include "base_laser_module.h"
namespace laser{
class NotuseLaserModule :public BaseLaserModule{
public:
    NotuseLaserModule(){}

    virtual ~NotuseLaserModule(){}

    virtual bool doOpen(){
        return false;
    };

    virtual void doStart(){
        return;
    };

    virtual void doStop(){
        return;
    };

    virtual void doClose() {
        return;
    };

    virtual bool getScanData(sros::core::LaserScan_ptr& scan_ptr){
        return false;
    };
};
}



#endif //SROS_NOTUSE_LASER_MODULE_HPP
