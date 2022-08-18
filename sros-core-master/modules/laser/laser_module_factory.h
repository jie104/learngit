//
// Created by lfc on 19-8-28.
//

#ifndef SROS_LASER_MODULE_FACTORY_H
#define SROS_LASER_MODULE_FACTORY_H

#include "base_laser_module.h"

#ifndef convertToStr
#define convertToStr(value_name) #value_name
#endif
namespace laser{
class LaserModuleFactory {
public:
    static std::shared_ptr<BaseLaserModule>  getLaserModule(LidarType type,const std::string device_address,std::string& device_name,int port = 0);
};

}


#endif //SROS_LASER_MODULE_FACTORY_H
