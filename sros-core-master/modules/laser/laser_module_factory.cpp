//
// Created by lfc on 19-8-28.
//

#include "laser_module_factory.h"
#include <memory>
#include "GAZEBOLASER/gazebo_laser.h"
#include "LMS1xx/lms1xx_module.h"
#include "LMS5xx/lms5xx_module.h"
#include "MICRO_SCAN3/micro_scan3_module.h"
#include "NAV350/nav350_module.h"
#include "OMD30M/omd30m_module.h"
#include "SIM_LASER/simlaser_module.h"
#include "TIM5xx/tim5xx_module.h"
#include "UST10LX/ust10lx_module.h"
#include "UTM30LX/utm30lx_module.h"
#include "UAM05LP/uam05lp_module.h"
#include "modules/laser/GENERAL/general_laser_module.h"
#include "notuse_laser_module.hpp"

namespace laser{
std::shared_ptr<BaseLaserModule> LaserModuleFactory::getLaserModule(const laser::LidarType type,
                                                                    const std::string device_address,
                                                                    std::string &device_name,
                                                                    int port) {
    std::shared_ptr<BaseLaserModule> laser_module;
    device_name += "_";
    switch (type) {
        case UTM30LX:
            LOG(INFO) << "!!!!will create the utm30lx module!";
            laser_module.reset(new Utm30lxModule);
            device_name += convertToStr(UTM30LX);
            break;
        case LMS151:
            LOG(INFO) << "!!!will create the lms1xx module!";
            laser_module.reset(new Lms1xxModule);
            device_name += convertToStr(LMS151);
            break;
        case UST10LX:
            LOG(INFO) << "!!!will create the ust10lx module!";
            printf("get the message of lidar type!\n");
            laser_module.reset(new Ust10lxModule);
            device_name += convertToStr(UST10LX);
            break;
        case UAM05LP:
            LOG(INFO) << "!!!will create the uam05lp module!";
            printf("get the message of lidar type!\n");
            laser_module.reset(new Uam05lpModule(device_address,port));
            device_name += convertToStr(UAM05LP);
            break;          
        case OMD30M:
            LOG(INFO) << "!!!will create the omd module!";
            printf("get the message of lidar type!\n");
            laser_module.reset(new Omd30mModule(device_address));
            device_name += convertToStr(OMD30M);
            break;
        case LMS511:
            LOG(INFO) << "!!!will create the lms511 module!";
            printf("get the message of lidar type!\n");
            laser_module.reset(new Lms5xxModule);
            device_name += convertToStr(LMS511);
            break;
        case TIM571:
            LOG(INFO) << "!!!will create the TIM571 module!";
            laser_module.reset(new Tim5xxModule(device_address,std::to_string(port)));
            device_name = +convertToStr(TIM571);
            break;
        case GAZEBOSIM:
            LOG(INFO) << "will create the gazebo module!";
            laser_module.reset(new GazeboLaserModule());
            device_name += convertToStr(GAZEBOSIM);
            break;
        case OMDUHD:
            LOG(INFO) << "!!!will create the OMDUHD module!";
            laser_module.reset(new Omd30mModule(OMDUHD));
            device_name += convertToStr(OMDUHD);
            break;
        case BAGSIM:
            LOG(INFO) << "will create the simlaser module!";
            laser_module.reset(new SimlaserModule());
            device_name += convertToStr(BAGSIM);
            break;
        case NANO:
        case MICRO_SCAN3:
            LOG(INFO) << "will create micro scan3 module!";
            device_name += convertToStr(MICRO_SCAN3);
            laser_module.reset(new MicroScan3Module(device_address,port));
            break;

        case WANJI:
            LOG(INFO) << "will create wanji lidar module!";
            device_name += convertToStr(WANJI);
            laser_module.reset(new GeneralLaserModule(device_address, port,convertToStr(WANJI)));
            break;
        case SIMINICSPAVO:
            LOG(INFO) << "will create siminics lidar module!";
            device_name += convertToStr(SIMINICSPAVO);
            laser_module.reset(new GeneralLaserModule(device_address, port,convertToStr(SIMINICSPAVO)));
            break;
        case KELI:
            LOG(INFO) << "will create keli lidar module!";
            device_name += convertToStr(KELI);
            laser_module.reset(new GeneralLaserModule(device_address, port,convertToStr(KELI)));
            break;
        case KELI_270:
            LOG(INFO) << "will create keli 270 lidar module!";
            device_name += convertToStr(KELI_270);
            laser_module.reset(new GeneralLaserModule(device_address, port,convertToStr(KELI_270)));
            break;
        case ORADAR_ORBB:
            LOG(INFO) << "will create orbbe lidar module!";
            device_name += convertToStr(ORADAR_ORBB);
            laser_module.reset(new GeneralLaserModule(device_address, port,convertToStr(ORADAR_ORBB)));
            break;

        case NOTUSE:
            LOG(INFO) << "will create the simlaser module!";
            laser_module.reset(new NotuseLaserModule());
            break;
        default:
            LOG(INFO) << "will create the utm30lx module!";
            laser_module.reset(new Utm30lxModule);
            break;
    }
    return laser_module;
}


}
