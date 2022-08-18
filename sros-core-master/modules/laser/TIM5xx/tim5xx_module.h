//
// Created by lfc on 17-2-11.
//

#ifndef SROS_TIM5XX_MODULE_H
#define SROS_TIM5XX_MODULE_H

#include <modules/laser/base_laser_module.h>
namespace sick_tim{
class SickTimCommon;
class SickTim5512050001Parser;
}
namespace laser{
class Tim5xxModule : public BaseLaserModule {

public:

    Tim5xxModule(std::string ip_address,std::string ip_port);

    virtual ~Tim5xxModule();

    virtual bool doOpen();

    virtual void doStart();

    virtual void doStop();

    virtual void doClose();

    virtual bool getScanData(sros::core::LaserScan_ptr& scan_ptr);

private:
    std::shared_ptr<sick_tim::SickTimCommon> tim_571;
    sick_tim::SickTim5512050001Parser* parser;
    std::string ip_add;
    std::string port;
    int time_limit;

    double start_angle;
    double end_angle;
    double angle_incerement;
};
}



#endif //SROS_LMS5XX_MODULE_H
