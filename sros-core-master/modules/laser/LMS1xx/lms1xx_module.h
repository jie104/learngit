//
// Created by lfc on 16-11-2.
//

#ifndef SROS_LMS1XX_MODULE_H
#define SROS_LMS1XX_MODULE_H

#include <modules/laser/base_laser_module.h>
#include "LMS1xx.h"
#include "lms_structs.h"

namespace laser {
class Lms1xxModule : public BaseLaserModule {
public:

    Lms1xxModule();

    virtual ~Lms1xxModule();

    virtual bool doOpen();

    virtual void doStart();

    virtual void doStop();

    virtual void doClose();

    virtual bool getScanData(sros::core::LaserScan_ptr& scan_ptr);

//    void resetLaser();

private:
    LMS1xx* laser;
    scanCfg cfg;
    scanOutputRange outputRange;
    scanDataCfg dataCfg;
    std::string host;



};
}


#endif //SROS_LMS1XX_MODULE_H
