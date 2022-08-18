//
// Created by lfc on 17-2-11.
//

#ifndef SROS_LMS5XX_MODULE_H
#define SROS_LMS5XX_MODULE_H

#include <modules/laser/base_laser_module.h>
#include "lms_structs.h"
namespace lms5xx{
class LMS5xx;
struct Lms5xxCfg{
    int frequency = 2500;
    int resolution = 2500;
};
}
namespace laser{
class Lms5xxModule : public BaseLaserModule {

public:

    Lms5xxModule();

    virtual ~Lms5xxModule();

    virtual bool doOpen();

    virtual void doStart();

    virtual void doStop();

    virtual void doClose();

    virtual bool getScanData(sros::core::LaserScan_ptr& scan_ptr);

private:
    static void systemExitTrace(int signum);

    std::shared_ptr<lms5xx::LMS5xx> lms_511;
    lms5xx::scanCfg cfg;
    lms5xx::scanOutputRange outputRange;
    lms5xx::scanDataCfg dataCfg;
    lms5xx::ScanEchoCfg echo_cfg;
    std::string host;
    lms5xx::Lms5xxCfg lms511_cfg;
};
}



#endif //SROS_LMS5XX_MODULE_H
