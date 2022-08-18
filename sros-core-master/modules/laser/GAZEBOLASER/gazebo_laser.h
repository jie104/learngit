//
// Created by lfc on 17-5-21.
//

#ifndef SROS_GAZEBO_LASER_H
#define SROS_GAZEBO_LASER_H
//#include "~/wo
#include "../base_laser_module.h"
#include "src/protocol/gazebo_laser_scan.hpp"
#include "boost/thread.hpp"
namespace src{
class GazeboLaserDevice;
}
namespace laser{
class GazeboLaserModule: public BaseLaserModule {
public:

    GazeboLaserModule();

    virtual ~GazeboLaserModule();

    virtual bool doOpen();

    virtual void doStart();

    virtual void doStop();

    virtual void doClose();

    virtual bool getScanData(sros::core::LaserScan_ptr& scan_ptr);
private:
     void getGazeboScan(network::GazeboLaserScanMsg_ptr scan);

    std::shared_ptr<src::GazeboLaserDevice> gazebo_laser;

    boost::mutex wakeup_mutex;
    boost::condition_variable_any condition;
    sros::core::LaserScan_ptr last_scan;
};

}


#endif //SROS_GAZEBO_LASER_H
