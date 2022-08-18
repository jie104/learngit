//
// Created by lfc on 17-7-13.
//

#ifndef SROS_SIMLASER_MODULE_H
#define SROS_SIMLASER_MODULE_H

#include <modules/laser/base_laser_module.h>
#include <boost/thread.hpp>
#include <modules/laser/SIM_LASER/network/connect_msg/base_msg.hpp>

namespace connection{
    class BaseMsg;
};
namespace laser{
class SimLaserDevice;

class SimlaserModule: public laser::BaseLaserModule {
public:
    SimlaserModule():BaseLaserModule(){

    }

    virtual bool doOpen();

    virtual void doStart();

    virtual void doStop();

    virtual void doClose();

    virtual bool getScanData(sros::core::LaserScan_ptr& scan_ptr);

private:

    std::shared_ptr<SimLaserDevice> laser_device;
    void scanCallback(std::shared_ptr<connection::BaseMsg> scan_ptr);

    boost::mutex thread_mutex;
    boost::mutex wakeup_mutex;
    boost::condition_variable_any condition;
    sros::core::LaserScan_ptr scan;
    std::string bag_name_with_path;
};
}



#endif //SROS_SIMLASER_MODULE_H
