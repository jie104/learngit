//
// Created by lfc on 17-7-10.
//

#ifndef PROJECT_LMK_MODULE_SINGLETON_H
#define PROJECT_LMK_MODULE_SINGLETON_H
#include <memory>
#include <Eigen/Dense>
#include <boost/thread.hpp>
#include <boost/function.hpp>
#include "processor_factory.h"
#include "msg/sensor_input_msg.hpp"
#include "msg/pose_msg.hpp"
#include "msg/scan_with_pose_msg.hpp"
#include "msg/map_output_msg.hpp"
#include <list>
#include <boost/thread/detail/thread.hpp>
#include <deque>
#include "msg/lmk_para_msg.hpp"
namespace record{
class ScanRecord;
}

namespace save_msg{
class ScanBackupManager;
}

namespace slam {
class SensorInputMsg;
class PoseMsg;
class MatchInputSetMsg;
class MatchOutputSetMsg;
class MatchInputSetMsg;

typedef boost::function<void(slam::MapOutputMsg_Ptr)> MapOutputCallback;

class LmkModuleSingleton {
public:
    static std::shared_ptr<LmkModuleSingleton> getInstance(LmkParaMsg_Ptr para);

    bool loadMap(std::string map_name);

    bool clearMap();

    bool haveMap();

    bool processMap(std::shared_ptr<SensorInputMsg> sensor_msg,std::shared_ptr<PoseMsg>& out_pose);

    bool processMap(std::shared_ptr<SensorInputMsg> sensor_msg);

    bool processLoc(std::shared_ptr<SensorInputMsg> sensor_msg,std::shared_ptr<PoseMsg>& out_pose);//数据共享,一旦内部修改时,外部调用的pose也同步修改

    bool processLoc(std::shared_ptr<SensorInputMsg> sensor_msg);//数据共享,一旦内部修改时,外部调用的pose也同步修改

    bool initialPoseLoc(std::shared_ptr<SensorInputMsg> sensor_msg,std::shared_ptr<PoseMsg>& out_pose);

    std::shared_ptr<PoseMsg> getLastPose();

    const std::list<slam::ScanWithPoseMsg>& getScanList();

    void continueMapping(std::string map_name);

    virtual ~LmkModuleSingleton();

    bool haveSameMap(std::string map_na_pa);

    void setMapOutputCallback(MapOutputCallback callback);

    void setLocOutputCallback(MapOutputCallback callback);

    void handleStopMapping(std::string map_name);

    void handleStartMapping(std::string map_name);

    void handleStartLocation(std::string map_name = "");

    void handleStopLocation(std::string map_name = "");

};

}
#endif //PROJECT_LMK_MODULE_SINGLETON_H
