//
// Created by john on 18-8-28.
//

#ifndef SROS_MISSION_INFO_H
#define SROS_MISSION_INFO_H

#include <string>
#include <vector>

#include "core/util/json.h"

#include "dataModel/step_begin.h"
#include "dataModel/step_end.h"
#include "dataModel/step_action.h"
#include "dataModel/step_decision.h"
#include "dataModel/step_move.h"
#include "dataModel/step_pause.h"
#include "dataModel/step_set_reg.h"
#include "dataModel/step_wait_reg_update.h"
#include "dataModel/step_wait_time.h"
#include "dataModel/step_mission.h"
#include "dataModel/step_parallel.h"

using namespace std;

namespace sros {
namespace core {

enum class MissionState {
    NA = 0,
    PENDING = 2, // 任务在队列中，但是又还没有启动的状态
    RUNNING = 3, // 任务正在执行
    PAUSED = 4,
    FINISHED = 5,
};

enum MissionResult {
    MISSION_RESULT_NA = 0, ///< 结果状态不可用
    MISSION_RESULT_OK = 1, ///< 任务执行完成
    MISSION_RESULT_CANCELED = 2, ///< 任务取消
    MISSION_RESULT_FAILED = 3, ///< 任务执行出错
};

class MissionInfo {
public:
    explicit MissionInfo();
    ~MissionInfo();

public:
    AbstractStepPtr getMissionStep(const std::string &step_id);
    AbstractStepPtr getNextMissionStep(const string &cur_step_id, bool decision = true);

    string getNextStepId(const string &cur_step_id, bool decision = true);
    string getStartStepId();

    // 递归查询指定任务步骤的层次,例如
    // 主任务a包含子任务步骤b,子任务b包含子任务步骤c,子任务c包含步骤X
    // 则传入步骤X的id返回结果:
    // [子任务步骤b, 子任务步骤c]
    // 如果直接在主任务中查询到,返回list size = 0
    // TODO 如果一个子任务出现多次,返回的层次关系(优先查询到的)可能是错误的
    static bool traverseStep (uint32_t mission_id, const std::string &step_id, vector<AbstractStepPtr> &step_list);

private:
    static MissionStepType getMissionStepType(nlohmann::json &mission_step);
    static AbstractStepPtr createMissionStep(MissionStepType step_type);
    static AbstractStepPtr createMissionStepFromJson(nlohmann::json &dat);

public:
    static const int MAX_NAME_LEN = 32;

    nlohmann::json getStep (std::string step_id);

    uint32_t id_;
    std::string name_; // len <= MAX_NAME_LEN
    std::string map_name_; // 程序关联的地图名
    int32_t create_time_; // len = 4
    int32_t last_modified_time_; // len = 4

    // 流程图, for example:
    /*
     * {
     *  1534832112540: {id: 1534832112540, type: "begin", selected: false, style: {left: "240px", top: "60px"}, targetConnections: [1534904301302]},
     *  1534832131789: {id: 1534832131789, type: "end", selected: false, style: {left: "240px", top: "420px"},sourceConnections: ["1534904301302", "1534905135447"], targetConnections: []},
     *  1534904301302: {id: 1534904301302, type: "decision", selected: false, style: {left: "200px", top: "180px"}, sourceConnections: [1534832112540], decisionInfo: {decisionType: "batteryPowerLessThan", batteryPower: 30},targetConnections: [1534832131789, 1534905135447]},
     *  1534905135447: {id: 1534905135447, type: "process", selected: false, style: {left: "440px", top: "260px"}, sourceConnections: [1534904301302], stepInfo: {stepType: "moveToStation", stationId: 1} targetConnections: [1534832131789]},
     * }
     */
    nlohmann::json body_;
};

}
}



#endif //SROS_MISSION_INFO_H
