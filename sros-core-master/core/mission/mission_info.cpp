//
// Created by john on 18-8-28.
//

#include "mission_info.h"
#include "mission_manager.h"

namespace sros {
namespace core {

nlohmann::json MissionInfo::getStep (std::string step_id) {
    if (body_.find(step_id) != body_.end()) {
        return body_.at(step_id);
    }
    return nlohmann::json();
}

MissionInfo::MissionInfo() {

}

MissionInfo::~MissionInfo() {

}

AbstractStepPtr MissionInfo::getMissionStep(const std::string &step_id) {
    if (step_id == "") {
        return nullptr;
    }

    try {
        nlohmann::json mission_step = body_.at(step_id);
        if (mission_step.is_null() || !mission_step.is_object()) {
            LOG(ERROR) << "MissionInfo::getMissionStep(): Step info is invalid, step_id=" << step_id;
            return nullptr;
        }

        return createMissionStepFromJson(mission_step);

    } catch (nlohmann::json::exception &e) {
        LOG(ERROR) << "MissionInfo::getMissionStep(): " << e.what() << " " << step_id;
    }
    return nullptr;
}

MissionStepType MissionInfo::getMissionStepType(nlohmann::json &mission_step) {
    MissionStepType e_step_type = MissionStepType::StepNone;
    try {
        if (mission_step.is_null() || !mission_step.is_object()) {
            LOG(ERROR) << "MissionInfo::getMissionStepType(): Step info is invalid, step_info=" << mission_step;
            return e_step_type;
        }

        std::string step_type = mission_step.at("stepInfo").at("stepType");
        if (step_type == "begin") {
            e_step_type = MissionStepType::StepBegin;
        } else if (step_type == "end") {
            e_step_type = MissionStepType::StepEnd;
        } else if (step_type == "moveToStation") {
            e_step_type = MissionStepType::StepMoveToStation;
        } else if (step_type == "action") {
            e_step_type = MissionStepType::StepAction;
        } else if (step_type == "wait") {
            e_step_type = MissionStepType::StepWait;
        } else if (step_type == "pause") {
            e_step_type = MissionStepType::StepPause;
        } else if (step_type == "decision") {
            e_step_type = MissionStepType::StepDecision;
        } else if (step_type == "setReg") {
            e_step_type = MissionStepType::StepSetReg;
        } else if (step_type == "waitRegUpdate") {
            e_step_type = MissionStepType::StepWaitRegUpdate;
        } else if (step_type == "mission") {
            e_step_type = MissionStepType::StepMission;
        } else if (step_type == "parallelStep") {
            e_step_type = MissionStepType::StepParallel;
        } else {
            LOG(ERROR) << "MissionInfo::getMissionStepType(): Unknown mission step: " << step_type;
        }
    } catch (std::exception &e) {
        LOG(ERROR) << "MissionInfo::getMissionStepType(): " << e.what();
    }
    return e_step_type;
}

AbstractStepPtr MissionInfo::createMissionStepFromJson(nlohmann::json &dat) {
    if (dat.is_null() || !dat.is_object()) {
        LOG(ERROR) << "MissionInfo::createMissionStepFromJson(): Step info is invalid";
        return nullptr;
    }

    MissionStepType step_type = getMissionStepType(dat);
    if (step_type == MissionStepType::StepNone) {
        LOG(INFO) << "MissionInfo::createMissionStepFromJson(): MissionStepType::StepNone";
        return nullptr;
    }

    AbstractStepPtr step_ptr = createMissionStep(step_type);
    if (step_ptr == nullptr) {
        LOG(INFO) << "MissionInfo::createMissionStepFromJson(): Failed to create mission step";
        return nullptr;
    }

    step_ptr->fromJson(dat);
    return step_ptr;
}

AbstractStepPtr MissionInfo::createMissionStep(MissionStepType step_type) {
    AbstractStepPtr step_instance = nullptr;
    switch (step_type) {
        case MissionStepType::StepBegin: {
            step_instance = make_shared<StepBegin>();
            break;
        }
        case MissionStepType::StepEnd: {
            step_instance = make_shared<StepEnd>();
            break;
        }
        case MissionStepType::StepMoveToStation: {
            step_instance = make_shared<StepMove>();
            break;
        }
        case MissionStepType::StepAction: {
            step_instance = make_shared<StepAction>();
            break;
        }
        case MissionStepType::StepSetReg: {
            step_instance = make_shared<StepSetReg>();
            break;
        }
        case MissionStepType::StepWaitRegUpdate: {
            step_instance = make_shared<StepWaitRegUpdate>();
            break;
        }
        case MissionStepType::StepWait: {
            step_instance = make_shared<StepWaitTime>();
            break;
        }
        case MissionStepType::StepPause: {
            step_instance = make_shared<StepPause>();
            break;
        }
        case MissionStepType::StepDecision: {
            step_instance = make_shared<StepDecision>();
            break;
        }
        case MissionStepType::StepMission: {
            step_instance = make_shared<StepMission>();
            break;
        }
        case MissionStepType::StepParallel: {
            step_instance = make_shared<StepParallel>();
        }
        default: {
            break;
        }
    }
    return step_instance;
}

string MissionInfo::getNextStepId(const string &cur_step_id, bool decision) {
    using nlohmann::json;
    std::string next_step_id = "";
    try {
        nlohmann::json target_connections = body_.at(cur_step_id).at("targetConnections");
        if (!target_connections.is_array()) {
            return next_step_id;
        }

        int index = decision ? 0 : 1; // true: 是, false: 否
        if (target_connections.size() <= index) {
            return next_step_id;
        }

        next_step_id  = target_connections.at(index).get<std::string>();
    } catch (std::exception &e) {
        LOG(ERROR) << "MissionInfo::getNextStepId(): " << e.what();
    }

    LOG(INFO) << "MissionModule::getNextStepId(): " << next_step_id;
    return next_step_id;
}

AbstractStepPtr MissionInfo::getNextMissionStep(const string &cur_step_id, bool decision) {
    return getMissionStep(getNextStepId(cur_step_id, decision));
}

string MissionInfo::getStartStepId() {
    using nlohmann::json;
    string step_id = "";
    try {
        for (json::iterator iter = body_.begin(); iter != body_.end(); ++iter) {
            if (getMissionStepType(iter.value()) == MissionStepType::StepBegin) {
                step_id = iter.value().at("id").get<std::string>();
                break;
            }
        }
    } catch (std::exception &e) {
        LOG(ERROR) << "MissionInfo::getStartStepId(): " << e.what();
    }

    if (step_id == "") {
        LOG(ERROR) << "MissionInfo::getStartStepId(): Can not find the begin node!";
    }

    return step_id;
}

bool MissionInfo::traverseStep(uint32_t mission_id, const std::string &step_id, vector<AbstractStepPtr> &step_list) {
    using nlohmann::json;
    MissionInfo mission_info;
    bool is_okay = MissionManager::getInstance()->loadMissionFromStorage(mission_id, mission_info);
    if (!is_okay) {
        LOG(ERROR) << "MissionInfo::traverseStep(): Can not find task from storage: " << mission_id;
        return false;
    }

    try {
        for (json::iterator iter = mission_info.body_.begin(); iter != mission_info.body_.end(); ++iter) {
            string id = iter.value().at("id").get<std::string>();
            if (strcmp(id.c_str(), step_id.c_str())==0) {
                return true; // 返回true说明已经找到
            }

            if (getMissionStepType(iter.value()) == MissionStepType::StepMission) {
                nlohmann::json mission_step = mission_info.body_.at(id);
                StepMissionPtr child_mission = std::dynamic_pointer_cast<StepMission>(createMissionStepFromJson(mission_step));
                if (!child_mission) {
                    LOG(ERROR) << "MissionInfo::traverseStep(): Pointer cast failed";
                    return false;
                }

                uint32_t child_mission_id = child_mission->getMissionId();

                // 如果已经找到,则把子任务步骤保存起来
                if (traverseStep(child_mission_id, step_id, step_list)) {
                    step_list.insert(step_list.begin(), child_mission);
                    return true;
                }
            }
        }
    } catch (nlohmann::json::exception &e) {
        LOG(ERROR) << "MissionInfo::traverseStep(): " << e.what() << " " << step_id;
        return false;
    }

    LOG(INFO) << "Cannot find step " << step_id << " in mission " << mission_id;
    return false;
}

}
}