//
// Created by caoyan on 1/15/21.
//

#include "action_7.h"
#include "core/logger.h"
#include "core/settings.h"
#include "core/msg_bus.h"
#include "core/state.h"
#include "core/exec_error.hpp"

using namespace std;
using namespace sros::core;

namespace ac {

void Action7::doStart() {
    //
    auto &s = sros::core::Settings::getInstance();
    bool enable_pgv_rectify = (s.getValue<std::string>("main.enable_pgv_rectify", "False") == "True");
    if (!enable_pgv_rectify) {
        LOG(ERROR) << "please set main.enable_pgv_rectify True and try again.";
        doActionFinishFailed(ERROR_CODE_ACTION_DOWN_CANMER_RECITIFY_DISABLED);
        throw EXEC_ERROR(ERROR_CODE_ACTION_DOWN_CANMER_RECITIFY_DISABLED,
                         "please set main.enable_pgv_rectify True and try again.");
    }
    if (src_sdk->getSRCVersion() < 4007000 && src_sdk->getVersion() != sdk::SRC_PROTO_VERSION_V2) {
        LOG(ERROR) << "Src version must greater or equal than v4.7.0 or is not srtos";
        doActionFinishFailed(ERROR_CODE_SRC_NOT_SUPPORE);
        throw EXEC_ERROR(ERROR_CODE_SRC_NOT_SUPPORE, "Src version must greater or equal than v4.7.0");
    }

    if (action_param0_ == 2) {  // 用当前站点矫正
        auto cur_station_no = g_state.station_no;
        if (cur_station_no == 0) {
            LOG(ERROR) << "current station is 0！";
            doActionFinishFailed(ERROR_CODE_ACTION_DOWN_CANMER_RECITIFY_CURRENT_STATION_IS_NONE);
            throw EXEC_ERROR(ERROR_CODE_ACTION_DOWN_CANMER_RECITIFY_CURRENT_STATION_IS_NONE,
                             "current station is 0！");
        }

        auto station = MapManager::getInstance()->getStation(cur_station_no);
        if (!station.dmcode_id.empty()) {
            DMCodeOffset offset(station.dmcode_id, station.dmcode_offset.x / 100, station.dmcode_offset.y / 100,
                                station.dmcode_offset.yaw);
            g_state.station_camera_offset.set(offset);

            auto imu_ins550 = s.getValue<int>("src.imu_ins550", 0);
            LOG(INFO) << "imu_ins550: " << imu_ins550;
            if (imu_ins550 / 10 % 10 == 2) {
                auto pgv_direction_offset =
                    Settings::getInstance().getValue<double>("main.pgv_direction_offset", 90.0);
                offset.yaw = normalizeYaw0To2Pi(offset.yaw - pgv_direction_offset * DEG_TO_RAD);
            }

            bool ret = src_sdk->setTargetDMCodeOffset(offset.id, offset.x, offset.y, offset.yaw);
            if (!ret) LOG(ERROR) << "设置pgv信息失败！";
            action_param0_ = 1;  // NOTE: 发给src执行的还是7,1,0
        } else {
            LOG(ERROR) << "current station dmcode is none！";
            doActionFinishFailed(ERROR_CODE_ACTION_DOWN_CANMER_RECITIFY_CURRENT_STATION_DMCODE_IS_NONE);
            throw EXEC_ERROR(ERROR_CODE_ACTION_DOWN_CANMER_RECITIFY_CURRENT_STATION_DMCODE_IS_NONE,
                             "current station dmcode is none！");
        }
    }

    enableSVC100Camera(true, sros::device::DEVICE_SVC100_DOWN);  // 准备开始执行矫正动作，启用SVC100

    // 向SRC发送动作指令
    src_sdk->executeAction(action_no_, action_id_, action_param0_, action_param1_);
}

bool Action7::onSrcAcFinishFirst() {
    // 当矫正动作执行结束后，不管成功还是失败，都需要关闭SVC100
    if (!g_state.needDebugInfo()) { // 需要调试信息除外，比如用户部署时，需要实时看到pgv的偏差
        enableSVC100Camera(false, sros::device::DEVICE_SVC100_DOWN);
    }

    return true;
}

}