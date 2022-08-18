/**
 * @file settings
 *
 * @author pengjiali
 * @date 20-4-16.
 *
 * @describe 这一层会加入一些系统的钩子，如设置参数时要动态修改一些系统内容
 *
 * @copyright Copyright (c) 2020 Standard Robots Co., Ltd. All rights reserved.
 */


#include "settings.h"
#include "core/state.h"


namespace sros {
namespace core {

Settings &Settings::getInstance() {
    try {
        static Settings settings;
        return settings;
    } catch (const std::exception &e) {
        LOG(ERROR) << "setting construct failed! " << e.what();
        throw e;
    }
}

bool Settings::updateItem(const ItemInfo &item_info) {
    if (SettingsDB::updateItem(item_info)) {
        // 设置系统参数后，需要动态更新系统的一些钩子
        if (item_info.key == "main.fleet_mode") {
            if (item_info.value == "FLEET_MODE_OFFLINE") {
                g_state.fleet_mode = FLEET_MODE_OFFLINE;
            } else if (item_info.value == "FLEET_MODE_ONLINE") {
                g_state.fleet_mode = FLEET_MODE_ONLINE;
            } else {
                LOG(ERROR) << "UNRACHABLE!!!";
            }
            g_state.updateFleetState();
        }
        return true;
    }
    
    return false;
}

}  // namespace core
}  // namespace sros