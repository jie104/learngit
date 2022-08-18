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

#ifndef SROS_SETTINGS_H
#define SROS_SETTINGS_H

#include "settings_db.h"

namespace sros {
namespace core {

class Settings : public SettingsDB {
 public:
    static Settings &getInstance();

 private:
    Settings() : SettingsDB() {}
    virtual ~Settings() = default;

    bool updateItem(const ItemInfo &item_info) override ;
};

}  // namespace core
}  // namespace sros

#endif  // SROS_SETTINGS_H
