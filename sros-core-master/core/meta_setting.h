/**
 * @file meta_setting.h
 *
 * @author pengjiali
 * @date 19-7-3.
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef CORE_META_SETTING_H_
#define CORE_META_SETTING_H_

#include "key_value_db.h"

namespace sros {
namespace core {
class MetaSetting : public KeyValueDB {
 public:
    virtual ~MetaSetting() = default;

    static MetaSetting &getInstance() {
        static MetaSetting setting;
        return setting;
    }

    template <typename T>
    T getValue(const std::string &key, const T &v) {
        return getValueSlave(key, v, false);
    }

    template <typename T>
    bool setValue(const std::string &key, const T &v) {
        return setValueSlave(key, v);
    }

 private:
    MetaSetting() : KeyValueDB("meta") {}
};

}  // namespace core
}  // namespace sros

#endif  // CORE_META_SETTING_H_
