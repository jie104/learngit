/*
 * module_manager.h
 *
 *  Created on: 2015年12月13日
 *      Author: lhx
 */

#ifndef MODULES_MODULE_MANAGER_H_
#define MODULES_MODULE_MANAGER_H_

#include <vector>

#include "core.h"
#include "module.h"

namespace sros {
namespace core {

class ModuleManager {
public:
    ModuleManager();
    virtual ~ModuleManager();

    bool registerModule(const module_ptr& module);

    void runAll();
private:
    std::vector<sros::core::module_ptr> module_list_;
};

} /* namespace core */
} /* namespace sros */

#endif /* MODULES_MODULE_MANAGER_H_ */
