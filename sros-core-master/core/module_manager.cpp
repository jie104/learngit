/*
 * module_manager.cpp
 *
 *  Created on: 2015年12月13日
 *      Author: lhx
 */

#include "module_manager.h"

namespace sros {
namespace core {

ModuleManager::ModuleManager() {

}

ModuleManager::~ModuleManager() {

}

bool ModuleManager::registerModule(const module_ptr& module) {
    LOG(INFO) << "register module: " << module->getName();
    module_list_.push_back(module);
    return true;
}

void ModuleManager::runAll() {
    for (auto module : module_list_) {
        LOG(INFO) << "start module: " << module->getName();
        module->start();
    }
}

} /* namespace core */
} /* namespace sros */
