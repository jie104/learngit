/**
 * @file security_module
 *
 * @author pengjiali
 * @date 20-1-21.
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef SROS_SECURITY_MODULE_H
#define SROS_SECURITY_MODULE_H

#include "core/module.h"
#include "security_state_handle.h"

namespace security {

class SecurityModule : public sros::core::Module {
 public:
    SecurityModule() : Module("SecurityModule") {}
    virtual ~SecurityModule() = default;

    void run() override;

 private:
    void handleSecurityState();

    SecurityStateHandle security_state_handle_;
};

}  // namespace security

#endif  // SROS_SECURITY_MODULE_H
