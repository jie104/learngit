//
// Created by penglei on 11/15/17.
//

#ifndef SROS_DEVICEMODULE_H
#define SROS_DEVICEMODULE_H

#include "core/core.h"
#include "core/module.h"
#include "core/usart/timeout_serial.h"
#include "core/hardware/R2100.h"

#include "core/device/device_manager.h"

#include <string>
#include <stdint.h>
#include <vector>

namespace device {

using namespace std;

class DeviceModule : public sros::core::Module {
public:
    DeviceModule();

    virtual ~DeviceModule();

    virtual void run();

private:
    /* r2100 data */
    sros::device::R2100_ptr r2100_ptr_;
    bool enable_r2100_device_;

    void onTimer_100ms(sros::core::base_msg_ptr msg);
};

} /* namespace device */
#endif //SROS_DEVICEMODULE_H
