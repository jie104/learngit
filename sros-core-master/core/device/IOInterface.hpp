/**
 * @file IOInterface.hpp
 *
 * @author pengjiali
 * @date 2019年5月20日
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */
#ifndef SROS_IOINTERFACE_HPP
#define SROS_IOINTERFACE_HPP

#include <stdint.h>
#include <functional>
#include <vector>

namespace sros {
namespace device {

typedef std::function<void(const std::vector<uint8_t> &data)> IOInterfaceReadDataFunc;

class IOInterface {
 public:
    virtual bool write(const std::vector<uint8_t> &data) = 0;
    virtual void setReadCallback(IOInterfaceReadDataFunc func) = 0;

    virtual bool isCanInterface() const { return false; } // 是否是can接口
};

}  // namespace device
}  // namespace sros

#endif  // SROS_IOINTERFACE_HPP
