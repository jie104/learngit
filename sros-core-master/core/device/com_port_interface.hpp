/**
 * @file com_port_interface.hpp
 *
 * @author pengjiali
 * @date 2019年05月21日
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */
#ifndef SROS_COM_PORT_INTERFACE_HPP
#define SROS_COM_PORT_INTERFACE_HPP

#include "IOInterface.hpp"
#include "core/usart/connection.hpp"

namespace sros {
namespace device {

template <class Frame>
class ComPortInterface : public IOInterface {
 public:
    ComPortInterface() { usart_ptr_ = std::make_shared<usart::Connection<Frame>>(); }

    bool open(const std::string &device_name, uint32_t baud_rate, bool rs485_mode = false) {
        return usart_ptr_->connect(device_name, baud_rate, rs485_mode);
    }

 protected:
    void setReadCallback(IOInterfaceReadDataFunc func) override final { usart_ptr_->setRecvDataCallback(func); }

    bool write(const std::vector<uint8_t> &data) override final { 
       usart_ptr_->sendData(data); 
       return true;
    }

 private:
    std::shared_ptr<usart::Connection<Frame>> usart_ptr_;
};

}  // namespace device
}  // namespace sros

#endif  // SROS_COM_PORT_INTERFACE_HPP
