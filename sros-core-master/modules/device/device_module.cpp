
#include "device_module.h"

#include <boost/asio.hpp>
#include <boost/utility.hpp>

#include "core/device/com_port_interface.hpp"
#include "core/msg/sonar_data_msg.hpp"
#include "core/msg/str_msg.hpp"
#include "core/settings.h"
#include "core/state.h"
#include "core/usart/frame_r2100.h"

//using namespace std;
using namespace sros::core;
using namespace sros::device;

namespace device
{

    DeviceModule::DeviceModule() : Module("DeviceModule"),
                                   enable_r2100_device_(false)
    {
    }

    DeviceModule::~DeviceModule()
    {
    }

    void DeviceModule::run()
    {

        LOG(INFO) << "DeviceModule module start running";

        auto &s = sros::core::Settings::getInstance();

        enable_r2100_device_ = (s.getValue<std::string>("device.enable_r2100", "False") == "True");

        if (enable_r2100_device_)
        {
            auto com_port = std::make_shared<ComPortInterface<usart::FrameR2100>>();

            auto r2100_device_name = s.getValue<std::string>("device.r2100_serial_device", "/dev/ttyUSB0");
            auto r2100_baud_rate = s.getValue<unsigned int>("device.r2100_serial_baud_rate", 115200);

            if (!com_port->open(r2100_device_name, r2100_baud_rate))
            {
                LOG(ERROR) << "R2100Module: r2100 serial device " << r2100_device_name << " open failed!";
            }
            else
            {
                LOG(INFO) << "successfull to open r2100!";
                r2100_ptr_ = createDevice<R2100>(DEVICE_R2100, DEVICE_ID_R2100, DEVICE_COMM_INTERFACE_TYPE_USB_0, com_port);
                CHECK(r2100_ptr_);

                r2100_ptr_->setModelNo("OMD8000-R2100-R2-2V15");
                r2100_ptr_->setStateOK();
            }
        }
        else
        {
            LOG(WARNING) << "R2100 stop running(disable)";
            stop();
            return;
        }

        subscribeTopic("TIMER_100MS", CALLBACK(&DeviceModule::onTimer_100ms));

        dispatch();
    }

    void DeviceModule::onTimer_100ms(sros::core::base_msg_ptr msg)
    {
        if (enable_r2100_device_ && r2100_ptr_)
        {
            // 构造需要发送的payload
            r2100_ptr_->asyncRequestData();
        }
    }

} /* namespace device */
