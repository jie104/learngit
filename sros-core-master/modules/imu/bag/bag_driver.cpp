#include "bag_driver.h"

namespace imu
{
    namespace driver
    {

        BagDriver::BagDriver(const std::string &data_dir, const std::string &bag_file)
            : bag_file_(bag_file)
        //   data_player_(std::make_unique<MsgBag<ImuData>>(data_dir))
        {
            msg_bag_.reset(new MsgBag<ImuData>(data_dir));
            msg_bag_->setMsgHandle([this](const ImuData &data) { handleData(data); });
        }

        bool BagDriver::open()
        {
            msg_bag_->playBack(bag_file_);
            return true;
        }
        void BagDriver::close()
        {
        }
    }
}