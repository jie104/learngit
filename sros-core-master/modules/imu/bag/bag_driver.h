/**
 * @file bag_driver.h
 * @author zmy (626670628@qq.com)
 * @brief 回放已录制数据的虚拟驱动接口
 * @version 0.1
 * @date 2021-04-30
 * 
 * 
 */

#ifndef IMU_BAG_DRIVER_H
#define IMU_BAG_DRIVER_H

#include <memory>

#include "driver_base.h"
#include "core/bag/msg_bag/message_bag.h"
template <typename Msg>
using MsgBag = sros::core::MsgBag<Msg>;

namespace imu
{
    namespace driver
    {
        class BagDriver : public DriverBase
        {
        public:
            BagDriver(const std::string &data_dir, const std::string &bag_file);
            virtual ~BagDriver() = default;

            bool open() override;
            void close() override;

        private:
            std::string bag_file_;
        };
    } // namespace driver
}

#endif