/**
 * @file bag_module.h
 * @author zmy (626670628@qq.com)
 * @brief 调用bag的sros模块
 * @version 0.1
 * @date 2021-05-27
 *
 *
 */

#ifndef NEW_SROS_BAG_MODULE_H
#define NEW_SROS_BAG_MODULE_H

#include "../sros-core/core/msg/base_msg.h"
#include "bag/include/bag/message_bag.h"
#include "core/core.h"
#include <memory>
#include <thread>

namespace srosbag
{

    class BagModule : public sros::core::Module
    {

    public:
        BagModule(/* args */);
        virtual ~BagModule() = default;
        virtual void run();

    private:
        void getStartParam();
        void for_test();

        //订阅所有传感器msg数据
        void subscribeSensorMsg();


        template <typename Msg>
        void msgCallback(sros::core::base_msg_ptr msg)
        {
            auto sensor_msg = std::dynamic_pointer_cast<Msg>(msg);
            // LOG(INFO) << "sensor_msg->mat_xyz_.size(): " << sensor_msg->mat_xyz_.size();
            // LOG(INFO) << "sensor_msg->mat_xyz_.type(): " << sensor_msg->mat_xyz_.type();
            // LOG(INFO) << "sensor_msg->mat_xyz_: " << sensor_msg->mat_xyz_.row(0);
            if (start_record_)
            {
                auto sensor_msg = std::dynamic_pointer_cast<Msg>(msg);
                std::string topic_name = msg->topic_;
                msg_bag_->dumpMsg<Msg>(*sensor_msg, topic_name);
            }
        }


    private:
        std::shared_ptr<bag::MsgBag> msg_bag_;
        bool start_record_;
        std::thread test_;
    };

} // namespace sros

#endif
