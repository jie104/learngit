/**
 * @file driver_base.h
 * @author zmy (626670628@qq.com)
 * @brief IMU驱动接口类
 * @version 0.1
 * @date 2021-03-29
 * 
 * 
 */

#ifndef ODOM_DRIVER_BASE_H
#define ODOM_DRIVER_BASE_H

#include "type.h"
#include <atomic>
#include <functional>
#include <iostream>
#include "core/device/device.h"
#include <glog/logging.h>
// #include "core/bag/msg_bag/message_bag.h"
// template <typename Msg>
// using MsgRecorder = sros::core::MsgBag<Msg>;

namespace imu
{
    namespace driver
    {
        class DriverBase:public sros::device::Device
        {
        public:
           DriverBase(std::string driver_name)
               : sros::device::Device(driver_name, sros::device::DEVICE_ID_IMU,
                                      sros::device::DEVICE_COMM_INTERFACE_TYPE_USB_1, sros::device::DEVICE_MOUNT_SROS),
                 callback_(nullptr)
           //   record_mode_(false),
           //   msg_bag_(nullptr),
           //   bag_dir_("/sros/message_bag")
           {
           }
           ~DriverBase() = default;

            virtual bool open()
            {
                std::cout << "has no imu device seted, please checking setting!" << std::endl;
                return false;
            }
            virtual void close() {}
            void setCallback(std::function<void(ImuData &)> callback) { callback_ = callback; }
            void handleData(ImuData &data)
            {
                if (!(callback_ == nullptr))
                    callback_(data);
            }

            void keepOK()
            {
                if ((2 * imu_increment_count_++) > imu_frequency_) {
                    imu_increment_count_ = 0;
                    keepAlive();
                }
            }
            // void setRecordMode(const bool is_recoder,
            //                    const std::string &bag_dir = "/sros/message_bag")
            // {
            //     record_mode_ = is_recoder;
            //     bag_dir_ = bag_dir;
            // }
            // void recorderMsg(const ImuData &imu_data)
            // {
            //     if (isRecordMode())
            //     {
            //         if (msg_bag_ == nullptr || !msg_bag_->isRecording())
            //             msg_bag_.reset(new MsgRecorder<ImuData>(bag_dir_));
            //         msg_bag_->dump(imu_data);
            //     }
            //     else if (msg_bag_ != nullptr)
            //     {
            //         msg_bag_->closeRecored();
            //     }
            // }

        private:
            // bool isRecordMode() const { return record_mode_; }

        protected:
            // std::unique_ptr<MsgRecorder<ImuData>> msg_bag_;
            int imu_frequency_ = 200;//200Hz
            int imu_increment_count_ = 0;
        private:

            std::function<void(ImuData &)> callback_;

            // std::atomic<bool> record_mode_;
            // std::string bag_dir_;
        };
    } // namespace imu::driver
}

#endif