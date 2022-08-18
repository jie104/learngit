/**
 * @file imu_module.cpp
 * @author zmy (626670628@qq.com)
 * @brief imu模块 实现
 * @version 0.1
 * @date 2021-04-12
 * 
 * 
 */

#include "imu_module.h"
#include "core/settings.h"
#include "imu_resolver.hpp"

namespace imu
{
    std::unique_ptr<ImuResolver> ImuModule::imu_resovler_(nullptr);

    ImuModule::ImuModule() : Module("ImuModule") {}

    ImuModule::~ImuModule() {}

    void ImuModule::run() {
        int imu_type = sros::core::Settings::getInstance().getValue<int>( //getValue = mainSetting.ini中参数
            "imu.imu_type", 0);
        std::string imu_dev = sros::core::Settings::getInstance().getValue<std::string>( //getValue = mainSetting.ini中参数
            "imu.imu_dev", "/dev/ttyUSB0");

        bool recorder_mode = sros::core::Settings::getInstance().getValue<bool>( //getValue = mainSetting.ini中参数
            "imu.data_recorder", false);
        std::string bag_dir = sros::core::Settings::getInstance().getValue<std::string>( //getValue = mainSetting.ini中参数
            "imu.bag_directory", "/sros/message_bag");

        double imu_roll = sros::core::Settings::getInstance().getValue<double>( //getValue = mainSetting.ini中参数
            "imu.ext_roll", 0);
        double imu_pitch = sros::core::Settings::getInstance().getValue<double>( //getValue = mainSetting.ini中参数
            "imu.ext_pitch", 0);
        double imu_yaw = sros::core::Settings::getInstance().getValue<double>( //getValue = mainSetting.ini中参数
            "imu.ext_yaw", 0);

        double imu_trans_x = sros::core::Settings::getInstance().getValue<double>( //getValue = mainSetting.ini中参数
            "imu.ext_trans_x", 0);
        double imu_trans_y = sros::core::Settings::getInstance().getValue<double>( //getValue = mainSetting.ini中参数
            "imu.ext_trans_y", 0);
        double imu_trans_z = sros::core::Settings::getInstance().getValue<double>( //getValue = mainSetting.ini中参数
            "imu.ext_trans_z", 0);
        float imu_frequency = sros::core::Settings::getInstance().getValue<float>( //getValue = mainSetting.ini中参数
            "imu.frequency", 200);

        imu_resovler_.reset(new ImuResolver(imu_type, imu_dev, imu_frequency));
        imu_resovler_->setExtTransform(imu_roll, imu_pitch, imu_yaw, imu_trans_x, imu_trans_y, imu_trans_z);
        LOG(INFO) << "begin to run the imu module,imu type:" << imu_type;
        //subscribeTopic("TOPIC_ODOPOSE", CALLBACK(&PoseManagerModule::onOdoPoseMsg));

        dispatch();
    }

    ImuData ImuModule::getImuWithIdx(const int index) {
        if (imu_resovler_)
            return imu_resovler_->getImuWithIdx(index);
        else
            return ImuData{};
    }

    ImuData ImuModule::getImuWithStamp(const int64_t stamp) {
        if (imu_resovler_)
            return imu_resovler_->getImuWithStamp(stamp);
        else
            return ImuData{};
    }

    void ImuModule::setStandstill(const bool is_standstill) {
        if (imu_resovler_)
            return imu_resovler_->setStandstill(is_standstill);
    }

} // namespace imu
