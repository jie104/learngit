/**
 * @file imu_module.h
 * @author zmy (626670628@qq.com)
 * @brief imu module
 * @version 0.1
 * @date 2021-04-12
 * 
 * 
 */

#ifndef SROS_IMU_LASER_MODULE_H
#define SROS_IMU_LASER_MODULE_H

#include "core/module.h"
#include "imu_resolver.hpp"

namespace imu {
class ImuModule : public sros::core::Module
{
 public:
    ImuModule();
    virtual ~ImuModule();

    virtual void run();

    /**
     * @brief Get the Imu With Idx object
     *        程序开始时有可能返回无效的ImuData，需要判断ImuData.status
     * @param index 容器索引
     * @return ImuData 
     */
    static ImuData getImuWithIdx(const int index = 0);

    /**
     * @brief Get the Imu With Stamp object
     *        程序开始时有可能返回无效的ImuData，需要判断ImuData.status
     * @param stamp  时间戳索引
     * @return ImuData 
     */
    static ImuData getImuWithStamp(const int64_t stamp);
    static void setStandstill(const bool is_standstill);

 public:
    static std::unique_ptr<ImuResolver> imu_resovler_;

};
} // namespace imu

#endif
