#pragma once

#include "extract_para.hpp"
#include "../feature/features.hpp"
//#include "srosbag/msg/pub_msg/laser_scan_msg.hpp"
//#include <sensor_msgs/LaserScan.h>
#include "glog/logging.h"

namespace extract
{
    class BaseFeatureExtracter
    {
    public:
        explicit BaseFeatureExtracter(ExtractPara_ptr para) : para_(para)
        {

        }

        virtual ~BaseFeatureExtracter() = default;//虚函数，使用默认的构造函数

        //提取特征
        virtual bool extract(const sros::core::LaserScan_ptr& scan, feature::FeatureContainer_Ptr &features) = 0;//纯虚函数

        //设置提取参数
        virtual void reset(ExtractPara_ptr para)
        {
            para_ = para;
        }

    protected:
        ExtractPara_ptr para_;
    };

    using BaseFeatureExtracter_ptr = std::shared_ptr<BaseFeatureExtracter>;

}
