#pragma once

#include "base_feature_extracter.hpp"

namespace extract
{
    using Point2f = feature::Point2f;

    class LandmarkFeatureExtracter : public virtual BaseFeatureExtracter
    {
    public:
        LandmarkFeatureExtracter(ExtractPara_ptr para) : BaseFeatureExtracter(para)
        {

        }

        virtual bool extract(const sros::core::LaserScan_ptr& scan, feature::FeatureContainer_Ptr &features) override
        {
            if (sin_table_.empty())
            {
                initSinCosTable(scan);
            }
            computeNormalizedIntensities(scan);//去掉强度与距离的相关性，原始强度小于32的点当做噪声归一化强度为0
//            computeNormalizedIntensitiesByIncidentAngle(scan);
            getLandmarks(scan);//根据强度和间距获取信标的起始和终止编号
            return getLandmarkFeatures(scan, features);//在上一步分段结果的基础上，筛选强点，包装成landmarkfeature所需要的格式的点
        }

    private:
        std::vector<float> sin_table_;  //sin表

        std::vector<float> cos_table_;  //cos表

        std::list<std::pair<unsigned int, unsigned int>> landmarks_; //所有可能存在的信标，存储信标首尾对应的序号

        std::vector<float> normalized_intensities_; //归一化强度(强度/标准强度)

        //初始化sin表和cos表,避免重复计算
        void initSinCosTable(const sros::core::LaserScan_ptr& scan)
        {
            sin_table_.reserve(scan->intensities.size());
            cos_table_.reserve(scan->intensities.size());
            for (unsigned int i = 0; i < scan->intensities.size(); i++)
            {
                const float angle = scan->angle_increment * static_cast<float>(i) + scan->angle_min;
                sin_table_.push_back(std::sin(angle));
                cos_table_.push_back(std::cos(angle));
            }
        }

        //计算归一化强度,噪点的归一化强度为0
        void computeNormalizedIntensities(const sros::core::LaserScan_ptr& scan)
        {
            normalized_intensities_.clear();
            normalized_intensities_.reserve(scan->intensities.size());
            float standard_intensity;//与距离相关的标准强度
            for (unsigned int i = 0; i < scan->intensities.size(); i++)
            {
                if (scan->ranges[i] < 2.f)
                {
                    standard_intensity = 1700.f;
                }
                else
                {
                    standard_intensity = 3600.f / (scan->ranges[i] + 1.6f) + 700.f;//距离越大强度越小，最小600
                }
                if (scan->intensities[i] < para_->intensity_weak_threshold_)//小于32，噪声
                {
                    normalized_intensities_.push_back(0.f);
                }
                else
                {
                    normalized_intensities_.push_back(scan->intensities[i] / standard_intensity);
                }
            }
        }

        /*
         * 判断是否是特征点
         * 强度归一化后， 0.5f对应的是cos(pi/3)
         * 也就是说入射角小于(pi/3)的反光贴可以被检测
         */
        bool isNormalizedIntensityStrong(const unsigned int index)
        {
            return normalized_intensities_[index] >= 0.35;//75°对应0.26，默认0.5//0.35
        }

        //判断范围是否接近,如不接近则认为是不连续的
        bool isRangeClose(const float lrange, const float rrange)
        {
            return std::abs(lrange - rrange) <= para_->range_close_threshold_;
        }

        //判断特征点个数是否满足要求
        bool isStrongCountEnough(const unsigned int count)
        {
            return count >= para_->strong_count_threshold_;
        }

        //获得所有可能存在的信标，存储信标首尾对应的序号
        void getLandmarks(const sros::core::LaserScan_ptr& scan)
        {
            landmarks_.clear();
            const unsigned int size = scan->intensities.size();
            unsigned int begin_index = 0;
            unsigned int end_index = 0;
            unsigned int loop_end_index = size;
            bool is_start_flag = false;
            unsigned int weak_count = 0;//目前为止，连续weak个数
            for (unsigned int index = 0; index < size; index++)
            {
                if (isNormalizedIntensityStrong(index))//是信标的强度
                {
                    if (!is_start_flag)
                    {
                        begin_index = index;
                        is_start_flag = true;
                    }
                    else
                    {
                        end_index = index - weak_count;
                        if (!isRangeClose(scan->ranges[index], scan->ranges[end_index - 1]))
                        {
                            if (begin_index == 0 && isNormalizedIntensityStrong(size - 1) && isRangeClose(scan->ranges.back(), scan->ranges.front()))//可以接上
                            {
                                loop_end_index = end_index;
                            }
                            else if (isStrongCountEnough(end_index - begin_index))
                            {
                                landmarks_.emplace_back(begin_index, end_index);
                            }
                            begin_index = index;
                        }
                        weak_count = 0;
                    }
                }
                else if (is_start_flag)
                {
                    end_index = index - weak_count;
                    if (++weak_count <= para_->weak_count_threshold_)
                    {
                        continue;
                    }
                    if (begin_index == 0 && isNormalizedIntensityStrong(size - 1) && isRangeClose(scan->ranges.back(), scan->ranges.front()))
                    {
                        loop_end_index = end_index;
                    }
                    else if (isStrongCountEnough(end_index - begin_index))
                    {
                        landmarks_.emplace_back(begin_index, end_index);
                    }
                    is_start_flag = false;
                    weak_count = 0;
                }
            }
            //首尾相接处的特殊处理
            if (loop_end_index != size && isStrongCountEnough(loop_end_index + size - begin_index))//loop_end_index == size代表第一段不需要接上
            {
                landmarks_.emplace_back(begin_index, loop_end_index + size);
            }
        }

        //获得所有信标的特征信息
        bool getLandmarkFeatures(const sros::core::LaserScan_ptr& scan, feature::FeatureContainer_Ptr features)
        {
            features->landmark_features.clear();
            auto iter = landmarks_.begin();
            unsigned int size = scan->intensities.size();
            while (iter != landmarks_.end())
            {
                std::vector<feature::LandmarkFeaturePoint> landmark_feature_points;
                landmark_feature_points.reserve(iter->second - iter->first);
                for (unsigned int i = iter->first; i < iter->second; i++)
                {
                    if (isNormalizedIntensityStrong(i))//只保留强度大的点
                    {
                        const unsigned int index = i % size;
                        feature::LandmarkFeaturePoint landmark_feature_point;
                        landmark_feature_point.coordinate_ = Point2f(scan->ranges[index] * cos_table_[index], scan->ranges[index] * sin_table_[index]);
                        landmark_feature_point.normalized_intensity_ = normalized_intensities_[index];
                        landmark_feature_points.push_back(landmark_feature_point);
                    }
                }
                feature::LandmarkFeature landmark_feature(landmark_feature_points);
                if (isLandmarkFeatureGood(landmark_feature))
                {
                    features->landmark_features.push_back(landmark_feature);
                    ++iter;
                }
                else
                {
                    iter = landmarks_.erase(iter);
                }
            }
            return !features->landmark_features.empty();
        }

        //判断信标特征是否满足条件
        bool isLandmarkFeatureGood(const feature::LandmarkFeature& landmark_feature) const
        {
            if (landmark_feature.type_ == feature::LandmarkFeatureType::UNKNOWN){
                return false;
            }
            if (landmark_feature.distance_ > para_->landmark_distance_threshold_){
                return false;
            }
            return true;//这里没有错误剔除掉对的信标
        }

    };

    using LandmarkFeatureExtracter_ptr = std::shared_ptr<LandmarkFeatureExtracter>;
}