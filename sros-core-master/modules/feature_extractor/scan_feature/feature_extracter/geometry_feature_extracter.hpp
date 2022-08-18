#pragma once

#include "base_feature_extracter.hpp"
#include <fstream>
namespace extract
{
    using Point2f = feature::Point2f;

    class GeometryFeatureExtracter : public virtual BaseFeatureExtracter
    {
    public:
        GeometryFeatureExtracter(ExtractPara_ptr para) : BaseFeatureExtracter(para)
        {

        }
        virtual bool extract(const sros::core::LaserScan_ptr& scan, feature::FeatureContainer_Ptr &features)//从scan分割，保存到features指向的位置.override保证继承类不会默认继承这里的内容
        {
            downSample(scan);//降采样，将一个区间内的点用一个点来表示
            getRegions();//根据r分段
            getLineSegments();//根据高线法以5个点，5cm高度为阈值继续分割，依然保存每一段的索引pair
            if (getLineSegmentFeatures(features))//提取线段特征
            {
                getCornerFeatures(features);
                return true;
            }
            return false;
        }

    private:
        /*
         * 断点判断系数k
         * abs(Ri - Ri-1) - Ri-1 * k > 3 * sigma时, Ri为断点
         * k = sin(delta_phi) / sin(lambda - delta_phi)   lambda=10，意思是，夹角小于10就不认为是连续的了
         */
        float _k = 0.f;

        std::vector<unsigned int> end_index_table_; //终止点序号表(对相邻序号间的点降采样,这里使用中位数滤波算法)

        std::vector<Eigen::Vector2f> cos_sin_pairs_;    //各降采样点角度对应的cos,sin值对

        std::vector<float> ranges_; //各降采样点的距离值

        std::vector<Point2f> points_;   //各降采样点的坐标

        std::list<std::pair<unsigned int, unsigned int>> regions_; //所有可能存在线段的区域，存储区域首尾对应的降采样点序号

        std::list<std::pair<unsigned int, unsigned int>> line_segments_; //所有可能为线段，存储线段首尾对应的降采样点序号


        std::vector<Eigen::Vector2f> break_points;

        //降采样（中位数滤波）
        void downSample(const sros::core::LaserScan_ptr& scan)
        {
            if (end_index_table_.empty())
            {
                initDownSampleInfos(scan);//分段，计算每一段的结尾，中位角度
            }
            computeRanges(scan);//计算每一段的距离中位数
            computePoints();//计算所有的降采样点
        }
        /*
         * 初始化降采样信息
         * 确定降采样点总数
         * 计算断点判断系数k
         * 确定各降采样点对应的原始数据点的起始和终止序号
         * 确定各降采样点对应的cos和sin值对
         */
        void initDownSampleInfos(const sros::core::LaserScan_ptr& scan)
        {
            if (scan->ranges.size() < para_->sample_point_size_)
            {
                para_->sample_point_size_ = scan->ranges.size();//降采样点数量要小于总点数
            }
            const unsigned int size = para_->sample_point_size_;
            const float delta_phi = (scan->angle_max - scan->angle_min) / static_cast<float>(size);
            _k = std::sin(delta_phi) / std::sin(para_->lambda_ - delta_phi);//只跟角度分辨率和lambda有关
            end_index_table_.reserve(size);
            cos_sin_pairs_.reserve(size);
            ranges_.reserve(size);
            points_.reserve(size);
            const float step = static_cast<float>(scan->ranges.size()) / static_cast<float>(size);//每隔多少点采一个
            unsigned int end_index, last_end_index = 0;
            float end_pos = step;
            while (end_index_table_.size() < size)
            {
                end_index = static_cast<unsigned int>(end_pos);//根据步长确定的点取整
                const float angle = scan->angle_increment * static_cast<float>((last_end_index + end_index) / 2) + scan->angle_min;//中位角度
                end_index_table_.push_back(end_index);//该区间的最后一个索引
                cos_sin_pairs_.emplace_back(std::cos(angle), std::sin(angle));//中位的角度
                last_end_index = end_index;
                end_pos += step;
            }//保存了每一步的最后结束索引，该区间内中位的角度
        }

        //判断序号i对应的距离是否有效
        bool isRangeValid(const sros::core::LaserScan_ptr& scan, const unsigned int i)
        {
            if (scan->ranges[i] < scan->range_min)
                return false;
            if (scan->ranges[i] > scan->range_max)
                return false;
            return true;
        }

        /*
         * 计算降采样点的距离值
         * 距离为该段所有点的中位数
         */
        void computeRanges(const sros::core::LaserScan_ptr& scan)
        {
            ranges_.clear();
            std::vector<float> sample_ranges;
            unsigned int front_index = 0;
            auto iter = end_index_table_.cbegin();//const迭代器，不可修改元素值
            while (iter != end_index_table_.cend())
            {
                for (unsigned int i = front_index; i < *iter; i++)
                {
                    if (isRangeValid(scan, i))
                    {
                        sample_ranges.push_back(scan->ranges[i]);
                    }
                }
                if (sample_ranges.empty())
                {
                    ranges_.push_back(0.f);
                }
                else
                {
                    std::nth_element(sample_ranges.begin(), sample_ranges.begin() + sample_ranges.size() / 2, sample_ranges.end());//中位数
                    ranges_.push_back(sample_ranges[sample_ranges.size() / 2]);//保存每一段r的中位数
                    sample_ranges.clear();
                }
                front_index = *iter++;
            }
        }

        //计算所有降采样点的x，y坐标
        void computePoints()
        {
            points_.clear();
            for (unsigned int i = 0; i < para_->sample_point_size_; i++)
            {
                points_.push_back(ranges_[i] * cos_sin_pairs_[i]);//计算成点，实际上使用降采样区间的中间角度作为角度，中位数作为距离
            }
        }

        //判断序号i对应的降采样点是否为断点
        bool isBreakIndex(const unsigned int i)
        {
            float range_diff = 0.f;
            float diff_ratio = 0.f;
            Eigen::Vector3f point_to_0 = Eigen::Vector3f::Zero();//由第i点指向原点的向量
            Eigen::Vector3f point_to_before = Eigen::Vector3f::Zero();//由第i点指向第i-1点的向量
            if (i == 0)
            {
                range_diff = std::abs(ranges_.front() - ranges_.back());
                diff_ratio = range_diff/ranges_.back();
                point_to_0.head(2) = -points_[0];
                point_to_before.head(2) = points_.back()-points_[0];
            }
            else
            {
                range_diff = std::abs(ranges_[i] - ranges_[i - 1]);
                diff_ratio = range_diff / ranges_[i-1];
                point_to_0.head(2) = -points_[i];
                point_to_before.head(2) = points_[i-1]-points_[i];
            }

//            float sin_angle = point_to_0.cross(point_to_before).norm() / (point_to_0.norm() * point_to_before.norm());
//            return (range_diff > (3.f * para_->sigma_) || (diff_ratio > para_->dist_ratio_thre_smear_ && sin_angle < std::sin(para_->angle_thre_smear_)));//大于15cm或者大于4%或者夹角小于 30°
            return (range_diff > (2.0f * para_->sigma_));//相邻点r差距7.25cm认为出现断点
//            return (value > (3.f * _para->sigma_) || sin_angle < std::sin(PI/6));//大于15cm或者夹角小于 30°
//            return value > (3.f * _para->sigma_);//大于15cm判断为断点，这里需要再细化！！！
        }

        //获得可能存在线段的区域，存储区域首尾对应的降采样点序号
        void getRegions()//距离足够远的断点区分
        {
            regions_.clear();

            break_points.clear();//

            int min_count = 3;
            unsigned int front_index = 0;//一开始默认从0开始
            for (unsigned int i = 1; i < ranges_.size(); i++)
            {
                if (isBreakIndex(i))//该处是断点，可能是跳跃或者拖尾
                {

                    break_points.push_back(points_[i]);//

                    if (ranges_[front_index] != 0.f && i-front_index >= min_count)
                    {

                        regions_.emplace_back(front_index, i - 1);//i处是断点，上一段就保存到i-1为止
                    }
                    front_index = i;//完整的第一段的开头
                }
            }
            //首尾相接处的特殊处理
            if (ranges_[front_index] != 0.f)
            {
                if (isBreakIndex(0))
                {
                    if(para_->sample_point_size_-front_index >= min_count){
                        regions_.emplace_back(front_index, para_->sample_point_size_ - 1);
                    }
                }
                else
                {
                    if(regions_.front().second+para_->sample_point_size_-front_index >= min_count){
                        regions_.front().first = front_index;//修改第一段
                        regions_.front().second += para_->sample_point_size_;
                    }
                }
            }
        }

        //获得所有可能的线段，存储线段首尾对应的降采样点序号
        void getLineSegments()
        {
            line_segments_.clear();//保存最大高度也低于一定要求的线段
            //把区域分割成线区域，滤除点数过少的区域
            while (!regions_.empty())
            {
                const auto region = regions_.front();
                regions_.pop_front();
                if (region.second - region.first >= para_->line_segment_point_size_threshold_)//至少5个点的region才会成为line
//                if((_points[region.second]-_points[region.first]).norm() >= _para->_min_length_for_split)//低于3cm不用高线法split
                {
                    const auto farthest = getFarthest(region);
                    if (farthest.first > para_->farthest_dist_threshold_)//超过5cm，需要高线法分开
                    {
                        regions_.emplace_front(farthest.second, region.second);
                        regions_.emplace_front(region.first, farthest.second - 1);
                    }
                    else
                    {
                        line_segments_.push_back(region);
                    }
                }
            }
        }

        //根据区域首尾点确定的直线，获得区域内距离该直线的最远点的序号以及对应的距离
        std::pair<float, unsigned int> getFarthest(const std::pair<unsigned int, unsigned int>& region) const
        {
            const unsigned int front_index = region.first % para_->sample_point_size_;
            const unsigned int back_index = region.second % para_->sample_point_size_;
            std::vector<Point2f> points{points_[front_index], points_[back_index]};
            std::vector<Eigen::Vector2f> pairs{cos_sin_pairs_[front_index], cos_sin_pairs_[back_index]};
            feature::LineSegmentFeature line(points, pairs);
            unsigned int farthest_index = region.first + 1;
            float dist, max_dist = 0.f;
            for (unsigned int i = region.first + 1; i < region.second; i++)
            {
                dist = line.distToPoint(points_[i % para_->sample_point_size_]);//类方法
                if (dist > max_dist)
                {
                    max_dist = dist;
                    farthest_index = i;
                }
            }
            return std::make_pair(max_dist, farthest_index);
        }

        //获得线段特征
        bool getLineSegmentFeatures(feature::FeatureContainer_Ptr features)
        {
            features->line_segment_features.clear();
            if (line_segments_.empty())
                return false;
            std::vector<Point2f> prev_points;
            std::vector<Eigen::Vector2f> prev_pairs;
            std::vector<Point2f> next_points;
            std::vector<Eigen::Vector2f> next_pairs;
            getPointsAndPairsByRegion(prev_points, prev_pairs, line_segments_.front());//line_segments只是通过高度分割出来的，有可能可以拟合成一个
            feature::LineSegmentFeature prev_line_segment(prev_points, prev_pairs);
            auto iter = line_segments_.cbegin();
            while (++iter != line_segments_.cend())
            {
                getPointsAndPairsByRegion(next_points, next_pairs, *iter);
                feature::LineSegmentFeature next_line_segment(next_points, next_pairs);
//                if (prev_line_segment.distToNext(next_line_segment) < 3.f * para_->sigma_ && prev_line_segment
//                                                                                                     .chiSquare(next_line_segment) < 2.77f * para_->sigma_ *para_->sigma_)
                if (prev_line_segment.distToNext(next_line_segment) < 3.f * para_->sigma_ && prev_line_segment.computeAngleWithLine(next_line_segment) < 0.01745*10)//夹角小于10°就满足
                {
                    prev_points.insert(prev_points.end(), next_points.begin(), next_points.end());//点合并，成为一点直线，再拟合
                    prev_pairs.insert(prev_pairs.end(), next_pairs.begin(), next_pairs.end());
                    prev_line_segment = feature::LineSegmentFeature(prev_points, prev_pairs);//
                }
                else
                {
//                    LOG(INFO) << prev_line_segment.back_edge_ << ", " << prev_line_segment.distToNext(next_line_segment) << ", " << prev_line_segment
//                            .chiSquare(next_line_segment);
                    if (isLineSegmentFeatureGood(prev_line_segment))
                        features->line_segment_features.push_back(prev_line_segment);
                    prev_line_segment = next_line_segment;
                    prev_points.swap(next_points);
                    prev_pairs.swap(next_pairs);
                }
            }
            if (isLineSegmentFeatureGood(prev_line_segment))
                features->line_segment_features.push_back(prev_line_segment);

            features->break_points.insert(features->break_points.begin(), break_points.begin(), break_points.end());//保存断点

            return !features->line_segment_features.empty();
        }

        //获得区域间的降采样点坐标及对应的cos,sin值对
        void getPointsAndPairsByRegion(std::vector<Point2f>& points, std::vector<Eigen::Vector2f>& pairs, const std::pair<unsigned int, unsigned int>& region)
        {
            points.clear();
            points.reserve(region.second - region.first + 1);
            pairs.clear();
            pairs.reserve(region.second - region.first + 1);
            for (unsigned int i = region.first; i <= region.second; i++)
            {
                points.push_back(points_[i % para_->sample_point_size_]);
                pairs.push_back(cos_sin_pairs_[i % para_->sample_point_size_]);
            }
        }

        //判断线段特征是否满足条件
        bool isLineSegmentFeatureGood(const feature::LineSegmentFeature& line_segment) const//足够长
        {
            if (line_segment.length_ < para_->line_segment_length_threshold_)//
                return false;
            return true;
        }

        //获得角点特征，并根据角点所属两条边的夹角将角点分成三角triangle和墙角corner两种类型
        bool getCornerFeatures(feature::FeatureContainer_Ptr features)
        {
            features->corner_features.clear();
            auto prev_iter = features->line_segment_features.begin();
            float prev_angle = prev_iter->polar_parameters_.x();
            auto next_iter = ++features->line_segment_features.begin();
            while (next_iter != features->line_segment_features.end())
            {
                //粗筛夹角是否在90°左右
                Eigen::Vector3f back = Eigen::Vector3f::Zero();
                back.head(2) = next_iter->back_edge_-next_iter->front_edge_;
                Eigen::Vector3f front = Eigen::Vector3f::Zero();
                front.head(2) = prev_iter->front_edge_-prev_iter->back_edge_;
                float sin_angle = back.cross(front).norm() / (back.norm() * front.norm());
                if(prev_iter->distToNext(*next_iter) < 2*para_->sigma_ && std::fabs(sin_angle) > std::sin(para_->min_angle_for_multi_type_corner)){//首尾足够近(原来是3sigma=15cm),夹角在30-150或者210-330之间
                    feature::CornerFeature corner(*prev_iter, *next_iter);
                    if(std::abs(corner.included_angle_ - para_->articorner_include_angle) < para_->articorner_include_angle_thre){
                        if(std::abs(corner.prev_.length_-para_->articorner_lenth) / para_->articorner_lenth < para_->articorner_lenth_ratio_thre && std::abs(corner.next_.length_ - para_->articorner_lenth) / para_->articorner_lenth < para_->articorner_lenth_ratio_thre){
                            features->articorner_features.push_back(corner);
//                            static float min_angle = 180.0;
//                            static float max_angle = 0.0;
//                            static float min_len = 0.5;
//                            static float max_len = 0.0;
//                            min_angle = min_angle < corner.included_angle_ ? min_angle : corner.included_angle_;
//                            max_angle = max_angle > corner.included_angle_ ? max_angle : corner.included_angle_;
//                            min_len = std::min(min_len, std::min(corner.prev_.length_, corner.next_.length_));
//                            max_len = std::max(max_len, std::max(corner.prev_.length_, corner.next_.length_));
//                            LOG(INFO) << corner.included_angle_/M_PI*180 << ", " << corner.prev_.length_ << ", " << corner.next_.length_;
//                            LOG(INFO) << max_angle/M_PI*180 << " " << min_angle/M_PI*180 << " " << max_len << " " << min_len;
//                            static int count = 0;
//                            static std::vector<float> angles;
//                            static std::vector<float> lenths;
//                            if(count < 2600){
//                                angles.push_back(corner.included_angle_);
//                                lenths.push_back(corner.prev_.length_);
//                                lenths.push_back(corner.next_.length_);
//                                count++;
//                            }else if(count == 2600){
////                                std::ofstream f1;
////                                f1.open("angle.txt");
////                                for(const auto& a : angles)
////                                    f1 << a << std::endl;
////                                f1.close();
//                                std::ofstream f2;
//                                f2.open("length.txt");
//                                for(const auto& l : lenths)
//                                    f2 << l << std::endl;
//                                f2.close();
//                            }
                        }
                    }else if(std::abs(corner.included_angle_ - para_->corner_include_angle ) < para_->corner_include_angle_thre){
                        if(corner.prev_.length_ > para_->corner_min_lengh && corner.next_.length_ > para_->corner_min_lengh){
                            features->corner_features.push_back(corner);
                        }
                    } else if(std::abs(corner.included_angle_ - para_->triangle_include_angle) < para_->triangle_include_angle_thre){
                        if(std::abs(corner.prev_.length_ - para_->triangle_length_) / para_->triangle_length_ < para_->triangle_lenth_ratio_thre && std::abs(corner.next_.length_ - para_->triangle_length_) / para_->triangle_length_ < para_->triangle_lenth_ratio_thre){
                            corner.direction_ = -corner.direction_;
//                            LOG(INFO) << corner.included_angle_/M_PI*180 << ", " << corner.prev_.length_ << ", " << corner.next_.length_;
                            features->triangle_features.push_back(corner);
                        }
                    }

//                    if(corner.included_angle_ > M_PI && corner.prev_.polar_parameters_(1) > para_->triangle_min_length_ && corner.next_.polar_parameters_(1) > para_->triangle_min_length_){//三角形
//                        features->triangle_features.push_back(corner);
//                    }else if(corner.included_angle_ < M_PI ){//墙角
//                        features->corner_features.push_back(corner);
//                    }
                    prev_iter->trimEdge(corner.coordinate_, false);//把角点作为线段的新端点，并重新计算线段的长度等属性
                    next_iter->trimEdge(corner.coordinate_, true);
                }
                ++prev_iter;
                ++next_iter;
            }
            return !features->corner_features.empty() ||  !features->triangle_features.empty();//至少一个非空
        }
    };

    using GeometryFeatureExtracter_ptr = std::shared_ptr<GeometryFeatureExtracter>;
}