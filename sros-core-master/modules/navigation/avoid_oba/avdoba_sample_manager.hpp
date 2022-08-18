//
// Created by lfc on 18-12-5.
//

#ifndef PROJECT_COLLIDE_SAMPLE_MANAGER_HPP
#define PROJECT_COLLIDE_SAMPLE_MANAGER_HPP

#include <vector>
#include "collide_region_manager.hpp"
#include "avdoba_module_para.hpp"
#include "particle_info_msg.hpp"

namespace avoidoba {

class AvdobaSampleManager {
public:

    virtual ~AvdobaSampleManager() {

    }

    AvdobaSampleManager(AvdobaModulePara_Ptr avd_para) : para_(avd_para) {
        slow_region_offset_.reset(new RegionOffsetPara);
        station_region_offset_.reset(new RegionOffsetPara);
        backward_offset_.reset(new RegionOffsetPara);//如果是后退时,车身尺寸offset与前进处理方法想反
        forward_offset_.reset(new RegionOffsetPara);
        deviation_path_offset_.reset(new RegionOffsetPara);
        rotate_offset_.reset(new RegionOffsetPara);//旋转时,车身尺寸offset与前进不同
        updatePara(avd_para);
    }

    void updatePara(AvdobaModulePara_Ptr avd_para){
        if (para_ != avd_para) {
            para_ = avd_para;
        }
        if (!sample_stop_line_interplation_) {
            sample_stop_line_interplation_.reset(
                new LineInterplation(0, para_->max_collide_region_count, para_->min_stop_collide_value,
                                     para_->max_stop_collide_value));//停止区线性插值求解器
        }else{
            sample_stop_line_interplation_->updatePara(0, para_->max_collide_region_count,
                                                       para_->min_stop_collide_value, para_->max_stop_collide_value);
        }
        if(!sample_slow_line_interplation_){
            sample_slow_line_interplation_.reset(
                new LineInterplation(0, para_->max_slow_region_count, para_->min_slow_collide_value,
                                     para_->max_slow_collide_value));//减速区线性插值求解器
        }else{
            sample_slow_line_interplation_->updatePara(0, para_->max_slow_region_count, para_->min_slow_collide_value,
                                                       para_->max_slow_collide_value);
        }
        if(!side_slow_line_interplation_){
            side_slow_line_interplation_.reset(
                new LineInterplation(0, para_->slow_width_offset, para_->min_slow_collide_value,
                                     para_->max_slow_collide_value));//边缘减速区的线性插值求解器
        }else{
            side_slow_line_interplation_->updatePara(0, para_->slow_width_offset, para_->min_slow_collide_value,
                                                     para_->max_slow_collide_value);
        }
        if (collide_regions_.size() != avd_para->max_collide_region_count) {
            collide_count_ = avd_para->max_collide_region_count;
            collide_regions_.resize(avd_para->max_collide_region_count);//限定停止与减速采样个数
            for (int i = 0; i < para_->max_collide_region_count; ++i) {
                double weight = sample_stop_line_interplation_->getDegreInterplation(i);
                if(collide_regions_[i]){
                    collide_regions_[i]->updateWeight(weight);
                }else{
                    collide_regions_[i].reset(new CollideRegionManager(weight, side_slow_line_interplation_));
                }
            }
            station_region_.reset(
                new CollideRegionManager(para_->max_collide_region_count, side_slow_line_interplation_, true));//当前站点处避障未启用.后期可根据需求使用
        }
        if (slow_regions_.size() != avd_para->max_slow_region_count) {
            slow_regions_.resize(avd_para->max_slow_region_count);
            slow_count_ = avd_para->max_slow_region_count;
            for (int i = 0; i < para_->max_slow_region_count; ++i) {
                double weight = sample_slow_line_interplation_->getDegreInterplation(i);
                if (slow_regions_[i]) {
                    slow_regions_[i]->updateWeight(weight);
                }else{
                    slow_regions_[i].reset(new CollideRegionManager(weight, side_slow_line_interplation_, true));//不包含减速余量区域,只有停止区两边需要包含减速余量区
                }
            }
        }
    }

    void updateSamplePoses(AvdobaPoseInfo &avdoba_poses, const OriginCollideSize_Ptr &origin_size,
                           const RegionOffsetPara_Ptr &offset_para,const RegionAsymmetricOffsetPara_Ptr &asymmetric_offset) {
        auto &collide_poses = avdoba_poses.collide_poses;
        auto &slow_poses = avdoba_poses.slow_poses;
        auto &station_pose = avdoba_poses.station_pose;

        *slow_region_offset_ = *offset_para;
        *backward_offset_ = *offset_para;
        *forward_offset_ = *offset_para;
        *rotate_offset_ = *offset_para;
        backward_offset_->stop_length_backward_offset = 0.0;
        backward_offset_->stop_length_forward_offset = 0.0;

        forward_offset_->stop_length_forward_offset = 0.0;
        forward_offset_->stop_length_backward_offset = 0.0;//如果不是原地旋转,则不需要考虑前侧避障,只有旋转才需要考虑前侧避障,因此设置为0

        collide_count_ = collide_poses.size();
        slow_count_ = slow_poses.size();
        if (collide_count_ > para_->max_collide_region_count) {
            LOG(INFO)<<"collide count is larger than max_region_count!";
            collide_count_ = para_->max_collide_region_count;
        }
        if (slow_count_ > para_->max_slow_region_count) {
            LOG(INFO)<<"slow count is larger than max_region_count!";
            slow_count_ = para_->max_slow_region_count;
        }

        origin_size->left_width = origin_size->width / 2.0;
        origin_size->right_width = -origin_size->width / 2.0;
        origin_size->head_length = origin_size->length / 2.0;
        origin_size->back_length = -origin_size->length / 2.0;
        origin_size->left_width += asymmetric_offset->left_offset;
        origin_size->right_width -= asymmetric_offset->right_offset;
        origin_size->head_length += asymmetric_offset->forward_offset;
        origin_size->back_length -= asymmetric_offset->backward_offset;
        origin_size->back_length += origin_size->no_avoid_oba_back_length;//针对识别栈板过程避障
        origin_size->head_length -= origin_size->no_avoid_oba_head_length;//针对其他情况
        //        std::vector<Eigen::Vector2d> obas;
        if (para_->enable_auto_enhance_oba_width) {
            if (collide_poses.size() >= 2) {
                if ((collide_poses[0].pose - collide_poses[1].pose).head<2>().norm() < para_->deviation_dist_thresh){
                    auto delta_point = (collide_poses[0].pose - collide_poses[1].pose).head<2>();
                    double origin_theta = collide_poses[1].pose[2];
                    Eigen::Vector2d local_point;
                    local_point[0] = delta_point[0] * cos(origin_theta) + delta_point[1] * sin(origin_theta);
                    local_point[1] = -delta_point[0] * sin(origin_theta) + delta_point[1] * cos(origin_theta);
                    collide_poses[0].pose.head<2>() = collide_poses[1].pose.head<2>();
                    if (local_point[1] > 0) {
                        origin_size->left_width += local_point[1];
                    }else{
                        origin_size->right_width += local_point[1];
                    }
                }
            }
        }
        OriginCollideSize_Ptr real_size(new OriginCollideSize);
        *real_size = *origin_size;

        if(collide_regions_.size()< collide_count_){
            LOG(INFO)<<"collide_regions_ size is wrong !";
            collide_count_ = collide_regions_.size();
        }
        for (int i = 0; i < collide_count_; ++i) {
            auto &collide_pose = collide_poses[i];

            if (collide_pose.move_state == BACKWARD_MOVE) {//当前粒子属性如果为后退,按照如下方法处理
                collide_regions_[i]->computeTransformPara(collide_pose.pose, real_size, backward_offset_);
            } else if (collide_pose.move_state == FORWARD_MOVE) {//当前粒子属性如果为前进,按照如下方法处理
                collide_regions_[i]->computeTransformPara(collide_pose.pose, real_size, forward_offset_);
            } else if (collide_pose.move_state == ROTATE_MOVE) {//当前粒子属性如果为旋转,按照如下方法处理
                if (use_synchronize_rotate) {
                    OriginCollideSize_Ptr rotate_size(new OriginCollideSize);
                    computeSynRotateAvdModel(real_size, rotate_size);
                    collide_regions_[i]->computeRotateTransformPara(collide_pose, rotate_size, rotate_offset_);
                    real_size->rotate_angle -= rotate::RotateCurveOperator::normalizeAngle(
                            collide_pose.end_angle - collide_pose.start_angle);
                } else {
                    collide_regions_[i]->computeRotateTransformPara(collide_pose, real_size, rotate_offset_);
                }
            } else {
                collide_regions_[i]->computeTransformPara(collide_pose.pose, real_size, forward_offset_);
            }
        }
        if (collide_poses.size() >= 2) {
            if ((collide_poses[0].pose - collide_poses[1].pose).head<2>().norm() > para_->deviation_dist_thresh) {
                LOG(WARNING) << "deviation too large!" <<"curr pose:"<<collide_poses[0].pose[0]<<","<<collide_poses[0].pose[1]<<",map to path pose:"
                        <<collide_poses[1].pose[0]<<","<<collide_poses[1].pose[1]<<",delta dist:"<<(collide_poses[0].pose - collide_poses[1].pose).head<2>().norm();
                if (collide_poses[0].move_state == BACKWARD_MOVE) {
                    *deviation_path_offset_ = *backward_offset_;
                } else if (collide_poses[0].move_state == FORWARD_MOVE) {
                    *deviation_path_offset_ = *forward_offset_;
                }
                deviation_path_offset_->stop_length_forward_offset = para_->deviation_large_head_stop_offset;
                deviation_path_offset_->stop_length_backward_offset = para_->deviation_large_back_stop_offset;
                deviation_path_offset_->stop_width_offset = para_->deviation_large_width_offset;
                collide_regions_[0]->computeTransformPara(collide_poses[0].pose, origin_size,
                                                                deviation_path_offset_);
            }
        }

        if(slow_regions_.size() < slow_count_){
            LOG(INFO) << "slow_regions_ size is wrong!";
            slow_count_ = slow_regions_.size();
        }
        for (int j = 0; j < slow_count_; ++j) {
            auto &slow_pose = slow_poses[j];
            if (slow_pose.move_state == ROTATE_MOVE) {
                if(use_synchronize_rotate) {
                    OriginCollideSize_Ptr rotate_size(new OriginCollideSize);
                    computeSynRotateAvdModel(real_size, rotate_size);
                    real_size->rotate_angle -= rotate::RotateCurveOperator::normalizeAngle(
                            slow_pose.end_angle - slow_pose.start_angle);
                    slow_regions_[j]->computeRotateTransformPara(slow_pose, rotate_size, slow_region_offset_);
                }else{
                    slow_regions_[j]->computeRotateTransformPara(slow_pose, real_size, slow_region_offset_);//对于减速粒子来讲,不需要考虑前进后退的影响,因此目前只要区分旋转和正常粒子即可
                }
            }else{
                slow_regions_[j]->computeTransformPara(slow_pose.pose, real_size, slow_region_offset_);
            }
        }

        if (collide_count_ < para_->max_collide_region_count) {//当接近目标站点时,才会出现,当前碰撞粒子个数小于粒子总数的情况,所以此时是接近了站点,需要添加站点粒子.但是当前站点粒子只是赋值,尚未启用
            *station_region_offset_ = *offset_para;
            if (station_pose.move_state==BACKWARD_MOVE) {
                station_region_offset_->stop_length_backward_offset += para_->station_stop_offset;
            } else {
                station_region_offset_->stop_length_forward_offset += para_->station_stop_offset;
            }
            station_region_->computeTransformPara(station_pose.pose, origin_size, station_region_offset_);
        }
    }

    void computeSynRotateAvdModel(const OriginCollideSize_Ptr& ref_size,const OriginCollideSize_Ptr& rotate_size){
        *rotate_size = *ref_size;
        rotate_size->length = ref_size->car_length;
        rotate_size->width = ref_size->car_width;
        rotate_size->head_length = ref_size->car_length / 2.0;
        rotate_size->back_length = -ref_size->car_length / 2.0;
        rotate_size->left_width = ref_size->car_width / 2.0;
        rotate_size->right_width = -ref_size->car_width / 2.0;
        rotate_size->rotate_angle = 0.0;
    }

    bool computeCollideValue(std::map<std::string,std::vector<Eigen::Vector2d>> &obas, CollideResultInfo &result_info) {
        CollideResultInfo collide_region_info;
        if (computeCollideRegionValue(obas, collide_region_info)) {//首先遍历所有碰撞粒子,如果有碰撞可能,则立即返回碰撞值
            if (collide_region_info.collide_value >= para_->min_stop_collide_value) {
                result_info = collide_region_info;
                return true;
            }
        }
        CollideResultInfo slow_region_info;
        if (computeSlowRegionValue(obas, slow_region_info)) {//再遍历所有减速粒子,如果有减速可能,则立即返回减速粒子最大碰撞值
            result_info = collide_region_info.collide_value > slow_region_info.collide_value ? collide_region_info
                                                                                             : slow_region_info;
            return true;
        }
        if (collide_region_info.collide_value >= para_->min_slow_collide_value) {
            result_info = collide_region_info;
            return true;
        }
        result_info.index = -1;
        return false;
    }

    void updatePara() {
        station_region_.reset(
                new CollideRegionManager(para_->max_collide_region_count, side_slow_line_interplation_, true));
        side_slow_line_interplation_->min_x_ = 0;
        side_slow_line_interplation_->max_x_ = para_->slow_width_offset;
    }

    void updateSynRotateState(bool curr_syn_rotate_state) {
        use_synchronize_rotate = curr_syn_rotate_state;
    }
    void updateSlowOffsetPara() {
        side_slow_line_interplation_->min_x_ = 0;
        side_slow_line_interplation_->max_x_ = para_->slow_width_offset;
    }

private:
    bool computeCollideRegionValue(std::map<std::string, std::vector<Eigen::Vector2d>> &obas, CollideResultInfo &result_info) {
        CollideResultInfo collide_region_info;
        collide_region_info.is_collide_sample = true;
        collide_region_info.collide_value = 0;
        collide_region_info.index = -1;
        for (int i = 0; i < collide_count_; ++i) {
            for (auto &oba_iter:obas) {
                auto& points = oba_iter.second;
                for(auto&oba:points) {
                    auto collide_value = collide_regions_[i]->computeCollideValue(oba);
                    if (collide_value >= para_->min_slow_collide_value) {//如果大于最小减速值,则认为一定至少会发生减速
                        if (collide_region_info.collide_value < collide_value) {
                            collide_region_info.collide_value = (int) round(collide_value);
                            collide_region_info.index = i;
                            collide_region_info.oba_name = oba_iter.first;
                            collide_region_info.collide_point = oba;
                        }
//                        if (collide_value > para_->max_slow_collide_value) {//如果大于最大减速值,则认为一定会发生碰撞
//                            result_info = collide_region_info;
//                            result_info.is_collide_sample = true;
//                            return true;
//                        }
                    }
                }
            }
        }
        result_info = collide_region_info;
        if (result_info.collide_value >= para_->min_slow_collide_value) {
            result_info.is_collide_sample = true;
            return true;
        }
        return false;
    }

    bool computeSlowRegionValue(std::map<std::string,std::vector<Eigen::Vector2d>> &obas, CollideResultInfo &result_info) {
        result_info.index = -1;
        for (int i = 0; i < slow_count_; ++i) {
            for (auto &oba:obas) {
                for (auto &oba_iter:obas) {
                    auto &points = oba_iter.second;
                    for (auto &oba : points) {
                        auto collide_value = slow_regions_[i]->computeCollideValue(oba);
                        if (collide_value >= para_->min_slow_collide_value) {
                            result_info.collide_value = (int)round(collide_value);
                            result_info.index = i;
                            result_info.is_collide_sample = false;
                            result_info.oba_name = oba_iter.first;
                            result_info.collide_point = oba;
                            return true;  //最先找到的障碍物得到的权重最高.考虑到粒子个数一定,那么id小的粒子权重大,所以碰撞值大
                        }
                    }
                }
            }
        }
        return false;
    }

    std::vector<CollideRegionManager_Ptr> collide_regions_;//碰撞粒子(将停止距离以一定步长进行采样得到)
    std::vector<CollideRegionManager_Ptr> slow_regions_;//减速粒子(将减速距离以一定步长进行采样)
    CollideRegionManager_Ptr station_region_;//站点区域
    AvdobaModulePara_Ptr para_;//配置参数

    LineInterplation_Ptr sample_stop_line_interplation_;//碰撞粒子差值模型
    LineInterplation_Ptr sample_slow_line_interplation_;//减速粒子差值模型
    LineInterplation_Ptr side_slow_line_interplation_;//两侧差值模型

    RegionOffsetPara_Ptr slow_region_offset_;//减速粒子碰撞区域offset
    RegionOffsetPara_Ptr backward_offset_;//后退时,碰撞区offset,对碰撞粒子有效
    RegionOffsetPara_Ptr rotate_offset_;//旋转时,offset
    RegionOffsetPara_Ptr forward_offset_;//前进时,碰撞区offset
    RegionOffsetPara_Ptr deviation_path_offset_;//前进时,碰撞区offset
    RegionOffsetPara_Ptr station_region_offset_;//站点粒子offset,目前未启用
    int collide_count_;//碰撞粒子个数,通过停止距离与步长计算得到,一旦设置,则固定
    int slow_count_;//减速粒子个数,通过减速距离与步长计算得到,一旦设置,则固定
    const double deviation_dist_thresh_ = 0.5;
    bool use_synchronize_rotate = false;
};
typedef std::shared_ptr<AvdobaSampleManager> AvdobaSampleManager_Ptr;

}

#endif //PROJECT_COLLIDE_SAMPLE_MANAGER_HPP
