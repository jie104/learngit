//
// Created by lfc on 2022/3/2.
//

#ifndef SROS_MANUAL_AVDOBA_PROCESSOR_HPP
#define SROS_MANUAL_AVDOBA_PROCESSOR_HPP
#include <glog/logging.h>

#include "avdoba_module_para.hpp"
#include "curve_sample/curve_sample_factory.h"
#include "avdoba_sample_manager.hpp"

namespace avoidoba{
class ManualAvdobaProcessor {
 public:
    ManualAvdobaProcessor(AvdobaModulePara_Ptr &module_para):para_(module_para){ updatePara(); }

    void checkManualVelocity(const Eigen::Vector3d& curr_pose,OriginCollideSize_Ptr origin_size,
                             AvdobaPoseInfo &avdoba_poses_info,Eigen::Vector2d& velocity){
        avdoba_poses_info.collide_poses.clear();
        avdoba_poses_info.slow_poses.clear();
        auto &stop_particles = avdoba_poses_info.collide_poses;
        auto &slow_particles = avdoba_poses_info.slow_poses;
        CollideResultInfo &collide_result_info = avdoba_poses_info.result_info;
        transversePath(curr_pose, velocity, stop_particles, slow_particles);
        avdoba_sample_manager_->updateSamplePoses(avdoba_poses_info, origin_size, offset_para_,asymmetric_para_);
        avdoba_sample_manager_->computeCollideValue(avdoba_poses_info.oba_points, collide_result_info);
        if (collide_result_info.collide_value >= para_->min_stop_collide_value) {
            velocity = Eigen::Vector2d::Zero();
        } else if (collide_result_info.collide_value >= para_->min_slow_collide_value) {
            auto curr_ratio = (float)(para_->max_slow_collide_value - collide_result_info.collide_value) / (float)para_->max_stop_collide_value;
            if (curr_ratio < 0.3f) {
                curr_ratio = 0.3f;
            }
            velocity *= curr_ratio;
        }
    }

    void updateOffsetPara(double slow_width_offset,double stop_width_offset,double stop_length_offset) {
        para_->slow_width_offset = slow_width_offset;
        para_->stop_width_offset = stop_width_offset;
        para_->stop_length_forward_offset = stop_length_offset;

        offset_para_->slow_width_offset = slow_width_offset;
        offset_para_->stop_width_offset = stop_width_offset;
        offset_para_->stop_length_forward_offset = stop_length_offset;
        if (offset_para_->stop_length_forward_offset < 0.05) {
            offset_para_->stop_length_forward_offset = 0.05;
        }
        avdoba_sample_manager_->updateSlowOffsetPara();
    }

    void updateAsymetricOffsetPara(double left, double right, double head, double back) {
        asymmetric_para_->left_offset = left;
        asymmetric_para_->right_offset = right;
        asymmetric_para_->forward_offset = head;
        asymmetric_para_->backward_offset = back;
    }

    void updatePara() {
        if (para_->forward_stop_distance == 0) {
            LOG(INFO) << "forward stop distance is wrong! will set to 1!";
            para_->forward_stop_distance = 1;
        }
        if (para_->backward_stop_distance == 0) {
            LOG(INFO)<<"backward stop distance is wrong! will set to 1!";
            para_->backward_stop_distance = 1;
        }
        if (para_->forward_slow_distance <= para_->forward_stop_distance) {
//            LOG(INFO) << "foward slow dist is smaller than stop distance! will set default value!";
            para_->forward_slow_distance = para_->forward_stop_distance + para_->sample_step;
        }
        para_->backward_step = para_->sample_step * para_->backward_stop_distance / para_->forward_stop_distance;//后退时的步长与前进时的步长满足一定关系,这样就可以确保采样个数固定
        para_->max_collide_region_count = int(para_->forward_stop_distance / para_->sample_step + 0.5);//计算停止采样个数,限定了停止距离和停止步长后,个数一定
        para_->max_slow_region_count = int(
            (para_->forward_slow_distance - para_->forward_stop_distance) / para_->sample_step + 0.5);//计算减速采样个数
        if (!avdoba_sample_manager_) {
            avdoba_sample_manager_.reset(new AvdobaSampleManager(para_));//根据采样粒子个数和碰撞值,创建采样管理器
        }else{
            avdoba_sample_manager_->updatePara(para_);
        }

        if (!offset_para_) {
            offset_para_.reset(new RegionOffsetPara);//计算基于车身尺寸生成的offset
        }
        offset_para_->slow_width_offset = para_->slow_width_offset;
        offset_para_->stop_length_forward_offset = para_->stop_length_forward_offset;
        offset_para_->stop_length_backward_offset = para_->stop_length_backward_offset;
        offset_para_->stop_width_offset = para_->stop_width_offset;
        if (!asymmetric_para_) {
            asymmetric_para_.reset(new RegionAsymmetricOffsetPara);
        }
    }
 private:

    void updateSamplePoses(AvdobaPoseInfo &avdoba_poses, const OriginCollideSize_Ptr &origin_size,
                           const RegionOffsetPara_Ptr &offset_para,const RegionAsymmetricOffsetPara_Ptr &asymmetric_offset) {
        origin_size->left_width = origin_size->width / 2.0;
        origin_size->right_width = -origin_size->width / 2.0;
        origin_size->head_length = origin_size->length / 2.0;
        origin_size->back_length = -origin_size->length / 2.0;
        origin_size->left_width += asymmetric_offset->left_offset;
        origin_size->right_width -= asymmetric_offset->right_offset;
        origin_size->head_length += asymmetric_offset->forward_offset;
        origin_size->back_length -= asymmetric_offset->backward_offset;
    }

    //TODO:针对于差速轮、舵轮等单自由度车型
    void transversePath(const Eigen::Vector3d& curr_pose,const Eigen::Vector2d& vel,std::vector<ParticleInfo>& stop,std::vector<ParticleInfo>& slow){
        const double delta_t = 0.05;
        double curr_dist = 0.0;
        double curr_rotate = 0.0;
        Eigen::Vector3d incre_pose = Eigen::Vector3d::Zero(), world_pose = curr_pose;
        ParticleMoveState move_state = FORWARD_MOVE;
        if (vel[0] < 0.01 && vel[1] > 0.05) {
            move_state = ROTATE_MOVE;
        } else if (vel[0] < 0) {
            move_state = BACKWARD_MOVE;
        }

        incre_pose[0] = vel[0] * delta_t;
        incre_pose[2] = vel[1] * delta_t;
        stop.emplace_back();
        stop.back().pose = world_pose;
        stop.back().move_state = move_state;
        while (curr_dist < para_->manual_stop_distance && curr_rotate < para_->manual_stop_rotate &&
               stop.size() < para_->max_collide_region_count) {
            curr_dist += fabs(incre_pose[0]);
            curr_rotate += fabs(incre_pose[2]);
            world_pose = addPose(world_pose, incre_pose);
            stop.emplace_back();
            stop.back().pose = world_pose;
            stop.back().move_state = move_state;
        }
        LOG(INFO) << "dist:" << curr_dist << "," << para_->manual_stop_distance << "," << stop.size() << ","
                  << curr_rotate;
        double max_dist = para_->manual_stop_distance + para_->manual_slow_distance;
        double max_rotate = para_->manual_stop_rotate + para_->manual_slow_rotate;
        slow.emplace_back();
        slow.back().pose = world_pose;
        slow.back().move_state = move_state;
        while (curr_dist < max_dist && curr_rotate < max_rotate && slow.size() < para_->max_slow_region_count) {
            curr_dist += fabs(incre_pose[0]);
            curr_rotate += fabs(incre_pose[2]);
            world_pose = addPose(world_pose, incre_pose);
            slow.emplace_back();
            slow.back().pose = world_pose;
            slow.back().move_state = move_state;
        }
    }

    Eigen::Vector3d addPose(const Eigen::Vector3d& curr_pose,const Eigen::Vector3d& delta_pose){
        Eigen::Affine2d origin_tf(Eigen::Translation2d(curr_pose[0], curr_pose[1]) *
            Eigen::Rotation2Dd(curr_pose[2]));
        auto point = origin_tf * delta_pose.head<2>();
        auto delta_yaw = delta_pose[2] + curr_pose[2];
        delta_yaw = atan2(sin(delta_yaw), cos(delta_yaw));
        return Eigen::Vector3d(point[0], point[1], delta_yaw);
    }

    AvdobaModulePara_Ptr para_;//配置参数
    AvdobaSampleManager_Ptr avdoba_sample_manager_;//采样避障管理器,通过curve_sample计算出要采样的粒子,输入到该管理器中,计算碰撞值,判断是否发生碰撞
    RegionOffsetPara_Ptr offset_para_;//碰撞尺寸offset
    RegionAsymmetricOffsetPara_Ptr asymmetric_para_;//碰撞尺寸offset
};
}


#endif  // SROS_MANUAL_AVDOBA_PROCESSOR_HPP
