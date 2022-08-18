//
// Created by lfc on 18-12-5.
//

#ifndef PROJECT_COLLIDE_REGION_MANAGER_HPP
#define PROJECT_COLLIDE_REGION_MANAGER_HPP

#include <memory>
#include <Eigen/Dense>
#include "particle_info_msg.hpp"
#include "curve_sample/rotate_curve_operator.hpp"
namespace avoidoba {



class CollideRegionManager {
public:

    struct TransformPara {
        double offset_x = 0;
        double offset_y = 0;
        double cos_theta = 0;
        double sin_theta = 0;
    };
    struct RegionBorder {
        double min_x = 0;
        double max_x = 0;
        double min_y = 0;
        double max_y = 0;
    };

    CollideRegionManager(const double weight, const LineInterplation_Ptr &inter, bool have_slow = true) : weight_(
            weight), line_interplation_(inter), have_slow_region_(have_slow) {
    }

    void updateWeight(const double& weight){ weight_ = weight; }

    void computeTransformPara(const Eigen::Vector3d &sample_pose, const OriginCollideSize_Ptr &origin_size,
                              const RegionOffsetPara_Ptr &offset_para) {
        is_rotate_sample_ = false;
        rotate_poses_.clear();

        auto cos_theta = cos(sample_pose[2] + origin_size->rotate_angle);
        auto sin_theta = sin(sample_pose[2] + origin_size->rotate_angle);
        tf_para_.offset_x = -cos_theta * sample_pose[0] - sin_theta * sample_pose[1];//tf_para的含义是坐标变换参数,用于提升运算效率
        tf_para_.offset_y = sin_theta * sample_pose[0] - cos_theta * sample_pose[1];
        tf_para_.cos_theta = cos_theta;
        tf_para_.sin_theta = sin_theta;

        stop_border_.min_x = origin_size->back_length - offset_para->stop_length_backward_offset;//以下计算当前位姿的避障尺寸.如果障碍点落到该尺寸范围内,则触发避障
        stop_border_.max_x = origin_size->head_length + offset_para->stop_length_forward_offset;
        stop_border_.min_y = origin_size->right_width - offset_para->stop_width_offset;
        stop_border_.max_y = origin_size->left_width + offset_para->stop_width_offset;

        slow_border_.min_x = stop_border_.min_x;
        slow_border_.max_x = stop_border_.max_x;
        slow_border_.min_y = stop_border_.min_y - offset_para->slow_width_offset;
        slow_border_.max_y = stop_border_.max_y + offset_para->slow_width_offset;
    }

    void computeRotateTransformPara(const ParticleInfo &sample_pose, const OriginCollideSize_Ptr &origin_size,
                              const RegionOffsetPara_Ptr &offset_para) {
        is_rotate_sample_ = true;//旋转粒子标志位
        rotate::RotateInfo_Ptr info(new rotate::RotateInfo);
        info->start_angle = sample_pose.start_angle;
        info->end_angle = sample_pose.end_angle;
        info->center_info = sample_pose.pose.head<2>();
        rotate_poses_.clear();
        rotate::RotateCurveOperator::splitCurveByStep(info, rotate_poses_, 0.04);//4cm步长,如果为旋转,则要求旋转时,不对停止距离产生影响,因此,只能在旋转处生成一个粒子,通过该粒子,生成多个旋转粒子,然后判断其矩形区域是否有障碍
        if (rotate_poses_.empty()) {
            rotate_poses_.push_back(sample_pose.pose);//考虑到,有时候旋转的初始角度和终止角度相等,这就需要将当前旋转粒子的位姿push进去
        }
        for(auto& rotate:rotate_poses_){
            rotate[2] += origin_size->rotate_angle;//计算避障尺寸中心位姿
        }
//        auto cos_theta = cos(sample_pose[2]);
//        auto sin_theta = sin(sample_pose[2]);
//        tf_para.offset_x = -cos_theta * sample_pose[0] - sin_theta * sample_pose[1];
//        tf_para.offset_y = sin_theta * sample_pose[0] - cos_theta * sample_pose[1];
//        tf_para.cos_theta = cos_theta;
//        tf_para.sin_theta = sin_theta;

        stop_border_.min_x = origin_size->back_length - offset_para->stop_length_forward_offset;//以下计算当前位姿的避障尺寸.如果障碍点落到该尺寸范围内,则触发避障
        stop_border_.max_x = origin_size->head_length + offset_para->stop_length_forward_offset;
        stop_border_.min_y = origin_size->right_width - offset_para->stop_width_offset;
        stop_border_.max_y = origin_size->left_width + offset_para->stop_width_offset;

        slow_border_.min_x = stop_border_.min_x;
        slow_border_.max_x = stop_border_.max_x;
        slow_border_.min_y = stop_border_.min_y - offset_para->slow_width_offset;
        slow_border_.max_y = stop_border_.max_y + offset_para->slow_width_offset;
    }

    double computeCollideValue(const Eigen::Vector2d &oba) {
        if (!is_rotate_sample_) {//如果不是旋转粒子,则进行正常的判定
            return computeValue(oba, tf_para_);
        }else {
            double max_collide_value = 0;
            for (auto &pose:rotate_poses_) {//遍历所有旋转粒子,如果有任何一个旋转位姿粒子检测到碰撞,则认为触发了碰撞
                double collide_value = computeValue(oba, computeTfPara(pose));//这里目前没有对旋转粒子进行权重分配,后期可以进行权重分配,实现旋转时的缓停效果
                if (collide_value > max_collide_value) {
                    max_collide_value = collide_value;
                }
            }
            return max_collide_value;
        }
    }

private:
    double computeValue(const Eigen::Vector2d &oba,const TransformPara& tf) {
        double local_x = tf.cos_theta * oba[0] + tf.sin_theta * oba[1] + tf.offset_x;//将障碍点转换到当前粒子位姿的局部坐标系
        double local_y = -tf.sin_theta * oba[0] + tf.cos_theta * oba[1] + tf.offset_y;
        if (inRegion(local_x, local_y, stop_border_)) {//在局部坐标系下,只要判断该点是否在矩形区即可
            return weight_;
        }
        if (have_slow_region_) {
            if (inRegion(local_x, local_y, slow_border_)) {
                double delta_y = fabs(fabs(local_y) - stop_border_.max_y);//这里计算距离停止边界距离，考虑到尺寸对称性
                return line_interplation_->getDegreInterplation(delta_y) / collide_max_value_ * weight_;
            }
        }

        return collide_min_value_;
    }

    TransformPara computeTfPara(const Eigen::Vector3d& pose){
        TransformPara tf;
        auto cos_theta = cos(pose[2]);
        auto sin_theta = sin(pose[2]);
        tf.offset_x = -cos_theta * pose[0] - sin_theta * pose[1];
        tf.offset_y = sin_theta * pose[0] - cos_theta * pose[1];
        tf.cos_theta = cos_theta;
        tf.sin_theta = sin_theta;
        return tf;
    }

    bool inRegion(const double &local_x, const double &local_y, const RegionBorder &border) {
        if (local_x >= border.min_x && local_x <= border.max_x && local_y >= border.min_y && local_y <= border.max_y) {
            return true;
        }
        return false;
    }

    double weight_;//权重,与该粒子距离当前位姿成反比,距离越远,权重越低,表示越不容易碰撞
    LineInterplation_Ptr line_interplation_;//如果带有减速区,则根据该障碍点距离中心位姿远近差值求解
    TransformPara tf_para_;//获取当前粒子坐标变换参数,目的是将全局坐标系障碍点转换成当前粒子位姿坐标系下
    RegionBorder stop_border_;//默认粒子碰撞边界
    RegionBorder slow_border_;//如果带有减速区,则该值为减速区边界.正常情况下,减速区边界范围大于碰撞边界
    bool have_slow_region_;//是否包含减速区
    const double collide_max_value_ = 1000;//最大碰撞值
    const double collide_min_value_ = 0;//最小碰撞值
    bool is_rotate_sample_ = false;//是否为旋转粒子.对于旋转粒子来讲,需要判断整个旋转过程是否会发生碰撞
    std::vector<Eigen::Vector3d> rotate_poses_;//旋转粒子生成的用于判断是否发生碰撞的粒子
};

typedef std::shared_ptr<CollideRegionManager> CollideRegionManager_Ptr;
}


#endif //PROJECT_COLLIDE_REGION_MANAGER_HPP
