//
// Created by lfc on 18-12-11.
//

#ifndef PROJECT_AVDOBA_PROCESSOR_HPP
#define PROJECT_AVDOBA_PROCESSOR_HPP

#include <glog/logging.h>

#include "avdoba_module_para.hpp"
#include "curve_sample/curve_sample_factory.h"
#include "avdoba_sample_manager.hpp"
#include "obstacle_points_manager.hpp"

namespace avoidoba {

// 障碍的方位
enum ObstacleDirection {
    OBSTACLE_DIRECTION_UNKNOWN = 0,  // 如：TIM320报的故障只知道有障碍，但不知道障碍的方向

    OBSTACLE_DIRECTION_FORWARD = 1,  // 前方
    OBSTACLE_DIRECTION_BACKWARD,     // 后方
    OBSTACLE_DIRECTION_LEFT,
    OBSTACLE_DIRECTION_RIGHT,
    OBSTACLE_DIRECTION_LEFT_FORWARD,
    OBSTACLE_DIRECTION_RIGHT_FORWARD,
    OBSTACLE_DIRECTION_LEFT_BACKWARD,
    OBSTACLE_DIRECTION_RIGHT_BACKWARD,
};

static std::vector<double> splitStrsToDoubles(const std::string &s, const char seperator) {
    std::vector<double> result;
    std::string::size_type i = 0;
    std::string::size_type j = 0;
    char c = s[0];
    if (c == '"')  // chip 设置的话一般带“”
        j = 1;
    // LOG(INFO) << "the s is:" << s;
    while (i < s.size()) {
        if (s[i] == seperator || i == s.size() - 1) {
            if (j != i || i == s.size() - 1) {
                auto len = (s[i] != seperator && i == s.size() - 1) ? (s.size() - j) : i - j;
                std::string item_s = s.substr(j, len);
                if (item_s == "\"") break;
                try {
                    double item = stod(item_s);
                    result.push_back(item);
                } catch (std::exception &e) {
                    LOG(ERROR) << "throw error:" << e.what() << item_s;
                    result.clear();
                    break;
                }
            }
            j = i + 1;
        }
        i++;
    }

    return result;
}

class AvoidobaProcessor {
public:
    AvoidobaProcessor(AvdobaModulePara_Ptr &module_para): para_(module_para) {
        updatePara();//根据外部传递过来的参数信息创建算法处理模块
    }


    void updateObstaclePoints(ObstaclePoints_Ptr& oba_points) {
        oba_manager_.updateObstaclePoints(oba_points);
    }

    void updatePathInfo(const sros::core::NavigationPath_vector& paths) {
        curve_samples_.clear();
        last_all_paths_.clear();
        last_checkpoint_ = 0;
        last_all_paths_ = paths;
    }

     void updateCheckPoint(const int no){
         auto &paths = last_all_paths_;
         int curr_check_point = no;
         if (curr_check_point < 0) {
             LOG(INFO) << "no is wrong!" << no;
             curr_check_point = last_all_paths_.size();
         }
         if (paths.size() < no) {
             LOG(INFO) << "curr no is too large!" << no << ", will use paths size!" << paths.size();
             curr_check_point = paths.size();
         }
         if (curve_samples_.size() > curr_check_point) {
             LOG(INFO) << "find one wrong checkpoint no! will clear sample! size:"<< curve_samples_.size() << "," << curr_check_point;
             curve_samples_.clear();
             last_checkpoint_ = 0;
         }
         int curr_end_index = last_checkpoint_;
         for (int i = last_checkpoint_; i < curr_check_point; ++i) {
             auto &path = paths[i];
             double step = para_->sample_step;
             if (path.direction_ == 2) {
                 step = para_->backward_step;
             }
             curr_end_index = i;
             curve_samples_.push_back(sample::CurveSampleFactory::getCurveSample(path, step,path.direction_));
         }
         if (last_checkpoint_ == curr_check_point) {
             return;
         }else{
             last_checkpoint_ = curr_check_point;
             if (!curve_samples_.empty()) {
                 auto &path = paths[curr_end_index];
                 station_info_.pose.head<2>() = curve_samples_.back()->getEndPoint();
                 station_info_.pose[2] = path.e_facing_;
                 LOG(INFO) << "direction:" << path.e_facing_ << "," << path.type_;

                 if(path.direction_ == 2) {
                     station_info_.move_state = BACKWARD_MOVE;
                 }else if(path.direction_ == 1){
                     station_info_.move_state = FORWARD_MOVE;
                 }else if(path.direction_ == 0x21) {
                     station_info_.move_state = NO_MOVE_STATE;
                 }else{
                     station_info_.move_state = FORWARD_MOVE;
                 }
             }
         }
    }

    std::pair<ObstacleDirection, std::string> outputObstacleInfo(const Eigen::Vector3d &curr_pose,const OriginCollideSize_Ptr &origin_size,const AvdobaPoseInfo &avdoba_poses_info){
        Eigen::Affine2d curr_tf(Eigen::Translation2d(curr_pose.head<2>()) * Eigen::Rotation2Dd(curr_pose[2]));
        std::stringstream output_info;
        auto obstacle_direction = OBSTACLE_DIRECTION_UNKNOWN;
        auto &result_info = avdoba_poses_info.result_info;
        if (result_info.is_region_oba) {
            output_info << result_info.oba_name<<" detect an obstacle! which is a region sensor";
        }else{
            if (result_info.collide_point.norm() == 0.0) {
                return std::make_pair(OBSTACLE_DIRECTION_UNKNOWN, output_info.str());
            }
            auto delta_point = curr_tf.inverse() * result_info.collide_point;
            const double &head_length = origin_size->head_length;
            const double &back_length = origin_size->back_length;
            const double &left_width = origin_size->left_width;
            const double &right_width = origin_size->right_width;
            const double &coord_x = delta_point[0];
            const double &coord_y = delta_point[1];
            output_info << result_info.oba_name<<" detect an obstacle! "<<"direction is:";
            if (coord_x < head_length && coord_x > back_length) {
                if (coord_y < 0) {
                    output_info << "on the right!";
                    obstacle_direction = OBSTACLE_DIRECTION_RIGHT;
                }else{
                    output_info << "on the left!";
                    obstacle_direction = OBSTACLE_DIRECTION_LEFT;
                }
            } else if (coord_y < left_width && coord_y > right_width) {
                if (coord_x > 0.0) {
                    output_info << "on the forward!";
                    obstacle_direction = OBSTACLE_DIRECTION_FORWARD;
                }else{
                    output_info << "on the backward!";
                    obstacle_direction = OBSTACLE_DIRECTION_BACKWARD;
                }
            } else if (coord_x > 0.0) {
                if (coord_y < 0) {
                    output_info << "on the right forward!";
                    obstacle_direction = OBSTACLE_DIRECTION_RIGHT_FORWARD;
                } else {
                    output_info << "on the left forward!";
                    obstacle_direction = OBSTACLE_DIRECTION_LEFT_FORWARD;
                }
            }else{
                if (coord_y < 0) {
                    output_info << "on the right backward!";
                    obstacle_direction = OBSTACLE_DIRECTION_RIGHT_BACKWARD;
                } else {
                    output_info << "on the left backward!";
                    obstacle_direction = OBSTACLE_DIRECTION_LEFT_BACKWARD;
                }
            }
            output_info << "robot pose is:" << curr_pose[0] << "," << curr_pose[1] << "," << curr_pose[2]
                        << ", coord is:" << result_info.collide_point[0] << "," << result_info.collide_point[1]
                        << std::endl;
            output_info << " coord to robot center is:" << coord_x << "," << coord_y
                        << ",angle is:" << atan2(coord_y, coord_x) * 180.0 / M_PI << ",dist is:" << delta_point.norm();
        }
//        LOG(INFO) << output_info.str();
        return std::make_pair(obstacle_direction, output_info.str());
    }

    void updateCurrPath(const sros::core::NavigationPath<double>& path,int path_no){
        int index_no = path_no - 1;
        if (curve_samples_.size() > index_no && index_no >= 0) {
            double step = para_->sample_step;
            if (path.direction_ == 2) {
                step = para_->backward_step;
            }
            curve_samples_[index_no] = sample::CurveSampleFactory::getCurveSample(path, step,path.direction_);
        }else{
            LOG(INFO) << "path no is wrong!" << path_no;
        }
    }

    int getAvdObaValue(uint16_t path_no,Eigen::Vector3d &curr_pose,OriginCollideSize_Ptr origin_size,
                       AvdobaPoseInfo &avdoba_poses_info) {
        if (path_no >= curve_samples_.size()) {
            LOG(INFO) << "path no is err!";
            return para_->max_stop_collide_value;
        }
        auto curve_size = curve_samples_.size();

        auto &stop_particles = avdoba_poses_info.collide_poses;
        auto &slow_particles = avdoba_poses_info.slow_poses;
        CollideResultInfo &collide_result_info = avdoba_poses_info.result_info;
        avdoba_poses_info.station_pose = station_info_;
        int curr_real_size = 0;
        int curr_no = path_no;

        pushBackCurrPose(curve_samples_[path_no], curr_pose, stop_particles);//将当前粒子存储到停止粒子内,目的是给系统一个初始值,也是为了保证当前位置避障依然有效

        if (!transverseCurves(path_no, para_->max_collide_region_count, stop_particles, curr_no)) {
            slow_particles.push_back(stop_particles.back());
            transverseCurves(curr_no, para_->max_slow_region_count, slow_particles, curr_no);
        }else{
            //处理站点信息
        }
//        std::vector<Eigen::Vector2d> obas;
        if (stop_particles.size() >= 2) {
            if ((stop_particles[0].pose - stop_particles[1].pose).head<2>().norm() > para_->deviation_dist_thresh)
            {

                LOG(INFO) << "curr pose no:" << path_no << ",path type:" << curve_samples_[path_no]->getType() << ",curr path end point:"
                          << curve_samples_[path_no]->getEndPoint()[0] << ","
                          << curve_samples_[path_no]->getEndPoint()[1];
            }
        }
        bool use_ifm_state = curve_samples_[path_no]->getType() != sample::TYPE_ROTATE_CURVE;//需要去除参数
        avdoba_sample_manager_->updateSamplePoses(avdoba_poses_info, origin_size, offset_para_,asymmetric_para_);
        getObstaclePoints(avdoba_poses_info,use_ifm_state,true);
        avdoba_sample_manager_->computeCollideValue(avdoba_poses_info.oba_points, collide_result_info);
        CollideResultInfo region_collide_info;
        region_collide_info.is_region_oba = true;
        int region_collide_value = getRegionCollideValue(region_collide_info);
        if (collide_result_info.collide_value < region_collide_value) {
            collide_result_info.collide_value = region_collide_value;
            collide_result_info.oba_name = region_collide_info.oba_name;
            collide_result_info.is_region_oba = region_collide_info.is_region_oba;
        }
        if (collide_result_info.collide_value >= para_->min_stop_collide_value) {//只有在停止状态下，才需要使用stop等待时间
            last_stop_collide_info_ = collide_result_info;
            last_stop_stamp_ = sros::core::util::get_time_in_us();
        }else{
            auto curr_time = sros::core::util::get_time_in_us();
            auto delta_time = curr_time - last_stop_stamp_;
            if (fabs(delta_time) < para_->delta_stop_time_stamp_in_us) {//如果时间差小于上次stop时间
                collide_result_info = last_stop_collide_info_;
            }
        }
        return collide_result_info.collide_value;
    }

    void updateSynRotateState(bool curr_syn_rotate_state){
        if (avdoba_sample_manager_) {
            avdoba_sample_manager_->updateSynRotateState(curr_syn_rotate_state);
        }
    }

    void getObstaclePoints(AvdobaPoseInfo &avdoba_poses_info,bool use_ifm_points = true,bool log_output = false) {
        oba_manager_.getObstaclePoints(avdoba_poses_info.oba_points, use_ifm_points, log_output);
    }

    void getObstaclePoints(std::map<std::string,std::vector<Eigen::Vector2d>> &oba_points,bool use_ifm_points = true,bool log_output = false) {
        oba_manager_.getObstaclePoints(oba_points, use_ifm_points, log_output);
    }

    void getObstaclePoints(std::vector<Eigen::Vector2d> &oba_points,bool use_ifm_points = true) {
        oba_manager_.getObstaclePoints(oba_points,use_ifm_points);
    }

    int getRegionCollideValue(CollideResultInfo &collide_result_info){
        std::map<std::string, ObstaclePoints::ObaState> oba_states;
        oba_manager_.getObstacleRegions(oba_states);
        int max_state = 0;
        int max_stop_value = para_->max_stop_collide_value;
        int min_stop_value = para_->min_stop_collide_value;
        for(auto&state_iter:oba_states){
            auto &state = state_iter.second;
            int curr_state = 0;
            switch(state){
                case ObstaclePoints::STATE_OBA_STOP_0:
                    curr_state = para_->max_stop_collide_value;
                    break;
                case ObstaclePoints::STATE_OBA_STOP_1:
                    curr_state = min_stop_value + (max_stop_value-min_stop_value)*0.75;
                    break;
                case ObstaclePoints::STATE_OBA_STOP_2:
                    curr_state = min_stop_value + (max_stop_value-min_stop_value)*0.5;
                    break;
                case ObstaclePoints::STATE_OBA_STOP_3:
                    curr_state = min_stop_value + (max_stop_value-min_stop_value)*0.25;
                    break;
                case ObstaclePoints::STATE_OBA_STOP_4:
                    curr_state = min_stop_value;
                    break;
                case ObstaclePoints::STATE_OBA_SLOW:
                    curr_state = para_->max_slow_collide_value;
                    break;
                case ObstaclePoints::STATE_OBA_FREE:
                    curr_state = 0;
                    break;
                default:
                    break;
            }
            if (max_state < curr_state) {
                max_state = curr_state;
                collide_result_info.oba_name = state_iter.first;
            }
        }
        return max_state;
    }

    bool transverseCurves(int beg_no, int particle_size, std::vector<avoidoba::ParticleInfo> &particles, int &curr_no){
        auto curve_size = curve_samples_.size();
        int curr_real_size = 0;
        for (uint16_t i = beg_no; i < curve_size; ++i) {
            curve_samples_[i]->getSomeSamples(particles, particle_size, curr_real_size);
            if (particles.size() >= particle_size) {
                curr_no = i;
                break;
            }
        }

        auto& curr_stop_pose = particles.back().pose;
//        LOG(INFO) << "stop particles:" << curr_stop_pose[0] << "," << curr_stop_pose[1] << "," << curr_stop_pose[2];
        if (particles.size() < particle_size) {
            return true;//遍历到了所有采样曲线
        }else {
            return false;//尚未遍历到所有采样曲线
        }
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

    void updateOffsetPara(double slow_width_offset,double stop_width_offset,double stop_length_offset) {
        para_->slow_width_offset = slow_width_offset;
        para_->stop_width_offset = stop_width_offset;
        para_->stop_length_forward_offset = stop_length_offset;

        offset_para_->slow_width_offset = slow_width_offset;
        offset_para_->stop_width_offset = stop_width_offset;
        offset_para_->stop_length_forward_offset = stop_length_offset;
        avdoba_sample_manager_->updateSlowOffsetPara();
    }
    
    std::string getPathTypeByNumber(int path_no,const Eigen::Vector3d& curr_pose) {
        int path_size = curve_samples_.size();
        if (path_no < path_size) {
            auto curve = curve_samples_[path_no];
            std::string path_type_str;
            std::string direction_str = " FORWARD";
            if (!curve->isForward()) {
                direction_str = " BACKWARD";
            }
            auto path_type = curve->getType();
            switch (path_type) {
                case sample::TYPE_LINE_CURVE:
                    path_type_str = "LINE";
                    break;
                case sample::TYPE_ROTATE_CURVE: {
                    double delta_pose = curve->getStartPose()[2] - curr_pose[2];
                    delta_pose = rotate::RotateCurveOperator::normalizeAngle(delta_pose);
                    if (delta_pose >= 0) {
                        path_type_str = "ROTATE LEFT";
                    } else {
                        path_type_str = "ROTATE RIGHT";
                    }

                    direction_str = "";
                } break;
                case sample::TYPE_BEZIER_CURVE:
                case sample::TYPE_CIRCLE_CURVE: {
                    const auto start_pose = curve->getStartPose();
                    const auto end_point = curve->getEndPoint();
                    Eigen::Vector2d start_direction(cos(start_pose[2]), sin(start_pose[2]));
                    Eigen::Vector2d delta_vector = end_point - start_pose.head<2>();
                    double cross_value = start_direction[0] * delta_vector[1] - start_direction[1] * delta_vector[0];
                    if (cross_value < 0) {
                        path_type_str = "TURN RIGHT";
                    } else {
                        path_type_str = "TURN LEFT";
                    }
                } break;
            }
            return path_type_str + direction_str;
        }
        return "NULL";
    }
    void updateAsymetricOffsetPara(double left, double right, double head, double back) {

        asymmetric_para_->left_offset = left;
        asymmetric_para_->right_offset = right;
        asymmetric_para_->forward_offset = head;
        asymmetric_para_->backward_offset = back;
    }

 private:
    void pushBackCurrPose(const sample::BaseCurveSample_Ptr& curr_curve,const Eigen::Vector3d& curr_pose,std::vector<avoidoba::ParticleInfo>& particles){
        avoidoba::ParticleInfo curr_particle;
        curr_particle.pose = curr_pose;
        if (!curr_curve->isForward()) {//判断输入粒子的状态,分为前进,和后退;旋转粒子可以当做前进粒子处理
            curr_particle.move_state = BACKWARD_MOVE;
        }else {
            curr_particle.move_state = FORWARD_MOVE;
        }
        particles.push_back(curr_particle);
    }

    AvdobaModulePara_Ptr para_;//配置参数
    std::vector<sample::BaseCurveSample_Ptr> curve_samples_;//当前规划的路径采样信息
    AvdobaSampleManager_Ptr avdoba_sample_manager_;//采样避障管理器,通过curve_sample计算出要采样的粒子,输入到该管理器中,计算碰撞值,判断是否发生碰撞
    avoidoba::ParticleInfo station_info_;//站点信息
    RegionOffsetPara_Ptr offset_para_;//碰撞尺寸offset
    RegionAsymmetricOffsetPara_Ptr asymmetric_para_;//碰撞尺寸offset
    ObstaclePointsManager oba_manager_;//障碍点管理器
    int64_t last_stop_stamp_ = 0;
    CollideResultInfo last_stop_collide_info_;
    RegionAsymmetricOffsetPara_Ptr last_asymmetric_para_;//前一个碰撞尺寸offset

    sros::core::NavigationPath_vector last_all_paths_;
    int last_checkpoint_;
};

}


#endif //PROJECT_AVDOBA_PROCESSOR_HPP
