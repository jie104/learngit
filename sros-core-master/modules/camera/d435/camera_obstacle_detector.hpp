//
// Created by zjWu on 2021/11/16.
//

#ifndef SROS_CAMERA_OBSTACLE_DETECTOR_H
#define SROS_CAMERA_OBSTACLE_DETECTOR_H

#include <Eigen/Dense>
#include <memory>
#include "layer_coarse_map.hpp"
#include "ground_plane_map.hpp"
#include "../camera_device/depth_image_backup.hpp"
#include "core/logger.h"
#include "grid_plane_builder.hpp"

#include "../stereo_point.h"
#include "math.hpp"

namespace threedim_camera {
struct PlaneInfo {              /**定义描述平面上点的数据结构*/
    Eigen::Vector3f norm;
    Eigen::Vector3f mean;
    float weight;
    uint16_t coord_x;
    uint16_t coord_y;
};
typedef std::shared_ptr<PlaneInfo> PlaneInfo_Ptr;

struct PlaneModelInfo {             /**定义平面的数据结构*/
    std::vector<PlaneInfo_Ptr> infos;
    PlaneInfo info;
};

class CameraObstacleDetector {

 public:

    CameraObstacleDetector() {
        ground_map_.reset(new GroundPlaneMap(ground_resolution, map_length, map_width, dimension, 0, mid_height_));
        high_coarse_map_.reset(new LayerCoarseMap<Eigen::Vector3f>(high_map_resolution, map_length, map_width, mid_height_, max_height_));
        low_coarse_map_.reset(new LayerCoarseMap<Eigen::Vector3f>(high_map_resolution, map_length, map_width, min_height_, mid_height_));
    }

    virtual ~CameraObstacleDetector(){

    }

    template <class Quaternion>
        static void EulerToQuaternion(double yaw, double roll, double pitch, Quaternion &quaternion_) {
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);

        quaternion_.w() = cr * cp * cy + sr * sp * sy;
        quaternion_.x() = sr * cp * cy - cr * sp * sy;
        quaternion_.y() = cr * sp * cy + sr * cp * sy;
        quaternion_.z() = cr * cp * sy - sr * sp * cy;
    }

    template <class Quaternion>
        static void QuaternionToEuler(const Quaternion& quaternion, double& yaw, double& roll, double& pitch) {
        auto w = quaternion.w();
        auto x = quaternion.x();
        auto y = quaternion.y();
        auto z = quaternion.z();
        //    yaw = atan2(2*(w * z + x * y), 1 - 2 * (z * z + x * x));
        //    roll = asin(2*(w * x - y * z));
        //    pitch = atan2(w * y + z * x, 1 - 2 * (x * x + y * y));
        double sinr_cosp = 2 * (w * x + y * z);
        double cosr_cosp = 1 - 2 * (x * x + y * y);
        roll = std::atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        double sinp = 2 * (w * y - z * x);
        if (std::abs(sinp) >= 1)
            pitch = std::copysign(M_PI / 2, sinp);  // use 90 degrees if out of range
            else
                pitch = std::asin(sinp);

            // yaw (z-axis rotation)
            double siny_cosp = 2 * (w * z + x * y);
            double cosy_cosp = 1 - 2 * (y * y + z * z);
            yaw = std::atan2(siny_cosp, cosy_cosp);
    }

    bool calibrateCamera(StereoPoints& points, int length, int width, Eigen::Matrix3f& install_matrix,
                         float& install_height, float& plane_height) {
        serialization::PointImage point_image(length, width);
        LOG(INFO) << "calibrate!";
        computePointImage(points, length, width, point_image);
        LOG(INFO) << "calibrate!" << length << "," << width;
        std::vector<PlaneInfo_Ptr> planes;
        GridPlaneBuilder::computeLocalPlanes(point_image, planes);
        std::sort(planes.begin(), planes.end(),
                  [](const PlaneInfo_Ptr& a, const PlaneInfo_Ptr& b) { return a->weight > b->weight; });
        std::vector<PlaneModelInfo> first_plane_models, second_plane_models;
        combinePlane(planes, first_plane_models);
        combinePlane(first_plane_models, second_plane_models);
        std::sort(second_plane_models.begin(), second_plane_models.end(),
                  [](const PlaneModelInfo& a, const PlaneModelInfo& b) { return a.infos.size() > b.infos.size(); });
        Eigen::Quaternionf initial_rotate(install_matrix);
        if (second_plane_models.size()) {
            double yaw, roll, pitch;
            QuaternionToEuler(initial_rotate, yaw, roll, pitch);
            LOGGER(INFO, SROS) << "initial angle info:yaw," << yaw << ",roll," << roll << ",pitch," << pitch;
            int max_count = 3;
            int curr_try_times = 0;
            for (auto& plane_model : second_plane_models) {
                curr_try_times++;
                if (curr_try_times > max_count) {
                    LOGGER(INFO, SROS) << "try times is large! will not try again!";
                    break;
                }
                std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> norm_pairs;
                norm_pairs.emplace_back();
                auto curr_norm = initial_rotate * plane_model.info.norm;
                norm_pairs.back().first = plane_model.info.norm;
                float z_direction = -1.0;
                if (plane_height < -0.1f) {
                    LOGGER(INFO, SROS) << "will use invert calibration method! plane height is:" << plane_height;
                    z_direction = 1.0f;
                }

                norm_pairs.back().second = Eigen::Vector3f(0, 0, z_direction);
                auto dot_value = curr_norm.dot(Eigen::Vector3f(0, 0, z_direction));
                LOGGER(INFO, SROS) << "plane_info:" << curr_norm[0] << "," << curr_norm[1] << "," << curr_norm[2] << ","
                << "origin_norm"
                << "," << plane_model.info.norm[0] << "," << plane_model.info.norm[1] << ","
                << plane_model.info.norm[2] << ",dot," << dot_value;
                if (dot_value >= 0.95) {
                    if (computeRotate(norm_pairs, initial_rotate)) {
                        QuaternionToEuler(initial_rotate, yaw, roll, pitch);
                        LOGGER(INFO, SROS) << "match angle info:yaw," << yaw << ",roll," << roll << ",pitch," << pitch;
                    } else {
                        LOGGER(INFO, SROS) << "err to compute rotate!";
                    }
                    auto mean = initial_rotate * plane_model.info.mean;
                    auto compute_install_height = -mean[2] - plane_height;
                    if (plane_height < -0.1f) {
                        compute_install_height -= 0.01f;//做个补偿。
                    }
                    auto delta_height_err = install_height - compute_install_height;
                    LOGGER(INFO, SROS) << "delta height err is:" << delta_height_err
                    << ",install height:" << install_height << ",height," << mean[2]
                    << ",plane_height," << plane_height
                    << ",compute height:" << compute_install_height;

                    if (fabs(delta_height_err) < 0.1) {
                        install_matrix = initial_rotate.toRotationMatrix();
                        install_height_ = compute_install_height;
                        install_height = install_height_;
                        rotate_ = initial_rotate;
                        computeQuatWithoutYaw(rotate_, rotate_);
                        return true;
                    } else {
                        LOGGER(INFO, SROS) << "install height is wrong! please configure it!" << install_height;
                    }
                }
            }
            LOGGER(INFO, SROS) << "cannot calibrate the D435! please check the initial para!";
            return false;
        }
        LOGGER(INFO, SROS) << "cannot get plane! please check the environment if have ground plane!";
        return false;
    }

    bool getEnoughtPlane(StereoPoints& points, int length, int width){
        serialization::PointImage point_image(length, width);
        LOG(INFO) << "calibrate!";
        computePointImage(points, length, width, point_image);
        LOG(INFO) << "calibrate!" << length << "," << width;
        std::vector<PlaneInfo_Ptr> planes;
        GridPlaneBuilder::computeLocalPlanes(point_image, planes);
        std::sort(planes.begin(), planes.end(),
                  [](const PlaneInfo_Ptr& a, const PlaneInfo_Ptr& b) { return a->weight > b->weight; });
        std::vector<PlaneModelInfo> first_plane_models, second_plane_models;
        combinePlane(planes, first_plane_models);
        combinePlane(first_plane_models, second_plane_models);
        return !second_plane_models.empty();
    }



    void setHeightThresh(float min_height, float max_height) {
        min_height_ = min_height, max_height_ = max_height;
        high_coarse_map_->setHeightRange(min_height, max_height);
        low_coarse_map_->setHeightRange(min_height, max_height);
    }

    void setMaxDistThresh(float max_dist) {
        max_detect_dist_ = max_dist;
        if (max_detect_dist_ > map_length) {
            LOGGER(INFO, SROS) << "max_detect_dist_ is wrong! will use max dist:" << map_length;
            max_detect_dist_ = map_length;
        }
    }

    void setCalibrationInfo(const Eigen::Matrix3f& install_matrix,const float& install_height) {
        rotate_ = Eigen::Quaternionf(install_matrix);
        computeQuatWithoutYaw(rotate_, rotate_);//需要去除yaw角，否则后面做旋转的时候，容易多加一层旋转
        install_height_ = install_height;
    }

    template <class StereoPoints>
        void computeObstaclePoints(StereoPoints& points, int length, int width, StereoPoints& oba_points) {
        high_coarse_map_->clear();
        low_coarse_map_->clear();
        ground_map_->clear();
        buildLocalMap(points, length, width);
        int length_int = high_coarse_map_->length();
        int width_int = high_coarse_map_->width();
        int low_size = 0;
        int high_size = 0;
        for (int l = 0; l < width_int; ++l) {
            for (int i = 0; i < length_int; ++i) {
                auto &high_cell = high_coarse_map_->index(i, l);
                if(!high_cell.empty()){
                    int sum_size = high_cell.size();
                    if (sum_size >= 3) {
                        for (auto &high : high_cell) {
                            oba_points.emplace_back();
                            oba_points.back().x = high[0];
                            oba_points.back().y = high[1];
                            oba_points.back().z = high[2];
                        }
                        continue;
                    }
                }
                auto &low_cell = low_coarse_map_->index(i, l);
                if (low_cell.size() >= 3) {
                    std::vector<Eigen::Vector3f> real_points;
                    for (auto &low : low_cell) {
                        auto coord_x = ground_map_->coordX(low[0]);
                        auto coord_y = ground_map_->coordY(low[1]);
                        uint16_t state = 0;
                        for (int j = -1; j < 2; ++j) {
                            for (int k = -1; k < 2; ++k) {
                                if (ground_map_->inGrid(coord_x + j, coord_y + k)) {
                                    state = state | ground_map_->index(coord_x + j, coord_y + k);
                                }
                            }
                        }
                        auto incre = ground_map_->heightStepCount(state);
                        int ratio = floorf(fabs(low[2]) / mid_height_ * dimension);
                        ratio = ratio < (dimension - 1) ? ratio : dimension - 1;
                        if (incre >= ratio) {
                            real_points.push_back(low);
                        }
                    }
                    if (real_points.size() >= 3) {
                        for(auto& point:real_points){
                            oba_points.emplace_back();
                            oba_points.back().x = point[0];
                            oba_points.back().y = point[1];
                            oba_points.back().z = point[2];
                        }
                    }
                }
            }
        }
    }

    void buildLocalMap(StereoPoints& stereo_points, int length, int width){
        auto vertices = stereo_points.points;
        Eigen::Quaternionf initial_quat_offset;
        EulerToQuaternion(0, 0, M_PI / 180.0, initial_quat_offset);//将D435故意向下倾斜，来使远处的特征高度阈值变大。
        auto real_rotate = initial_quat_offset * rotate_;
        for (int j = 0; j < width; ++j) {
            for (int i = 0; i < length; ++i) {
                auto vertice = vertices[j * length + i];
                if (!std::isfinite(vertice.x) || !std::isfinite(vertice.y) || !std::isfinite(vertice.z)) {
                } else if (math::isZero(vertice.x) && math::isZero(vertice.y) && math::isZero(vertice.z)) {
                } else {
                    auto point = Eigen::Vector3f(vertice.z, -vertice.x, -vertice.y);
                    Eigen::Vector3f world_point = real_rotate * point;
                    world_point[2] += install_height_;
                    if (world_point[0] < max_detect_dist_) {//只需要限定水平距离即可
                        if (high_coarse_map_->inMap(world_point[0], world_point[1])) {//先判断是否在范围内。
                            if (high_coarse_map_->inHeight(world_point[2])) {
                                high_coarse_map_->addPoint(world_point[0], world_point[1], world_point);
                            }else{
                                if (low_coarse_map_->inHeight(world_point[2])) {
                                    low_coarse_map_->addPoint(world_point[0], world_point[1], world_point);
                                }
                                if (ground_map_->inHeight(world_point[2])) {
                                    ground_map_->addPoint(world_point[0], world_point[1], world_point);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    void computePointImage(StereoPoints& stereoPoints, int length, int width, serialization::PointImage& image){
        auto vertices = stereoPoints.points;
        for (int i = 0; i < width; ++i) {
            for (int j = 0; j < length; ++j) {
                auto vertice = vertices[i * length + j];
                auto &point = image(j, i);
                point.coord_x = j, point.coord_y = i;
                if (vertice.z < 0.2f) {
                    point.valid = false;
                }/* else if (math::isZero(vertice.x) && math::isZero(vertice.y) && math::isZero(vertice.z)) {
                    point.valid = false;
                }*/
                else {
                    point.point = Eigen::Vector3f(vertice.z, -vertice.x, -vertice.y);
                    point.valid = true;
                }
            }
        }
    }


 private:
    void computeQuatWithoutYaw(const Eigen::Quaternionf& input_rotate,Eigen::Quaternionf& output_rotate){
        double yaw,roll,pitch;
        QuaternionToEuler(input_rotate, yaw, roll, pitch);
        EulerToQuaternion(0.0, roll, pitch, output_rotate);
    }

    bool computeRotate(std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>& norm_pairs, Eigen::Quaternionf& quat) {
        //计算初始值
        if (!norm_pairs.empty()) {
            auto initial_quat = Eigen::Quaternionf::FromTwoVectors(
                norm_pairs[0].first, norm_pairs[0].second);  //使用第一个向量计算到目标旋转向量
                quat = initial_quat;
                return true;
        }
        return false;
    }

    void updatePlane(const PlaneInfo& cand_plane, PlaneInfo& model_info) {
        model_info.mean = (model_info.mean * model_info.weight + cand_plane.mean * cand_plane.weight) /
            (model_info.weight + cand_plane.weight);
        model_info.norm = (model_info.norm * model_info.weight + cand_plane.norm * cand_plane.weight);
        model_info.weight = model_info.norm.norm();
        model_info.norm.normalize();
    }

    void combinePlane(std::vector<PlaneInfo_Ptr>& cand_planes, std::vector<PlaneModelInfo>& output_planes) {
        for (auto& plane_ptr : cand_planes) {
            auto& plane = *plane_ptr;
            bool find_model = false;
            for (auto& model : output_planes) {
                auto& model_info = model.info;
                auto dot_norm = model_info.norm.dot(plane.norm);
                auto delta_vector = plane.mean - model_info.mean;
                auto project_dist = fabs(model_info.norm.dot(delta_vector));  //投影高度
                if (dot_norm > 0.98 && project_dist < 0.1) {
                    find_model = true;
                    updatePlane(plane, model_info);
                    model.infos.push_back(plane_ptr);
                    break;
                }
            }
            if (!find_model) {
                output_planes.emplace_back();
                output_planes.back().info = plane;
                output_planes.back().infos.push_back(plane_ptr);
            }
        }
    }

    void combinePlane(std::vector<PlaneModelInfo>& cand_planes, std::vector<PlaneModelInfo>& output_planes) {
        for (auto& first_model : cand_planes) {
            bool find_model = false;
            auto& plane = first_model.info;
            for (auto& model : output_planes) {
                auto& model_info = model.info;
                auto dot_norm = model_info.norm.dot(plane.norm);
                auto delta_vector = plane.mean - model_info.mean;
                auto project_dist = fabs(model_info.norm.dot(delta_vector));  //投影高度
                if (dot_norm > 0.98 && project_dist < 0.1) {
                    find_model = true;
                    updatePlane(plane, model_info);
                    for (auto& info : first_model.infos) {
                        model.infos.push_back(info);
                    }
                    break;
                }
            }
            if (!find_model) {
                output_planes.emplace_back();
                output_planes.back().info = plane;
                output_planes.back().infos = first_model.infos;
            }
        }
    }

    float install_height_ = 0.0f;
    float min_height_ = 0.03f;
    float max_height_ = 1.0f;
    float max_detect_dist_ = 2.0f;
    const float mid_height_ = 0.06f;//区分低矮物体和高处物体,一般不需要设置，程序默认
    const float map_length = 2.5f;//生成平面长度方向（相机前向X值）
    const float map_width = 3.0f;//生成平面水平方向（相机Y值）
    const float ground_resolution = 0.005f;//地面网格分辨率
    const float high_map_resolution = 0.10f;//在高度方向上生成地图分辨率
    const int dimension = 8;//低矮物体检测高度维度
    const int obs_point_min_size = 3;//d435避障点滤波大小

    Eigen::Quaternionf rotate_;
    std::shared_ptr<GroundPlaneMap> ground_map_;
    std::shared_ptr<LayerCoarseMap<Eigen::Vector3f>> high_coarse_map_;
    std::shared_ptr<LayerCoarseMap<Eigen::Vector3f>> low_coarse_map_;
};

}  // namespace threedim_camera


#endif  // SROS_CAMERA_OBSTACLE_DETECTOR_H
