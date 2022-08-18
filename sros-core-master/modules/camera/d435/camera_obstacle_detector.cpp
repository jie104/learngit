//
// Created by ehl on 2022/07/07.
//
#include "camera_obstacle_detector.h"
#include "math.hpp"

using namespace threedim_camera;

CameraObstacleDetector::CameraObstacleDetector() {
    ground_map_.reset(new GroundPlaneMap(ground_resolution, map_length, map_width, dimension, 0, mid_height_));
    high_coarse_map_.reset(new LayerCoarseMap<Eigen::Vector3f>(high_map_resolution, map_length, map_width, mid_height_, max_height_));
    low_coarse_map_.reset(new LayerCoarseMap<Eigen::Vector3f>(high_map_resolution, map_length, map_width, min_height_, mid_height_));
}

CameraObstacleDetector::~CameraObstacleDetector(){

}

bool CameraObstacleDetector::calibrateCamera(StereoPoints& points, int length, int width, Eigen::Matrix3f& install_matrix,
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

bool CameraObstacleDetector::getEnoughtPlane(StereoPoints& points, int length, int width){
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



void CameraObstacleDetector::setHeightThresh(float min_height, float max_height) {
    min_height_ = min_height, max_height_ = max_height;
    high_coarse_map_->setHeightRange(min_height, max_height);
    low_coarse_map_->setHeightRange(min_height, max_height);
}

void CameraObstacleDetector::setMaxDistThresh(float max_dist) {
    max_detect_dist_ = max_dist;
    if (max_detect_dist_ > map_length) {
        LOGGER(INFO, SROS) << "max_detect_dist_ is wrong! will use max dist:" << map_length;
        max_detect_dist_ = map_length;
    }
}

void CameraObstacleDetector::setCalibrationInfo(const Eigen::Matrix3f& install_matrix,const float& install_height) {
    rotate_ = Eigen::Quaternionf(install_matrix);
    computeQuatWithoutYaw(rotate_, rotate_);//需要去除yaw角，否则后面做旋转的时候，容易多加一层旋转
    install_height_ = install_height;
}

void CameraObstacleDetector::buildLocalMap(StereoPoints& stereo_points, int length, int width){
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

void CameraObstacleDetector::computePointImage(StereoPoints& stereoPoints, int length, int width, serialization::PointImage& image){
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

void CameraObstacleDetector::computeQuatWithoutYaw(const Eigen::Quaternionf& input_rotate,Eigen::Quaternionf& output_rotate){
    double yaw,roll,pitch;
    QuaternionToEuler(input_rotate, yaw, roll, pitch);
    EulerToQuaternion(0.0, roll, pitch, output_rotate);
}

bool CameraObstacleDetector::computeRotate(std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>& norm_pairs, Eigen::Quaternionf& quat) {
    //计算初始值
    if (!norm_pairs.empty()) {
        auto initial_quat = Eigen::Quaternionf::FromTwoVectors(
            norm_pairs[0].first, norm_pairs[0].second);  //使用第一个向量计算到目标旋转向量
            quat = initial_quat;
            return true;
    }
    return false;
}

void CameraObstacleDetector::updatePlane(const PlaneInfo& cand_plane, PlaneInfo& model_info) {
    model_info.mean = (model_info.mean * model_info.weight + cand_plane.mean * cand_plane.weight) /
        (model_info.weight + cand_plane.weight);
    model_info.norm = (model_info.norm * model_info.weight + cand_plane.norm * cand_plane.weight);
    model_info.weight = model_info.norm.norm();
    model_info.norm.normalize();
}

void CameraObstacleDetector::combinePlane(std::vector<PlaneInfo_Ptr>& cand_planes, std::vector<PlaneModelInfo>& output_planes) {
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

void CameraObstacleDetector::combinePlane(std::vector<PlaneModelInfo>& cand_planes, std::vector<PlaneModelInfo>& output_planes) {
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
