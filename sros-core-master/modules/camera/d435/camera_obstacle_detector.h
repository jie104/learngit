//
// Created by ehl on 2022/07/07.
//

#ifndef SROS_CAMERA_OBSTACLE_DETECTOR_H
#define SROS_CAMERA_OBSTACLE_DETECTOR_H

#include "layer_coarse_map.hpp"
#include "ground_plane_map.hpp"
#include "../camera_device/depth_image_backup.hpp"
#include "core/logger.h"
#include "grid_plane_builder.hpp"

#include "../stereo_point.h"

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

    CameraObstacleDetector();

    virtual ~CameraObstacleDetector();

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
                         float& install_height, float& plane_height);

    bool getEnoughtPlane(StereoPoints& points, int length, int width);



    void setHeightThresh(float min_height, float max_height);

    void setMaxDistThresh(float max_dist);

    void setCalibrationInfo(const Eigen::Matrix3f& install_matrix,const float& install_height);

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

    void buildLocalMap(StereoPoints& stereo_points, int length, int width);

    void computePointImage(StereoPoints& stereoPoints, int length, int width, serialization::PointImage& image);


 private:
    void computeQuatWithoutYaw(const Eigen::Quaternionf& input_rotate,Eigen::Quaternionf& output_rotate);

    bool computeRotate(std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>& norm_pairs, Eigen::Quaternionf& quat);

    void updatePlane(const PlaneInfo& cand_plane, PlaneInfo& model_info);

    void combinePlane(std::vector<PlaneInfo_Ptr>& cand_planes, std::vector<PlaneModelInfo>& output_planes);

    void combinePlane(std::vector<PlaneModelInfo>& cand_planes, std::vector<PlaneModelInfo>& output_planes);

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
