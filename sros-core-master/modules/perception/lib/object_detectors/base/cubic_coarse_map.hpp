//
// Created by lfc on 2022/1/8.
//

#ifndef TSDF_SLAM_CUBIC_COARSE_MAP_HPP
#define TSDF_SLAM_CUBIC_COARSE_MAP_HPP
/*
 * 这是存到搜索地图里
void pushToSearchMap(LivoxPoint &point) {
    if (coarse_map_.inMap(point.x, point.y, point.z)) {
        LivoxPoint_Ptr livox_point(new LivoxPoint(point));
        livox_point->index = point.index;
        auto &livox_points = coarse_map_.value(point.x, point.y, point.z);
        if (livox_points.empty()) {
            searched_points_.push_back(livox_point);//这个vector存在的目的是快速索引到哪个空间网格里有数据
        }
        livox_points.push_back(livox_point);
    }
}
 */
/*
 * 这是通过搜索地图拿数据
        void computeScanPoints(std::vector<mapping5::Scan3DPoint> &scan_points) {
            for (auto &search_point:searched_points_) {//search points表示一共存储了多少个网格，每增加一个网格，就把该点云存进去
                auto &points = coarse_map_.value(search_point->x, search_point->y, search_point->z);
                if (points.size() < point_step) {//如果一个网格点云很多，
                    computeLackPoints(points, scan_points);
                } else {
                    int point_count = 0;
                    int max_num_count = points.size() / point_step;
                    int num_count = 1;
                    std::vector<Eigen::Vector3d> voxel_points;
                    for (auto &point:points) {
                        point_count++;
                        voxel_points.push_back(Eigen::Vector3d(point->x, point->y, point->z));
                        if (point_count >= point_step && num_count < max_num_count) {
                            point_count = 0;
                            num_count++;
                            scan_points.emplace_back();
                            computeScanPoint(voxel_points, scan_points.back());
                            voxel_points.clear();
                        }
                    }
                    if (voxel_points.size()) {
                        scan_points.emplace_back();
                        computeScanPoint(voxel_points, scan_points.back());
                    }
                }
            }
        }
 */

namespace standard {
    template <class Element>
    class CubicCoarseMap {
    public:
        CubicCoarseMap(const Eigen::Vector3f pose,const float length, const float width,const float height,const float resolution)
                : length_(length), width_(width), height_(height),grid_resolution_(resolution) {
            inv_grid_resolution_ = 1.0f / grid_resolution_;
            width_2_ = width / 2.0f;
            length_2_ = length / 2.0f;
            height_2_ = height / 2.0f;
            grid_width_ = int(floorf(width / resolution + 0.5f));
            grid_length_ = int(floorf(length / resolution + 0.5f));
            grid_height_ = int(floorf(height / resolution + 0.5f));
            LOG(INFO) << "length:" << grid_length_ << "," << grid_height_ << "," << grid_width_;
            LOG(INFO) << "length:" << width << "," << length << "," << height << "," << resolution<<","<<grid_resolution_;
            creatGrid(grid_length_, grid_width_, grid_height_);
            updateCenterPose(pose);
        }

        bool inMap(const float coord_x,const float coord_y,const float coord_z){
            auto int_x = coordX(coord_x, coord_y);
            auto int_y = coordY(coord_x, coord_y);
            auto int_z = coordZ(coord_z);
            return ((int_x >= 0) && (int_x < grid_length_)) & ((int_y >= 0) && (int_y < grid_width_)) &&
                   (int_z >= 0 && int_z < grid_height_);
        }

        Element &value(const float coord_x,const float coord_y,const float coord_z){
            auto int_x = coordX(coord_x, coord_y);
            auto int_y = coordY(coord_x, coord_y);
            auto int_z = coordZ(coord_z);
            return map_[int_x][int_y][int_z];
        }

        bool inGridMap(const int int_x,const int int_y,const int int_z){
            return ((int_x >= 0) && (int_x < grid_length_)) & ((int_y >= 0) && (int_y < grid_width_)) &&
                   (int_z >= 0 && int_z < grid_height_);
        }

        Element &valueByMap(const int int_x,const int int_y,const int int_z){
            return map_[int_x][int_y][int_z];
        }

        void setValue(const float coord_x,const float coord_y,const float coord_z,const Element& ele){
            auto int_x = coordX(coord_x, coord_y);
            auto int_y = coordY(coord_x, coord_y);
            auto int_z = coordZ(coord_z);
            map_[int_x][int_y][int_z] = ele;
        }

        const int coordX(const float &coord_x, const float &coord_y) const {
            const float &x = coord_x * cos_theta_ + coord_y * sin_theta_ + delta_x_;
            return int(floorf(x * inv_grid_resolution_ + 0.5f));
        }

        const int coordZ(const float &height) const {
            const float z = height + delta_z_;
            return int(floorf(z * inv_grid_resolution_ + 0.5f));
        }

        const int coordY(const float &coord_x, const float &coord_y) const {
            const float &y = -coord_x * sin_theta_ + coord_y * cos_theta_ + delta_y_;
            return int(floorf(y * inv_grid_resolution_ + 0.5f));
        }

        void creatGrid(const int grid_length, const int grid_width,const int grid_height) {
            if (grid_length == 0 || grid_width == 0 || grid_height == 0) {
                LOG(INFO) << "cannot creat!";
                return;
            }
            map_.resize(grid_length);
            for (auto &map:map_) {
                map.resize(grid_width);
                for (auto &m:map) {
                    m.resize(grid_height);
                }
            }
        }

        virtual void updateCenterPose(const Eigen::Vector3f &pose) {
            map_center_pose_ = pose;

            Eigen::Affine2f center_tf(Eigen::Translation2f(map_center_pose_[0], map_center_pose_[1]) *
                    Eigen::Rotation2Df(map_center_pose_[2]));
            Eigen::Vector2f point = center_tf * Eigen::Vector2f(-length_2_, -width_2_);
            map_origin_pose_ = Eigen::Vector3f(point[0], point[1], map_center_pose_[2]);

            cos_theta_ = cosf(map_origin_pose_[2]);
            sin_theta_ = sinf(map_origin_pose_[2]);
            auto pose_cos_x = map_origin_pose_[0] * cos_theta_;
            auto pose_cos_y = map_origin_pose_[1] * cos_theta_;
            auto pose_sin_x = map_origin_pose_[0] * sin_theta_;
            auto pose_sin_y = map_origin_pose_[1] * sin_theta_;
            delta_x_ = -pose_cos_x - pose_sin_y;
            delta_y_ = pose_sin_x - pose_cos_y;
            delta_z_ = height_2_;
        }
    private:


        float grid_resolution_;
        float inv_grid_resolution_;
        float width_;
        float length_;
        float height_;
        float width_2_;
        float length_2_;
        float height_2_;
        int grid_width_;
        int grid_length_;
        int grid_height_;
        float cos_theta_;
        float sin_theta_;
        float delta_x_;
        float delta_y_;
        float delta_z_;
        std::vector<std::vector<std::vector<Element>>> map_;

        Eigen::Vector3f map_origin_pose_;
        Eigen::Vector3f map_center_pose_;
    };
}


#endif //TSDF_SLAM_CUBIC_COARSE_MAP_HPP
