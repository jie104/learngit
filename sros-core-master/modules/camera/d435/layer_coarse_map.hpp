//
// Created by lfc on 2021/4/25.
//

#ifndef D435_CALIBRATION_LAYER_COARSE_MAP_HPP
#define D435_CALIBRATION_LAYER_COARSE_MAP_HPP
#include <vector>
#include <Eigen/Dense>
#include <memory>
#include <math.h>
template <class Point_Ptr>
class LayerCoarseMap {
 public:
    LayerCoarseMap(float resolution,float length,float width,float min_height,float max_height) : resolution_(resolution), min_height_(min_height), max_height_(max_height) {
        length_ = roundf(length / resolution);
        width_ = roundf(width / resolution);
        map_.resize(length_ * width_);
        index_map_.resize(length_ * width_);
        center_x_offset_ = 0.0;
        center_y_offset_ = width / 2.0;
        clear();
    }

    void clear(){
        for (auto& map : map_) {
            map.clear();
        }
        for (auto& index : index_map_) {
            if (index.empty()) {
                index.resize(fine_times * fine_times);
            }
            for(auto& id:index){
                id = -1;
            }
        }
    }

    int length()const { return length_; }

    int width()const { return width_; }

    bool inMap(float coord_x,float coord_y){ return inGrid(coordX(coord_x), coordY(coord_y)); }

    bool inHeight(const float height) const { return height >= min_height_ && height <= max_height_; }

    void setHeightRange(const float min_height, const float max_height){
        min_height_ = min_height;
        max_height_ = max_height;
    }

    bool inGrid(const int x, const int y) const { return x >= 0 && x < length_ && y >= 0 && y < width_; }

    int coordX(const float coord_x)const{ return (int)roundf((coord_x + center_x_offset_) / resolution_); }

    int coordY(const float coord_y)const{ return (int)roundf((coord_y + center_y_offset_) / resolution_); }

    void addPoint(float coord_x,float coord_y,Point_Ptr& point){
        int coord_x_int = coordX(coord_x);
        int coord_y_int = coordY(coord_y);
        if (inGrid(coord_x_int, coord_y_int)) {
            auto& indexes = mapIndex(coord_x_int, coord_y_int);
            int delta_step = fine_times - 1;
            int index_x = (int)roundf(((coord_x+center_x_offset_) / resolution_ - coord_x_int + 0.5) * delta_step) % fine_times;
            int index_y = (int)roundf(((coord_y+center_y_offset_) / resolution_ - coord_y_int + 0.5) * delta_step) % fine_times;
            auto &curr_index = indexes[index_y * fine_times + index_y];
            if (curr_index >= 0) {
                auto& curr_point = index(coord_x_int, coord_y_int)[curr_index];
                curr_point = curr_point.z() >= point.z() ? curr_point : point;
            }else{
                curr_index = index(coord_x_int, coord_y_int).size();
                index(coord_x_int, coord_y_int).push_back(point);
            }
        }
    }

    std::vector<Point_Ptr>& index(int x,int y){ return map_[length_ * y + x]; }
 private:
    std::vector<int> &mapIndex(int x,int y){ return index_map_[length_ * y + x]; }

    const int fine_times = 6;
    int length_,width_;
    float center_x_offset_,center_y_offset_;
    float resolution_;
    float min_height_;
    float max_height_;
    std::vector<std::vector<Point_Ptr>> map_;
    std::vector<std::vector<int>> index_map_;
};

#endif  // D435_CALIBRATION_LAYER_COARSE_MAP_HPP
