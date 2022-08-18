//
// Created by lfc on 2021/4/25.
//

#ifndef D435_CALIBRATION_GROUND_PLANE_MAP_HPP
#define D435_CALIBRATION_GROUND_PLANE_MAP_HPP
#include <vector>

class GroundPlaneMap {
 public:
    GroundPlaneMap(float resolution,float length,float width, uint16_t height_dimension, float min_height, float max_height)
        : resolution_(resolution), min_height_(min_height), max_height_(max_height),height_dimension_(height_dimension) {
        length_ = roundf(length / resolution);
        width_ = roundf(width / resolution);
        height_step_ = (float)height_dimension / (max_height - min_height);
        map_.resize(length_ * width_);
        center_x_offset_ = 0.0;
        center_y_offset_ = width / 2.0;
        clear();
    }

    void clear(){
        memset(map_.data(), 0, sizeof(uint16_t)*map_.size());
    }

    bool inHeight(const float height) const { return height >= min_height_ && height <= max_height_; }

    bool inGrid(const int x, const int y) const { return x >= 0 && x < length_ && y >= 0 && y < width_; }

    int heightStepCount(uint16_t state) {
        int countx = 0;
        while (state) {
            countx++;
            state = state & (state - 1);
        }
        return countx;
    }

    int coordX(const float coord_x)const{ return (int)roundf((coord_x + center_x_offset_) / resolution_); }

    int coordY(const float coord_y)const{ return (int)roundf((coord_y + center_y_offset_) / resolution_); }

    const uint16_t operator()(const float coord_x,const float coord_y)const{
        int coord_x_int = coordX(coord_x);
        int coord_y_int = coordY(coord_y);
        if (inGrid(coord_x_int, coord_y_int)) {
            return map_[coord_y_int * length_ + coord_x_int];
        }
        return 0;
    }

    template <class Point>
    void addPoint(float coord_x,float coord_y,Point& point){
        int coord_x_int = coordX(coord_x);
        int coord_y_int = coordY(coord_y);
        if (inGrid(coord_x_int, coord_y_int)) {
            if (inHeight(point.z())) {
                int height = roundf((point.z() - min_height_) * height_step_);
                index(coord_x_int, coord_y_int) |= (1 << height);
            }
        }
    }

    uint16_t &index(int x,int y){ return map_[y * length_ + x]; }
 private:

    std::vector<uint16_t> map_;
    uint16_t height_dimension_;
    int length_,width_;
    float center_x_offset_;
    float center_y_offset_;
    float resolution_;
    float min_height_;
    float max_height_;
    float height_step_;
};

#endif  // D435_CALIBRATION_GROUND_PLANE_MAP_HPP
