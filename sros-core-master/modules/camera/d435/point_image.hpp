#pragma once

namespace obstacle_detection {
//栅格化点
struct RasterizedPoint {
    float x, y, z;   //三维坐标
    bool validity;   //有效性
    float intensity; //强度
    int front_index; //前点的序数
    int back_index;  //后点的序数
};

class PointImage {
public:
    PointImage(const int height, const int width) : height_(height), width_(width) {
        data_ = new RasterizedPoint[height_ * width_];
    }

    ~PointImage() {
        delete[] data_;
    }

private:
    const int height_;
    const int width_;
    RasterizedPoint *data_;

public:
    const RasterizedPoint &operator()(const int row, const int col) const {
        return data_[row * width_ + col];
    }

    RasterizedPoint &operator()(const int row, const int col) {
        return data_[row * width_ + col];
    }
};
} // namespace obstacle_detection