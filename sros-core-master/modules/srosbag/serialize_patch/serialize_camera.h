//
// Created by duan on 2021/6/16.
//

#ifndef SROS_SERIALIZE_CAMERA_H
#define SROS_SERIALIZE_CAMERA_H

#include <opencv2/opencv.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>
// #include "../../message/camera_msg.hpp"
#include "../sros-core/core/msg/image_msg.hpp"

using CameraData = sros::core::ImageMsg;

namespace boost{
namespace serialization {

template <class Archive>
void serialize(Archive &ar, CameraData &data, const unsigned int version){
    ar &data.time_;
    ar &data.mat_;
}

template<class Archive>
void serialize(Archive &ar, cv::Mat& mat, const unsigned int version){
    int cols, rows, type;
    bool continuous;

    if (Archive::is_saving::value) {
        cols = mat.cols;
        rows = mat.rows;
        type = mat.type();
        continuous = mat.isContinuous();
    }

    ar & cols & rows & type & continuous;

    if (Archive::is_loading::value)
        mat.create(rows, cols, type);

    if (continuous) {
        const unsigned int data_size = rows * cols * mat.elemSize();
        ar & boost::serialization::make_array(mat.ptr(), data_size);
    } else {
        const unsigned int row_size = cols * mat.elemSize();
        for (int i = 0; i < rows; i++) {
            ar & boost::serialization::make_array(mat.ptr(i), row_size);
        }
    }
}

}
}


#endif  // SROS_SERIALIZE_CAMERA_H
