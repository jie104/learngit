//
// Created by lfc on 2023/1/3.
//

#ifndef RACK_DETECTOR_RECTANGLE_DETECTOR_HPP
#define RACK_DETECTOR_RECTANGLE_DETECTOR_HPP
#include "triangle_detecor.hpp"
namespace rack_detection{
struct RectangleInfo{
    std::vector<TriangleInfo_Ptr> triangles;
    Eigen::Vector2f direction;
    Eigen::Vector2f mean;
    float length;
    float width;
    void computeRectangleInfo(){
        direction = triangles[0]->length_point->mean - triangles[0]->corner_point->mean;
        length = triangles[0]->length;
        width = triangles[0]->width;
        mean.setZero();
        for (auto& triangle : triangles) {
            Eigen::Vector2f curr_direction = triangle->length_point->mean - triangle->corner_point->mean;
            float dot_value = direction.dot(curr_direction) / (direction.norm() * curr_direction.norm());   //cos
            if (fabs(dot_value) > 0.5) {
                curr_direction = dot_value > 0 ? curr_direction : (Eigen::Vector2f)-curr_direction;
                direction += curr_direction;
            }
            mean += triangle->mean;
        }
        direction.normalized();
        mean = mean / (float(triangles.size()));
    }
};
typedef std::shared_ptr<RectangleInfo> RectangleInfo_Ptr;

class RectangleDetector {
 public:
    void filterRectangleBySize(std::vector<RectangleInfo_Ptr>& rectangles,float length,float width,float dist_thresh = 0.05){
        std::vector<RectangleInfo_Ptr> filtered_rects;
        for(auto& rect:rectangles) {
            if (fabs(rect->length - length) < dist_thresh && fabs(rect->width - width) < dist_thresh) {
                filtered_rects.push_back(rect);
            }
        }
        rectangles.swap(filtered_rects);
    }

    void computeRectangle(const std::vector<TriangleInfo_Ptr> &triangles,std::vector<RectangleInfo_Ptr>& rectangles){
        int angle_size = triangles.size();
        std::vector<bool> states(angle_size, false);
        for (int i = 0; i < angle_size; ++i) {
            if (!states[i]) {
                rectangles.emplace_back(new RectangleInfo);
                rectangles.back()->triangles.push_back(triangles[i]);
                for (int j = i + 1; j < angle_size; ++j) {
                    if (triangles[i]->equal(triangles[j])) {
                        rectangles.back()->triangles.push_back(triangles[j]);
                        states[j] = true;
                    }
                }
                rectangles.back()->computeRectangleInfo();
            }
        }
    }
 private:

};

}



#endif  // RACK_DETECTOR_RECTANGLE_DETECTOR_HPP
