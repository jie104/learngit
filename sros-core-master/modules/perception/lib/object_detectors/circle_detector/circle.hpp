/**
 * @file circle.h
 * @brief 简述文件内容
 * 
 * 此处写文件的详细内容，注意需要与上下内容用空格分开。
 * 
 * @author zhangxu@standard-robots.com
 * @date create date：2020/12/16
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef __CIRCLE_H__
#define __CIRCLE_H__

// INCLUDE

//CODE
namespace perception {
/**
 * @description : TODO
 * @author      : zhangxu
 * @date        : 2020/12/16 上午11:13
 */
class Circle {
public:
    Circle() = default;

    ~Circle() = default;

    inline int getId() const {
        return id_;
    };

    inline void setId(const int id) {
        this->id_ = id;
    }

    inline float getRadius() const {
        return radius_;
    }

    inline void setRadius(const float radius){
        this->radius_ = radius;
    }

    /**
     * @brief set the target length (uint: meter).
     * @param[in] angle input target length
     */
    inline void setLength(const float &length) {
        this->length_ = length;
    }

    /**
     * @brief get the target length (uint: meter).
     * @return target length
     */
    inline float getLength() const {
        return this->length_;
    }

    /**
    * @brief set the target width (uint: meter).
    * @param[in] angle input target width
    */
    inline void setWidth(const float &width) {
        this->width_ = width;
    }

    /**
     * @brief get the target width (uint: meter).
     * @return target width
     */
    inline float getWidth() const {
        return this->width_;
    }
private:

    int id_;
    float radius_;

    /** @brief target total length.(uint: meter) */
    float length_{};

    /** @brief target total length.(uint: meter) */
    float width_{};
};

using Circle = perception::Circle;

} // end of namespace perception
#endif //PERCEPTION_SOLUTION_CIRCLE_H
