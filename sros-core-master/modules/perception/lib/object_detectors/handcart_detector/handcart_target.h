
#ifndef __HANDCART_TARGET_H__
#define __HANDCART_TARGET_H__

// INCLUDE
#include "../base/cluster.h"
#include "../base/target.hpp"

#include <vector>
#include <opencv/cv.h>

//CODE
namespace perception {

/**
 * @brief target of the card detect.
 */
class HandcartTarget : public Target {
public:

    /**
     * @brief constructor.
     */
    HandcartTarget();

    /**
     * @brief destructor.
     */
    ~HandcartTarget() override;

    void reset() override;

    /**
     * @brief set pallets index.
     * @param[in] start_index start index of pallet in pallet list.
     * @param[in] num number of the pallets.
     */
    void setMatsIndices(int start_index, int num);

    /**
     * @brief get target rect(x,y,width, height) in target image.
     * @return target rect
     */
    inline cv::Rect getRect() const {
        return this->rect_;
    }

    /**
     * @brief set target rect(x,y,width, height) in target image.
     * @param[in] rect target rect
     */
    inline void setRect(const cv::Rect &rect) {
        this->rect_ = rect;
    }

    /**
     * @brief set the bottom left point in target image.
     * @param[in] point x.y in the picture.
     */
    inline void setLeftDownPoint(const cv::Point &point) {
        this->left_down_ = point;
    }

    /**
     * @brief get the bottom left point in target image.
     * @return point
     */
    inline cv::Point getLeftDownPoint() const {
        return left_down_;
    }

    /**
     * @brief set the bottom left point in target image.
     * @param[in] point
     */
    inline void setRightDownPoint(const cv::Point &point) {
        this->right_down_ = point;
    }

    /**
     * @brief set the bottom right point of target rect.
     * @param[out] cv::point 2dPoint.
     */
    inline cv::Point getRightDownPoint() const {
        return right_down_;
    }

    /**
     * @brief get card height
     * @return out put handcart height
     */
    inline float getHeight() const {
        return this->height_;
    }

    /**
     * @brief set card height.
     * @param[in] width input handcart height.
     */
    inline void setHeight(const float &width) {
        this->height_ = width;
    }

    /**
     * @brief get card width
     * @return out put handcart width
     */
    inline float getWidth() const {
        return this->width_;
    }

    /**
     * @brief set card width.
     * @param[in] width input handcart width.
     */
    inline void setWidth(const float &width) {
        this->width_ = width;
    }

    /**
     * @brief get pallet rectangles.
     * @return output mat rectangles.
     */
    inline cv::Rect getMatRect() const {
        return this->rect_;
    }

    /**
     * @brief set pallet rectangles.
     * @param[in] pallets input pallet rectangles.
     */
    void setMatRect(const cv::Rect &rect){
        this->rect_ = rect;
    }


private:

    /** @brief  across width. */
    float width_;

    /** @brief  across height. */
    float height_;

    /** @brief  the target bottom left point. */
    cv::Point left_down_;

    /** @brief  the target bottom right point. */
    cv::Point right_down_;

    /** @brief  target bounding rectangle. */
    cv::Rect rect_;

};

using HandcartTarget = perception::HandcartTarget;

} // end of namespace perception
#endif //SRC_TARGET_H
