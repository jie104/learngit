
#ifndef __CARD_TARGET_H__
#define __CARD_TARGET_H__

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
class CardTarget : public Target {
public:

    /**
     * @brief constructor.
     */
    CardTarget();

    /**
     * @brief destructor.
     */
    ~CardTarget() override;

    void reset() override;

    /**
     * @brief get index of the card pallets.
     * @return pallet index.
     */
    inline std::vector<int> getPalletsIndices() const {
        return this->pallet_indices_;
    }

    /**
     * @brief set index of the card pallets
     * @param[in] indices pallet index
     */
    inline void setPalletsIndices(const std::vector<int> &indices) {
        this->pallet_indices_.resize(indices.size());
        copy(indices.begin(), indices.end(), this->pallet_indices_.begin());
    }

    /**
     * @brief set pallets index.
     * @param[in] start_index start index of pallet in pallet list.
     * @param[in] num number of the pallets.
     */
    void setPalletsIndices(int start_index, int num);

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
     * @brief get card width
     * @return out put card width
     */
    inline float getCardWidth() const {
        return this->card_width_;
    }

    /**
     * @brief set card width.
     * @param[in] width input card width.
     */
    inline void setCardWidth(const float &card_width) {
        this->card_width_ = card_width;
    }

    /**
     * @brief get hole height.
     * @return output hole height.
     */
    inline float getHoleHeight() const {
        return this->hole_height_;
    }

    /**
     * @brief set hole height.
     * @param[in] height input hole height.
     */
    inline void setHoleHeight(const float &hole_height) {
        this->hole_height_ = hole_height;
    }

    /**
     * @brief get hole rectangles.
     * @return output hole rectangles.
     */
    inline std::vector<cv::Rect> getHoleRect() const {
        return this->hole_rect_;
    }

    /**
     * @brief set hole rectangles.
     * @param[in] holes input hole rectangles.
     */
    inline void setHoleRect(const std::vector<cv::Rect> &holes) {
        this->hole_rect_.resize(holes.size());
        copy(holes.begin(), holes.end(), this->hole_rect_.begin());
    }

    /**
     * @brief get pallet rectangles.
     * @return output pallet rectangles.
     */
    inline std::vector<cv::Rect> getPalletRect() const {
        return this->pallet_rect_;
    }

    /**
     * @brief set pallet rectangles.
     * @param[in] pallets input pallet rectangles.
     */
    void setPalletRect(const std::vector<cv::Rect> &pallets);

private:

    /** @brief  card width. */
    float card_width_;

    /** @brief  hole height. */
    float hole_height_;

    /** @brief  the target bottom left point. */
    cv::Point left_down_;

    /** @brief  the target bottom right point. */
    cv::Point right_down_;

    /** @brief  target bounding rectangle. */
    cv::Rect rect_;

    /** @brief  index of the target card pallet in pallet list. */
    std::vector<int> pallet_indices_;

    /** @brief  card hole rectangles. */
    std::vector<cv::Rect> hole_rect_;

    /** @brief  card pallet rectangles. */
    std::vector<cv::Rect> pallet_rect_;
};

using CardTarget = perception::CardTarget;

} // end of namespace perception
#endif //CARD_TARGET_H
