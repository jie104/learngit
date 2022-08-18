/**
 * @file card.h
 * @brief card information
 * 
 * card bash data, such as pallets width, pallets height, pallet pixel width in
 * deep image, pallet pixel width in deep image.
 * 
 * @author zhangxu@standard-robots.com
 * @date create date：2020/9/22
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef __HANDCART_H__
#define __HANDCART_H__

// INCLUDE
#include <vector>
#include "../base/cluster.h"

//CODE
namespace perception {

/** @brief card information */
class Handcart {
public:
    /**
     * @brief default constructor.
     */
    Handcart() = default;

    /**
     * @brief default destructor
     */
    ~Handcart() = default;

    /**
     * @brief set card id.
     * @param[in] id input card id.
     */
    inline void setId(int id) {
        this->id_ = id;
    }

    /**
     * @brief get card id.
     * @return output card id.
     */
    inline int getId() const {
        return this->id_;
    }

    /**
     * @brief set across thickness.
     * @param[in] across_thickness value of across height.
     */
    inline void setAcrossHeight(const float across_thickness){
        this->across_height_ = across_thickness;
    }

    /**
     * @brief get across thickness.
     * @return output across height.
     */
    inline float getAcrossHeight() const {
        return this->across_height_;
    }

    /**
     * @brief set width of handcart.
     * @param[in] width width of handcart.
     */
    inline void setWidth(const float width) {
        this->width_ = width;
    }

    /**
     * @brief get handcart width.
     * @return output handcart width.
     */
    inline float getWidth() const {
        return this->width_;
    }

    /**
 * @brief set total width of handcart.
 * @param[in] width width of handcart.
 */
    inline void setLength(const float length) {
        this->length_ = length;
    }

    /**
     * @brief get handcart width.
     * @return output handcart width.
     */
    inline float getLength() const {
        return this->length_;
    }

private:
    /** @brief card id. */
    int id_;

    /** @brief height of handcart base. (Unit: m) */
    float across_height_;

    /** @brief total width of handcart base. (Unit: m) */
    float width_;

    /** @brief length of handcart base. (Unit: m) */
    float length_;
};

using Handcart = perception::Handcart;

} // end of namespace perception
#endif //HANDCART_H
