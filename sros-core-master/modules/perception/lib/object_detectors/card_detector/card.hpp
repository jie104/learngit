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

#ifndef __CARD_H__
#define __CARD_H__

// INCLUDE
#include <vector>
#include "../base/cluster.h"

//CODE
namespace perception {

/** @brief card information */
class Card {
public:
    /**
     * @brief default constructor.
     */
    Card() = default;

    /**
     * @brief default destructor
     */
    ~Card() = default;

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
     * @brief get number of the holes.
     * @return output number of the holes.
     */
    inline int getHolesSize() const {
        return this->holes_width_.size();
    }

    /**
     * @brief get number of the pallet.
     * @return output number of the pallet.
     */
    inline int getPalletSize() const {
        return this->pallets_width_.size();
    }

    /**
     * @brief set pallet height.
     * @param[in] height input pallet height.
     */
    inline void setPalletHeight(const float &height) {
        this->pallet_height_ = height;
    }

    /**
     * @brief get pallet height.
     * @return[out] output pallet height.
     */
    inline float getPalletHeight() const {
        return this->pallet_height_;
    }

    /**
     * @brief set pallet pixel-height in deep image.
     * @param[in] pixel_height input pixel_height.
     */
    inline void setPalletPixelHeight(const int &pixel_height) {
        this->pallet_pixel_height_ = pixel_height;
    }

    /**
     * @brief get pallet pixel height in deep image.
     * @return output pixel_height.
     */
    inline int getPalletPixelHeight() const {
        return this->pallet_pixel_height_;
    }

    /**
     * @brief set hole height.
     * @param[in] height input hole height.
     */
    inline void setHoleHeight(const float &height) {
        this->hole_height_ = height;
    }

    /**
     * @brief get hole height.
     * @return output hole height.
     */
    inline float getHoleHeight() const {
        return this->hole_height_;
    }

    /**
     * @brief set hole pixel height in deep image.
     * @param[in] pixel_height input hole pixel height.
     */
    inline void setHolePixelHeight(const int &pixel_height) {
        this->hole_pixel_height_ = pixel_height;
    }

    /**
     * @brief get hole pixel height in deep image.
     * @return output pixel height.
     */
    inline int getHolePixelHeight() const {
        return this->hole_pixel_height_;
    }

    /**
     * @brief set hole width.
     * @param[in] hole_width input hole width.
     */
    inline void setHoleWidth(const std::vector<float> &hole_width) {
        this->holes_width_.resize(hole_width.size());
        copy(hole_width.begin(), hole_width.end(), this->holes_width_.begin());
    }

    /**
     * @brief get hole width.
     * @return output hole width.
     */
    inline std::vector<float> getHolesWidth() const {
        return this->holes_width_;
    }

    /**
     * @brief set pallet width.
     * @param[in] pallet_width input pallet width.
     */
    inline void setPalletWidth(const std::vector<float> &pallet_width) {
        this->pallets_width_.resize(pallet_width.size());
        copy(pallet_width.begin(), pallet_width.end(), this->pallets_width_.begin());
    }

    /**
     * @brief get pallet width.
     * @return output pallet width.
     */
    inline std::vector<float> getPalletsWidth() const {
        return this->pallets_width_;
    }

    /**
     * @brief set hole pixel width in deep image.
     * @param[in] hole_pixel_width input pixel width.
     */
    inline void setHolePixelWidth(const std::vector<int> &hole_pixel_width) {
        this->holes_pixel_width_.resize(hole_pixel_width.size());
        copy(hole_pixel_width.begin(), hole_pixel_width.end(), this->holes_pixel_width_.begin());
    }

    /**
     * @brief get hole pixel width in deep image.
     * @return output pixel width.
     */
    inline std::vector<int> getHolePixelWidth() const {
        return this->holes_pixel_width_;
    }

    /**
     * @brief set pallet pixel width in deep image.
     * @param[in] pallet_pixel_width input pallet pixel width.
     */
    inline void setPalletPixelWidth(const std::vector<int> &pallet_pixel_width) {
        this->pallets_pixel_width_.resize(pallet_pixel_width.size());
        copy(pallet_pixel_width.begin(), pallet_pixel_width.end(), this->pallets_pixel_width_.begin());
    }

    /**
     * @brief get pallet pixel width in deep image.
     * @return output pixel width.
     */
    inline std::vector<int> getPalletPixelWidth() const {
        return this->pallets_pixel_width_;
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

    /** @brief card id. */
    int id_;

    /** @brief target total length.(uint: meter) */
    float length_{};

    /** @brief target total length.(uint: meter) */
    float width_{};

    /** @brief pallet height. (Unit: m) */
    float pallet_height_;

    /** @brief hole height. (Unit: m) */
    float hole_height_;

    /** @brief pallet pixel height in deep image. */
    int pallet_pixel_height_;

    /** @brief hole pixel height in deep image. */
    int hole_pixel_height_;

    /** @brief holes width. (Unit: m) */
    std::vector<float> holes_width_;

    /** @brief pallet width. (Unit: m) */
    std::vector<float> pallets_width_;

    /** @brief card pallets pixel width list. */
    std::vector<int> pallets_pixel_width_;

    /** @brief card holes pixel width list. */
    std::vector<int> holes_pixel_width_;
};

using Card = perception::Card;

} // end of namespace perception
#endif //CARD_H
