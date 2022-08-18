/**
 * @file qrcode.hpp
 * @brief qrcode information
 * @author wuchaohuo@standard-robots.com
 * @date create date：2022/05/23
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef __QRCODE_HPP__
#define __QRCODE_HPP__

// INCLUDE
#include <vector>

//CODE
namespace nav {

/** @brief Qrcode information */
class QRCode {
public:
    /**
     * @brief default constructor.
     */
    QRCode() = default;

    /**
     * @brief default destructor
     */
    ~QRCode() = default;

    /**
     * @brief set QRCode id.
     * @param[in] id input QRCode id.
     */
    inline void setId(const std::string& id) {
        this->id_ = id;
    }

    /**
     * @brief get QRCode id.
     * @return output QRCode id.
     */
    inline std::string getId() const {
        return this->id_;
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

    /** @brief QRCode id. */
    std::string id_;

    /** @brief target good length.(uint: meter) */
    float length_{};

    /** @brief target good length.(uint: meter) */
    float width_{};

};

using QRCode = nav::QRCode;

} // end of namespace nav
#endif //__QRCODE_HPP__
