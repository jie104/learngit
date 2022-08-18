/**
 * @file card_result.hpp
 * @brief data of detect result.
 *
 * data of card detect result. include card center pose and angle in AGV coordinate system.
 *
 * @author zhangxu@standard-robots.com
 * @date create date：2020/12/9
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */

#ifndef __DETECCT_RESULT_HPP__
#define __DETECCT_RESULT_HPP__

// INCLUDE
#include "../../../common/normal.hpp"

/**
 * @brief data of detect result.
 * @author      : zhangxu
 * @date        : 2020/12/9 上午11:16
 */
struct DetectResult
{
    /** @brief Card number. */
    int id = -1;

    /** @brief The value of the target in X direction in AGV coordinate system. Unit(m) */
    float x = 0;

    /** @brief The value of the target in Y direction in AGV coordinate system.Unit(m) */
    float y = 0;

    /** @brief The value of the target in Z direction in AGV coordinate system. Unit(m)*/
    float z = 0;

    /** @brief Angle of target plate relative to AGV. Unit(rad)*/
    float angle = 0;

    /** @brief Card board width. Unit(m)*/
    float width = 0;

    /** @brief Height of hole. Unit(m)*/
    float height = 0;

    /** @brief If the result is not available, detect if false. */
    bool is_available = false;

    bool is_stacktop = false;

    /** @brief normal of the card center. */
    Normal normal;

    /**
     * @brief clear the runtime data.
     */
    void reset(){
        this->id = -1;
        this->x = 0;
        this->y = 0;
        this->z = 0;
        this->angle = 0;
        this->width = 0;
        this->height = 0;
        this->is_available = false;
        this->is_stacktop = false;
        this->normal.normal_x = 0;
        this->normal.normal_y = 0;
        this->normal.normal_z = 0;
    }
};
#endif //PERCEPTION_SOLUTION_CARD_RESULT_HPP
