
/**
 * @file DMCodeMark.hpp
 *
 * @author pengjiali
 * @date 2019年1月23日
 *
 * @describe
 *
 * @copyright Copyright (c) 2019 Standard Robots Co., Ltd. All rights reserved.
 */
#ifndef SROS_MAP_DMCODEMARK_HPP
#define SROS_MAP_DMCODEMARK_HPP

#include "Mark.hpp"

namespace sros {
namespace map {

class DMCodeMark : public MarkPointf {
 public:
    DMCodeMark() = default;
    DMCodeMark(int id, const std::string &no, double x_, double y_, double yaw_) : MarkPointf(x_, y_, yaw_), id(id), no(no) {}

    bool isValid() const { return no.empty(); }

 public:
    int id = 0;
    std::string no; // 默认值为空代表着该DMCodeMark无效
};

typedef MarkGroup<DMCodeMark, std::string> DMCodeMarkGroup;
}  // namespace map
}  // namespace sros
#endif  // SROS_MAP_DMCODEMARK_HPP
