#pragma once

#include <cmath>

namespace math {
static constexpr float g_limit_zero = 1e-6f;

//判断是否为0
static bool isZero(const float value) {
    return std::abs(value) < g_limit_zero;
}

//平方
template<typename T>
inline T pow2(const T value) {
    return value * value;
}

//模长
template<typename T>
inline T hypot(const T x, const T y, const T z) {
    return std::sqrt(pow2(x) + pow2(y) + pow2(z));
}
} // namespace math