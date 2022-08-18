#ifndef MATH_HPP_
#define MATH_HPP_

#include <cmath>
#include <Eigen/Dense>

namespace math {
template <typename T>
inline T pow2(const T value) {
  return value * value;
}

template <typename T>
inline T hypot(const T x, const T y, const T z) {
  return std::sqrt(pow2(x) + pow2(y) + pow2(z));
}
}  // namespace math
#endif /* !MATH_HPP_ */
