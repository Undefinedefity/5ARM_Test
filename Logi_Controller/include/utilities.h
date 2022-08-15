/*!
 * @file utilities.h
 * @brief Common utility functions
 */

#ifndef PROJECT_UTILITIES_H
#define PROJECT_UTILITIES_H

#include <eigen3/Eigen/Dense>
/*!
 * Convert boolean to string (true, false)
 */
static inline std::string boolToString(bool b)
{
  return std::string(b ? "true" : "false");
}

/*!
 * Convert eigen type to std::string.
 */
template <typename T>
std::string eigenToString(Eigen::MatrixBase<T> &value)
{
  std::stringstream ss;
  ss << value;
  return ss.str();
}

/*!
 * Apply deadband
 * @param x : input
 * @param range : deadband (+/- range around 0)
 * @return result
 */
template <typename T>
T deadband(T x, T range) {
  if (x < range && x > -range) x = T(0);
  return x;
}
/*!
 * Apply deadband to eigen type
 */
template <typename T>
void eigenDeadband(Eigen::MatrixBase<T>& v, typename T::Scalar band) {
  for (size_t i = 0; i < T::RowsAtCompileTime; i++) {
    for (size_t j = 0; j < T::ColsAtCompileTime; j++) {
      v(i, j) = deadband(v(i, j), band);
    }
  }
}

#endif // PROJECT_UTILITIES_H
