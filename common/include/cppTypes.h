/*! @file cTypes.h
 *  @brief Common types that are only valid in C++
 *
 *  This file contains types which are only used in C++ code.  This includes
 * Eigen types, template types, aliases, ...
 */

#ifndef PROJECT_CPPTYPES_H
#define PROJECT_CPPTYPES_H

#include <vector>
#include "cTypes.h"
#include <eigen3/Eigen/Dense>

// Rotation Matrix
template <typename T>
using RotMat = typename Eigen::Matrix<T, 3, 3>;

// 1x1 Vector
template <typename T>
using Vec1 = typename Eigen::Matrix<T, 1, 1>;

// 2x1 Vector
template <typename T>
using Vec2 = typename Eigen::Matrix<T, 2, 1>;

// 3x1 Vector
template <typename T>
using Vec3 = typename Eigen::Matrix<T, 3, 1>;

// 4x1 Vector
template <typename T>
using Vec4 = typename Eigen::Matrix<T, 4, 1>;

// 2000x12 Vector
template <typename T>
using Mat2000_12 = typename Eigen::Matrix<T, 2000, 12>;

// 2x4 Vector
template <typename T>
using Vec2_4 = typename Eigen::Matrix<T, 2, 4>;

// 6x1 Vector
template <typename T>
using Vec6 = Eigen::Matrix<T, 6, 1>;

// 7x1 Vector
template <typename T>
using Vec7 = Eigen::Matrix<T, 7, 1>;

// 28x1 Vector
template <typename T>
using Vec28 = Eigen::Matrix<T, 28, 1>;

// 10x1 Vector
template <typename T>
using Vec10 = Eigen::Matrix<T, 10, 1>;

// 10x4 Vector
template <typename T>
using Mat9_4 = Eigen::Matrix<T, 9, 4>;

// 12x1 Vector
template <typename T>
using Vec12 = Eigen::Matrix<T, 12, 1>;

// 24x1 Vector
template <typename T>
using Vec24 = Eigen::Matrix<T, 24, 1>;

// 16x1 Vector
template <typename T>
using Vec16 = Eigen::Matrix<T, 16, 1>;

// 18x1 Vector
template <typename T>
using Vec18 = Eigen::Matrix<T, 18, 1>;

// 28x1 vector
template <typename T>
using Vec28 = Eigen::Matrix<T, 28, 1>;

// 36x1 vector
template <typename T>
using Vec36 = Eigen::Matrix<T, 36, 1>;

// 3x3 Matrix
template <typename T>
using Mat3 = typename Eigen::Matrix<T, 3, 3>;

// 42x4 Matrix
template <typename T>
using Mat42_4 = typename Eigen::Matrix<T, 42, 4>;

// 7x4 Matrix
template <typename T>
using Mat7_4 = typename Eigen::Matrix<T, 7, 4>;

//13x4 Matrix
template <typename T>
using Mat13_4 = typename Eigen::Matrix<T, 13, 4>;

// 4x1 Vector
template <typename T>
using Quat = typename Eigen::Matrix<T, 4, 1>;

// 1x4 Vector
template <typename T>
using Mat1_4 = typename Eigen::Matrix<T, 1, 4>;

// Spatial Vector (6x1, all subspaces)
template <typename T>
using SVec = typename Eigen::Matrix<T, 6, 1>;

// Spatial Transform (6x6)
template <typename T>
using SXform = typename Eigen::Matrix<T, 6, 6>;

// 6x6 Matrix
template <typename T>
using Mat6 = typename Eigen::Matrix<T, 6, 6>;

// 12x12 Matrix
template <typename T>
using Mat12 = typename Eigen::Matrix<T, 12, 12>;

// 18x18 Matrix
template <typename T>
using Mat18 = Eigen::Matrix<T, 18, 18>;

// 28x28 Matrix
template <typename T>
using Mat28 = Eigen::Matrix<T, 28, 28>;

// 3x4 Matrix
template <typename T>
using Mat34 = Eigen::Matrix<T, 3, 4>;

// 3x4 Matrix
template <typename T>
using Mat23 = Eigen::Matrix<T, 2, 3>;

// 4x4 Matrix
template <typename T>
using Mat4 = typename Eigen::Matrix<T, 4, 4>;

// 10x1 Vector
template <typename T>
using MassProperties = typename Eigen::Matrix<T, 10, 1>;

// Dynamically sized vector
template <typename T>
using DVec = typename Eigen::Matrix<T, Eigen::Dynamic, 1>;

// Dynamically sized matrix
template <typename T>
using DMat = typename Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

// Dynamically sized matrix with spatial vector columns
template <typename T>
using D6Mat = typename Eigen::Matrix<T, 6, Eigen::Dynamic>;

// Dynamically sized matrix with cartesian vector columns
template <typename T>
using D3Mat = typename Eigen::Matrix<T, 3, Eigen::Dynamic>;

// std::vector (a list) of Eigen things
template <typename T>
using vectorAligned = typename std::vector<T, Eigen::aligned_allocator<T>>;

enum class RobotType { CHEETAH_3, MINI_CHEETAH };

#endif  // PROJECT_CPPTYPES_H
