// Copyright (c) 2020 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef HECTOR_STABILITY_METRICS_TYPES_H
#define HECTOR_STABILITY_METRICS_TYPES_H

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <vector>

namespace hector_stability_metrics
{
template<typename Scalar>
using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;
using Matrix3f = Matrix3<float>;
using Matrix3d = Matrix3<double>;

template<typename Scalar>
using Vector2 = Eigen::Matrix<Scalar, 2, 1>;
using Vector2f = Vector2<float>;
using Vector2d = Vector2<double>;
template<typename Scalar>
using Vector2List = std::vector<Vector2<Scalar>, Eigen::aligned_allocator<Vector2<Scalar> > >;
using Vector2fList = Vector2List<float>;
using Vector2dList = Vector2List<double>;

template<typename Scalar>
using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
using Vector3f = Vector3<float>;
using Vector3d = Vector3<double>;
template<typename Scalar>
using Vector3List = std::vector<Vector3<Scalar> >;
using Vector3fList = Vector3List<float>;
using Vector3dList = Vector3List<double>;

template<typename Scalar>
using VectorX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
using VectorXf = VectorX<float>;
using VectorXd = VectorX<double>;
template<typename Scalar>
using VectorXList = std::vector<VectorX<Scalar> >;
using VectorXfList = VectorXList<float>;
using VectorXdList = VectorXList<double>;

template<typename Scalar>
using Isometry3 = Eigen::Transform<Scalar, 3, Eigen::Isometry>;
using Isometry3f = Isometry3<float>;
using Isometry3d = Isometry3<double>;
}  // namespace hector_stability_metrics

#endif  // HECTOR_STABILITY_METRICS_TYPES_H
