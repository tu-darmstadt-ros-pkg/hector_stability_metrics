/*
 * Copyright (C) 2020  Stefan Fabian
 *
 * This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef HECTOR_STABILITY_METRICS_SUPPORT_POLYGON_H
#define HECTOR_STABILITY_METRICS_SUPPORT_POLYGON_H

#include "hector_stability_metrics/math/hull.h"
#include "hector_stability_metrics/math/types.h"

namespace hector_stability_metrics
{

template <typename Scalar>
struct SupportPolygon
{
  //! The convex hull of the contact points of the robot with the ground. The contact points should ordered clockwise
  //! when viewed from above.
  Vector3List<Scalar> contact_hull_points;
  //! The stability of each axis. Index 0 is the stability of the axis from hull point 0 to hull point 1.
  std::vector<Scalar> axis_stabilities;
};

using SupportPolygonf = SupportPolygon<float>;
using SupportPolygond = SupportPolygon<double>;

template <typename Container, typename Scalar=typename Eigen::DenseBase<typename Container::value_type>::Scalar>
void supportPolygonFromSortedContactPoints( const Container &points, SupportPolygon<Scalar> &result )
{
  math::convexHull(points, result.contact_hull_points);
}

template <typename Container, typename Scalar=typename Eigen::DenseBase<typename Container::value_type>::Scalar>
SupportPolygon<Scalar> supportPolygonFromSortedContactPoints( const Container &points )
{
  SupportPolygon<Scalar> result;
  math::convexHull(points, result.contact_hull_points);
  return result;
}

template <typename Container, typename Scalar=typename Eigen::DenseBase<typename Container::value_type>::Scalar>
SupportPolygon<Scalar> supportPolygonFromUnsortedContactPoints( const Container &points)
{
  Container copy = points;
  std::sort(copy.begin(), copy.end(), [](const typename Container::value_type &a, const typename Container::value_type &b) {
    if (a.y() < b.y()) return true;
    return a.y() == b.y() && a.x() < b.x();
  });
  return supportPolygonFromSortedContactPoints(copy);
}
}  // namespace hector_stability_metrics

#endif  // HECTOR_STABILITY_METRICS_SUPPORT_POLYGON_H