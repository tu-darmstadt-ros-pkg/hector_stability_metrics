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

#include <Eigen/Core>
#include <vector>

namespace hector_stability_metrics
{

template <typename Scalar>
struct SupportPolygon
{
  //! The convex hull of the contact points of the robot with the ground. The contact points should ordered clockwise
  //! when viewed from above.
  std::vector<Eigen::Matrix<Scalar, 3, 1>, Eigen::aligned_allocator<Eigen::Matrix<Scalar, 3, 1> > > contact_hull_points;
  //! The stability of each axis. Index 0 is the stability of the axis from hull point 0 to hull point 1.
  std::vector<Scalar> axis_stabilities;
};

typedef SupportPolygon<double> SupportPolygond;
typedef SupportPolygon<float> SupportPolygonf;
}  // namespace hector_stability_metrics

#endif  // HECTOR_STABILITY_METRICS_SUPPORT_POLYGON_H