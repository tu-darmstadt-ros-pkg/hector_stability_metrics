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

#ifndef HECTOR_STABILITY_METRICS_FORCE_ANGLE_STABILITY_MEASURE_H
#define HECTOR_STABILITY_METRICS_FORCE_ANGLE_STABILITY_MEASURE_H

#include "hector_stability_metrics/support_polygon.h"
#include "hector_stability_metrics/types.h"

#include <Eigen/Geometry>

namespace hector_stability_metrics
{

/*!
 * Computes the Force Angle Stability Measure (Papadopoulos, Rey, 1996) for the given support polygon and returns the
 * index of the least stable axis.
 * The stabilities for each axis are stored in the support_polygon. Previous values are overwritten.
 * @param support_polygon A support polygon where it is assumed that the points are ordered clockwise when viewed from
 * above.
 * @param com The position of the center of mass
 * @param external_force External force acting on the COM. Usually the gravity vector
 * @param normalization_factor A normalization factor that is multiplied with each beta which can be used  to normalize
 * against a base stability, e.g., the robot's stability when standing on flat ground (Note: The min beta may still
 * become greater than 1)
 * @return The index of the least stable axis
 */
template<typename Scalar>
size_t computeForceAngleStabilityMeasure( SupportPolygonWithStabilities<Scalar> &support_polygon,
                                          const Vector3<Scalar> &com,
                                          const Vector3<Scalar> &external_force,
                                          Scalar normalization_factor = Scalar( 1.0 ))
{
  support_polygon.axis_stabilities.resize( support_polygon.contact_hull_points.size());

  Scalar min_beta = std::numeric_limits<Scalar>::infinity();
  size_t min_index = 0;
  for ( size_t i = 0; i < support_polygon.contact_hull_points.size(); ++i )
  {
    size_t b = i + 1;
    if ( b == support_polygon.contact_hull_points.size())
      b = 0;
    Vector3<Scalar> axis =
      (support_polygon.contact_hull_points[b] - support_polygon.contact_hull_points[i]).normalized();
    Eigen::Matrix<Scalar, 3, 3> projection = Eigen::Matrix<Scalar, 3, 3>::Identity() - axis * axis.transpose();
    Vector3<Scalar> axis_normal = projection * (support_polygon.contact_hull_points[b] - com);
    // Since the mass normally doesn't change, we omit it
    Vector3<Scalar> force_component = projection * external_force;
    Scalar force_component_norm = force_component.norm();
    Vector3<Scalar> force_component_normalized = force_component / force_component_norm;
    Scalar distance =
      (-axis_normal + axis_normal.dot( force_component_normalized ) * force_component_normalized).norm();
    axis_normal.normalize();
    Scalar sigma = axis_normal.cross( force_component_normalized ).dot( axis ) < 0 ? 1 : -1;
    Scalar theta = sigma * std::acos( force_component_normalized.dot( axis_normal ));
    Scalar beta = theta * distance * force_component_norm;
    support_polygon.axis_stabilities[i] = beta * normalization_factor;
    if ( beta < min_beta )
    {
      min_beta = beta;
      min_index = i;
    }
  }
  return min_index;
}
}

#endif  // HECTOR_STABILITY_METRICS_FORCE_ANGLE_STABILITY_MEASURE_H