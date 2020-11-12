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
#include "hector_stability_metrics/metrics/stability_metric_base.h"

#include <Eigen/Geometry>

namespace hector_stability_metrics
{
template <typename Scalar, typename DataStruct>
class ForceAngleStabilityMargin : public StabilityMetricBase<ForceAngleStabilityMargin<Scalar, DataStruct>>
{
public:
  ForceAngleStabilityMargin(MinimumFunction<Scalar> minimum_function = standardMinimum)
    : StabilityMetricBase<ForceAngleStabilityMargin<Scalar, DataStruct>>(minimum_function)
  {}
  /*!
   * Computes the Force Angle Stability Measure (Papadopoulos, Rey, 1996) for the given support polygon and returns the
   * index of the least stable axis.
   * The stabilities for each axis are stored in the stability_vector. Previous values are overwritten.
   * @param support_polygon A support polygon where it is assumed that the points are ordered clockwise when viewed from
   * above.
   * @param data A data struct containing all addition
   * @param stability_vector A vector containting the stability values for all edges of the support polygon
   */
  void computeStabilityValueForAllEdgesImpl(const SupportPolygon<Scalar>& support_polygon, const DataStruct& data, std::vector<Scalar>& stability_vector)
  {
    stability_vector.resize(support_polygon.size());

    for (size_t i = 0; i < support_polygon.size(); ++i)
    {
      size_t b = i + 1;
      if (b == support_polygon.size())
        b = 0;
      Vector3<Scalar> axis = (support_polygon[b] - support_polygon[i]).normalized();
      Eigen::Matrix<Scalar, 3, 3> projection = Eigen::Matrix<Scalar, 3, 3>::Identity() - axis * axis.transpose();
      Vector3<Scalar> axis_normal = projection * (support_polygon[b] - data.com);
      // Since the mass normally doesn't change, we omit it
      Vector3<Scalar> force_component = projection * data.external_force;
      Scalar force_component_norm = force_component.norm();
      Vector3<Scalar> force_component_normalized = force_component / force_component_norm;
      Scalar distance = (-axis_normal + axis_normal.dot(force_component_normalized) * force_component_normalized).norm();
      axis_normal.normalize();
      Scalar sigma = axis_normal.cross(force_component_normalized).dot(axis) < 0 ? 1 : -1;
      Scalar theta = sigma * std::acos(force_component_normalized.dot(axis_normal));
      Scalar beta = theta * distance * force_component_norm;
      stability_vector[i] = beta * data.normalization_factor;
    }
  }
};

STABLITY_CREATE_DERIVED_METRIC_TRAITS(ForceAngleStabilityMargin)
}  // namespace hector_stability_metrics

#endif  // HECTOR_STABILITY_METRICS_FORCE_ANGLE_STABILITY_MEASURE_H
