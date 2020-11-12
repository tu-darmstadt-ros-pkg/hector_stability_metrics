/*
 * Copyright (C) 2020  Felix Biemüller
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

#ifndef HECTOR_STABILITY_METRICS_STATIC_STABILITY_MARGIN_H
#define HECTOR_STABILITY_METRICS_STATIC_STABILITY_MARGIN_H

#include "hector_stability_metrics/support_polygon.h"
#include "hector_stability_metrics/types.h"
#include "hector_stability_metrics/metrics/stability_metric_base.h"

namespace hector_stability_metrics
{
template <typename Scalar, typename DataStruct = CommonData<Scalar>>
class StaticStabilityMargin : public StabilityMetricBase<StaticStabilityMargin<Scalar, DataStruct>>
{
public:
  /** @brief StaticStabilityMargin This class computes the Static Stability Margin defined in "On the Stability Properties of Quadruped Creeping Gaits" (McGhee, Frank, 1968).
   * @param minimum_function The minimum function that should be used for determining the overall stability.
   * @param use_signed_version If set to false the original unsigened version as defined by Garcia and De Santos is used.
   */
  StaticStabilityMargin(MinimumFunction<Scalar> minimum_function = standardMinimum)
    : StabilityMetricBase<StaticStabilityMargin<Scalar, DataStruct>>(minimum_function)
  {}

  void computeStabilityValueForAllEdgesImpl(const SupportPolygon<Scalar>& support_polygon, const DataStruct& data, std::vector<Scalar>& stability_vector)
  {
    size_t nr_of_edges = support_polygon.size();
    stability_vector.resize(nr_of_edges);

    Matrix3<Scalar> project_2d;
    project_2d.setZero();
    project_2d(0, 0) = 1;
    project_2d(1, 1) = 1;
    for (int i = 0; i < nr_of_edges; i++)
    {
      Vector3<Scalar> edge = getSupportPolygonEdge(support_polygon, i);
      Vector3<Scalar> projected_edge = project_2d * edge;
      Vector3<Scalar> projected_edge_point = project_2d * support_polygon[i];
      Vector3<Scalar> projected_com = project_2d * data.com;

      Scalar signed_distance_to_edge = (projected_edge.normalized().cross(projected_edge_point - projected_com))(2);
      stability_vector[i] = signed_distance_to_edge;
    }
  }
};

STABLITY_CREATE_DERIVED_METRIC_TRAITS(StaticStabilityMargin)

}  // namespace hector_stability_metrics

#endif  // HECTOR_STABILITY_METRICS_STATIC_STABILITY_MARGIN_H
