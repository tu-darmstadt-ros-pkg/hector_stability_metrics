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

#ifndef HECTOR_STABILITY_METRICS_ENERGY_STABILITY_MARGIN_H
#define HECTOR_STABILITY_METRICS_ENERGY_STABILITY_MARGIN_H

#include "hector_stability_metrics/support_polygon.h"
#include "hector_stability_metrics/types.h"
#include "hector_stability_metrics/metrics/stability_metric_base.h"
#include "hector_stability_metrics/math/sign_functions.h"

namespace hector_stability_metrics
{
template <typename Scalar, typename DataStruct = CommonData<Scalar>>
class NormalizedEnergyStabilityMargin : public StabilityMetricBase<NormalizedEnergyStabilityMargin<Scalar, DataStruct>>
{
public:
  /**
   * @brief NormalizedEnergyStabilityMargin This class computes the Normalized Energy Stability Margin by Hirose et al. as defined in "An improved energy stability margin for
   * walking machines subject to dynamic effects" (Garcia, De Santos, 2005). Additionally to the original formulation, a sign function can be given in order for  the stability
   * value to be negative for unstable edges of the support polygon. This is helpful if used to analyze or optimize the stability. The original unsigned version is used in the
   * default case by setting sign_function to a function allways returning 1.
   * @param minimum_function The minimum function that should be used for determining the overall stability.
   * @param sign_function If set to ConstantOne the original unsigened version as defined by Garcia and De Santos is used. If set to the standard sign function or a function
   * approximating it the stability values for unstable edges will be negative. As the sign function is not contiously differentiable it is recommended to use an approximation when
   * this metric is used for gradient based optimization.
   */
  NormalizedEnergyStabilityMargin(MinimumFunction<Scalar> minimum_function = standardMinimum, SignFunction<Scalar> sign_function = ConstantOne)
    : StabilityMetricBase<NormalizedEnergyStabilityMargin<Scalar, DataStruct>>(minimum_function)
  {
    sign_function_ = sign_function;
  }

  static Scalar ConstantOne(const Scalar& value) { return Scalar(1); }

  void computeStabilityValueForAllEdgesImpl(const SupportPolygon<Scalar>& support_polygon, const DataStruct& data, std::vector<Scalar>& stability_vector)
  {
    size_t nr_of_edges = support_polygon.size();
    stability_vector.resize(nr_of_edges);

    for (size_t i = 0; i < support_polygon.size(); ++i)
    {
      Vector3<Scalar> A = support_polygon[i];
      Vector3<Scalar> edge = getSupportPolygonEdge(support_polygon, i);
      Vector3<Scalar> corner_to_com = data.com - A;

      // linear algebra calculations adapted from Ericson Real Time Collision Detection
      Vector3<Scalar> closest_point_on_edge_to_com = A + corner_to_com.dot(edge) / edge.dot(edge) * edge;

      // compute normal of the vertical plane containing the line
      Vector3<Scalar> plane_normal = edge.cross(Vector3<Scalar>(0, 0, 1));
      Scalar distance_com_to_plane = plane_normal.normalized().dot(corner_to_com);

      // variables R, theta, psi and h based on "An improved energy stability margin for walking machines subject to dynamic effects" (Garcia, De Santos, 2005)
      Vector3<Scalar> R = data.com - closest_point_on_edge_to_com;

      // rotation around the edge neccessary to turn com into the vertical plane of the edge
      Scalar theta = asin(distance_com_to_plane / R.norm());
      // edge inclination relative to horizontal plane
      Scalar psi = asin(edge(2) / edge.norm());

      // mass and acceleration due to gravity are ignored in the normalized version, as they only change the scale
      stability_vector[i] = R.norm() * (1 - cos(theta)) * cos(psi);

      // multiply stability with the given sign_function of theta to enable returning negative stabilites if the vehicle is unstable
      stability_vector[i] *= sign_function_(theta);
    }
  }

private:
  SignFunction<Scalar> sign_function_;
};

STABLITY_CREATE_DERIVED_METRIC_TRAITS(NormalizedEnergyStabilityMargin)

}  // namespace hector_stability_metrics

#endif  // HECTOR_STABILITY_METRICS_ENERGY_STABILITY_MARGIN_H
