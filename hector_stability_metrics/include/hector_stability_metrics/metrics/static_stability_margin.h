/*
 * Copyright (C) 2020  Felix Biemüller, Stefan Fabian
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

#ifndef HECTOR_STABILITY_METRICS_METRICS_STATIC_STABILITY_MARGIN_H
#define HECTOR_STABILITY_METRICS_METRICS_STATIC_STABILITY_MARGIN_H

#include "hector_stability_metrics/support_polygon.h"
#include "hector_stability_metrics/types.h"

namespace hector_stability_metrics
{

/** @brief This method computes the Static Stability Margin defined in "On the Stability Properties of Quadruped Creeping Gaits" (McGhee, Frank, 1968).
 * 
 * @tparam Scalar The scalar type used for computations.
 * @tparam DataStruct A data struct with additional information required by the stability measure.
 * 
 * @param support_polygon A support polygon where it is assumed that the points are ordered clockwise when viewed from
 * above.
 * @param data In this case a method named @b centerOfMass returning a Vector3<Scalar> representing the center of mass is required.
 *   Optionally, you can also provide a method normalizationFactor returning a Scalar normalization factor that is applied to all edges.
 * @param edge_stabilities Output vector for the stability values for all edges of the support polygon
 */
template<typename Scalar, typename DataStruct>
void computeStaticStabilityMargin( const SupportPolygon <Scalar> &support_polygon, const DataStruct &data,
                                   std::vector<Scalar> &edge_stabilities )
{
  const size_t number_of_edges = support_polygon.size();
  edge_stabilities.resize( number_of_edges );

  Matrix3<Scalar> project_2d = Matrix3<Scalar>::Zero();
  project_2d( 0, 0 ) = 1;
  project_2d( 1, 1 ) = 1;
  const Vector3<Scalar> &projected_com = project_2d * data.centerOfMass();
  for ( int i = 0; i < number_of_edges; i++ )
  {
    const Vector3<Scalar> &edge = getSupportPolygonEdge( support_polygon, i );
    const Vector3<Scalar> &projected_edge = project_2d * edge;
    const Vector3<Scalar> &projected_edge_point = project_2d * support_polygon[i];

    Scalar signed_distance_to_edge = (projected_edge.normalized().cross( projected_edge_point - projected_com ))( 2 );
    edge_stabilities[i] = normalize( signed_distance_to_edge, data );
  }
}

template<typename Scalar, typename DataStruct>
Scalar computeStaticStabilityMarginLeastStableEdgeValue( const SupportPolygon <Scalar> &support_polygon,
                                                         const DataStruct &data, std::vector<Scalar> &edge_stabilities,
                                                         size_t &least_stable_edge )
{
  return computeLeastStableEdgeValue<Scalar, DataStruct, computeStaticStabilityMargin<Scalar, DataStruct>>(
    support_polygon, data, edge_stabilities, least_stable_edge );
}

template<typename Scalar, typename DataStruct>
Scalar computeStaticStabilityMarginLeastStableEdgeValue( const SupportPolygonWithStabilities <Scalar> &support_polygon,
                                                         const DataStruct &data, size_t &least_stable_edge )
{
  return computeLeastStableEdgeValue<Scalar, DataStruct, computeStaticStabilityMargin<Scalar, DataStruct>>(
    support_polygon, data, least_stable_edge );
}

template<typename Scalar, typename DataStruct, math::MinimumFunction <Scalar> minimum = math::standardMinimum < Scalar>>

Scalar computeStaticStabilityMarginMinimumStabilityValue( const SupportPolygon <Scalar> &support_polygon,
                                                          const DataStruct &data,
                                                          std::vector<Scalar> &edge_stabilities )
{
  return computeMinimumStabilityValue<Scalar, DataStruct, computeStaticStabilityMargin<Scalar, DataStruct>, minimum>(
    support_polygon, data, edge_stabilities );
}

template<typename Scalar, typename DataStruct, math::MinimumFunction <Scalar> minimum = math::standardMinimum < Scalar>>

Scalar computeStaticStabilityMarginMinimumStabilityValue( const SupportPolygonWithStabilities <Scalar> &support_polygon,
                                                          const DataStruct &data )
{
  return computeMinimumStabilityValue<Scalar, DataStruct, computeStaticStabilityMargin<Scalar, DataStruct>, minimum>(
    support_polygon, data );
}
}  // namespace hector_stability_metrics

#endif  // HECTOR_STABILITY_METRICS_STATIC_STABILITY_MARGIN_H
