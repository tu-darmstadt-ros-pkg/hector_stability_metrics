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

#ifndef HECTOR_STABILITY_METRICS_METRICS_NORMALIZED_ENERGY_STABILITY_MARGIN_H
#define HECTOR_STABILITY_METRICS_METRICS_NORMALIZED_ENERGY_STABILITY_MARGIN_H

#include "hector_stability_metrics/math/sign_functions.h"
#include "hector_stability_metrics/support_polygon.h"
#include "hector_stability_metrics/types.h"

namespace hector_stability_metrics
{

/**
 * @brief This method computes the Normalized Energy Stability Margin by Hirose et al. as defined in "An improved energy stability margin for
 * walking machines subject to dynamic effects" (Garcia, De Santos, 2005). Additionally to the original formulation, a sign function can be given in order for  the stability
 * value to be negative for unstable edges of the support polygon. This is helpful if used to analyze or optimize the stability.
 * Per default a fast signbit based method is used. The original unsigned version can be used by passing hector_stability_metrics::math::constantOneSignum as signFunction.
 * @tparam Scalar The scalar type used for computations.
 * @tparam DataStruct A data struct with additional information required by the stability measure.
 * @tparam signFunction A function returning the sign for the passed value. Check math/sign_functions.h for provided methods.
 *   Use hector_stability_metrics::math::constantOneSignum to obtain the original unsigned formulation.
 *   Since the default function is not continously differentiable, it is recommended to use an approximation when this metric is used for gradient based optimization.
 * 
 * @param support_polygon A support polygon where it is assumed that the points are ordered clockwise when viewed from
 * above.
 * @param data In this case a method named @b centerOfMass returning a Vector3<Scalar> representing the center of mass and a method @b externalForce returning a Vector3<Scalar> representing the sum of all forces acting on the center of mass are required.
 *   Optionally, you can also provide a method normalizationFactor returning a Scalar normalization factor that is applied to all edges.
 * @param edge_stabilities Output vector for the stability values for all edges of the support polygon
 */
template<typename Scalar, typename DataStruct, math::SignFunction<Scalar> signFunction = math::quickSignum>
void computeNormalizedEnergyStabilityMargin( const SupportPolygon<Scalar> &support_polygon, const DataStruct &data,
                                             std::vector<Scalar> &edge_stabilities )
{
  const size_t number_of_edges = support_polygon.size();
  edge_stabilities.resize( number_of_edges );

  for ( size_t i = 0; i < number_of_edges; ++i )
  {
    const Vector3<Scalar> &A = support_polygon[i];
    const Vector3<Scalar> &edge = getSupportPolygonEdge( support_polygon, i );
    const Vector3<Scalar> &corner_to_com = data.centerOfMass() - A;

    // linear algebra calculations adapted from Ericson Real Time Collision Detection
    const Vector3<Scalar> &closest_point_on_edge_to_com = A + corner_to_com.dot( edge ) / edge.dot( edge ) * edge;

    // compute normal of the vertical plane containing the line
    const Vector3<Scalar> &plane_normal = edge.cross( Vector3<Scalar>( 0, 0, 1 ));
    const Scalar distance_com_to_plane = plane_normal.normalized().dot( corner_to_com );

    // variables R, theta, psi and h based on "An improved energy stability margin for walking machines subject to dynamic effects" (Garcia, De Santos, 2005)
    const Vector3<Scalar> &R = data.centerOfMass() - closest_point_on_edge_to_com;

    // rotation around the edge neccessary to turn com into the vertical plane of the edge
    const Scalar theta = asin( distance_com_to_plane / R.norm());
    // edge inclination relative to horizontal plane
    const Scalar psi = asin( edge( 2 ) / edge.norm());

    // mass and acceleration due to gravity are ignored in the normalized version, as they only change the scale
    // multiply stability with the given sign_function of theta to enable returning negative stabilites if the vehicle is unstable
    edge_stabilities[i] = normalize((R.norm() * (1 - cos( theta )) * cos( psi )) * signFunction( theta ), data );
  }
}

template<typename Scalar, typename DataStruct, math::SignFunction<Scalar> signFunction = math::quickSignum<Scalar>>
Scalar computeNormalizedEnergyStabilityMarginLeastStableEdgeValue( const SupportPolygon<Scalar> &support_polygon,
                                                                   const DataStruct &data,
                                                                   std::vector<Scalar> &edge_stabilities,
                                                                   size_t &least_stable_edge )
{
  return computeLeastStableEdgeValue<Scalar, DataStruct, computeNormalizedEnergyStabilityMargin<Scalar, DataStruct, signFunction>>(
    support_polygon, data, edge_stabilities, least_stable_edge );
}

template<typename Scalar, typename DataStruct, math::SignFunction<Scalar> signFunction = math::quickSignum<Scalar>>
Scalar computeNormalizedEnergyStabilityMarginLeastStableEdgeValue(
  const SupportPolygonWithStabilities<Scalar> &support_polygon, const DataStruct &data, size_t &least_stable_edge )
{
  return computeLeastStableEdgeValue<Scalar, DataStruct, computeNormalizedEnergyStabilityMargin<Scalar, DataStruct, signFunction>>(
    support_polygon, data, least_stable_edge );
}

template<typename Scalar, typename DataStruct, math::SignFunction<Scalar> signFunction = math::quickSignum<Scalar>, math::MinimumFunction<Scalar> minimum = math::standardMinimum<Scalar>>
Scalar computeNormalizedEnergyStabilityMarginMinimumStabilityValue( const SupportPolygon<Scalar> &support_polygon,
                                                                    const DataStruct &data,
                                                                    std::vector<Scalar> &edge_stabilities )
{
  return computeMinimumStabilityValue<Scalar, DataStruct, computeNormalizedEnergyStabilityMargin<Scalar, DataStruct, signFunction>, minimum>(
    support_polygon, data, edge_stabilities );
}

template<typename Scalar, typename DataStruct, math::SignFunction<Scalar> signFunction = math::quickSignum<Scalar>, math::MinimumFunction<Scalar> minimum = math::standardMinimum<Scalar>>
Scalar computeNormalizedEnergyStabilityMarginMinimumStabilityValue(
  const SupportPolygonWithStabilities<Scalar> &support_polygon, const DataStruct &data )
{
  return computeMinimumStabilityValue<Scalar, DataStruct, computeNormalizedEnergyStabilityMargin<Scalar, DataStruct, signFunction>, minimum>(
    support_polygon, data );
}
}  // namespace hector_stability_metrics

#endif  // HECTOR_STABILITY_METRICS_METRICS_NORMALIZED_ENERGY_STABILITY_MARGIN_H
