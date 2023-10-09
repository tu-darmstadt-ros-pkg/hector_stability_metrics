// Copyright (c) 2020 Felix Biemüller, Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef HECTOR_STABILITY_METRICS_METRICS_NORMALIZED_ENERGY_STABILITY_MARGIN_H
#define HECTOR_STABILITY_METRICS_METRICS_NORMALIZED_ENERGY_STABILITY_MARGIN_H

#include "hector_stability_metrics/math/sign_functions.h"
#include "hector_stability_metrics/math/support_polygon.h"
#include "hector_stability_metrics/metrics/common.h"

namespace hector_stability_metrics
{
/*!
 * @defgroup NESM Normalized Energy Stability Margin
 *
 * @brief These methods compute the Normalized Energy Stability Margin by Hirose et al. as defined in "An improved energy stability margin for
 * walking machines subject to dynamic effects" (Garcia, De Santos, 2005). Additionally to the original formulation, a sign function can be given in order for  the stability
 * value to be negative for unstable edges of the support polygon. This is helpful if used to analyze or optimize the stability.
 * Per default a fast signbit based method is used. The original unsigned version can be used by passing hector_stability_metrics::math::constantOneSignum as signFunction.
 * @tparam Scalar The scalar type used for computations.
 * @tparam signFunction A function returning the sign for the passed value. Check math/sign_functions.h for provided methods.
 *   Use hector_stability_metrics::math::constantOneSignum to obtain the original unsigned formulation.
 *   Since the default function is not continously differentiable, it is recommended to use an approximation when this metric is used for gradient based optimization.
 *
 * @param support_polygon A support polygon where it is assumed that the points are ordered clockwise when viewed from above.
 * @param edge_stabilities Output vector for the stability values for each edge in the support_polygon. Existing content is erased.
 * @param center_of_mass A Vector3<Scalar> representing the center of mass.
 * @param normalization_factor Optionally, you can also provide a Scalar normalization factor that is applied to all edges.
 *
 * @{
 */

template<typename Scalar, math::SignFunction<Scalar> signFunction = math::quickSignum>
void computeNormalizedEnergyStabilityMargin( const math::Vector3List<Scalar> &support_polygon,
                                             std::vector<Scalar> &edge_stabilities,
                                             const math::Vector3<Scalar> &center_of_mass,
                                             Scalar normalization_factor = Scalar( 1 ) );

/*!
 * @return The minimum stability value of all edges.
 */
template<typename Scalar, math::SignFunction<Scalar> signFunction = math::quickSignum>
Scalar computeNormalizedEnergyStabilityMarginValue( const math::Vector3List<Scalar> &support_polygon,
                                                    std::vector<Scalar> &edge_stabilities,
                                                    const math::Vector3<Scalar> &center_of_mass,
                                                    Scalar normalization_factor = Scalar( 1 ) );

/*!
 * @return The index of the minimum stability value of all edges.
 */
template<typename Scalar, math::SignFunction<Scalar> signFunction = math::quickSignum>
size_t computeNormalizedEnergyStabilityMarginLeastStableEdgeIndex(
    const math::Vector3List<Scalar> &support_polygon, std::vector<Scalar> &edge_stabilities,
    const math::Vector3<Scalar> &center_of_mass, Scalar normalization_factor = Scalar( 1 ) );

/*! @} */

namespace impl
{
template<typename Scalar, math::SignFunction<Scalar> signFunction = math::quickSignum,
         typename MinimumType = Scalar,
         typename MinimumSelector = math::MinimumSelector<Scalar, MinimumType>>
typename MinimumSelector::ReturnType computeNormalizedEnergyStabilityMargin(
    const math::Vector3List<Scalar> &support_polygon, std::vector<Scalar> &edge_stabilities,
    const math::Vector3<Scalar> &center_of_mass, Scalar normalization_factor = Scalar( 1 ) )
{
  const size_t number_of_edges = support_polygon.size();
  edge_stabilities.resize( number_of_edges );
  MinimumSelector minimum_selector;

  for ( size_t i = 0; i < number_of_edges; ++i ) {
    const math::Vector3<Scalar> &A = support_polygon[i];
    const math::Vector3<Scalar> &edge = math::getSupportPolygonEdge( support_polygon, i );
    const math::Vector3<Scalar> &corner_to_com = center_of_mass - A;

    // linear algebra calculations adapted from Ericson Real Time Collision Detection
    const math::Vector3<Scalar> &closest_point_on_edge_to_com =
        A + corner_to_com.dot( edge ) / edge.dot( edge ) * edge;

    // compute normal of the vertical plane containing the line
    const math::Vector3<Scalar> &plane_normal = edge.cross( math::Vector3<Scalar>( 0, 0, 1 ) );
    // this is computed as Eigen's normalization is not useable with automatic differentiation
    const math::Vector3<Scalar> &plane_normal_normalized = plane_normal / plane_normal.norm();
    const Scalar distance_com_to_plane = plane_normal_normalized.dot( corner_to_com );

    // variables R, theta, psi and h based on "An improved energy stability margin for walking machines subject to dynamic effects" (Garcia, De Santos, 2005)
    const math::Vector3<Scalar> &R = center_of_mass - closest_point_on_edge_to_com;

    // rotation around the edge neccessary to turn com into the vertical plane of the edge
    const Scalar theta = asin( distance_com_to_plane / R.norm() );
    // edge inclination relative to horizontal plane
    const Scalar psi = asin( edge( 2 ) / edge.norm() );

    // mass and acceleration due to gravity are ignored in the normalized version, as they only change the scale
    // multiply stability with the given sign_function of theta to enable returning negative stabilites if the vehicle is unstable
    const Scalar value = normalization_factor * ( R.norm() * ( 1 - cos( theta ) ) * cos( psi ) ) *
                         signFunction( theta );
    minimum_selector.updateMinimum( i, value );
    edge_stabilities[i] = value;
  }
  return minimum_selector.getMinimum();
}
} // namespace impl

template<typename Scalar, math::SignFunction<Scalar> signFunction>
void computeNormalizedEnergyStabilityMargin( const math::Vector3List<Scalar> &support_polygon,
                                             std::vector<Scalar> &edge_stabilities,
                                             const math::Vector3<Scalar> &center_of_mass,
                                             Scalar normalization_factor )
{
  impl::computeNormalizedEnergyStabilityMargin<Scalar, signFunction, void>(
      support_polygon, edge_stabilities, center_of_mass, normalization_factor );
}

template<typename Scalar, math::SignFunction<Scalar> signFunction>
Scalar computeNormalizedEnergyStabilityMarginValue( const math::Vector3List<Scalar> &support_polygon,
                                                    std::vector<Scalar> &edge_stabilities,
                                                    const math::Vector3<Scalar> &center_of_mass,
                                                    Scalar normalization_factor )
{
  return impl::computeNormalizedEnergyStabilityMargin<Scalar, signFunction>(
      support_polygon, edge_stabilities, center_of_mass, normalization_factor );
}

template<typename Scalar, math::SignFunction<Scalar> signFunction>
size_t computeNormalizedEnergyStabilityMarginLeastStableEdgeIndex(
    const math::Vector3List<Scalar> &support_polygon, std::vector<Scalar> &edge_stabilities,
    const math::Vector3<Scalar> &center_of_mass, Scalar normalization_factor )
{
  return impl::computeNormalizedEnergyStabilityMargin<Scalar, signFunction, size_t>(
      support_polygon, edge_stabilities, center_of_mass, normalization_factor );
}
} // namespace hector_stability_metrics

#endif // HECTOR_STABILITY_METRICS_METRICS_NORMALIZED_ENERGY_STABILITY_MARGIN_H
