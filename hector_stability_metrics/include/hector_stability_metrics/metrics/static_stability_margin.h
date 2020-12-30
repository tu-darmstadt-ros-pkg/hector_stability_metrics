// Copyright (c) 2020 Felix Biemüller, Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef HECTOR_STABILITY_METRICS_METRICS_STATIC_STABILITY_MARGIN_H
#define HECTOR_STABILITY_METRICS_METRICS_STATIC_STABILITY_MARGIN_H

#include "hector_stability_metrics/math/support_polygon.h"
#include "hector_stability_metrics/metrics/common.h"

namespace hector_stability_metrics
{

/*!
 * @defgroup SSM Static Stability Margin
 * @brief These methods compute the Static Stability Margin defined in "On the Stability Properties of Quadruped Creeping Gaits" (McGhee, Frank, 1968).
 * 
 * @tparam Scalar The scalar type used for computations.
 * 
 * @param support_polygon A support polygon where it is assumed that the points are ordered clockwise when viewed from
 * above.
 * @param edge_stabilities Output vector for the stability values for each edge in the support_polygon. Existing content is erased.
 * @param center_of_mass A Vector3<Scalar> representing the center of mass is required.
 * @param normalization_factor Optionally, you can also provide a Scalar normalization factor that is applied to all edges.
 *
 * @{
 */
template<typename Scalar>
void computeStaticStabilityMargin( const math::Vector3List <Scalar> &support_polygon,
                                   std::vector<Scalar> &edge_stabilities,
                                   const math::Vector3 <Scalar> &center_of_mass,
                                   Scalar normalization_factor = Scalar( 1 ));

//! @return The minimum stability value of all edges.
template<typename Scalar>
Scalar computeStaticStabilityMarginValue( const math::Vector3List <Scalar> &support_polygon,
                                          std::vector<Scalar> &edge_stabilities,
                                          const math::Vector3 <Scalar> &center_of_mass,
                                          Scalar normalization_factor = Scalar( 1 ));

//! @return The index of the minimum stability value of all edges.
template<typename Scalar>
size_t computeStaticStabilityMarginLeastStableEdgeIndex( const math::Vector3List <Scalar> &support_polygon,
                                                         std::vector<Scalar> &edge_stabilities,
                                                         const math::Vector3 <Scalar> &center_of_mass,
                                                         Scalar normalization_factor = Scalar( 1 ));

/*! @} */

namespace impl
{
template<typename Scalar, typename MinimumType = Scalar, typename MinimumSelector = math::MinimumSelector <Scalar, MinimumType>>
typename MinimumSelector::ReturnType computeStaticStabilityMargin( const math::Vector3List <Scalar> &support_polygon,
                                                                   std::vector<Scalar> &edge_stabilities,
                                                                   const math::Vector3 <Scalar> &center_of_mass,
                                                                   Scalar normalization_factor = Scalar( 1 ))
{
  const size_t number_of_edges = support_polygon.size();
  edge_stabilities.resize( number_of_edges );
  MinimumSelector minimum_selector;

  math::Matrix3<Scalar> project_2d = math::Matrix3<Scalar>::Zero();
  project_2d( 0, 0 ) = 1;
  project_2d( 1, 1 ) = 1;
  const math::Vector3<Scalar> &projected_com = project_2d * center_of_mass;
  for ( int i = 0; i < number_of_edges; i++ )
  {
    const math::Vector3<Scalar> &edge = math::getSupportPolygonEdge( support_polygon, i );
    const math::Vector3<Scalar> &projected_edge = project_2d * edge;
    const math::Vector3<Scalar> &projected_edge_point = project_2d * support_polygon[i];

    const Scalar signed_distance_to_edge = (projected_edge.normalized().cross( projected_edge_point - projected_com ))(
      2 );
    const Scalar value = normalization_factor * signed_distance_to_edge;
    edge_stabilities[i] = signed_distance_to_edge;
    minimum_selector.updateMinimum( i, value );
  }
  return minimum_selector.getMinimum();
}
}

template<typename Scalar>
void computeStaticStabilityMargin( const math::Vector3List <Scalar> &support_polygon,
                                   std::vector<Scalar> &edge_stabilities,
                                   const math::Vector3 <Scalar> &center_of_mass, Scalar normalization_factor )
{
  impl::computeStaticStabilityMargin<Scalar, void>( support_polygon, edge_stabilities,
                                                    center_of_mass, normalization_factor );
}

template<typename Scalar>
Scalar computeStaticStabilityMarginValue( const math::Vector3List <Scalar> &support_polygon,
                                          std::vector<Scalar> &edge_stabilities,
                                          const math::Vector3 <Scalar> &center_of_mass,
                                          Scalar normalization_factor )
{
  return impl::computeStaticStabilityMargin<Scalar>( support_polygon, edge_stabilities,
                                                     center_of_mass, normalization_factor );
}

template<typename Scalar>
size_t computeStaticStabilityMarginLeastStableEdgeIndex( const math::Vector3List <Scalar> &support_polygon,
                                                         std::vector<Scalar> &edge_stabilities,
                                                         const math::Vector3 <Scalar> &center_of_mass,
                                                         Scalar normalization_factor )
{
  return impl::computeStaticStabilityMargin<Scalar, size_t>( support_polygon, edge_stabilities,
                                                             center_of_mass, normalization_factor );
}
}  // namespace hector_stability_metrics

#endif  // HECTOR_STABILITY_METRICS_STATIC_STABILITY_MARGIN_H
