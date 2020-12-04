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
 * @param center_of_mass A Vector3<Scalar> representing the center of mass is required.
 * @param normalization_factor Optionally, you can also provide a Scalar normalization factor that is applied to all edges.
 *
 * @{
 */
/*!
 * @param edge_stabilities Output vector for the stability values for each edge in the support_polygon. Existing content is erased.
 */
template<typename Scalar>
void computeStaticStabilityMargin( SupportPolygon<Scalar> &support_polygon, std::vector<Scalar> &edge_stabilities,
                                   const Vector3<Scalar> &center_of_mass, Scalar normalization_factor = Scalar( 1 ));

template<typename Scalar>
void computeStaticStabilityMargin( SupportPolygonWithStabilities<Scalar> &support_polygon,
                                   const Vector3<Scalar> &center_of_mass, Scalar normalization_factor = Scalar( 1 ));

/*!
 * @param edge_stabilities Output vector for the stability values for each edge in the support_polygon. Existing content is erased.
 * @return The minimum stability value of all edges.
 */
template<typename Scalar>
Scalar computeStaticStabilityMarginValue( SupportPolygon<Scalar> &support_polygon,
                                          std::vector<Scalar> &edge_stabilities,
                                          const Vector3<Scalar> &center_of_mass,
                                          Scalar normalization_factor = Scalar( 1 ));

/*!
 * @return The minimum stability value of all edges.
 */
template<typename Scalar>
Scalar computeStaticStabilityMarginValue( SupportPolygonWithStabilities<Scalar> &support_polygon,
                                          const Vector3<Scalar> &center_of_mass,
                                          Scalar normalization_factor = Scalar( 1 ));

/*!
 * @param edge_stabilities Output vector for the stability values for each edge in the support_polygon. Existing content is erased.
 * @return The index of the minimum stability value of all edges.
 */
template<typename Scalar>
size_t computeStaticStabilityMarginLeastStableEdgeIndex( SupportPolygon<Scalar> &support_polygon,
                                                         std::vector<Scalar> &edge_stabilities,
                                                         const Vector3<Scalar> &center_of_mass,
                                                         Scalar normalization_factor = Scalar( 1 ));

/*!
 * @return The index of the minimum stability value of all edges.
 */
template<typename Scalar>
size_t computeStaticStabilityMarginLeastStableEdgeIndex( SupportPolygonWithStabilities<Scalar> &support_polygon,
                                                         const Vector3<Scalar> &center_of_mass,
                                                         Scalar normalization_factor = Scalar( 1 ));

/*! @} */

namespace impl
{
template<typename Scalar, typename MinimumType = Scalar, typename MinimumSelector = MinimumSelector<Scalar, MinimumType>>
typename MinimumSelector::ReturnType computeStaticStabilityMargin( const SupportPolygon<Scalar> &support_polygon,
                                                                   std::vector<Scalar> &edge_stabilities,
                                                                   const Vector3<Scalar> &center_of_mass,
                                                                   Scalar normalization_factor = Scalar( 1 ))
{
  const size_t number_of_edges = support_polygon.size();
  edge_stabilities.resize( number_of_edges );
  MinimumSelector minimum_selector;

  Matrix3<Scalar> project_2d = Matrix3<Scalar>::Zero();
  project_2d( 0, 0 ) = 1;
  project_2d( 1, 1 ) = 1;
  const Vector3<Scalar> &projected_com = project_2d * center_of_mass;
  for ( int i = 0; i < number_of_edges; i++ )
  {
    const Vector3<Scalar> &edge = getSupportPolygonEdge( support_polygon, i );
    const Vector3<Scalar> &projected_edge = project_2d * edge;
    const Vector3<Scalar> &projected_edge_point = project_2d * support_polygon[i];

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
void computeStaticStabilityMargin( SupportPolygon<Scalar> &support_polygon, std::vector<Scalar> &edge_stabilities,
                                   const Vector3<Scalar> &center_of_mass, Scalar normalization_factor )
{
  impl::computeStaticStabilityMargin<Scalar, void>( support_polygon, edge_stabilities,
                                                    center_of_mass, normalization_factor );
}

template<typename Scalar>
void computeStaticStabilityMargin( SupportPolygonWithStabilities<Scalar> &support_polygon,
                                   const Vector3<Scalar> &center_of_mass, Scalar normalization_factor )
{
  impl::computeStaticStabilityMargin<Scalar, void>( support_polygon.contact_hull_points,
                                                    support_polygon.edge_stabilities,
                                                    center_of_mass, normalization_factor );
}

template<typename Scalar>
Scalar computeStaticStabilityMarginValue( SupportPolygon<Scalar> &support_polygon,
                                          std::vector<Scalar> &edge_stabilities, const Vector3<Scalar> &center_of_mass,
                                          Scalar normalization_factor )
{
  impl::computeStaticStabilityMargin<Scalar>( support_polygon, edge_stabilities,
                                              center_of_mass, normalization_factor );
}

template<typename Scalar>
Scalar computeStaticStabilityMarginValue( SupportPolygonWithStabilities<Scalar> &support_polygon,
                                          const Vector3<Scalar> &center_of_mass, Scalar normalization_factor )
{
  impl::computeStaticStabilityMargin<Scalar>( support_polygon.contact_hull_points, support_polygon.edge_stabilities,
                                              center_of_mass, normalization_factor );
}

template<typename Scalar>
size_t computeStaticStabilityMarginLeastStableEdgeIndex( SupportPolygon<Scalar> &support_polygon,
                                                         std::vector<Scalar> &edge_stabilities,
                                                         const Vector3<Scalar> &center_of_mass,
                                                         Scalar normalization_factor )
{
  impl::computeStaticStabilityMargin<Scalar, size_t>( support_polygon, edge_stabilities,
                                                      center_of_mass, normalization_factor );
}

template<typename Scalar>
size_t computeStaticStabilityMarginLeastStableEdgeIndex( SupportPolygonWithStabilities<Scalar> &support_polygon,
                                                         const Vector3<Scalar> &center_of_mass,
                                                         Scalar normalization_factor )
{
  impl::computeStaticStabilityMargin<Scalar, size_t>( support_polygon.contact_hull_points,
                                                      support_polygon.edge_stabilities,
                                                      center_of_mass, normalization_factor );
}
}  // namespace hector_stability_metrics

#endif  // HECTOR_STABILITY_METRICS_STATIC_STABILITY_MARGIN_H
