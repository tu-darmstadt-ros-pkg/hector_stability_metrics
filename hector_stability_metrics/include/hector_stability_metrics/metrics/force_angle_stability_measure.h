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

#include "hector_stability_metrics/metrics/common.h"

#include <Eigen/Geometry>

namespace hector_stability_metrics
{
/*!
 * @defgroup FASM Force-Angle Stability Measure
 *
 * These methods compute the Force Angle Stability Measure (Papadopoulos, Rey, 1996) for the given support polygon.
 * The stabilities for each axis are stored in the edge_stabilities vector. Previous values are overwritten.
 *
 * @tparam Scalar The scalar type used for computations.
 *
 * @param support_polygon A support polygon where it is assumed that the points are ordered clockwise when viewed from above.
 * @param center_of_mass A Vector3<Scalar> representing the robot's center of mass.
 * @param external_force A Vector3<Scalar> representing the sum of all forces acting on the center of mass.
 * @param normalization_factor Optionally, you can also provide a Scalar normalization factor that is applied to all edges.
 *
 * @{
 */
/*!
 * @param edge_stabilities Output vector for the stability values for each edge in the support_polygon. Existing content is erased.
 */
template<typename Scalar>
void computeForceAngleStabilityMeasure( SupportPolygon<Scalar> &support_polygon,
                                        std::vector<Scalar> &edge_stabilities,
                                        const Vector3<Scalar> &center_of_mass,
                                        const Vector3<Scalar> &external_force,
                                        Scalar normalization_factor = Scalar( 1 ));

template<typename Scalar>
void computeForceAngleStabilityMeasure( SupportPolygonWithStabilities<Scalar> &support_polygon,
                                        const Vector3<Scalar> &center_of_mass,
                                        const Vector3<Scalar> &external_force,
                                        Scalar normalization_factor = Scalar( 1 ));

/*!
 * @param edge_stabilities Output vector for the stability values for each edge in the support_polygon. Existing content is erased.
 * @return The minimum stability value of all edges.
 */
template<typename Scalar>
Scalar computeForceAngleStabilityMeasureValue( SupportPolygon<Scalar> &support_polygon,
                                               std::vector<Scalar> &edge_stabilities,
                                               const Vector3<Scalar> &center_of_mass,
                                               const Vector3<Scalar> &external_force,
                                               Scalar normalization_factor = Scalar( 1 ));

/*!
 * @return The minimum stability value of all edges.
 */
template<typename Scalar>
Scalar computeForceAngleStabilityMeasureValue( SupportPolygonWithStabilities<Scalar> &support_polygon,
                                               const Vector3<Scalar> &center_of_mass,
                                               const Vector3<Scalar> &external_force,
                                               Scalar normalization_factor = Scalar( 1 ));

/*!
 * @param edge_stabilities Output vector for the stability values for each edge in the support_polygon. Existing content is erased.
 * @return The index of the minimum stability value of all edges.
 */
template<typename Scalar>
size_t computeForceAngleStabilityMeasureLeastStableEdgeIndex( SupportPolygon<Scalar> &support_polygon,
                                                              std::vector<Scalar> &edge_stabilities,
                                                              const Vector3<Scalar> &center_of_mass,
                                                              const Vector3<Scalar> &external_force,
                                                              Scalar normalization_factor = Scalar( 1 ));

/*!
 * @return The index of the minimum stability value of all edges.
 */
template<typename Scalar>
size_t computeForceAngleStabilityMeasureLeastStableEdgeIndex( SupportPolygonWithStabilities<Scalar> &support_polygon,
                                                              const Vector3<Scalar> &center_of_mass,
                                                              const Vector3<Scalar> &external_force,
                                                              Scalar normalization_factor = Scalar( 1 ));
/*! @} */

namespace impl
{

template<typename Scalar, typename MinimumType = Scalar, typename MinimumSelector = MinimumSelector<Scalar, MinimumType>>
typename MinimumSelector::ReturnType computeForceAngleStabilityMeasure( const SupportPolygon<Scalar> &support_polygon,
                                                                        std::vector<Scalar> &edge_stabilities,
                                                                        const Vector3<Scalar> &center_of_mass,
                                                                        const Vector3<Scalar> &external_force,
                                                                        Scalar normalization_factor = Scalar( 1 ))
{
  const size_t number_of_edges = support_polygon.size();
  edge_stabilities.resize( number_of_edges );
  MinimumSelector minimum_selector;

  for ( size_t i = 0; i < number_of_edges; ++i )
  {
    const Vector3<Scalar> &axis = (getSupportPolygonEdge( support_polygon, i )).normalized();
    const Matrix3<Scalar> &projection = Matrix3<Scalar>::Identity() - axis * axis.transpose();
    Vector3<Scalar> axis_normal = projection * (support_polygon[i] - center_of_mass);
    // Since the mass normally doesn't change, we omit it
    const Vector3<Scalar> &force_component = projection * external_force;
    const Scalar force_component_norm = force_component.norm();
    const Vector3<Scalar> &force_component_normalized = force_component / force_component_norm;
    const Scalar distance = (-axis_normal +
                             axis_normal.dot( force_component_normalized ) * force_component_normalized).norm();
    axis_normal.normalize();
    // TODO: Use atan2 formulation
    const Scalar sigma = axis_normal.cross( force_component_normalized ).dot( axis ) < 0 ? 1 : -1;
    const Scalar theta = sigma * std::acos( force_component_normalized.dot( axis_normal ));
    const Scalar beta = theta * distance * force_component_norm;
    const Scalar value = beta * normalization_factor;
    edge_stabilities[i] = value;
    minimum_selector.updateMinimum( i, value );
  }
  return minimum_selector.getMinimum();
}
}

template<typename Scalar>
void computeForceAngleStabilityMeasure( SupportPolygon<Scalar> &support_polygon,
                                        std::vector<Scalar> &edge_stabilities,
                                        const Vector3<Scalar> &center_of_mass,
                                        const Vector3<Scalar> &external_force,
                                        Scalar normalization_factor )
{
  impl::computeForceAngleStabilityMeasure<Scalar, void>( support_polygon, edge_stabilities, center_of_mass,
                                                         external_force, normalization_factor );
}

template<typename Scalar>
void computeForceAngleStabilityMeasure( SupportPolygonWithStabilities<Scalar> &support_polygon,
                                        const Vector3<Scalar> &center_of_mass,
                                        const Vector3<Scalar> &external_force,
                                        Scalar normalization_factor )
{
  impl::computeForceAngleStabilityMeasure<Scalar, void>( support_polygon.contact_hull_points,
                                                         support_polygon.edge_stabilities,
                                                         center_of_mass, external_force, normalization_factor );
}

template<typename Scalar>
Scalar computeForceAngleStabilityMeasureValue( SupportPolygon<Scalar> &support_polygon,
                                               std::vector<Scalar> &edge_stabilities,
                                               const Vector3<Scalar> &center_of_mass,
                                               const Vector3<Scalar> &external_force,
                                               Scalar normalization_factor )
{
  impl::computeForceAngleStabilityMeasure<Scalar>( support_polygon,
                                                   edge_stabilities,
                                                   center_of_mass, external_force, normalization_factor );
}

template<typename Scalar>
Scalar computeForceAngleStabilityMeasureValue( SupportPolygonWithStabilities<Scalar> &support_polygon,
                                               const Vector3<Scalar> &center_of_mass,
                                               const Vector3<Scalar> &external_force,
                                               Scalar normalization_factor )
{
  impl::computeForceAngleStabilityMeasure<Scalar>( support_polygon.contact_hull_points,
                                                   support_polygon.edge_stabilities,
                                                   center_of_mass, external_force, normalization_factor );
}

template<typename Scalar>
size_t computeForceAngleStabilityMeasureLeastStableEdgeIndex( SupportPolygon<Scalar> &support_polygon,
                                                              std::vector<Scalar> &edge_stabilities,
                                                              const Vector3<Scalar> &center_of_mass,
                                                              const Vector3<Scalar> &external_force,
                                                              Scalar normalization_factor )
{
  impl::computeForceAngleStabilityMeasure<Scalar, size_t>( support_polygon, edge_stabilities, center_of_mass,
                                                           external_force, normalization_factor );
}

template<typename Scalar>
size_t computeForceAngleStabilityMeasureLeastStableEdgeIndex( SupportPolygonWithStabilities<Scalar> &support_polygon,
                                                              const Vector3<Scalar> &center_of_mass,
                                                              const Vector3<Scalar> &external_force,
                                                              Scalar normalization_factor )
{
  impl::computeForceAngleStabilityMeasure<Scalar, size_t>( support_polygon.contact_hull_points,
                                                           support_polygon.edge_stabilities,
                                                           center_of_mass, external_force, normalization_factor );
}
}  // namespace hector_stability_metrics

#endif  // HECTOR_STABILITY_METRICS_FORCE_ANGLE_STABILITY_MEASURE_H
