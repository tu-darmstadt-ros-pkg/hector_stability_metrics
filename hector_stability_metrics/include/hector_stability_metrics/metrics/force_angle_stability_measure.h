// Copyright (c) 2020 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef HECTOR_STABILITY_METRICS_FORCE_ANGLE_STABILITY_MEASURE_H
#define HECTOR_STABILITY_METRICS_FORCE_ANGLE_STABILITY_MEASURE_H

#include "hector_stability_metrics/math/support_polygon.h"
#include "hector_stability_metrics/metrics/common.h"

#include <Eigen/Geometry>

namespace hector_stability_metrics
{
namespace non_differentiable
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
 * @param edge_stabilities Output vector for the stability values for each edge in the support_polygon. Existing content is erased.
 * @param center_of_mass A Vector3<Scalar> representing the robot's center of mass.
 * @param external_force A Vector3<Scalar> representing the sum of all forces acting on the center of mass.
 * @param normalization_factor Optionally, you can also provide a Scalar normalization factor that is applied to all edges.
 *
 * @{
 */
template<typename Scalar>
void computeForceAngleStabilityMeasure( const Vector3List<Scalar> &support_polygon,
                                        std::vector<Scalar> &edge_stabilities,
                                        const Vector3<Scalar> &center_of_mass,
                                        const Vector3<Scalar> &external_force,
                                        Scalar normalization_factor = Scalar( 1 ));

//! @return The minimum stability value of all edges.
template<typename Scalar>
Scalar computeForceAngleStabilityMeasureValue( const Vector3List<Scalar> &support_polygon,
                                               std::vector<Scalar> &edge_stabilities,
                                               const Vector3<Scalar> &center_of_mass,
                                               const Vector3<Scalar> &external_force,
                                               Scalar normalization_factor = Scalar( 1 ));

//! @return The index of the minimum stability value of all edges.
template<typename Scalar>
size_t computeForceAngleStabilityMeasureLeastStableEdgeIndex( const Vector3List<Scalar> &support_polygon,
                                                              std::vector<Scalar> &edge_stabilities,
                                                              const Vector3<Scalar> &center_of_mass,
                                                              const Vector3<Scalar> &external_force,
                                                              Scalar normalization_factor = Scalar( 1 ));
/*! @} */

namespace impl
{

template<typename Scalar, typename MinimumType = Scalar, typename MinimumSelector = math::MinimumSelector<Scalar, MinimumType>>
typename MinimumSelector::ReturnType computeForceAngleStabilityMeasure( const Vector3List<Scalar> &support_polygon,
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
    const Vector3<Scalar> &axis = (math::getSupportPolygonEdge( support_polygon, i )).normalized();
    const Matrix3<Scalar> &projection = Matrix3<Scalar>::Identity() - axis * axis.transpose();
    Vector3<Scalar> axis_normal = projection * (support_polygon[i] - center_of_mass);
    // Since the mass normally doesn't change, we omit it
    const Vector3<Scalar> &force_component = projection * external_force;
    const Scalar force_component_norm = force_component.norm();
    const Vector3<Scalar> &force_component_normalized = force_component / force_component_norm;
    const Scalar distance = (-axis_normal +
                             axis_normal.dot( force_component_normalized ) * force_component_normalized).norm();
    axis_normal.normalize();
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
void computeForceAngleStabilityMeasure( const Vector3List<Scalar> &support_polygon,
                                        std::vector<Scalar> &edge_stabilities,
                                        const Vector3<Scalar> &center_of_mass,
                                        const Vector3<Scalar> &external_force,
                                        Scalar normalization_factor )
{
  impl::computeForceAngleStabilityMeasure<Scalar, void>( support_polygon, edge_stabilities, center_of_mass,
                                                         external_force, normalization_factor );
}

template<typename Scalar>
Scalar computeForceAngleStabilityMeasureValue( const Vector3List<Scalar> &support_polygon,
                                               std::vector<Scalar> &edge_stabilities,
                                               const Vector3<Scalar> &center_of_mass,
                                               const Vector3<Scalar> &external_force,
                                               Scalar normalization_factor )
{
  return impl::computeForceAngleStabilityMeasure<Scalar>( support_polygon, edge_stabilities,
                                                          center_of_mass, external_force, normalization_factor );
}

template<typename Scalar>
size_t computeForceAngleStabilityMeasureLeastStableEdgeIndex( const Vector3List<Scalar> &support_polygon,
                                                              std::vector<Scalar> &edge_stabilities,
                                                              const Vector3<Scalar> &center_of_mass,
                                                              const Vector3<Scalar> &external_force,
                                                              Scalar normalization_factor )
{
  return impl::computeForceAngleStabilityMeasure<Scalar, size_t>( support_polygon, edge_stabilities, center_of_mass,
                                                                  external_force, normalization_factor );
}
}  // non_differentiable
}  // namespace hector_stability_metrics

#endif  // HECTOR_STABILITY_METRICS_FORCE_ANGLE_STABILITY_MEASURE_H
