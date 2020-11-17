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

#include "hector_stability_metrics/math/normalization.h"
#include "hector_stability_metrics/metrics/common.h"

#include <Eigen/Geometry>

namespace hector_stability_metrics
{

/*!
  * Computes the Force Angle Stability Measure (Papadopoulos, Rey, 1996) for the given support polygon and returns the
  * index of the least stable axis.
  * The stabilities for each axis are stored in the stability_vector. Previous values are overwritten.
  * 
  * @tparam Scalar The scalar type used for computations.
  * @tparam DataStruct A data struct with additional information required by the stability measure.
  * 
  * @param support_polygon A support polygon where it is assumed that the points are ordered clockwise when viewed from above.
  * @param data In this case a method named @b centerOfMass returning a Vector3<Scalar> representing the center of mass and a method @b externalForce returning a Vector3<Scalar> representing the sum of all forces acting on the center of mass are required.
 *   Optionally, you can also provide a method normalizationFactor returning a Scalar normalization factor that is applied to all edges.
  * @param edge_stabilities Output vector for the stability values for all edges of the support polygon
  */
template<typename Scalar, typename DataStruct>
void computeForceAngleStabilityMeasure( const SupportPolygon<Scalar> &support_polygon, const DataStruct &data,
                                        std::vector<Scalar> &edge_stabilities )
{
  const size_t number_of_edges = support_polygon.size();
  edge_stabilities.resize( number_of_edges );

  for ( size_t i = 0; i < number_of_edges; ++i )
  {
    const Vector3<Scalar> &axis = (getSupportPolygonEdge( support_polygon, i )).normalized();
    const Matrix3<Scalar> &projection = Matrix3<Scalar>::Identity() - axis * axis.transpose();
    Vector3<Scalar> axis_normal = projection * (support_polygon[i] - data.centerOfMass());
    // Since the mass normally doesn't change, we omit it
    const Vector3<Scalar> &force_component = projection * data.externalForce();
    const Scalar force_component_norm = force_component.norm();
    const Vector3<Scalar> &force_component_normalized = force_component / force_component_norm;
    const Scalar distance = (-axis_normal +
                             axis_normal.dot( force_component_normalized ) * force_component_normalized).norm();
    axis_normal.normalize();
    // TODO: Use atan2 formulation
    const Scalar sigma = axis_normal.cross( force_component_normalized ).dot( axis ) < 0 ? 1 : -1;
    const Scalar theta = sigma * std::acos( force_component_normalized.dot( axis_normal ));
    const Scalar beta = theta * distance * force_component_norm;
    edge_stabilities[i] = normalize( beta, data );
  }
}

template<typename Scalar, typename DataStruct>
Scalar computeForceAngleStabilityMeasureLeastStableEdgeValue( const SupportPolygon<Scalar> &support_polygon,
                                                              const DataStruct &data,
                                                              std::vector<Scalar> &edge_stabilities,
                                                              size_t &least_stable_edge )
{
  return computeLeastStableEdgeValue<Scalar, DataStruct, computeForceAngleStabilityMeasure<Scalar, DataStruct>>(
    support_polygon, data, edge_stabilities, least_stable_edge );
}

template<typename Scalar, typename DataStruct>
Scalar computeForceAngleStabilityMeasureLeastStableEdgeValue(
  const SupportPolygonWithStabilities<Scalar> &support_polygon, const DataStruct &data, size_t &least_stable_edge )
{
  return computeLeastStableEdgeValue<Scalar, DataStruct, computeForceAngleStabilityMeasure<Scalar, DataStruct>>(
    support_polygon, data, least_stable_edge );
}

template<typename Scalar, typename DataStruct, math::MinimumFunction<Scalar> minimum = math::standardMinimum<Scalar>>
Scalar computeForceAngleStabilityMeasureMinimumStabilityValue( const SupportPolygon<Scalar> &support_polygon,
                                                               const DataStruct &data,
                                                               std::vector<Scalar> &edge_stabilities )
{
  return computeMinimumStabilityValue<Scalar, DataStruct, computeForceAngleStabilityMeasure<Scalar, DataStruct>, minimum>(
    support_polygon, data, edge_stabilities );
}

template<typename Scalar, typename DataStruct, math::MinimumFunction<Scalar> minimum = math::standardMinimum<Scalar>>
Scalar computeForceAngleStabilityMeasureMinimumStabilityValue(
  const SupportPolygonWithStabilities<Scalar> &support_polygon, const DataStruct &data )
{
  return computeMinimumStabilityValue<Scalar, DataStruct, computeForceAngleStabilityMeasure<Scalar, DataStruct>, minimum>(
    support_polygon, data );
}
}  // namespace hector_stability_metrics

#endif  // HECTOR_STABILITY_METRICS_FORCE_ANGLE_STABILITY_MEASURE_H
