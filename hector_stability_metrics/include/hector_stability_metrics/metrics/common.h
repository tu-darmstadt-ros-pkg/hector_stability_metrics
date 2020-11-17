#ifndef HECTOR_STABILITY_METRICS_METRICS_COMMON_DATA_H
#define HECTOR_STABILITY_METRICS_METRICS_COMMON_DATA_H

#include "hector_stability_metrics/math/minimum.h"
#include "hector_stability_metrics/support_polygon.h"
#include "hector_stability_metrics/types.h"

#include <functional>

namespace hector_stability_metrics
{

template<typename Scalar, typename DataStruct>
using StabilityFunction = void( const SupportPolygon<Scalar> &, const DataStruct &,
                                std::vector<Scalar> &edge_stabilities );

template<typename Scalar>
struct CenterOfMassData
{
  Vector3<Scalar> center_of_mass;

  const Vector3<Scalar> &centerOfMass() const { return center_of_mass; }
};

template<typename Scalar>
struct ForceData
{
  Vector3<Scalar> external_force;

  const Vector3<Scalar> &externalForce() const { return external_force; }
};

//! @brief Provides a normalization constant that is applied to all edge stabilities.
template<typename Scalar>
struct NormalizationData
{
  Scalar normalization_factor;

  Scalar normalizationFactor() const { return normalization_factor; }
};

//! @brief Finds and returns the value and index of the edge with the lowest stability.
template<typename Scalar>
Scalar getLeastStableEdgeValue( const std::vector<Scalar> &edge_stabilities, size_t &least_stable_edge )
{
  Scalar min_value = std::numeric_limits<Scalar>::infinity();
  for ( int i = 0; i < edge_stabilities.size(); i++ )
  {
    if ( edge_stabilities[i] < min_value )
    {
      least_stable_edge = i;
      min_value = edge_stabilities[i];
    }
  }
  return min_value;
}

//! @brief Finds and returns the value and index of the edge with the lowest stability.
template<typename Scalar>
Scalar
getLeastStableEdgeValue( const SupportPolygonWithStabilities<Scalar> &support_polygon, size_t &least_stable_edge )
{
  return getLeastStableEdgeValue( support_polygon.axis_stabilities, least_stable_edge );
}

//! @brief Finds and returns the value of the edge with the lowest stability.
template<typename Scalar>
Scalar getLeastStableEdgeValue( const std::vector<Scalar> &edge_stabilities )
{
  size_t index;
  return getLeastStableEdgeValue( edge_stabilities, index );
}

//! @brief Finds and returns the value of the edge with the lowest stability.
template<typename Scalar>
Scalar getLeastStableEdgeValue( const SupportPolygonWithStabilities<Scalar> &support_polygon )
{
  return getLeastStableEdgeValue( support_polygon.axis_stabilities );
}

//! @brief Computes a stability value for the given stabilities using the provided minimum function. Default: Minimum of all values.
template<typename Scalar, math::MinimumFunction<Scalar> minimum = math::standardMinimum<Scalar>>
Scalar getMinimumStabilityValue( const std::vector<Scalar> &edge_stabilities )
{
  return minimum( edge_stabilities );
}

template<typename Scalar, math::MinimumFunction<Scalar> minimum = math::standardMinimum<Scalar>>
Scalar getMinimumStabilityValue( const SupportPolygonWithStabilities<Scalar> &support_polygon )
{
  return minimum( support_polygon.axis_stabilities );
}

template<typename Scalar, typename DataStruct, StabilityFunction<Scalar, DataStruct> computeStability>
Scalar computeLeastStableEdgeValue( const SupportPolygon<Scalar> &support_polygon, const DataStruct &data,
                                    std::vector<Scalar> &edge_stabilities, size_t &least_stable_edge )
{
  computeStability( support_polygon, data, edge_stabilities );
  return getLeastStableEdgeValue( edge_stabilities, least_stable_edge );
}

template<typename Scalar, typename DataStruct, StabilityFunction<Scalar, DataStruct> computeStability>
Scalar computeLeastStableEdgeValue( SupportPolygonWithStabilities<Scalar> &support_polygon, const DataStruct &data,
                                    size_t &least_stable_edge )
{
  computeStability( support_polygon, data );
  return getLeastStableEdgeValue( support_polygon.axis_stabilities, least_stable_edge );
}

template<typename Scalar, typename DataStruct, StabilityFunction<Scalar, DataStruct> computeStability, math::MinimumFunction<Scalar> minimum = math::standardMinimum<Scalar>>
Scalar computeMinimumStabilityValue( const SupportPolygon<Scalar> &support_polygon, const DataStruct &data,
                                     std::vector<Scalar> &edge_stabilities )
{
  computeStability( support_polygon, data, edge_stabilities );
  return getMinimumStabilityValue<Scalar, minimum>( edge_stabilities );
}

template<typename Scalar, typename DataStruct, StabilityFunction<Scalar, DataStruct> computeStability, math::MinimumFunction<Scalar> minimum = math::standardMinimum<Scalar>>
Scalar computeMinimumStabilityValue( const SupportPolygonWithStabilities<Scalar> &support_polygon,
                                     const DataStruct &data )
{
  computeStability( support_polygon, data );
  return getMinimumStabilityValue<Scalar, minimum>( support_polygon.axis_stabilities );
}
}  // namespace hector_stability_metrics

#endif  // HECTOR_STABILITY_METRICS_METRICS_COMMON_DATA_H
