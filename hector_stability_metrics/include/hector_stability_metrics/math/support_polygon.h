// Copyright (c) 2020 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef HECTOR_STABILITY_METRICS_SUPPORT_POLYGON_H
#define HECTOR_STABILITY_METRICS_SUPPORT_POLYGON_H

#include "hector_stability_metrics/math/hull.h"
#include "hector_stability_metrics/math/types.h"

namespace hector_stability_metrics
{
namespace math
{

template<typename Container, typename Scalar = typename Eigen::DenseBase<typename Container::value_type>::Scalar>
void supportPolygonFromSortedContactPoints( const Container &points, Vector3List<Scalar> &result, Scalar threshold=Scalar(0.0) )
{
  math::convexHull( points, result, threshold );
}

template<typename Container, typename Scalar = typename Eigen::DenseBase<typename Container::value_type>::Scalar>
Vector3List<Scalar> supportPolygonFromSortedContactPoints( const Container &points, Scalar threshold=Scalar(0.0) )
{
  Vector3List<Scalar> result;
  math::convexHull( points, result, threshold );
  return result;
}

template<typename Container, typename Scalar = typename Eigen::DenseBase<typename Container::value_type>::Scalar>
Vector3List<Scalar> supportPolygonFromUnsortedContactPoints( const Container &points, Scalar threshold=Scalar(0.0) )
{
  Container copy = points;
  std::sort( copy.begin(), copy.end(),
             []( const typename Container::value_type &a, const typename Container::value_type &b )
             {
               if ( a.y() < b.y())
                 return true;
               return a.y() == b.y() && a.x() < b.x();
             } );
  return supportPolygonFromSortedContactPoints( copy, threshold );
}

/**
 * @brief getSupportPolygonEdge returns a vector pointing from one corner of a support polygon to the next.
 * @param support_polygon A vector of the corner points in the support polygon in clockwise order when viewed from above.
 * @param index The index of the corner of the suport polygon where the edge starts.
 * @return The vector from the corner with the specified index to the next corner.
 */
template<typename Scalar>
Vector3<Scalar> getSupportPolygonEdge( const Vector3List<Scalar> &support_polygon, size_t index )
{
  assert( index < support_polygon.size());

  size_t next_index = index + 1;
  if ( next_index == support_polygon.size())
  {
    next_index = 0;
  }
  return (support_polygon[next_index] - support_polygon[index]);
}
}  // namespace math
}  // namespace hector_stability_metrics

#endif  // HECTOR_STABILITY_METRICS_SUPPORT_POLYGON_H
