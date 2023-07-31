// Copyright (c) 2020 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef HECTOR_STABILITY_METRICS_HULL_H
#define HECTOR_STABILITY_METRICS_HULL_H

#include <cstddef>

namespace hector_stability_metrics
{
namespace math
{

template<typename T>
inline bool isClockwiseTurn( const T ox, const T oy, const T ax, const T ay, const T bx, const T by, T threshold=T(0.0) )
{
  // It's a counter clockwise turn if (ax - ox) * (by - oy) - (ay - oy) * (bx - ox) is greater than 0
  return (ax - ox) * (by - oy) < (ay - oy) * (bx - ox) - threshold;
}

template<typename MatrixBase1, typename MatrixBase2, typename MatrixBase3>
inline bool isClockwiseTurn( const MatrixBase1 &o, const MatrixBase2 &a, const MatrixBase3 &b, typename MatrixBase1::value_type threshold=typename MatrixBase1::value_type(0.0) )
{
  // It's a counter clockwise turn if (ax - ox) * (by - oy) - (ay - oy) * (bx - ox) is greater than 0
  return (a.x() - o.x()) * (b.y() - o.y()) < (a.y() - o.y()) * (b.x() - o.x()) - threshold;
}

/*!
  * Slightly modified implementation of Andrew's monotone chain 2D convex hull algorithm.
  * It is assumed that the points are already sorted by their x- or y- coordinate (ties are broken by the other coordinate).
  * The hull is passed via parameter so when making multiple calls, the PointList can be allocated once and passed each time
  * reducing costly allocations.
  *
  * @param points The candidate points for which the hull is computed
  * @param result The convex hull formed by a subset of the given points. The points are listed in clockwise order.
  * @param threshold Positive values increase the strictness for determining convexity and, therefore, simplify the support polygon.
  */
template<typename Container, typename Scalar = typename Eigen::DenseBase<typename Container::value_type>::Scalar>
inline void convexHull( const Container &points, Container &result, Scalar threshold=Scalar(0.0) )
{
  size_t n = points.size();
  result.clear();
  if ( n <= 2 )
  {
    result.reserve( 2 );
    result.insert( result.begin(), points.begin(), points.end());
    return;
  }

  size_t k = 0;
  result.resize( 2 * n );
  for ( size_t i = 0; i < n; ++i, ++k )
  {
    while ( k >= 2 && !isClockwiseTurn( result[k - 2], result[k - 1], points[i], threshold )) --k;
    result[k] = points[i];
  }
  for ( size_t i = n, t = k + 1; i > 0; --i, ++k )
  {
    const auto &pt = points[i - 1];
    while ( k >= t && !isClockwiseTurn( result[k - 2], result[k - 1], pt, threshold )) --k;
    result[k] = pt;
  }
  result.resize( k - 1 );
}
}
}  // namespace hector_stability_metrics

#endif  // HECTOR_STABILITY_METRICS_HULL_H