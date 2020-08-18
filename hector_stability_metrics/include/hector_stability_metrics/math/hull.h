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

#ifndef HECTOR_STABILITY_METRICS_HULL_H
#define HECTOR_STABILITY_METRICS_HULL_H

namespace hector_stability_metrics
{
namespace math
{

template<typename T>
inline bool isClockwiseTurn( const T ox, const T oy, const T ax, const T ay, const T bx, const T by )
{
  // It's a counter clockwise turn if (ax - ox) * (by - oy) - (ay - oy) * (bx - ox) is greater than 0
  return (ax - ox) * (by - oy) < (ay - oy) * (bx - ox);
}

template<typename MatrixBase1, typename MatrixBase2, typename MatrixBase3>
inline bool isClockwiseTurn( const MatrixBase1 &o, const MatrixBase2 &a, const MatrixBase3 &b )
{
  // It's a counter clockwise turn if (ax - ox) * (by - oy) - (ay - oy) * (bx - ox) is greater than 0
  return (a.x() - o.x()) * (b.y() - o.y()) < (a.y() - o.y()) * (b.x() - o.x());
}

/*!
  * Slightly modified implementation of Andrew's monotone chain 2D convex hull algorithm.
  * It is assumed that the points are already sorted by their x- or y- coordinate (ties are broken by the other coordinate).
  * The hull is passed via parameter so when making multiple calls, the PointList can be allocated once and passed each time
  * reducing costly allocations.
  *
  * @param points The candidate points for which the hull is computed
  * @param hull The convex hull formed by a subset of the give points. The points are listed in clockwise order.
  */
template<typename Container>
inline void convexHull( const Container &points, Container &result )
{
  size_t n = points.size();
  result.clear();
  if ( n <= 3 )
  {
    result.reserve( 3 );
    result.insert( result.begin(), points.begin(), points.end());
    return;
  }

  size_t k = 0;
  result.resize( 2 * n );
  for ( size_t i = 0; i < n; ++i, ++k )
  {
    while ( k >= 2 && !isClockwiseTurn( result[k - 2], result[k - 1], points[i] )) --k;
    result[k] = points[i];
  }
  for ( size_t i = n, t = k + 1; i > 0; --i, ++k )
  {
    const auto &pt = points[i - 1];
    while ( k >= t && !isClockwiseTurn( result[k - 2], result[k - 1], pt )) --k;
    result[k] = pt;
  }
  result.resize( k - 1 );
}
}
}  // namespace hector_stability_metrics

#endif  // HECTOR_STABILITY_METRICS_HULL_H