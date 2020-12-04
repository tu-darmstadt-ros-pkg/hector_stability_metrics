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

#ifndef HECTOR_STABILITY_METRICS_SUPPORT_POLYGON_H
#define HECTOR_STABILITY_METRICS_SUPPORT_POLYGON_H

#include "hector_stability_metrics/math/hull.h"
#include "hector_stability_metrics/types.h"

namespace hector_stability_metrics
{
template<typename Scalar>
using SupportPolygon = Vector3List<Scalar>;

using SupportPolygonf = SupportPolygon<float>;
using SupportPolygond = SupportPolygon<double>;

template<typename Scalar>
struct SupportPolygonWithStabilities
{
  //! The convex hull of the contact points of the robot with the ground. The contact points should be ordered clockwise
  //! when viewed from above.
  SupportPolygon<Scalar> contact_hull_points;
  //! The stability of each edge. Index 0 is the stability of the edge from hull point 0 to hull point 1.
  std::vector<Scalar> edge_stabilities;
};

using SupportPolygonWithStabilitiesf = SupportPolygonWithStabilities<float>;
using SupportPolygonWithStabilitiesd = SupportPolygonWithStabilities<double>;

template<typename Container, typename Scalar = typename Eigen::DenseBase<typename Container::value_type>::Scalar>
void supportPolygonFromSortedContactPoints( const Container &points, SupportPolygon<Scalar> &result )
{
  math::convexHull( points, result );
}

template<typename Container, typename Scalar = typename Eigen::DenseBase<typename Container::value_type>::Scalar>
SupportPolygon<Scalar> supportPolygonFromSortedContactPoints( const Container &points )
{
  SupportPolygon<Scalar> result;
  math::convexHull( points, result );
  return result;
}

template<typename Container, typename Scalar = typename Eigen::DenseBase<typename Container::value_type>::Scalar>
SupportPolygon<Scalar> supportPolygonFromUnsortedContactPoints( const Container &points )
{
  Container copy = points;
  std::sort( copy.begin(), copy.end(),
             []( const typename Container::value_type &a, const typename Container::value_type &b )
             {
               if ( a.y() < b.y())
                 return true;
               return a.y() == b.y() && a.x() < b.x();
             } );
  return supportPolygonFromSortedContactPoints( copy );
}

/**
 * @brief getSupportPolygonEdge returns a vector pointing from one corner of a support polygon to the next.
 * @param support_polygon A vector of the corner points in the support polygon in clockwise order when viewed from above.
 * @param index The index of the corner of the suport polygon where the edge starts.
 * @return The vector from the corner with the specified index to the next corner.
 */
template<typename Scalar>
Vector3<Scalar> getSupportPolygonEdge( const SupportPolygon<Scalar> &support_polygon, size_t index )
{
  assert( index < support_polygon.size());

  size_t next_index = index + 1;
  if ( next_index == support_polygon.size())
  {
    next_index = 0;
  }
  return (support_polygon[next_index] - support_polygon[index]);
}
}  // namespace hector_stability_metrics

#endif  // HECTOR_STABILITY_METRICS_SUPPORT_POLYGON_H
