/*
 * Copyright (C) 2020  Felix Biemüller
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

#ifndef WALKER_CHAIR_STABILITY_METRIC_BASE_H
#define WALKER_CHAIR_STABILITY_METRIC_BASE_H

#include <functional>
#include <memory>

#include <hector_stability_metrics/support_polygon.h>
#include <hector_stability_metrics/types.h>
#include <hector_stability_metrics/math/minimum.h>
#include <hector_stability_metrics/metrics/common.h>

// clang-format off
#define STABLITY_CREATE_DERIVED_METRIC_TRAITS(DerivedMetric)          \
template <typename _Scalar, typename _DataStruct>               \
  struct base_traits<DerivedMetric<_Scalar, _DataStruct>>       \
  {                                                             \
    typedef _Scalar Scalar;                                     \
    typedef _DataStruct DataStruct;                             \
  };
// clang-format on

namespace walker_chair_stability_metrics
{
template <typename Derived>
struct base_traits;

template <typename Derived>
class StabilityMetricBase
{
public:
  typedef typename base_traits<Derived>::Scalar Scalar;
  typedef typename base_traits<Derived>::DataStruct DataStruct;

  StabilityMetricBase(hector_stability_metrics::math::MinimumFunction<Scalar> minimum_function = hector_stability_metrics::math::standardMinimum) { minimum_function_ = minimum_function; }

  Scalar getStabilityValue(const hector_stability_metrics::SupportPolygon<Scalar>& support_polygon, const DataStruct& data)
  {
    std::vector<Scalar> stability_values;
    return getStabilityValue(support_polygon, data, stability_values);
  }

  Scalar getStabilityValue(const hector_stability_metrics::SupportPolygon<Scalar>& support_polygon, const DataStruct& data, std::vector<Scalar>& stability_values)
  {
    computeStabilityValueForAllEdges(support_polygon, data, stability_values);
    return minimum_function_(stability_values);
  }

  Scalar getLeastStableEdgeValue(const std::vector<Scalar>& stability_values)
  {
    size_t least_stable_edge;
    return getLeastStableEdgeValue(stability_values, least_stable_edge);
  }

  Scalar getLeastStableEdgeValue(const std::vector<Scalar>& stability_values, size_t& least_stable_edge)
  {
    return hector_stability_metrics::getLeastStableEdgeValue(stability_values, least_stable_edge);
  }

  void computeStabilityValueForAllEdges(const hector_stability_metrics::SupportPolygon<Scalar>& support_polygon, const DataStruct& data, std::vector<Scalar>& stability_vector)
  {
    derived().computeStabilityValueForAllEdgesImpl(support_polygon, data, stability_vector);
  }

private:
  hector_stability_metrics::math::MinimumFunction<Scalar> minimum_function_;

  Derived& derived() { return *static_cast<Derived*>(this); }
  const Derived& derived() const { return *static_cast<const Derived*>(this); }
};

}  // namespace walker_chair_stability_metrics

#endif  // WALKER_CHAIR_STABILITY_METRIC_BASE_H
