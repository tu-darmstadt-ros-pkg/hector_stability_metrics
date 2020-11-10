#ifndef HECTOR_STABILITY_METRICS_STABILITY_METRIC_BASE_H
#define HECTOR_STABILITY_METRICS_STABILITY_METRIC_BASE_H

#include <functional>
#include <memory>

#include "hector_stability_metrics/support_polygon.h"
#include "hector_stability_metrics/types.h"
#include "hector_stability_metrics/math/minimum_functions.h"
#include "hector_stability_metrics/metrics/common_data.h"

// clang-format off
#define STABLITY_CREATE_DERIVED_METRIC_TRAITS(DerivedMetric)          \
template <typename _Scalar, typename _DataStruct>               \
  struct base_traits<DerivedMetric<_Scalar, _DataStruct>>       \
  {                                                             \
    typedef _Scalar Scalar;                                     \
    typedef _DataStruct DataStruct;                             \
  };
// clang-format on

namespace hector_stability_metrics
{
template <typename Derived>
struct base_traits;

template <typename Derived>
class StabilityMetricBase
{
public:
  typedef typename base_traits<Derived>::Scalar Scalar;
  typedef typename base_traits<Derived>::DataStruct DataStruct;

  StabilityMetricBase(MinimumFunction<Scalar> minimum_function = StandardMinimum) { minimum_function_ = minimum_function; }

  Scalar getStabilityValue(const SupportPolygon<Scalar>& support_polygon, const DataStruct& data)
  {
    std::vector<Scalar> stability_values;
    return getStabilityValue(support_polygon, data, stability_values);
  }

  Scalar getStabilityValue(const SupportPolygon<Scalar>& support_polygon, const DataStruct& data, std::vector<Scalar>& stability_values)
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
    Scalar min_value = stability_values[0];
    for (int i = 1; i < stability_values.size(); i++)
    {
      if (stability_values[i] < min_value)
      {
        least_stable_edge = i;
        min_value = stability_values[i];
      }
    }
    return min_value;
  }

  void computeStabilityValueForAllEdges(const SupportPolygon<Scalar>& support_polygon, const DataStruct& data, std::vector<Scalar>& stability_vector)
  {
    derived().computeStabilityValueForAllEdgesImpl(support_polygon, data, stability_vector);
  }

private:
  MinimumFunction<Scalar> minimum_function_;

  Derived& derived() { return *static_cast<Derived*>(this); }
  const Derived& derived() const { return *static_cast<const Derived*>(this); }
};

}  // namespace hector_stability_metrics

#endif  // HECTOR_STABILITY_METRICS_STABILITY_METRIC_BASE_H
