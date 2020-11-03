#ifndef HECTOR_STABILITY_METRICS_STABILITY_METRIC_BASE_H
#define HECTOR_STABILITY_METRICS_STABILITY_METRIC_BASE_H

#include <functional>
#include <memory>

#include "hector_stability_metrics/support_polygon.h"
#include "hector_stability_metrics/types.h"
#include "hector_stability_metrics/math/minimum_functions.h"
#include "hector_stability_metrics/metrics/common_data.h"

namespace hector_stability_metrics
{
template <typename Scalar, typename Derived, typename DataStruct = CommonData<Scalar>>
class StabilityMetricBase
{
public:
  StabilityMetricBase(std::function<Scalar(VectorX<Scalar>)> minimum_function = StandardMinimum) { minimum_function_ = minimum_function; }

  Scalar getStabilityValue(const SupportPolygon<Scalar>& support_polygon, const DataStruct& data)
  {
    std::vector<Scalar> stability_values;
    return getStabilityValue(support_polygon, data, stability_values);
  }

  Scalar getStabilityValue(const SupportPolygon<Scalar>& support_polygon, const DataStruct& data, std::vector<Scalar>& stability_values)
  {
    getStabilityValueForAllEdges(data, stability_values);
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

  void computeStabilityValueForAllEdges(const SupportPolygon<Scalar>& support_polygon, const DataStruct& data, VectorX<Scalar>& stability_vector)
  {
    derived().getStabilityValueForAllEdgesImpl(support_polygon, data, stability_vector);
  }

private:
  std::function<Scalar(VectorX<Scalar>&)> minimum_function_;

  StabilityMetricBase& derived() { return *static_cast<Derived*>(this); }
  const StabilityMetricBase& derived() const { return *static_cast<const Derived*>(this); }
};

}  // namespace hector_stability_metrics

#endif  // HECTOR_STABILITY_METRICS_STABILITY_METRIC_BASE_H
