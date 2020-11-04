#ifndef HECTOR_STABILITY_METRICS_ENERGY_STABILITY_MARGIN_H
#define HECTOR_STABILITY_METRICS_ENERGY_STABILITY_MARGIN_H

#include "hector_stability_metrics/support_polygon.h"
#include "hector_stability_metrics/types.h"
#include "hector_stability_metrics/metrics/stability_metric_base.h"

namespace hector_stability_metrics
{
template <typename Scalar, typename DataStruct = CommonData<Scalar>>
class EnergyStabilityMargin : public StabilityMetricBase<EnergyStabilityMargin<Scalar, DataStruct>>
{
public:
  EnergyStabilityMargin(MinimumFunction<Scalar> minimum_function = StandardMinimum)
    : StabilityMetricBase<EnergyStabilityMargin<Scalar, DataStruct>>(minimum_function)
  {}

  void computeStabilityValueForAllEdgesImpl(const SupportPolygon<Scalar>& support_polygon, const DataStruct& data, std::vector<Scalar>& stability_vector)
  {
    stability_vector.resize(support_polygon.size());

    for (Scalar& s : stability_vector)
    {
      s = 0.5;
    }
  }
};

STABLITY_CREATE_DERIVED_METRIC_TRAITS(EnergyStabilityMargin)

}  // namespace hector_stability_metrics

#endif  // HECTOR_STABILITY_METRICS_ENERGY_STABILITY_MARGIN_H
