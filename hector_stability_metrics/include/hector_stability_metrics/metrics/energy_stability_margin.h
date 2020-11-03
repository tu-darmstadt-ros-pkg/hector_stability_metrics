#ifndef HECTOR_STABILITY_METRICS_ENERGY_STABILITY_MARGIN_H
#define HECTOR_STABILITY_METRICS_ENERGY_STABILITY_MARGIN_H

#include "hector_stability_metrics/support_polygon.h"
#include "hector_stability_metrics/types.h"
#include "hector_stability_metrics/metrics/stability_metric_base2.h"

namespace hector_stability_metrics
{
template <typename Scalar, typename DataStruct = CommonData<Scalar>>
class EnergyStabilityMargin : StabilityMetricBase<Scalar, EnergyStabilityMargin<Scalar, DataStruct>, DataStruct>
{
  VectorX<Scalar> getStabilityValueForAllEdgesImpl()
  {
    this->data_ptr_->support_polygon;
    this->data_ptr_->com;
  }
};
}  // namespace hector_stability_metrics

#endif  // HECTOR_STABILITY_METRICS_ENERGY_STABILITY_MARGIN_H
